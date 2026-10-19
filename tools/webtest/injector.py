#!/usr/bin/env python3
"""
注入器：在没有 Gazebo 的环境里，向 ROS 图发布真实的 /map、/scan、TF、/plan。

**为什么需要它**
本机沙箱有硬限制：/dev/shm 只有 10MB、进程最多约 60 个线程，Gazebo 一 spawn
模型就 SIGABRT 崩溃。所以没法用仿真栈来测网页控制台的显示链路。

但网关真正依赖的是**话题数据**，不是 Gazebo 本身。这个脚本按标准消息格式
发布与真机等价的 OccupancyGrid / LaserScan / TF / Path，把网关当成黑盒，
验证它能不能正确接收、编码、下发前端 —— 也就是交接文档里"完全没在真机跑过"
的那几个点：
  ① aiohttp 与 rclpy 线程交互是否真的通
  ② 地图 PNG 编码的真实视觉效果
  ③ 位姿来自 map→base 的 TF 能否查到
  ④ 激光点能否按位姿变换到地图坐标系

发布内容（模拟一个 6m x 4m 房间，中间有根柱子）：
  /map      20 格/米的栅格地图，transient_local（晚订阅也能拿到）
  /scan    360° 激光，障碍距离按几何计算
  TF        map→odom→base_footprint，机器人会原地缓慢转圈
  /plan     一条直线路径
  /agv/task_state 周期性任务状态
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from rclpy.clock import Clock, ClockType
from rosgraph_msgs.msg import Clock as ClockMsg
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from agv_interfaces.msg import TaskState

RES = 0.05                 # 米/格
W, H = 120, 80             # 6m x 4m
ORIGIN = (-3.0, -2.0)      # 左下角世界坐标


def build_grid():
    """造一张 6x4 米房间的地图：四周墙壁 + 中心一根柱子。"""
    data = [0] * (W * H)
    for y in range(H):
        for x in range(W):
            wx = ORIGIN[0] + (x + 0.5) * RES
            wy = ORIGIN[1] + (y + 0.5) * RES
            wall = (abs(wx) > 2.85 or abs(wy) > 1.85)          # 外墙
            pillar = (abs(wx - 0.9) < 0.35 and abs(wy - 0.4) < 0.35)  # 柱子
            if wall or pillar:
                data[y * W + x] = 100                          # 占据
    return data


class ClockSource(Node):
    """独立发 /clock 的节点，**必须走系统时钟**。

    ★ 为什么单独一个节点：主注入器在 --sim 模式下 use_sim_time=True，
      它的定时器要等 /clock 才推进；如果让它自己发 /clock，就永远等不到
      第一帧 —— 死锁。Gazebo 里不存在这个问题，因为 /clock 是 Gazebo
      进程（真实时间驱动）发的，不是受时钟节制的 ROS 节点发的。
      这里就用一个走 wall time 的节点忠实还原这个角色。
    """

    def __init__(self):
        super().__init__('agv_test_clock_source',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.Parameter.Type.BOOL, False)])
        self.pub = self.create_publisher(ClockMsg, '/clock', 10)
        self._t0 = time.monotonic()
        self.create_timer(0.02, self._tick)      # 50 Hz，像 Gazebo

    def _tick(self):
        m = ClockMsg()
        t = time.monotonic() - self._t0            # 仿真时间从 0 起
        m.clock.sec = int(t)
        m.clock.nanosec = int((t - int(t)) * 1e9)
        self.pub.publish(m)


class Injector(Node):
    def __init__(self, sim=False):
        # ★ sim=True 时本节点也切到仿真时钟（和 Gazebo 栈一致）。
        #   这是复现"两个模式都看不到小车"的必要条件：
        #   上游用 sim time 发 TF、网关用 wall time 判新旧 → 差 57 年。
        super().__init__('agv_test_injector',
                         parameter_overrides=(
                             [rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.Parameter.Type.BOOL, True)]
                             if sim else None))
        self.sim = sim
        qos_map = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', qos_map)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_plan = self.create_publisher(Path, '/plan', 10)
        self.pub_task = self.create_publisher(TaskState, '/agv/task_state', 10)
        self.pub_manual = self.create_publisher(Bool, '/agv/manual_mode', 10)
        self.pub_raw = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.pub_web = self.create_publisher(Twist, '/agv/web_teleop', 10)
        self.tf = TransformBroadcaster(self)
        # ★ /clock 不能由本节点发：本节点 use_sim_time=True 时，它的定时器
        #   要等 /clock 到来才推进 —— 自己发给自己 = 死锁（第一次就这么挂了，
        #   表现是 spin 里抛 ExternalShutdownException，一个话题都没发出去）。
        #   所以 /clock 由一个独立的、走系统时钟的节点发（见 ClockSource）。

        self.grid = build_grid()
        self.t0 = time.time()
        if self.sim:
            self.get_logger().info(
                '注入器已启动（★ 仿真时基模式）：发 /clock，时间戳从 0 起；'
                '用于复现"网关 wall time vs 上游 sim time"导致小车不显示')
        # 订阅网关发出的速度，用于验证"网页→ROS"方向真的通了
        self.create_subscription(Twist, '/cmd_vel_raw', self.on_raw, 10)
        self.create_subscription(Twist, '/agv/web_teleop', self.on_web, 10)
        self.got_raw = 0
        self.got_web = 0
        self.last_raw = (0.0, 0.0)
        self.last_web = (0.0, 0.0)

        self.create_timer(1.0, self.tick_map)
        self.create_timer(0.1, self.tick_fast)
        self.get_logger().info('注入器已启动：发布 /map /scan TF /plan')

    def on_raw(self, msg):
        self.got_raw += 1
        self.last_raw = (msg.linear.x, msg.angular.z)
        print(f'[RAW] #{self.got_raw} v={msg.linear.x:.3f} w={msg.angular.z:.3f}',
              flush=True)

    def on_web(self, msg):
        self.got_web += 1
        self.last_web = (msg.linear.x, msg.angular.z)
        print(f'[WEB] #{self.got_web} v={msg.linear.x:.3f} w={msg.angular.z:.3f}',
              flush=True)

    def _tick_clock(self):
        """（保留但不再使用）详见 ClockSource 的注释：不能自己给自己发时钟。"""
        return

    def pose(self):
        """机器人原地慢慢自转，方便看位姿箭头是否跟着动。"""
        t = time.time() - self.t0
        return 0.0, 0.0, math.sin(t * 0.3) * 1.2      # x, y, yaw(±1.2rad 摆动)

    def tick_map(self):
        x, y, yaw = self.pose()
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = RES
        m.info.width = W
        m.info.height = H
        m.info.origin.position.x = ORIGIN[0]
        m.info.origin.position.y = ORIGIN[1]
        m.info.origin.orientation.w = 1.0
        m.data = self.grid
        self.pub_map.publish(m)

        # TF 链：map→odom（这里让 odom 与 map 重合）→base_footprint
        now = self.get_clock().now().to_msg()
        t1 = TransformStamped()
        t1.header.frame_id = 'map'
        t1.header.stamp = now
        t1.child_frame_id = 'odom'
        t1.transform.rotation.w = 1.0
        self.tf.sendTransform(t1)

        t2 = TransformStamped()
        t2.header.frame_id = 'odom'
        t2.header.stamp = now
        t2.child_frame_id = 'base_footprint'
        t2.transform.translation.x = x
        t2.transform.translation.y = y
        t2.transform.rotation.z = math.sin(yaw / 2.0)
        t2.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf.sendTransform(t2)

        # 全局路径：一条从当前位置往右上的直线
        p = Path()
        p.header.frame_id = 'map'
        p.header.stamp = now
        for i in range(30):
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = now
            ps.pose.position.x = x + i * 0.1
            ps.pose.position.y = y + i * 0.05
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.pub_plan.publish(p)

        ts = TaskState()
        ts.task_id = 'TEST-001'
        ts.state = 'RUNNING'
        ts.target_location = '货架A'
        ts.progress = 0.42
        ts.message = '注入器模拟任务'
        self.pub_task.publish(ts)
        self.pub_manual.publish(Bool(data=False))

    def tick_fast(self):
        """10 Hz 激光：按当前位姿和地图几何算每个方向的最近障碍。"""
        x, y, yaw = self.pose()
        n = 360
        s = LaserScan()
        s.header.frame_id = 'laser'
        s.header.stamp = self.get_clock().now().to_msg()
        s.angle_min = -math.pi
        s.angle_max = math.pi
        s.angle_increment = 2 * math.pi / n
        s.range_min = 0.12
        s.range_max = 8.0
        ranges = []
        for i in range(n):
            a = s.angle_min + i * s.angle_increment + yaw
            d = 0.12
            # 沿射线步进直到撞墙或撞柱子
            for step in range(1, 160):
                d = step * 0.05
                px = x + d * math.cos(a)
                py = y + d * math.sin(a)
                if abs(px) > 2.85 or abs(py) > 1.85:
                    break
                if abs(px - 0.9) < 0.35 and abs(py - 0.4) < 0.35:
                    break
            ranges.append(d if d <= s.range_max else float('inf'))
        s.ranges = ranges
        self.pub_scan.publish(s)


def main():
    # ★ --sim：本节点连同时间戳都切到仿真时基（并由 ClockSource 从 0 起发 /clock）。
    #   用来复现"上游 sim time / 网关 wall time"这个真实故障场景。
    sim = '--sim' in sys.argv[1:]
    rclpy.init()
    clock_node = ClockSource() if sim else None
    node = Injector(sim=sim)
    try:
        if clock_node is not None:
            # 两个节点一起 spin：时钟源走系统时钟，注入器走仿真时钟
            from rclpy.executors import SingleThreadedExecutor
            ex = SingleThreadedExecutor()
            ex.add_node(clock_node)
            ex.add_node(node)
            ex.spin()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        # ★ 不要静默吞：注入器挂了会让 e2e 全线报"没有地图/没有位姿"，
        #   看起来像网关坏了，实际是测试工具没起来。
        print(f'注入器异常退出：{type(exc).__name__}: {exc}', file=sys.stderr)
    finally:
        for n in (node, clock_node):
            if n is not None:
                try:
                    n.destroy_node()
                except Exception:
                    pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
