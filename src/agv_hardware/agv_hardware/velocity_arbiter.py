"""
velocity_arbiter —— 差速速度仲裁节点（硬件边界上的"最后一道"速度处理）。

它在这一层的职责（规格书 §3.2 原则二"硬件无关边界"）：
    上半层（Nav2 / 任务层 / 手动遥控）把"想走的速度"发到 /cmd_vel；
    本节点读取 /scan，用 agv_common.ReactiveAvoider 做反应式避障（近障减速 +
    左右择空 + 缓回导航），把最终速度发到 /cmd_vel_raw —— 也就是 Gazebo 差速
    插件（实车阶段是下位机）真正听的那个话题。

为什么这样接（不破坏 Nav2 链路）：
    Nav2 内部链路 controller_server → cmd_vel_nav → velocity_smoother → cmd_vel
    一个字都不用改。本节点只订阅 Nav2 的最终输出 /cmd_vel，改完再发给硬件。
    任务层急停也是发到 /cmd_vel（零速），所以会自然经过本节点转发到硬件——
    急停依然有效，不需要额外接线。

订正说明（与"避免撞墙"需求对应）：
    · 近障减速：距离越近线速度越低，太近直接为 0（防撞 D2）。
    · 择向接管：低速且前方很近时，比较左右哪边更空，确定性偏向空的一侧
      （不用随机——随机不可复现、无法统计成功率、答辩讲不清）。
    · 缓回导航：障碍远离后角速度分步交还给导航，避免蛇形。
    · 后方保护：倒车时改看车尾窄锥（`rear_half_angle_deg`，默认 20°），
      车尾太近同样刹停/减速。前进时后方数据完全不参与。
      ★ 锥角必须窄：槽内净宽 0.722、车心离侧壁仅 0.361，锥一开宽就会读到
        平行于车身的槽侧壁，倒车被自己压住 → "能进不能出"复发。

参数（可在 agv_bringup/config 覆盖）：
    ～ ReactiveAvoider 的全部阈值（见 agv_common.reactive_avoid）
    max_linear / max_angular   最终速度限幅（安全约束唯一收口）
    scan_topic / cmd_in / cmd_out  话题名（默认 /scan、/cmd_vel、/cmd_vel_raw）
    cmd_timeout    上层速度超时(s)：超时即输出零速（默认 0.5）
    scan_timeout   激光超时(s)：超时则不做避障，只限幅放行（默认 0.5）
    publish_rate   仲裁输出频率(Hz)：按此频率重发（默认 20.0）

★ 参数默认值与 config/default/hardware.yaml 保持一致：那份 YAML 是实际生效值，
  两边若不一致会让人误判避障距离，改动时请同步。
"""

import math                                        # use 角度换算
import time                                        # use dt 计算

import rclpy                                       # use ROS 2 Python 客户端
from rclpy.node import Node                        # def 节点基类
from geometry_msgs.msg import Twist                # def 速度消息
from sensor_msgs.msg import LaserScan              # def 激光消息

from agv_common.reactive_avoid import ReactiveAvoider   # use 反应式避障策略
from agv_common.limits import VelocityLimits            # use 速度限幅


class VelocityArbiter(Node):                       # out 速度仲裁节点
    # 初始化：读参数、建避障器与限幅器、建收发话题
    # 参数：无
    # 返回：无
    def __init__(self):                            # out 对外构造函数
        super().__init__('velocity_arbiter')       # use 节点名

        # ── 参数：避障阈值（默认值与 config/default/hardware.yaml 保持一致，可覆盖）──
        self.declare_parameter('stop_distance', 0.30)          # io 刹停距离
        self.declare_parameter('slow_distance', 0.55)          # io 减速起点
        self.declare_parameter('take_over_distance', 0.45)     # io 择向接管距离
        self.declare_parameter('front_half_angle_deg', 45.0)   # io 前方扇区半角（度）
        self.declare_parameter('side_max_angle_deg', 90.0)     # io 侧方最大角（度）
        self.declare_parameter('takeover_half_angle_deg', 20.0)  # io 接管闸门锥半角（度）：★ 别开大
        self.declare_parameter('rear_half_angle_deg', 20.0)    # io 后方扇区半角（度）：★ 别开大
        self.declare_parameter('yaw_handback_rate', 0.30)      # io 角速度交还步长
        self.declare_parameter('takeover_max_speed', 0.25)     # io 接管车速上限
        self.declare_parameter('slow_min_speed', 0.10)         # io 减速下限
        # ── 参数：限幅与话题 ──
        self.declare_parameter('max_linear', 0.5)              # io 线速度上限
        self.declare_parameter('max_angular', 1.0)             # io 角速度上限
        self.declare_parameter('scan_topic', '/scan')          # io 激光话题
        self.declare_parameter('cmd_in', '/cmd_vel')           # io 导航/上层速度入
        self.declare_parameter('cmd_out', '/cmd_vel_raw')      # io 硬件速度出
        self.declare_parameter('scan_timeout', 0.5)            # io 激光超时：超时则原样放行
        self.declare_parameter('cmd_timeout', 0.5)             # io 上层速度超时：超时停车
        self.declare_parameter('publish_rate', 20.0)           # io 仲裁输出频率(Hz)

        gp = lambda n: self.get_parameter(n).value             # use 读参数简写
        self.avoider = ReactiveAvoider(                        # def 反应式避障器
            stop_distance=float(gp('stop_distance')),
            slow_distance=float(gp('slow_distance')),
            take_over_distance=float(gp('take_over_distance')),
            front_half_angle=math.radians(float(gp('front_half_angle_deg'))),
            side_max_angle=math.radians(float(gp('side_max_angle_deg'))),
            takeover_half_angle=math.radians(float(gp('takeover_half_angle_deg'))),
            rear_half_angle=math.radians(float(gp('rear_half_angle_deg'))),
            slow_min_speed=float(gp('slow_min_speed')),
            takeover_max_speed=float(gp('takeover_max_speed')),
            yaw_handback_rate=float(gp('yaw_handback_rate')),
        )
        self.limits = VelocityLimits(max_linear=float(gp('max_linear')),
                                     max_angular=float(gp('max_angular')))   # def 限幅器
        self.scan_timeout = float(gp('scan_timeout'))
        self.cmd_timeout = float(gp('cmd_timeout'))
        self.cmd_in = gp('cmd_in')
        self.cmd_out = gp('cmd_out')
        scan_topic = gp('scan_topic')

        # ── 状态 ──
        self._scan = None                          # def 最近一帧激光
        self._scan_stamp = 0.0                     # def 激光到达时刻（monotonic）
        self._last_nav = Twist()                   # def 最近一条导航速度
        self._last_cmd_stamp = 0.0                 # def 上层速度到达时刻（monotonic）
        self._last_tick = time.monotonic()         # def 上次处理时刻（算 dt）

        # ── 收发话题 ──
        self.pub = self.create_publisher(Twist, self.cmd_out, 10)      # io 硬件速度出
        self.create_subscription(Twist, self.cmd_in, self.on_cmd, 10)  # io 上层速度入
        self.create_subscription(LaserScan, scan_topic, self.on_scan, 10)  # io 激光入
        # 定时器：以固定频率重发（保证即使上层不发新速度，减速/刹停也持续生效）
        rate = max(float(gp('publish_rate')), 1.0)                     # def 输出频率(Hz)
        self.create_timer(1.0 / rate, lambda: self.publish(self._last_nav))  # io 周期输出

        self.get_logger().info(                    # io 启动日志
            f'速度仲裁启动：{self.cmd_in} →（近障减速/择空）→ {self.cmd_out}，'
            f'激光 {scan_topic}，限幅 线≤{self.limits.max_linear} 角≤{self.limits.max_angular}')

    # 上层速度回调：记住最新"想走的速度"，立即处理一次
    # 参数：msg - Twist
    # 返回：无
    def on_cmd(self, msg):                         # in 内部回调
        self._last_nav = msg
        self._last_cmd_stamp = time.monotonic()    # use 记住到达时刻（超时判据）
        self.publish(msg)

    # 激光回调：缓存最近一帧
    # 参数：msg - LaserScan
    # 返回：无
    def on_scan(self, msg):                        # in 内部回调
        self._scan = msg
        self._scan_stamp = time.monotonic()

    # 处理并发布：把导航速度经避障/限幅后发出
    # 参数：nav - 上层想要的 Twist
    # 返回：无
    def publish(self, nav):                        # out 对外接口
        now = time.monotonic()
        dt = max(now - self._last_tick, 1e-3)      # def 距上次的时间
        self._last_tick = now

        v_nav = float(nav.linear.x)                # def 导航线速度
        w_nav = float(nav.angular.z)               # def 导航角速度
        # 上层速度超时：说明没人再发了（遥控松手 / 上游退出），必须停车——
        # 不能把最后一条速度无限重发，否则车会一直跑下去。
        if (now - self._last_cmd_stamp) > self.cmd_timeout:
            v_nav, w_nav = 0.0, 0.0

        scan = self._scan                          # use 最近激光
        fresh = (scan is not None) and (now - self._scan_stamp) <= self.scan_timeout
        if fresh:                                  # # 有新鲜激光：走避障仲裁
            v_out, w_out, info = self.avoider.arbitrate(
                v_nav, w_nav,
                scan.ranges, float(scan.angle_min), float(scan.angle_increment),
                range_max=float(scan.range_max) if scan.range_max > 0 else 8.0,
                dt=dt)
        else:                                      # # 无激光（还没起/超时）：原样放行，但要限幅
            v_out, w_out = v_nav, w_nav
            info = {'front': -1.0, 'engaged': False}

        v_out, w_out = self.limits.clamp_twist(v_out, w_out)   # use 安全限幅（唯一收口）

        out = Twist()                              # def 发给硬件的速度
        out.linear.x = float(v_out)
        out.angular.z = float(w_out)
        self.pub.publish(out)                      # io 发布

        # 接管/减速时打一条限流日志，便于观察（每秒最多一条，避免刷屏）
        front = info.get('front', -1.0)
        near = (front >= 0.0) and (front < self.avoider.slow_distance)
        if info.get('engaged') or near:
            rear = info.get('rear', -1.0)
            self.get_logger().info(
                f'[避障] 前方 {front:.2f}m（闸门 {info.get("front_gate", -1.0):.2f}m）'
                f' 左 {info.get("left", -1.0):.2f} 右 {info.get("right", -1.0):.2f}'
                f' 后 {rear:.2f} → v {v_nav:.2f}→{v_out:.2f}, '
                f'w {w_nav:.2f}→{w_out:.2f}{"，择向接管" if info.get("engaged") else ""}',
                throttle_duration_sec=1.0)


# 程序入口
# 参数：无
# 返回：无
def main(args=None):                               # out 程序入口
    rclpy.init(args=args)                          # io 初始化 ROS 2
    node = VelocityArbiter()                       # def 建节点
    try:
        rclpy.spin(node)                           # io 转圈
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()                        # io 释放资源
        rclpy.shutdown()                           # io 关闭 ROS 2


if __name__ == '__main__':
    main()                                         # use 直接运行时进入主函数
