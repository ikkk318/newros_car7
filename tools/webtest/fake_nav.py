#!/usr/bin/env python3
"""fake_nav —— 最小可用的假 Nav2，用来在无 Gazebo 环境里跑通任务流程。

★ 为什么需要它
  沙箱里 /dev/shm 只有 10MB，Gazebo Classic 起来就 SIGABRT，而 Nav2 全套
  （bt_navigator / controller / planner / amcl / map_server …）需要仿真环境
  才有意义。但"多任务只执行第一个"这个 bug 的核心在于**任务层的时序**，
  跟 Nav2 内部没关系 —— 只要有一个能"接受目标 → 过几秒报成功"的东西，
  task_manager 就能走完 GO_LOAD → LIFTING → TRANSPORT → LOWERING → DONE。

  所以这个脚本只造 Nav2 `BasicNavigator` 会去问的那几个接口：
    · amcl/get_state          生命周期服务（返回 active）
    · bt_navigator/get_state  同上
    · /amcl_pose              位姿（BasicNavigator 要收到它才认为定位就绪）
    · /navigate_to_pose       NavigateToPose action（收目标 → 延时 → 成功）
      /follow_path            同理（rclpy 的 BasicNavigator 只用前者，留着兼容）

★ 它还会把收到的目标数写进 tools/webtest/fake_nav.count，
  这是"车真的动了 N 次"的硬证据 —— 比看日志可靠得多。
"""

import os
import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy,
                       QoSDurabilityPolicy)

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped  # noqa: F401
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

# 每单"行驶"时长：短一点让测试跑得快，但要长到能观察到中间态
DRIVE_SEC = float(os.environ.get('FAKE_NAV_SEC', '2.0'))
COUNT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          'fake_nav.count')


class FakeNav(Node):
    def __init__(self):
        super().__init__('fake_nav')
        self.group = ReentrantCallbackGroup()
        self.count = 0
        self.lock = threading.Lock()
        self._write_count()

        # ── 生命周期查询服务：BasicNavigator 靠它判断"节点激活了没" ──
        for name in ('amcl/get_state', 'bt_navigator/get_state',
                     'controller_server/get_state', 'planner_server/get_state'):
            self.create_service(GetState, name, self._state_cb,
                                callback_group=self.group)

        # ── /amcl_pose：BasicNavigator 收到它才算定位就绪 ──
        # ★ QoS 必须和 nav2_simple_commander 订阅的一致：
        #   TRANSIENT_LOCAL + RELIABLE + KEEP_LAST(1)。
        #   用默认 VOLATILE 发，订阅端只会打
        #   "requesting incompatible QoS ... Last incompatible policy:
        #    DURABILITY" 然后永远收不到 —— 表现就是 BasicNavigator 卡在
        #   "Waiting for amcl_pose to be received" 无限循环。
        amcl_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', amcl_qos)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)
        self.create_timer(0.5, lambda: self.pose_pub.publish(
            self._pose_msg()))

        # ── TF：map → odom → base_footprint（网关 refresh_pose 要用）──
        self.tf = StaticTransformBroadcaster(self)
        self._broadcast_tf()

        # ── 导航 action ──
        self.nav_srv = ActionServer(
            self, NavigateToPose, '/navigate_to_pose',
            self._execute_nav, callback_group=self.group)
        self.get_logger().info(
            f'假 Nav2 就绪：每单行驶 {DRIVE_SEC:.1f}s；目标计数写入 {COUNT_FILE}')

    def _pose_msg(self):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.orientation.w = 1.0
        return m

    def _broadcast_tf(self):
        now = self.get_clock().now().to_msg()
        for parent, child in (('map', 'odom'), ('odom', 'base_footprint')):
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            self.tf.sendTransform(t)

    def _state_cb(self, request, response):
        # ★ primary_state 必须是 ACTIVE，否则 BasicNavigator 会永远卡在
        #   "waiting for amcl to become active"（真机上这段能等到超时）。
        response.current_state.id = State.PRIMARY_STATE_ACTIVE
        response.current_state.label = 'active'
        return response

    def _write_count(self):
        try:
            with open(COUNT_FILE, 'w', encoding='utf-8') as f:
                f.write(str(self.count))
        except OSError:
            pass

    def _execute_nav(self, goal_handle):
        with self.lock:
            self.count += 1
            n = self.count
            self._write_count()
        self.get_logger().info(f'[假Nav2] 收到第 {n} 个导航目标，行驶中…')
        t0 = time.monotonic()
        fb = NavigateToPose.Feedback()
        while time.monotonic() - t0 < DRIVE_SEC:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn(f'[假Nav2] 第 {n} 个目标被取消')
                return NavigateToPose.Result()
            # 反馈里 distance_remaining 要一路减小，goto_pose 会打印它
            left = DRIVE_SEC - (time.monotonic() - t0)
            fb.distance_remaining = max(0.0, left)
            fb.number_of_recoveries = 0
            goal_handle.publish_feedback(fb)
            time.sleep(0.2)
        goal_handle.succeed()
        self.get_logger().info(f'[假Nav2] 第 {n} 个目标到达')
        return NavigateToPose.Result()


def main():
    rclpy.init()
    node = FakeNav()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
