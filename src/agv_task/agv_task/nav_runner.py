"""
nav_runner —— 导航执行器（段导航 + 多点串联 + 可中止等待）。

从原 task_manager_node 里抽出来的一段独立职责：
    「给一个位姿，调 Nav2 走过去；给一串位姿，依次走过去；中途可被中止。」

为什么单独成模块：
    原节点 524 行里，导航执行、搬货调用、状态上报、指令处理、安全心跳全混在一起。
    其中"导航执行"是需要持有 BasicNavigator、处理超时、响应中止标志的一块，
    与"指令怎么解析"毫无关系，单独放这里后两边都能读得懂。

本模块不做 ROS 节点的事：它只是**被节点持有**的一个执行器，
回调组与日志都由宿主节点传入，因此可以脱离 ROS 单独审视逻辑边界。

★ 超时语义：单段导航超过 timeout_s 就取消该目标并返回 False（不抛异常），
  由调用方决定是报错还是重规划。这与"任务层死等"不同——死等会让整条任务
  卡住直到外层超时，而这里会把失败尽早交给状态机。
"""

import math                              # use 位姿差/角度换算
import time                              # use 超时与中止轮询

import rclpy                             # use 判断 ROS 是否还在运行
from nav2_simple_commander.robot_navigator import TaskResult   # use 导航结果枚举
from rclpy.time import Time              # use "取最新可用变换"

from agv_task.goto_pose import goto_pose  # use 分段导航执行
from agv_task.slot_approach import (      # use 槽内到点判据（纯逻辑）
    SlotArrive, DEFAULT_ARRIVE_XY, DEFAULT_ARRIVE_YAW, DEFAULT_ARRIVE_HOLD)


class NavRunner:                         # out 导航执行器
    # 初始化
    # 参数：node - 宿主节点；frame - 坐标系；print_period - 反馈打印周期(s)
    #       timeout_s - 单段导航超时(s)；abort_event - 中止标志（threading.Event）
    # 返回：无
    def __init__(self, node, frame='map', print_period=1.0,
                 timeout_s=180.0, abort_event=None):    # out 对外构造函数
        self.node = node                 # def 宿主节点
        self.frame = str(frame or 'map')      # def 坐标系
        self.print_period = float(print_period or 1.0)   # def 打印周期
        self.timeout_s = float(timeout_s or 180.0)       # def 单段超时
        self.abort = abort_event         # def 中止标志（可为 None）
        self._navigator = None           # def BasicNavigator（首次导航才建）
        self._tf = None                  # def tf2 Buffer（槽内软到点才用，懒建）

    # 是否被请求中止
    # 参数：无
    # 返回：bool
    def aborted(self):                   # in 内部方法
        return self.abort is not None and self.abort.is_set()

    # 懒建 BasicNavigator（没起 Nav2 时不至于在构造期就报错）
    # 参数：无
    # 返回：BasicNavigator 或 None
    def _nav(self):                      # in 内部方法
        if self._navigator is None:
            from nav2_simple_commander.robot_navigator import BasicNavigator
            self._navigator = BasicNavigator()
            # ★ 把这个 logger 压到 WARN（用户："日志打印速度太快根本看不过来"）。
            #   BasicNavigator 每调一次 waitUntilNav2Active() 就打一条
            #   "Nav2 is ready for use!"，每发一个目标又打一条
            #   "Navigating to goal: x y..."。实测 221 行日志里这两句共 33 条，
            #   全是我们自己触发的、且完全没有信息量（我们在 goto_pose 里
            #   已经打了"发送导航目标：x=… y=… yaw=…"，那条还更全）。
            #   在源头压级别比在下游过滤干净：下游过滤要写正则猜句式，
            #   而且原文还会进 ROS 终端和 rosbag。
            try:
                from rclpy.logging import set_logger_level
                set_logger_level('basic_navigator', 40)   # 40 = WARN
            except Exception:
                pass          # 拿不到（老版本 rclpy）就算了，不影响导航
        return self._navigator

    # 读当前位姿（map→base_footprint），返回 (x, y, yaw)
    # 参数：无
    # 返回：(float, float, float)；TF 还没起来时抛异常，由调用方吞掉
    #
    # ★ 为什么用 tf 而不是 /amcl_pose：
    #   /amcl_pose 只在 AMCL 更新时发（update_min_d/update_min_a 触发），
    #   到点停稳后可能几秒不更新，软到点会因此错过窗口。TF 是连续可查的。
    def _pose_now(self):                 # in 内部方法
        if self._tf is None:
            from tf2_ros import Buffer, TransformListener
            self._tf = Buffer()
            # spin_thread=False：宿主节点已经在 MultiThreadedExecutor 里转，
            # 再起线程会多一份回调、也可能和主执行器抢 TF 订阅。
            TransformListener(self._tf, self.node)
        tf = self._tf.lookup_transform(self.frame, 'base_footprint', Time())
        tr = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return float(tr.x), float(tr.y), float(yaw)

    # 造一个"槽内软到点"判据给 goto_pose 用
    # 参数：pose - 目标位姿（含 name）；soft - {'xy','yaw','hold'} 容差配置
    # 返回：回调 () -> None | {'kind','text'}
    def _arrive_checker(self, pose, soft):    # in 内部方法
        watcher = SlotArrive(
            xy=soft.get('xy', DEFAULT_ARRIVE_XY),
            yaw=soft.get('yaw', DEFAULT_ARRIVE_YAW),
            hold=soft.get('hold', DEFAULT_ARRIVE_HOLD))
        target = (float(pose['x']), float(pose['y']), float(pose.get('yaw', 0.0)))
        name = pose.get('name') or f'({target[0]:.2f}, {target[1]:.2f})'

        def check():
            x, y, yaw = self._pose_now()
            verdict, dist, dyaw = watcher.feed(x, y, yaw, target, time.monotonic())
            if verdict == 'arrive':
                return {'kind': 'arrive',
                        'text': f'[导航] 已到「{name}」：位置差 {dist:.2f} 米，'
                                f'角度差 {math.degrees(dyaw):.0f}°'
                                f'（槽内不再纠偏，直接执行搬货）'}
            if verdict == 'stuck':
                return {'kind': 'fail',
                        'text': f'[导航] 已在「{name}」位置，但航向差 '
                                f'{math.degrees(dyaw):.0f}°，'
                                f'槽内空间不足以原地纠正 → 放弃本次导航'}
            return None

        return check

    # 走一段：到点返回 True
    # 参数：pose - {'x','y','yaw'}，可选 'name' 与 'soft'（槽内软到点容差）
    # 返回：bool
    def step(self, pose):                # out 对外接口
        if self.aborted():               # # 已被取消/急停
            return False
        try:
            nav = self._nav()            # use 懒建
        except Exception as exc:         # # Nav2 不可用：判失败，不让节点崩
            self.node.get_logger().error(f'导航不可用：{exc}')
            return False
        # soft 只在槽内点位上出现（由 TaskFlow 按点名后缀写入）。
        # 普通点保持 Nav2 原生判据，不改既有行为。
        soft = pose.get('soft')
        checker = self._arrive_checker(pose, soft) if soft else None
        result = goto_pose(nav, pose['x'], pose['y'], pose.get('yaw', 0.0),
                           frame_id=self.frame, print_period=self.print_period,
                           logger=self.node.get_logger(), timeout_s=self.timeout_s,
                           abort_event=self.abort,
                           arrive_check=checker)   # use 软到点（可为 None）
        if not rclpy.ok():               # # ROS 已关：直接失败
            return False
        return result == TaskResult.SUCCEEDED and not self.aborted()

    # 依次走一串点：全部到点才 True
    # 参数：points - [{'name','x','y','yaw'}, ...]
    # 返回：bool
    def sequence(self, points):          # out 对外接口
        for pt in points:
            if self.aborted():
                return False
            name = pt.get('name') or f'({pt["x"]:.2f}, {pt["y"]:.2f})'
            self.node.get_logger().info(f'[路线] 前往途经点「{name}」')
            if not self.step(pt):        # use 单段
                self.node.get_logger().error(f'[路线] 途经点「{name}」导航失败')
                return False
        return True

    # 可中止的等待（用中止标志代替长 sleep，取消后能立刻退出）
    # 参数：seconds - 等待时长(s)
    # 返回：无
    def sleep(self, seconds):            # out 对外接口
        deadline = time.monotonic() + max(float(seconds), 0.0)
        while time.monotonic() < deadline:
            if self.aborted():
                return
            time.sleep(0.05)

    # 取消当前 Nav2 目标（急停/取消任务时调）
    # 参数：无
    # 返回：无
    def cancel(self):                    # out 对外接口
        if self._navigator is None:
            return
        try:
            self._navigator.cancelTask()  # use 取消当前目标
        except Exception:                # # Nav2 不在/已关：忽略
            pass
