"""
task_manager_node —— 任务层节点（只做接线：收发、参数、调度）。

职责边界（这是本次重构的重点）：
    本节点**不再**包含业务逻辑。流程怎么走、点位怎么查、状态怎么拼，
    分别由下面这些可独立阅读/测试的模块负责：

        command_router.py    指令门卫：白名单 / 参数 / 去重 / 分派
        goal_check.py        目标点合法性的**唯一判据**（地图优先）
        task_flow.py         两条流程：单点导航、完整搬运
        nav_runner.py        导航执行：单段/多点/可中止等待
        cargo_client.py      搬货调用（顶升/下降）
        store_client.py      点位/路线查询
        state_reporter.py    状态组装（急停 > 搬运 > 导航 > 空闲）
        task_manager.py      搬运状态机（纯逻辑）
        goto_controller.py   单点导航状态机（纯逻辑）
        safety_monitor.py    安全判定（纯逻辑）

    本节点只负责：读参数、建发布/订阅/服务/客户端、把回调转给上面的模块、
    起后台线程跑流程、周期上报状态。

话题：
    /agv/command      in   AgvCommand  指令入
    /agv/web_teleop   in   Twist       网页遥控（互锁校验后转发到 /cmd_vel）
    /cmd_vel          in   Twist       速度心跳（判"上层还在发指令"）
    /cmd_vel          out  Twist       遥控转发帧 + 急停/超时时的零速停车帧
    /agv/manual_mode  in   Bool        手动模式（互锁：手动时禁导航）
    /agv/task_state   out  TaskState   状态上报（1Hz 周期 + 变化时）
服务：
    /agv/query        AgvQuery         运行时状态 / 位姿查询
客户端：
    /agv/store                         点位与路线的统一增删查列
    /agv/cargo                         搬货服务（顶升/下降）

★ 关于 /cmd_vel 自发帧的自锁问题（本次修复）：
    本节点发零速到 /cmd_vel，同时又把 /cmd_vel 当心跳订阅。发出的零速会绕回来
    喂给自己的心跳计时，把刚判定的"指令超时"立刻清掉——自己解除自己的超时。
    现在发之前调 safety.note_self_publish() 记账，收到自己的回声就冲销不计心跳；
    并且只在自己发的车/急停路径上记账，不影响真正的上层指令。
"""

import threading                        # use 后台跑导航/搬运，避免阻塞回调
import time                             # use 超时与节流

import rclpy                            # use ROS 2 Python 客户端
from rclpy.callback_groups import (      # use 回调组：阻塞式调用与普通回调隔离
    MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup)
from rclpy.executors import MultiThreadedExecutor   # use 多线程执行器
from rclpy.node import Node             # def 节点基类
from geometry_msgs.msg import Twist     # def 速度消息
from std_msgs.msg import Bool           # def 布尔消息

from agv_interfaces.msg import AgvCommand, TaskState           # use 指令/状态消息
from agv_interfaces.srv import AgvQuery, CargoCmd              # use 服务类型

from agv_common.goal_check import GoalValidator         # use 目标点唯一判据
from agv_task.cargo_client import CargoClient           # use 搬货客户端
from agv_task.command_router import CommandRouter       # use 指令门卫
from agv_task.goto_controller import GotoController     # use 单点导航状态机
from agv_task.nav_runner import NavRunner               # use 导航执行器
from agv_task.robot_pose import get_current_pose        # use 实时位姿
from agv_task.safety_monitor import SafetyMonitor       # use 安全监控
from agv_task.state_reporter import compose             # use 状态组装
from agv_task.store_client import StoreClient           # use 点位/路线客户端
from agv_task.task_flow import TaskFlow                 # use 流程执行体
from agv_task.task_manager import TaskManager           # use 搬运状态机


class TaskManagerNode(Node):             # out 任务层节点
    # 初始化：读参数、建模块、接线收发、注册指令
    # 参数：无
    # 返回：无
    def __init__(self):                  # out 对外构造函数
        super().__init__('task_manager_node')

        # ── 参数 ──
        self.declare_parameter('cmd_timeout_ms', 500)    # 安全层：速度指令超时
        self.declare_parameter('estop_budget_ms', 200)   # 安全层：急停停车时限
        self.declare_parameter('waypoint_frame', 'map')
        self.declare_parameter('nav_print_period', 1.0)
        self.declare_parameter('nav_timeout_s', 180.0)
        self.declare_parameter('cargo_timeout_s', 5.0)
        self.declare_parameter('robot_clearance', 0.30)
        self.declare_parameter('map_yaml', '')
        # 槽内点位进槽策略（见 slot_approach.py）：
        #   slot_approach    先到「X号货架」对位再直行进槽（关掉则退回直接进槽）
        #   slot_arrive_*    槽内软到点容差：位置 0.12 m / 角度 0.35 rad(≈20°) / 稳定 0.6 s
        # ★ 角度容差不能给太大：顶升是平板往上顶，偏太多会顶偏货物。
        #   0.35 rad 对应车体横向多占 0.17 m，槽内 0.80 m 净宽仍放得下。
        self.declare_parameter('slot_approach', True)
        self.declare_parameter('slot_arrive_xy', 0.12)
        self.declare_parameter('slot_arrive_yaw', 0.35)
        self.declare_parameter('slot_arrive_hold', 0.6)
        # 兜底矩形范围：★ 只在没有地图时生效，不再是"第二个判据"
        self.declare_parameter('field_min', -50.0)
        self.declare_parameter('field_max', 50.0)
        self.declare_parameter('field_min_y', -50.0)
        self.declare_parameter('field_max_y', 50.0)
        gp = lambda n: self.get_parameter(n).value       # use 读参数简写
        self.frame = str(gp('waypoint_frame') or 'map')
        self.nav_timeout_s = float(gp('nav_timeout_s') or 180.0)
        cargo_timeout_s = float(gp('cargo_timeout_s') or 5.0)

        # ── 目标点唯一判据（有地图以地图为准，否则退回矩形范围）──
        self.validator = GoalValidator(      # def 点位合法性判据
            map_yaml=str(gp('map_yaml') or ''),
            robot_clearance=float(gp('robot_clearance') or 0.30),
            field={'min_x': float(gp('field_min')), 'max_x': float(gp('field_max')),
                   'min_y': float(gp('field_min_y')), 'max_y': float(gp('field_max_y'))})

        # ── 纯逻辑组件 ──
        self.router = CommandRouter(validator=self.validator)   # def 指令门卫
        self.safety = SafetyMonitor(cmd_timeout_ms=int(gp('cmd_timeout_ms')),
                                    estop_budget_ms=int(gp('estop_budget_ms')))  # def 安全监控
        self.manager = TaskManager()         # def 搬运状态机
        self.goto = GotoController()         # def 单点导航状态机

        # ── 回调组：状态类回调串行；阻塞式服务/客户端放可重入组 ──
        # 说明：查点/查路线是"同步等服务应答"，必须放在可重入组，否则它阻塞时
        #       也占住默认互斥组 → 应答回来没人处理 → 死锁。
        self._cb_group = MutuallyExclusiveCallbackGroup()
        self._io_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()       # def 保护状态读取

        # ── 收发 ──
        self.state_pub = self.create_publisher(TaskState, '/agv/task_state', 10)   # io 状态出
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)            # io 停车帧出
        self.create_subscription(AgvCommand, '/agv/command', self.on_command, 10,
                                 callback_group=self._cb_group)                    # io 指令入
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10,
                                 callback_group=self._cb_group)                    # io 速度心跳
        # ★ 网页遥控入口。缺了这条订阅，导航模式下网页按方向键车一动不动 ——
        #   网关把速度发到 /agv/web_teleop，而全项目没有任何节点接它。
        self.create_subscription(Twist, '/agv/web_teleop', self.on_web_teleop, 10,
                                 callback_group=self._cb_group)                    # io 网页遥控
        self.create_subscription(Bool, '/agv/manual_mode', self.on_manual_mode, 10,
                                 callback_group=self._cb_group)                    # io 手动模式
        self.create_service(AgvQuery, '/agv/query', self.on_query,
                            callback_group=self._io_group)                         # io 查询

        # ── 运行态 ──
        self._abort_task = threading.Event()  # def 请求中止后台线程（取消/急停）
        self._runner = None                  # def 当前后台线程（导航/搬运）
        self._last_tick = time.monotonic()   # def 上次安全推进时刻

        # ── 业务模块（都注入同一套纯逻辑组件）──
        self.nav = NavRunner(self, frame=self.frame,
                             print_period=float(gp('nav_print_period') or 1.0),
                             timeout_s=self.nav_timeout_s,
                             abort_event=self._abort_task)                 # def 导航执行器
        self.store = StoreClient(self, timeout_s=3.0)                      # def 点位/路线客户端
        self.cargo = CargoClient(self, CargoCmd, timeout_s=cargo_timeout_s,
                                 abort_event=self._abort_task)             # def 搬货客户端
        self.flow = TaskFlow(self, self.manager, self.goto, self.nav, self.cargo,
                             self.store, self.validator, self._abort_task,
                             self._publish_zero_twist,
                             slot_approach=bool(gp('slot_approach')),
                             slot_soft={'xy': float(gp('slot_arrive_xy')),
                                        'yaw': float(gp('slot_arrive_yaw')),
                                        'hold': float(gp('slot_arrive_hold'))})  # def 流程执行体
        self.flow.on_state = self.publish_state                            # use 流程回报状态

        # ── 指令注册 ──
        self.router.register('goto', self.handle_goto)              # def 点名/坐标导航
        self.router.register('start_task', self.handle_start_task)  # def 完整搬运
        self.router.register('cancel_task', self.handle_cancel)     # def 取消
        self.router.register('stop', self.handle_stop)              # def 急停（常受理）
        self.router.register('resume', self.handle_resume)          # def 恢复

        # ── 定时器 ──
        self.create_timer(0.1, self.on_safety_tick)      # io 安全推进
        self.create_timer(1.0, self.publish_state)       # io 周期上报状态

        self.publish_state()                             # io 启动先报一次 IDLE
        self.get_logger().info(
            f'任务层启动：坐标系 {self.frame}，导航超时 {self.nav_timeout_s:.0f}s；'
            f'目标点判据 {self.validator.describe()}')

    # ══════════════════ 状态上报 ══════════════════

    # 组装并发布一次 /agv/task_state
    # 参数：无
    # 返回：无
    def publish_state(self):             # out 对外接口
        info = compose(self.manager, self.goto, self.safety)   # use 纯逻辑组装
        msg = TaskState()                # def 状态消息
        msg.task_id = info['task_id']
        msg.state = info['state']
        msg.target_location = info['target']
        msg.progress = float(info['progress'])
        msg.message = info['message']
        self.state_pub.publish(msg)      # io 发布

    # ══════════════════ 回调 ══════════════════

    # 指令总线回调：交给门卫校验/去重/分派（处理器只"受理并起线程"，不阻塞）
    # 参数：msg - AgvCommand
    # 返回：无
    def on_command(self, msg):           # in 内部回调
        accepted, reason = self.router.route(msg)     # use 门卫
        # 注意：不要在 info/warn 之间复用缓存的绑定方法（rclpy 会报
        # "Logger severity cannot be changed between calls"），按结果分别调用。
        if accepted:
            self.get_logger().info(
                f'[指令] {msg.command} from={msg.source} → 受理：{reason}')
        else:
            self.get_logger().warn(
                f'[指令] {msg.command} from={msg.source} → 拒绝：{reason}')
        self.publish_state()                          # io 受理后立刻刷一次状态

    # 速度心跳：刷新安全层的"上层还在发指令"计时
    # 参数：msg - Twist
    # 返回：无
    def on_cmd_vel(self, msg):           # in 内部回调
        self.safety.on_cmd_vel(float(msg.linear.x))

    # 网页遥控入口：互锁校验后转发到底盘链路
    # 参数：msg - Twist（线速度 / 角速度）
    # 返回：无
    def on_web_teleop(self, msg):        # in 内部回调
        """把网页遥控从 /agv/web_teleop 转到 /cmd_vel（→ 仲裁 → /cmd_vel_raw）。

        ★ 为什么必须由本节点转发，而不是网页直发 /cmd_vel：
          手动模式与急停态必须能当场拒绝遥控，这里是唯一的互锁闸门。
          绕过它就等于急停时用户按方向键车照样会动。

        ★ 为什么转发时**不**调 safety.note_self_publish()：
          这个方法只用于本节点自己生成的停车帧（否则"自己发的零速"会被
          当成上层心跳，把自己的超时判据清掉）。网页就是真正的上层，
          它发的帧本来就应该刷新心跳计时。

        ★ 为什么急停/手动时不 return 而是发零速：
          车可能正带着上一次的速度在跑，只丢掉这一帧不会让它停；
          必须在拒绝的同时把零速压上去。
        """
        if self.safety.estopped() or self.safety.manual():
            self._publish_zero_twist()
            return
        self.cmd_vel_pub.publish(msg)

    # 手动模式互锁：手动时禁止导航
    # 参数：msg - Bool（true=手动）
    # 返回：无
    def on_manual_mode(self, msg):       # in 内部回调
        self.safety.set_manual(bool(msg.data))
        self.get_logger().info('切入手动模式：禁止导航' if msg.data else '切入自动模式：允许导航')
        self.publish_state()

    # 安全推进：指令超时判定（超时则补一帧零速，保证车停得住）
    # 参数：无
    # 返回：无
    def on_safety_tick(self):            # in 内部回调
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        self.safety.tick(dt)
        if self.safety.timeout_edge():   # # ★ 只在刚超时那一次补零速（边沿，不反复发）
            self._publish_zero_twist()
            self.get_logger().warn('指令超时（上层已不再发速度），已发零速停车',
                                   throttle_duration_sec=2.0)

    # 查询服务：运行时状态 / 位姿
    # 参数：request - AgvQuery.Request；response - 待填 Response
    # 返回：填好的 response
    def on_query(self, request, response):    # in 内部回调
        what = str(request.what or 'state').strip().lower()
        response.ok = True
        response.state = self.manager.state()
        response.x = response.y = response.yaw = 0.0
        if what == 'pose':                        # # 当前位置
            pose = get_current_pose(self) or {}
            if pose:
                response.x = float(pose.get('x', 0.0))
                response.y = float(pose.get('y', 0.0))
                response.yaw = float(pose.get('yaw', 0.0))
                response.message = (f'当前位姿 x={response.x:.2f} y={response.y:.2f} '
                                    f'yaw={response.yaw:.2f}')
            else:
                response.ok = False
                response.message = '暂时取不到位姿（定位/TF 未就绪）'
        else:                                     # # 默认：任务状态
            progress, message = self.manager.progress_info()
            response.message = (f'{self.manager.state()}（{message}）'
                                f'目标 {self.manager.target_location() or "-"} '
                                f'进度 {progress:.0%}，导航 {self.goto.state()}')
        return response

    # ══════════════════ 指令处理器（只受理 + 起线程） ══════════════════

    # 点名 / 坐标导航（可选路线）
    # 参数：args - [点名] 或 [x, y, yaw]；msg - 原始指令（读 route）
    # 返回：(accepted, reason)
    def handle_goto(self, args, msg):    # in 指令处理器
        ok, reason = self._can_start()        # use 前置检查
        if not ok:
            return False, reason
        route = str(getattr(msg, 'route', '') or '').strip()   # def 本次路线名
        target = [str(a).strip() for a in args if str(a).strip()]
        if not target:
            return False, 'goto 缺少目标（点名或坐标）'
        self.goto.reset()                     # use 清掉上次的 DONE/ERROR
        self._abort_task.clear()
        self._spawn(self.flow.run_goto, target, route)   # use 后台执行
        return True, f'已下发导航（{"点名 " + target[0] if len(target) == 1 else "坐标"}）'

    # 完整搬运（可选路线）
    # 参数：args - [取货点名, 放货点名]；msg - 原始指令（读 route）
    # 返回：(accepted, reason)
    def handle_start_task(self, args, msg):   # in 指令处理器
        ok, reason = self._can_start()        # use 前置检查
        if not ok:
            return False, reason
        names = [str(a).strip() for a in args if str(a).strip()]
        if len(names) < 2:
            return False, 'start_task 需要两个参数：取货点名 与 放货点名'
        route = str(getattr(msg, 'route', '') or '').strip()   # def 本次路线名
        self._abort_task.clear()
        # ★ 受理 = 状态机真的进格（本轮修的关键）。
        #   原来这里只 _spawn 后台线程，manager.start() 要等线程里查完
        #   两个点位（服务未就绪时每次最长 6 秒）才执行，task_id 迟迟不
        #   生成。而网关正是用"task_id 有没有变"判断受理的：6 秒等不到
        #   就判"没启动"并重发，重发又被 _runner.is_alive() 拒掉，
        #   最后报一句含糊的"指令被拒绝"—— 用户看到的就是
        #   "任务一直没能启动，车也没动"。
        #   放在回调里只是纯状态机操作（微秒级），不做任何服务调用，
        #   不会拖住急停（on_command 与 stop 共用互斥回调组）。
        ok, why = self.manager.start(names[0], names[1])
        if not ok:
            return False, why
        self._spawn(self.flow.run_transport, names[0], names[1], route)   # use 后台执行
        return True, (f'已开始搬运：{names[0]} → {names[1]}'
                      f'（{self.manager.task_id()}）')

    # 取消当前任务
    # 参数：args - 忽略；msg - 忽略
    # 返回：(accepted, reason)
    def handle_cancel(self, args, msg):   # in 指令处理器
        had_task = not self.manager.is_idle() or self.goto.state() != 'IDLE'
        self._abort_task.set()                # use 让后台线程尽快退出
        self.nav.cancel()                     # use 取消 Nav2 目标
        self.manager.cancel()                 # use 状态机回空闲
        self.goto.cancel()
        self._publish_zero_twist()
        self.safety.clear_task()
        # ★ 取消后必须立刻重发一次状态。
        #   以前这里漏了 publish_state()，后果很直观：状态机内部已经回 IDLE，
        #   但 /agv/task_state 的最后一帧还停在 GO_LOAD/TRANSPORT，
        #   网页永远显示"执行中"，而车其实早就停了。要等 1 Hz 的定时器
        #   下一拍才纠正 —— 最长差 1 秒看似不多，但如果刚好那一刻
        #   网关没收到（限流/丢帧），界面就一直卡着，用户只能重启网关。
        #   凡是"改变了状态机"的地方，都要紧跟一次 publish_state()。
        self.publish_state()
        # ★ 顺手把后台线程收干净（这是"中止后再也开不了新任务"的第二半原因）。
        #   _can_start() 里有一条 `self._runner.is_alive()` 的串行保护：
        #   取消是异步的，线程可能还卡在 goto_pose 里没退出来（要等 Nav2
        #   目标真正取消、或 waitUntilNav2Active 返回）。如果此刻用户马上
        #   点「开始执行」，就会被"已有任务在执行"拒掉，而他刚才明明点了中止。
        #   这里同步等线程收尾：不阻塞太久（1.5 秒足够 goto_pose 的中止轮询
        #   走完一圈），并且绝不在指令回调里无限期等待。
        runner = self._runner
        if runner is not None and runner.is_alive():
            runner.join(timeout=1.0)
            if runner.is_alive():
                self.get_logger().warn(
                    '后台任务线程仍在收尾，稍后（约 1 秒）才能接新任务')
            else:
                self._runner = None           # use 已收干净：解除串行锁
        return True, '已取消任务并停车' if had_task else '当前无任务可取消'

    # 急停（最高优先级，任何状态都受理）
    # 参数：args - 忽略；msg - 忽略
    # 返回：(accepted, reason)
    def handle_stop(self, args, msg):     # in 指令处理器
        t0 = time.monotonic()                 # use 量一下"多久发出停车"
        self.safety.on_estop()                # use 进急停态
        self._publish_zero_twist()            # io 立刻发零速
        self.safety.mark_stop_published((time.monotonic() - t0) * 1000.0)
        self._abort_task.set()                # use 中止后台导航/搬运
        self.nav.cancel()                     # use 取消 Nav2 目标
        self.manager.on_estop()               # use 任务进 PAUSED
        self.goto.cancel()
        self.publish_state()
        timely = '达标' if self.safety.estop_timely() else '超标'
        return True, f'已急停（停车耗时 {self.safety.stop_published_within_ms()}ms，{timely}）'

    # 急停 / 暂停后恢复（清急停 + 清任务 + 回空闲，等新指令）
    # ★ 不"续跑暂停前那一格"：急停已取消 Nav2 目标并中止后台线程，那一段已作废。
    # 参数：args - 忽略；msg - 忽略
    # 返回：(accepted, reason)
    def handle_resume(self, args, msg):   # in 指令处理器
        self.safety.clear_estop()             # use 清急停
        self.safety.clear_task()              # use 任务状态已清理
        self._abort_task.set()                # use 让残留线程退出
        self.manager.reset()                  # use 回空闲
        self.goto.reset()
        self._abort_task.clear()
        self.publish_state()
        self.get_logger().info('已恢复：急停解除、任务清空，回到空闲待命')
        return True, '已恢复，回到空闲待命'

    # ══════════════════ 小工具 ══════════════════

    # 前置检查：能不能接新任务
    # 参数：无
    # 返回：(ok, reason)
    def _can_start(self):                # in 内部方法
        if not rclpy.ok():
            return False, '系统正在关闭'
        if self.safety.estopped():       # # 急停态：先 resume
            return False, '处于急停状态，请先恢复（resume）'
        if self.safety.manual():         # # 手动模式互锁
            return False, '当前是手动模式，禁止导航'
        if self._runner is not None and self._runner.is_alive():   # # 串行执行
            return False, '已有任务在执行，请先取消或等它完成'
        if self.manager.state() not in ('IDLE', 'DONE'):
            return False, f'搬运状态为 {self.manager.state()}，无法接新任务'
        return True, '可以开始'

    # 起一个后台线程执行阻塞流程（导航/搬运）
    # 参数：target - 可调用；*args - 它的参数
    # 返回：无
    def _spawn(self, target, *args):     # in 内部方法
        self._runner = threading.Thread(target=self._guarded, args=(target,) + args,
                                        daemon=True)
        self._runner.start()

    # 后台线程外壳：兜住异常，避免线程静默死掉
    # 参数：target / args
    # 返回：无
    def _guarded(self, target, *args):   # in 内部方法
        try:
            target(*args)
        except Exception as exc:         # # 异常不能让线程无声退出
            self.get_logger().error(f'后台任务异常：{exc}')
            self.manager.fail()
            self.goto.on_nav_failed()
            self.publish_state()

    # 发一帧零速到 /cmd_vel（会经速度仲裁转发到硬件）
    # ★ 发之前记账，避免这帧回声把自己的心跳计时刷新（自解锁缺陷）
    # 参数：无
    # 返回：无
    def _publish_zero_twist(self):       # in 内部方法
        self.safety.note_self_publish()  # use 声明"这帧是我自己发的"
        self.cmd_vel_pub.publish(Twist())    # io 默认全 0 = 停车


# 程序入口
# 参数：无
# 返回：无
def main(args=None):                     # out 程序入口
    rclpy.init(args=args)                # io 初始化 ROS 2
    node = TaskManagerNode()             # def 建节点
    executor = MultiThreadedExecutor()   # def 多线程执行器（回调互不饿死）
    executor.add_node(node)
    try:
        executor.spin()                  # io 转圈
    except KeyboardInterrupt:            # # Ctrl-C 正常退出
        pass
    finally:
        node.destroy_node()              # io 释放资源
        rclpy.shutdown()                 # io 关闭 ROS 2


if __name__ == '__main__':
    main()                               # use 直接运行时进入主函数
