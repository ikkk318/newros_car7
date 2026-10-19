"""
commands.py —— 节点的"动作一半"：遥控、模式、指令/服务、任务缓冲、地图库。

**和 state.py 的分工**
  state.py     ROS 回调 → `_gw_*` 字段 → `snapshot()`（数据往里流）
  commands.py  浏览器点击 → 话题/服务/子进程（动作往外发）

**三条必须守住的老规矩**（都踩过坑，别"顺手优化"）
① `MODE_TEXT[mode]` 是**故意**直接下标：模式集合只有两种，传进来怪值就该
   立刻抛 KeyError 暴露调用方写错，而不是悄悄显示"未启动"。
② `api_tasks` 的 add 分支**不做** `or 1` 归一化，与 replace / `_load_tasks`
   故意不同 —— 前端已经保证 loops ≥ 1，这里多一层会让"用户填 0 应报错"被吞掉。
③ `set_initial_pose` 之后必须 `reset_runtime_cache('reloc')`：位姿跳变后
   TF 缓存、激光残留全要作废，否则网页上的车会拖一条假轨迹。
"""



import math                                # use 角度归一化
import os                                  # use 工作区探测 / 存图路径
import pathlib                             # use 地图库目录
import shlex                               # use 拼 map_saver 命令
import subprocess                          # use 调 map_saver_cli
import tempfile                            # use 存图临时目录
import threading                           # use 任务缓冲后台线程
import time                                # use 计时/超时

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

from agv_interfaces.msg import AgvCommand
from agv_interfaces.srv import AgvStore, CargoCmd

from ..stack_manager import (              # use 模式常量与遥控话题选择
    MODES, MODE_NAV, MODE_TEXT, teleop_topic_for,
)
from .util import _load_tasks, _save_tasks


class CommandMixin:
    """节点动作：遥控、模式切换、指令总线、任务缓冲、地图库。

    ★ 同样**不定义 `__init__`**，理由见 `state.py` 的类文档。
    """




    # ── 动作：都由 asyncio 一侧调用 ──
    def publish_twist(self, v, w):             # out 发遥控速度（已限幅）
        v = max(-self.max_linear, min(self.max_linear, float(v)))
        w = max(-self.max_angular, min(self.max_angular, float(w)))
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        # ★ 按当前模式选 publisher：建图栈没有 arbiter，只有 /cmd_vel_raw 能驱动底盘
        pub = self.pub_by_mode.get(self.mode) or self.pub_by_mode[MODE_NAV]
        self.cmd_pub = pub
        pub.publish(msg)
        return v, w

    def set_mode(self, mode):                  # out 切换工作模式并换遥控通道
        """切换模式：只改遥控话题，不负责起停栈（那是 StackRunner 的事）。"""
        m = str(mode or '').strip().lower()
        if m not in MODES:
            return False
        old_topic = self.teleop_topic
        with self._gw_lock:
            changed = self.mode != m
            self.mode = m
            self.cmd_pub = self.pub_by_mode[m]
            self.teleop_topic = teleop_topic_for(m)
        if old_topic != self.teleop_topic:
            self.log('info', f'遥控通道：{old_topic} → {self.teleop_topic}')
        # ★ 模式真的变了 = 换了一套栈，旧栈的地图/位姿/激光/路径全部作废。
        #   见 reset_runtime_cache 的注释：不清理就会出现"重影"和小车停在旧位置。
        if changed:
            self.reset_runtime_cache(f'切到{MODE_TEXT.get(m, m)}')
        return True

    def reset_runtime_cache(self, reason=''):  # out 作废上一套栈的运行态缓存
        """把"上一套栈留下的运行态"全部清掉，并推进地图代次。

        **为什么必须有它（真机问题）**
        网页上的地图、激光点、路径、小车位置全部来自本节点缓存的"最后一帧"。
        切换建图/导航时，旧栈的节点会被 SIGINT 杀掉，但**缓存不会自动消失** ——
        新栈的地图要等 slam/map_server 发布 /map 才来（建图模式要几秒，
        导航模式要等 map_server 激活），位姿要等 AMCL/TF 就绪才来。
        这段空窗期里前端一直在收到 snapshot，里面仍然是旧栈的 PNG + 旧位姿，
        于是画出"两张地图叠在一起（重影）"和"小车停在切换前的最后位置"。
        等新数据到了，如果新地图的 origin/尺寸和旧的接近，前端还会因为
        版本号判断/缓存没变而继续用旧图，重影就一直留着。

        所以换栈时必须**主动**把这几样置空，并让 map_epoch +1，
        前端看到 epoch 变了就会重置视图、清掉选中点、重新拉图。

        参数：reason - 写进日志的原因（方便对时间线）
        返回：无
        """
        with self._gw_lock:
            had_map = self._gw_map_png is not None
            self._gw_map_png = None
            self._gw_map_meta = None
            self._gw_map_epoch += 1
            self._gw_pose = None
            self._gw_scan = None
            self._gw_plan = None
            self._gw_task_state = None
            # ★ 位姿来源的时间戳/时钟也要一起清：新栈的时钟可能不是同一个
            #   （比如改了 use_sim_time），留着旧 stamp 会让"时间戳是否在前进"
            #   的判断拿新值去比旧基准，误判成 "变了" → 把旧位姿当新鲜的。
            self._gw_pose_stamp = 0.0
            self._gw_pose_stamp_at = 0.0
            self._gw_pose_clock = None
            self._gw_pose_fresh = False
            self._gw_pose_stale = False
            # ★ 底盘速度也要清：新栈的 /cmd_vel_raw 还没起来，
            #   留着旧栈的速度会让仪表显示"0.30 · -0.28"，而车根本没动。
            self._gw_speed = None
            # ★ 上一套栈的地图加载错误也要清：新栈还没起来时自检如果照旧
            #   念旧错误，会把人带到上一张图上去查。
            self._gw_map_error = None
            epoch = self._gw_map_epoch
        self._gw_last_pose_ok = 0.0
        self._gw_last_map_encode = 0.0
        # ★ 日志限流状态也要清：不清的话新栈第一条避障会被旧栈的
        #   5 秒窗口吞掉，而那正是最需要确认"新栈避障生效"的一刻。
        try:
            self._gw_logf.reset()
        except Exception:
            pass
        # 跨通道去重窗口也清：新栈的"第一次"不该被旧栈的记录当成重复丢掉。
        with self._gw_lock:
            self._gw_log_recent = []
        # ★★ 必须连 tf2 的历史缓存一起清（这是本轮最隐蔽的一层）。
        #   Buffer 默认缓存 10 秒的变换。旧栈的节点被杀之后，那段历史仍然
        #   躺在 buffer 里，`lookup_transform(map, base, Time())` 会继续
        #   返回"最后一个已知变换" —— 于是我们刚 set 成 None 的 _gw_pose，
        #   在下一个 0.2 秒的 tick 里就被旧 TF 重新填满了。
        #   现象就是：缓存清理日志明明打了，界面上的小车却纹丝不动地
        #   停在切换前的位置，你说它不对、它又"有条有理"地跟着旧数据动。
        #   clear() 无参数、无返回值，清掉全部历史。
        try:
            self.tf_buffer.clear()
        except Exception as exc:
            # 个别版本没有 clear()，退化成"记一条日志别静默"
            self.log('warn', f'tf 缓存清理失败（{type(exc).__name__}）：{exc}；'
                             '换栈后位姿可能短暂沿用旧值')
        self.log('info', f'{reason}：已清理上一套栈的地图/位姿/激光/路径缓存'
                         f'（地图代次 {epoch}{"" if had_map else "，原本就没有地图"}）')
        return epoch


    def send_command(self, command, args=None, route='', request_id=''):  # out 发业务指令
        msg = AgvCommand()
        msg.source = 'web'
        msg.command = str(command)
        msg.args = [str(a) for a in (args or [])]
        msg.route = str(route or '')
        msg.confidence = 1.0
        msg.request_id = str(request_id or f'web-{int(time.time() * 1000)}')
        self.cmd_web_pub.publish(msg)
        return msg.request_id

    def call_service(self, client, request, timeout=2.0):   # out 同步调服务（带超时）
        if not client.service_is_ready():
            return None, '服务尚未就绪（业务节点可能还没起来）'
        try:
            fut = client.call_async(request)
            deadline = time.time() + timeout
            while not fut.done() and time.time() < deadline:
                time.sleep(0.01)
            if not fut.done():
                return None, '服务调用超时'
            return fut.result(), None
        except Exception as exc:
            return None, f'服务调用异常：{exc}'

    def store_request(self, resource, action, name='', x=0.0, y=0.0,
                      yaw=0.0, kind='', point_names=None):  # out 调 /agv/store
        req = AgvStore.Request()
        req.resource = resource
        req.action = action
        req.name = name
        req.x, req.y, req.yaw = float(x), float(y), float(yaw)
        req.kind = kind
        req.point_names = list(point_names or [])
        return self.call_service(self.store_cli, req)

    def cargo_request(self, command):          # out 调 /agv/cargo（顶升/下降）
        req = CargoCmd.Request()
        req.command = command
        req.request_id = f'web-{int(time.time() * 1000)}'
        return self.call_service(self.cargo_cli, req)

    def set_initial_pose(self, x, y, yaw):      # out 重新定位（等价 RViz 2D Pose Estimate）
        """发布 /initialpose，让 AMCL 在该位姿附近重新撒粒子。

        **以前"重新定位用不了"的两层原因（本轮修）**

        ① 前端语义错了：旧代码把**当前位姿**原样发回来，等于"我在原地
           告诉 AMCL 我在原地" —— 除了把协方差重置成 0.25 以外没有任何
           定位修正作用。真正的 2D Pose Estimate 是由人指定"车实际在哪、
           朝哪"，所以前端必须让人改坐标（见 uiPose）。

        ② 后端**哑发**：只要一 publish 就回 {'ok': True}。可是在
           · 建图模式（栈里根本没有 AMCL）
           · 导航栈还没激活 / AMCL 已崩
           这两种情况下消息发出去没有任何订阅者，界面却显示"已重新定位"，
           用户点几次都一样，自然得出"这个按钮没用"的结论。
           现在先看订阅者数量，并在消息里说实话。

        参数：x, y, yaw - 目标位姿（地图坐标系，米 / 弧度）
        返回：(ok, message)。ok=False 表示这条消息没人接，等于没生效。
        """
        # ★ get_subscription_count() 是"此刻有多少个订阅者"，对 latching
        #   无要求 —— /initialpose 本来就是事件型 topic，没订阅者就是白发。
        try:
            n_sub = self.initial_pub.get_subscription_count()
        except Exception:
            n_sub = -1                     # 老版本 rclpy 没这接口，不阻塞
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        # 位置/朝向给一个中等不确定度，让 AMCL 在该点附近搜索而不是全图乱撒
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25
        cov[35] = 0.0685
        msg.pose.covariance = cov
        self.initial_pub.publish(msg)
        # ★ 发布后立刻把本地缓存位姿改成目标值：AMCL 收敛要 1~3 秒，
        #   这期间界面上小车还停在旧位置，用户会以为没生效又点一次。
        #   先"跳"到目标点、再由 AMCL 的真实结果覆盖，体感才对。
        with self._gw_lock:
            self._gw_pose = (float(x), float(y), float(yaw), time.time())
            # ★ 重定位是"人指定"的位姿，不是来自 TF，所以没有来源 stamp。
            #   stamp 置 0 表示"没有可比的来源时间戳"，让 _tf_is_fresh 走
            #   "第一次见"分支，不会拿 AMCL 的旧 stamp 来否掉它。
            self._gw_pose_stamp = 0.0
            self._gw_pose_stamp_at = 0.0
            self._gw_pose_clock = None
            self._gw_pose_fresh = True
            self._gw_pose_stale = False
        self._gw_last_pose_ok = time.monotonic()
        if n_sub == 0:
            return False, ('已发布 /initialpose，但当前没有任何节点订阅它'
                           '（建图模式没有 AMCL，或导航栈还没激活）——'
                           '这条重定位不会生效')
        return True, f'已发布重定位 (x {x:.2f}, y {y:.2f}, θ {math.degrees(float(yaw)):.0f}°)'

    # ── 任务缓冲：读写 tasks.yaml（等价终端控制台的 [9] 任务系统）──
    def get_task_store(self):                  # out 取任务缓冲读写器
        """返回 {'load': fn, 'save': fn}，文件是"当前地图目录下的 tasks.yaml"。

        ★ 与终端控制台共用同一个 tasks.yaml：网页加的任务，终端能看到，反之亦然。
          绝不能各存一份，否则两边看到的任务列表会不一样。
        """
        mm = self.get_map_manager()
        if mm is None:
            return None
        if not self.current_map:
            # 没选地图时退回默认地图目录（项目约定 maps/world666map）
            try:
                maps = mm.list_maps()
                if not maps:
                    return None
                self.current_map = maps[0]
            except Exception:
                return None
        path = mm.tasks_file(self.current_map)
        return {'load': lambda: _load_tasks(path),
                'save': lambda items: _save_tasks(path, items)}

    def start_task_buffer(self, items):        # out 后台逐条执行任务缓冲
        """把任务缓冲跑起来。每条任务按 loops 次数重复，失败即停并记录。

        ★ 为什么用后台线程而不是 asyncio 任务：里面要同步等 ROS 服务/状态，
          放进 event loop 会把整个网页卡住（连状态推送都停）。

        ★★ 为什么还要 join 旧线程（本轮修的真机 bug）：
          `stop_tasks()` 为了让界面立刻能点"开始执行"，会马上把
          `task_running` 置 False —— 但那一刻旧的后台线程可能还没从
          `_wait_task_outcome` 里退出（它每 0.3s 才看一次事件）。
          如果只凭 `task_running` 判断，用户在中止后立刻再点开始，
          就会**同时跑两个任务线程**：两个线程各自下发 start_task，
          task_manager 只受理第一单、拒绝第二单，于是"后面的任务
          得不到执行"，而且日志看起来像在跑、车却不动。
          这里先等旧线程收尾（stop 事件已置位，通常 <1s 退出），
          等不到就明确拒绝，绝不让两个线程叠在一起。
        """
        old = self._gw_task_thread
        if old is not None and old.is_alive():
            old.join(timeout=3.0)
            if old.is_alive():
                self.log('error', '上一个任务线程还没退出，请稍等一秒再点开始')
                return False
        if self.task_running:
            return False
        self._gw_task_stop.clear()
        self._gw_task_thread = threading.Thread(
            target=self._run_tasks, args=(list(items),), daemon=True)
        self.task_running = True
        self._gw_task_thread.start()
        return True

    def _run_tasks(self, items):               # in 内部：任务执行循环
        total = sum(max(1, int(t.get('loops', 1))) for t in items)
        done = idx = 0
        # ★ 记下总单数与当前单号：仪表要显示"搬运中 2/5"。
        #   为什么不从 task_manager 的 TaskState 推：它只报单条任务的
        #   内部进度（取货/顶升/运输），不知道缓冲里还有几单。这两个数
        #   只有网关自己知道，所以必须在这里维护。
        with self._gw_lock:
            self._gw_task_total = total
            self._gw_task_idx = 0
        self.log('info', f'开始执行任务缓冲：{len(items)} 条 / 共 {total} 次')
        try:
            for t in items:
                loops = max(1, int(t.get('loops', 1)))
                for _ in range(loops):
                    if self._gw_task_stop.is_set():
                        self.log('warn', f'任务缓冲已中止（完成 {done}/{total}）')
                        return
                    idx += 1
                    with self._gw_lock:
                        self._gw_task_idx = idx
                    pick, drop = t.get('pick', ''), t.get('drop', '')
                    route = t.get('route', '')
                    self.log('info', f'[{idx}/{total}] {pick} ⇒ {drop}'
                                     + (f'（经路线「{route}」）' if route else ''))
                    # ★ 关键：先把上一单残留的状态清掉，并记下"上一单的
                    #   task_id"。少了这两步，第二单刚开始就会被上一单残留的
                    #   DONE 立刻判成"已完成" —— 现象就是"只有第一单真跑了，
                    #   后面的唰唰全过，日志还说全部完成"。
                    outcome = self._start_one(idx, total, pick, drop, route)
                    if outcome == 'done':
                        done += 1
                        self.log('ok', f'[{idx}/{total}] 搬运完成')
                    elif outcome == 'cancel':
                        self.log('warn', '任务被取消，停止后续')
                        return
                    else:
                        self.log('error', f'[{idx}/{total}] 任务失败或超时，停止后续')
                        return
            self.log('ok', f'任务缓冲全部完成（{done}/{total} 次搬运）')
        finally:
            self.task_running = False
            # 结束后清掉进度：否则仪表会一直挂着"搬运中 5/5"，
            # 而实际上已经空闲（task_running=False 已能兜住显示，但
            # 留着旧单号下次开始时可能闪现上一轮的 5/5）。
            with self._gw_lock:
                self._gw_task_idx = 0
                self._gw_task_total = 0

    # 下发一单并等它跑完（带重试）
    # 参数：idx/total - 序号（日志用）；pick/drop - 点位名；route - 路线名
    # 返回：'done' / 'cancel' / 'fail'
    def _start_one(self, idx, total, pick, drop, route):   # in 内部：跑一单
        """下发 start_task 并等结果，遇到"没启动起来"就重发。

        ★ 为什么需要重试（真机 bug，第 2 单往后直接不动）：
          task_manager 跑完一单的收尾是「先发 DONE → 停 0.5 秒 → 状态机回
          IDLE」。网关一看到 DONE 就往下发第二单，此刻 task_manager 的
          `_can_start()` 里 `self._runner.is_alive()` 还是 True，于是
          第二单被**当场拒绝**（"已有任务在执行"）——但它照样 publish_state，
          网关读到的还是上一单的 DONE，看起来像"这单也成功了"，实际车没动。
          用户看到的就是"放多个任务，只执行第一个"。
          这里对"没启动起来"做有限重试（拒绝是瞬时的，0.5 秒后就正常），
          并把最终失败如实报出来，不再假装成功。
        """
        for attempt in range(3):
            prev = self._current_task_id()     # 上一单的 task_id（区分新旧帧）
            self.reset_task_state()
            self.send_command('start_task', [pick, drop], route=route)
            outcome = self._wait_task_outcome(prev, timeout=300.0,
                                              start_timeout=6.0)
            if outcome != 'nostart':
                return outcome
            if self._gw_task_stop.is_set():
                return 'cancel'
            self.log('warn', f'[{idx}/{total}] 任务没能启动（第 {attempt + 1} 次），'
                             f'1 秒后重发')
            time.sleep(1.0)
        # ★ 别猜原因。网关是通过话题下发指令的，**拿不到响应**，只能靠
        #   task_id 有没有变来推断。task_manager 拒绝时会打一句
        #   "[指令] start_task → 拒绝：<真实原因>"，on_rosout 已经缓存。
        #   以前这里硬写"检查是否处于手动模式 / 急停 / 路线不存在"，
        #   其中「路线「」不存在」在没选路线时必然是个空名字 —— 用户照着
        #   去查路线只会白费时间。现在有真原因就说真原因，没有就说没有。
        hint = ''
        rec = getattr(self, '_gw_last_reject', None)   # ★ getattr：测试用 AST 抽方法，不跑 __init__
        if rec and (time.monotonic() - rec[0]) <= 30.0:
            hint = f'；task_manager 回：{rec[1]}'
        self.log('error',
                 f'[{idx}/{total}] 任务没启动：6 秒内没收到新的 task_id'
                 + hint
                 + ('' if hint else '；task_manager 未响应或未上线，'
                                    '先确认导航栈已在运行、自检全绿'))
        return 'fail'

    def _current_task_id(self):                # in 内部：最近一帧的 task_id
        """取最近一帧 TaskState 的 task_id（没有则空串）。

        ★ 用途：任务缓冲要判断"这一单是不是新的一单"。
          task_manager 每 start 一次就把序号 +1（task-0001、task-0002…），
          所以 task_id 变化 = 新任务真的被受理了。靠它比靠"看到 IDLE"
          可靠得多 —— 上一单的 DONE / IDLE 帧都会带着**旧** task_id。
        """
        with self._gw_lock:
            ts = self._gw_task_state
        return str((ts or {}).get('task_id', '') or '')

    def _wait_task_outcome(self, prev_task_id='', timeout=300.0,
                           start_timeout=12.0, settle_timeout=8.0):
        """等**当前这一单**结束，返回 'done'/'cancel'/'fail'/'timeout'/'nostart'。

        参数：prev_task_id   - 下发前那一帧的 task_id（用来区分新旧单）
              timeout       - 整单最长等待（秒）
              start_timeout - 多久没看到"本单真在跑"就判定没启动
              settle_timeout- 看到 DONE 后再等多久让 task_manager 收尾回 IDLE

        ★ 为什么必须带 prev_task_id（这是一整类 bug 的根源）：
          `/agv/task_state` 是 1 Hz 周期发布，网关缓存的是**最后一帧**。
          上一单跑完留下 DONE，下一秒才回 IDLE。任务缓冲如果只判断
          "state == DONE"，那么在第二单下发后的头几百毫秒里读到的还是
          上一单的 DONE → 立刻返回 done → 第三单、第四单同样瞬间通过。
          结果就是"只执行第一个任务，后面的全跳过"。用 task_id 就能把
          "旧帧"和"本单的真实进展"分开。

        ★ 为什么看到 DONE 还要再等一会儿（settle）：
          task_manager 的 run_transport 结尾是「报 DONE → 睡 0.5s →
          状态机回 IDLE」。如果看到 DONE 立刻返回，调用方马上发下一单，
          那时后台线程还活着（`_runner.is_alive()`），会被 `_can_start()`
          拒掉。等它真正回到 IDLE 再返回，下一单才接得住。
        """
        ACTIVE = ('GO_LOAD', 'LIFTING', 'TRANSPORT', 'LOWERING')
        t0 = time.time()
        deadline = t0 + max(float(timeout), 1.0)
        done_at = 0.0          # 首次看到 DONE 的时刻（用于 settle）
        while time.time() < deadline:
            if self._gw_task_stop.is_set():
                return 'cancel'
            with self._gw_lock:
                ts = self._gw_task_state
            if ts is None:                     # 刚 reset 过，等第一帧
                if (time.time() - t0) > start_timeout:
                    return 'nostart'
                time.sleep(0.15)
                continue
            tid = str(ts.get('task_id', '') or '')
            st = str(ts.get('state', '')).upper()
            # 「本单被受理了」的判据：task_id 与下发前不同。
            #   task_manager 每次 start 成功都把序号 +1（task-0001 → 0002），
            #   **被拒绝时不会变** —— 所以这一条能干净地把"指令被拒/没响应"
            #   和"真的开始跑了"分开。
            #   兼容 goto 单点导航（task_id 恒为空串）：那种情况只认活跃态。
            if tid:
                accepted = (tid != prev_task_id)
            else:
                accepted = st in ACTIVE
            if not accepted:
                if (time.time() - t0) > start_timeout:
                    return 'nostart'
                time.sleep(0.15)
                continue
            # 受理了但状态还没走到活跃态：正常，继续等下一步
            if st == 'DONE':
                # ★ 看到 DONE 不等于"task_manager 已经能接下一单"。
                #   run_transport 的最后是「报 DONE → 睡 0.5s → 状态机回
                #   IDLE」，只等 DONE 就下发下一单，会被 _can_start() 里
                #   的 `_runner.is_alive()` 当场拒掉（"已有任务在执行"）。
                #   所以这里再等它真正回 IDLE，最多 settle_timeout 秒。
                if not done_at:
                    done_at = time.time()
                if (time.time() - done_at) > settle_timeout:
                    return 'done'              # 等不到 IDLE 也不卡死
                time.sleep(0.2)
                with self._gw_lock:
                    cur = str(((self._gw_task_state or {}).get('state', '')) or '')
                if cur in ('', 'IDLE'):
                    return 'done'              # 已收尾，可以安全接下一单
                continue
            if st in ('ERROR', 'FAILED'):
                return 'fail'
            if st in ('CANCELLED', 'CANCELED', 'PAUSED'):
                return 'cancel'
            time.sleep(0.25)
        return 'timeout'

    def stop_tasks(self, notify=True):         # out 中止任务缓冲
        """中止任务缓冲：置停止事件 + （可选）通知 task_manager 停车。

        ★ 为什么两件事都要做（以前只发了命令，是个 bug）：
          · 只发 `cancel_task`：task_manager 收到后确实取消导航、状态机回 IDLE，
            但网关自己的 `_run_tasks` 后台线程还卡在 `_wait_task_outcome`
            的循环里 —— 它退出的条件是 `_gw_task_stop` 被置位（或收到
            CANCELLED 状态）。而 task_manager 取消后几乎没有 CANCELLED
            这个瞬态（直接回 IDLE），于是线程会一直等到 300 秒超时才退出，
            期间 `task_running` 始终是 True → 界面永远"执行中"，
            而且因为 `task_running` 为真，连新任务都启动不了。
          · 只置 Event：任务缓冲停了，但车上的导航目标还在跑。
          所以必须成对做。

        参数：notify - 是否顺带发 cancel_task。急停场景传 False，
              由调用方自己发 `stop`（stop 本身就会取消任务，不必重复发）。
        """
        self._gw_task_stop.set()
        if notify:
            self.send_command('cancel_task')
        # 立刻把对外状态置 False：让界面马上能点"开始执行"，
        # 不用等后台线程的下一次循环判断。后台线程随后会自己收尾。
        self.task_running = False

    def reset_task_state(self):                # out 清掉上一次任务残留状态
        """把"上一次任务的状态"清干净，避免界面显示上一单的 DONE/ERROR。

        ★ 为什么需要：`/agv/task_state` 是 task_manager 的**周期发布**，
          最后一帧可能停在 DONE 或 ERROR 上（DONE 之后虽然内部回 IDLE，
          但下一帧要靠 1 Hz 定时器才发出来；如果那一刻网关刚好在忙，
          网页就会一直显示上一单的结果）。开新任务前清一下，界面才干净。
        """
        with self._gw_lock:
            self._gw_task_state = None

    # ── 保存地图（调 map_saver_cli，阻塞操作，放线程里跑）──
    def save_map_now(self, name, mm):          # out 保存当前 SLAM 地图
        """调 map_saver_cli 存当前 /map，再交给 MapManager 归位到地图库。

        参数：name - 地图名；mm - MapManager 实例
        返回：(ok, message)

        ★ 为什么要 `bash -lc 'source /opt/ros/humble/setup.bash && ...'`：
          本节点可能由 launch 起在一个只 source 了工作区、没有 source ROS 环境
          的 shell 里；不重新 source 会找不到 map_saver_cli 可执行文件。
        """
        tmp = tempfile.mkdtemp(prefix='agv_websave.')
        out = os.path.join(tmp, name)
        cmd = ('source /opt/ros/humble/setup.bash && '
               f'ros2 run nav2_map_server map_saver_cli -f {shlex.quote(out)}')
        try:
            r = subprocess.run(['bash', '-lc', cmd],
                               capture_output=True, text=True, timeout=45)
        except subprocess.TimeoutExpired:
            return False, '保存超时：建图栈还在跑吗？/map 有数据吗？'
        except Exception as exc:
            return False, f'调用 map_saver_cli 失败：{exc}'
        if not os.path.isfile(out + '.pgm'):
            tail = ((r.stdout or '')[-200:] + ' ' + (r.stderr or '')[-150:]).strip()
            return False, f'没生成 pgm，检查建图栈是否在跑、是否走过场地。{tail}'
        try:
            # ★ 建图时存的点位/路线落在 maps/_mapping 草稿目录（地图名要等
            #   保存这一刻才定），save_map 会在建空模板之前先把草稿并进来。
            return mm.save_map(name, out + '.pgm', out + '.yaml', overwrite=True)
        except Exception as exc:
            return False, f'写入地图库失败：{exc}'

    # ── 工作区定位（起停栈时需要知道 install/setup.bash 在哪）──
    def get_workspace_root(self):              # out 工作区根目录（找不到返回 '')
        """定位 colcon 工作区根目录（含 src/ 与 install/ 的那一层）。

        优先级：ws_root 参数 > AGV_WS 环境变量 > 从本文件位置往上找。
        ★ 不能只看 __file__：本文件装进 install/ 后的路径是
          install/agv_web/lib/python3.x/site-packages/agv_web/…，
          用 parents[N] 硬数层数在不同 colcon 布局下会错位；
          所以改成"逐级往上找，看到同时有 src 和 install 的那层就停"。
        """
        if self.ws_root_override and os.path.isdir(self.ws_root_override):
            return os.path.abspath(self.ws_root_override)
        env = os.environ.get('AGV_WS', '').strip()
        if env and os.path.isdir(env):
            return os.path.abspath(env)
        cur = pathlib.Path(__file__).resolve()
        for parent in [cur] + list(cur.parents):
            if (parent / 'src').is_dir() and (parent / 'install').is_dir():
                return str(parent)
        # 退路：源码树里调试运行时，src/ 的上层就是工作区
        for parent in [cur] + list(cur.parents):
            if (parent / 'src').is_dir():
                return str(parent)
        return ''

    # ── 地图库（懒建，纯逻辑模块，不依赖 ROS）──
    def get_map_manager(self):                 # out 取地图库管理器
        if self._gw_mm is not None:
            return self._gw_mm
        try:
            from agv_common.map_manager import MapManager, default_maps_root
            root = self.get_workspace_root()
            ws = pathlib.Path(root) if root else pathlib.Path(__file__).resolve().parents[3]
            self._gw_mm = MapManager(default_maps_root(ws=str(ws)))
        except Exception as exc:
            self.log('warn', f'地图库不可用：{exc}')
            return None
        return self._gw_mm
