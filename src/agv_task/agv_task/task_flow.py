"""
task_flow —— 两条业务流程的执行体（单点导航 / 完整搬运）。

从原 task_manager_node 里抽出的大块：原节点把"指令解析""流程执行""ROS 收发"
全塞在一个类里，524 行非注释代码，是项目里最大的单体。
这里只负责**流程怎么走**，收发包由节点负责：

    单点导航：目标 →（路线各点）→ 目标点
    完整搬运：取货点 → 顶升 →（路线各点）→ 放货点 → 下降 → 完成

依赖都以构造参数注入（nav/cargo/store/validator），因此本模块不 new 任何 ROS 实体，
流程本身可以被读、被替换、被单测式地审视，不受节点接线方式牵连。

失败一律收敛到 _task_fail / on_nav_failed：状态机进 ERROR，并发零速停车，
不留"卡在中间态"的可能。
"""


class TaskFlow:                          # out 业务流程执行体
    # 初始化
    # 参数：node       - 宿主节点（日志/状态上报）
    #       manager    - 搬运状态机；goto - 单点导航状态机
    #       nav        - NavRunner；cargo - CargoClient；store - StoreClient
    #       validator  - GoalValidator（点位预检）；abort - 中止标志
    #       publish_zero - 发零速的回调（停车用）
    # 返回：无
    def __init__(self, node, manager, goto, nav, cargo, store,
                 validator, abort, publish_zero,
                 slot_approach=True, slot_soft=None):    # out 对外构造函数
        self.node = node                 # def 宿主节点
        self.manager = manager           # def 搬运状态机
        self.goto = goto                 # def 单点导航状态机
        self.nav = nav                   # def 导航执行器
        self.cargo = cargo               # def 搬货客户端
        self.store = store               # def 点位/路线客户端
        self.validator = validator       # def 点位预检判据
        self.abort = abort               # def 中止标志
        self.publish_zero = publish_zero  # def 发零速
        self.on_state = lambda: None     # def 状态上报回调（节点注入）
        # 槽内点位进槽策略（见 slot_approach.py）：
        #   slot_approach=True 时先到「X号货架」对位，再直行进槽；
        #   slot_soft 是槽内软到点的容差（None 用默认值）
        self.slot_approach = bool(slot_approach)
        self.slot_soft = dict(slot_soft or {})

    # 日志简写
    # 参数：无
    # 返回：Logger
    def _log(self):                      # in 内部方法
        return self.node.get_logger()

    # 是否已被取消/急停
    # 参数：无
    # 返回：bool
    def _aborted(self):                  # in 内部方法
        return self.abort is not None and self.abort.is_set()

    # 中止收尾：状态机回空闲 + 停车 + 上报
    # 参数：无
    # 返回：无
    #
    # ★ 为什么流程自己要收尾，而不是靠 handle_cancel 顺手清掉：
    #   取消/急停是在**另一个线程**（指令回调）里置标志的，流程线程可能
    #   正卡在 nav.step() 里，等它反应过来时已经过去几百毫秒。这段时间里
    #   状态机停在 GO_LOAD / LIFTING / TRANSPORT —— 网页照旧显示"执行中"，
    #   而且如果 handle_cancel 的清理和流程的回退交叉执行，还可能把刚清好
    #   的状态又盖回去。让"发现中止的那一方"负责收尾，语义才闭合：
    #   谁看到中止，谁就把状态机摆回 IDLE。
    #   重复调用是安全的（manager.cancel / goto.cancel 都是幂等赋值）。
    def _stop_quietly(self):             # in 内部方法
        self.manager.cancel()            # use 搬运状态机回 IDLE
        self.goto.cancel()               # use 单点导航状态机回 IDLE
        self.publish_zero()              # io 停车（取消/急停都要确保车停住）
        self.on_state()

    # 中止的统一收尾：打一句日志 + 静默停车回空闲
    # 参数：stage - '导航' / '搬运'（用于日志措辞）
    # 返回：无
    #
    # ★ 为什么要收成一个方法（代码可省略处之一）：
    #   原来 5 处都是这两行的复制粘贴 ——
    #       log.warn('[搬运] 已被取消，停止任务（不算失败）')
    #       self._stop_quietly()
    #       return
    #   复制带来的真实问题是**措辞会漂**：导航阶段打的是"[导航] 已被取消"，
    #   搬运阶段打的是"[搬运] 已被取消，停止任务（不算失败）"，同一件事
    #   两种说法，用户以为是两回事。收成一处后日志格式统一，
    #   以后要加"(已跑 12.3 秒)"这种信息也只改一个地方。
    def _abort_stop(self, stage='搬运'):  # in 内部方法
        self._log().warn(f'[{stage}] 已被取消，停止任务（不算失败）')
        self._stop_quietly()

    # ══════════════ 槽内点位的进槽策略 ══════════════

    # 把一个业务点位拆成"导航腿"列表
    # 参数：name - 点名；pose - {'x','y','yaw'}
    # 返回：list[dict]（每项含 name/x/y/yaw，槽内那条带 soft）
    #
    # ★ 为什么槽内点要拆两腿（实车故障根因）：
    #   凵型槽净宽 0.80 m、车体 0.50 m，进槽后四周只剩 0.19 m，
    #   原地纠偏角度根本转不动。而 Nav2 到点要求航向 ≤0.20 rad。
    #   只要进槽时偏一点，就会「到点→转不动→恢复行为→越转越偏→任务超时」，
    #   顶升/下降永远等不到。把对位放到槽口（X号货架，周围有空间）做，
    #   最后一段就是 1.37 m 直线，本来不需要转身。
    def _legs(self, name, pose):      # in 内部方法
        from agv_task.slot_approach import is_slot_point, mouth_name
        plain = dict(pose, name=name)
        if not (self.slot_approach and is_slot_point(name)):
            return [plain]
        mouth = mouth_name(name)
        mouth_pose = self.store.waypoint(mouth) if mouth else None
        if mouth_pose is None:
            # 对位点被别人改名/删掉了：退回"直接进槽 + 软到点"，
            # 至少不会因为查不到点而整单失败。
            return [dict(plain, soft=self.slot_soft or None)]
        return [dict(mouth_pose, name=f'{mouth}（槽口对位）'),
                dict(plain, soft=self.slot_soft or None)]

    # 导航到一个业务点位（槽内点自动带上槽口对位）
    # 参数：name - 点名；pose - {'x','y','yaw'}
    # 返回：bool（True=到点）
    def _go(self, name, pose):        # in 内部方法
        legs = self._legs(name, pose)
        if len(legs) > 1:
            self._log().info(
                f'[进槽] 先到「{legs[0]["name"]}」对正车头，再直行进槽'
                f'（槽内原地纠不了角度，对位必须在槽外完成）')
        for leg in legs:
            if not self.nav.step(leg):
                return False
        return True

    # ══════════════ 单点 / 多点导航 ══════════════

    # 单点/多点导航：点名或坐标 →（路线各点）→ 目标点
    # 参数：target - [点名] 或 [x, y, yaw]；route - 路线名（可空）
    # 返回：无
    def run_goto(self, target, route):   # out 对外接口
        log = self._log()
        if len(target) == 1:             # # 点名：查点位表拿坐标
            name = target[0]
            pose = self.store.waypoint(name)
            if pose is None:
                log.error(f'点位「{name}」不存在或查点服务不可用，导航取消')
                self.goto.on_nav_failed()
                self.on_state()
                return
        else:                            # # 坐标
            try:
                x, y = float(target[0]), float(target[1])
                yaw = float(target[2]) if len(target) >= 3 else 0.0
            except (TypeError, ValueError):
                log.error('坐标格式非法，导航取消')
                self.goto.on_nav_failed()
                self.on_state()
                return
            pose = {'x': x, 'y': y, 'yaw': yaw}
            name = f'({x:.2f}, {y:.2f})'

        mid = self.store.route(route)    # def 路线途经点（list 或 None）
        if route and mid is None:        # # 选了路线却取不到：直接失败，不偷偷改成单点
            log.error(f'路线「{route}」取不到，导航取消')
            self.goto.on_nav_failed()
            self.on_state()
            return
        if not self.precheck(name, pose):     # use 点位预检
            self.goto.on_nav_failed()
            self.on_state()
            return

        ok, reason = self.goto.start(name, pose)   # use 启动导航状态机
        if not ok:
            log.warn(reason)
            return
        log.info(reason)
        self.on_state()                  # io 立刻上报"导航中"
        if mid:                          # # 先跑路线各点
            log.info(f'导航路线「{route}」：共 {len(mid)} 个途经点')
            if not self.nav.sequence(mid):
                if self._aborted():      # # 取消不是失败，别把状态机推到 ERROR
                    self._abort_stop('导航')
                    return
                self.goto.on_nav_failed()
                self.on_state()
                return
        if self._go(name, pose):         # use 目标点（槽内点会先到槽口对位）
            self.goto.on_nav_succeeded()
        elif self._aborted():
            self._abort_stop('导航')
            return
        else:
            self.goto.on_nav_failed()
        self.on_state()

    # ══════════════ 完整搬运 ══════════════

    # 完整搬运：取货点 → 顶升 →（路线各点）→ 放货点 → 下降 → 完成
    # 参数：pick - 取货点名；drop - 放货点名；route - 路线名（可空）
    # 返回：无
    def run_transport(self, pick, drop, route):   # out 对外接口
        log = self._log()
        # ★ 这条必须打得早：它命中网页日志的 [搬运] 白名单，是
        #   "指令真的到了 task_manager" 的唯一证据。以前第一句日志在
        #   两次查点之后，查点一慢（服务未就绪时单次最长 6 秒），
        #   网页上就只剩"任务没能启动"，完全看不出任务其实已受理。
        log.info(f'[搬运] 开始：{pick} → {drop}（校验点位…）')
        pick_pose = self.store.waypoint(pick)
        drop_pose = self.store.waypoint(drop)
        if pick_pose is None:
            return self.task_fail(f'取货点「{pick}」不存在或查点服务不可用')
        if drop_pose is None:
            return self.task_fail(f'放货点「{drop}」不存在或查点服务不可用')
        mid = self.store.route(route)    # def 路线途经点
        if route and mid is None:
            return self.task_fail(f'路线「{route}」取不到')
        for nm, ps in ((pick, pick_pose), (drop, drop_pose)):   # use 两点都预检
            if not self.precheck(nm, ps):
                return self.task_fail(f'点位「{nm}」预检未通过')

        # ★ 查点这几秒里用户可能已经点了取消/急停。这时候不该把状态
        #   推到 ERROR（ERROR 是 _can_start() 的硬拒绝态，会连累下一单），
        #   按"中止"安静收尾即可。
        if self._aborted():
            return self._abort_stop('搬运')

        # ★ 兜底：正常路径已经在 handle_start_task 里调过 manager.start()
        #   （受理即建单 —— 网关靠 task_id 变化判断任务有没有真被受理，
        #   等线程里查完点再 start 会让网关 6 秒判不出结果）。
        #   这里只在"直接调用本方法"的场景补一次（单测、其它调用方）。
        if self.manager.is_idle():
            ok, reason = self.manager.start(pick, drop)     # use 启动搬运状态机
            if not ok:
                return self.task_fail(reason)
            log.info(reason)
            self.on_state()

        # ── 第 1 步：前往取货点 ──
        log.info(f'[搬运] 前往取货点「{pick}」')
        if not self._go(pick, pick_pose):
            # ★ 关键：中止导致的"没走到"不算失败。
            #   以前这里一律 task_fail，会把状态机推到 ERROR；而 ERROR 是
            #   _can_start() 里明确拒绝的状态（"搬运状态为 ERROR，无法接
            #   新任务"）。于是用户点了中止 → 状态 ERROR → 之后再也开不了
            #   新任务，只能重启节点。这就是"中止之后既不能终止也不能启动
            #   新任务"的真根因。
            #   取消是用户的正常操作，收尾应该回 IDLE，不是 ERROR。
            if self._aborted():
                self._abort_stop('搬运')
                return
            return self.task_fail('前往取货点失败')
        self.manager.step_success()      # GO_LOAD → LIFTING

        # ── 第 2 步：顶升 ──
        self.on_state()
        # ★ 措辞统一成 "[顶升] 开始 / 成功 / 失败" 三态（后两个由 CargoClient 打）。
        #   以前是 "[搬运] 顶升"，和结束的 "[搬货] lift 成功" 对不上，
        #   回看日志时找不出这一对边界 —— 而用户明确要求这两个事件要能看清。
        #   2026-10-18 又按用户要求把"结束"改成"成功/失败"：
        #   网页日志靠文本判红绿（见 web_gateway.on_rosout），"结束"太模糊。
        log.info(f'[顶升] 开始（{pick}）')
        if not self.cargo.step('lift', pick):
            if self._aborted():
                self._abort_stop('搬运')
                return
            return self.task_fail('顶升失败（搬货服务返回失败）')
        self.manager.step_success()      # LIFTING → TRANSPORT

        # ── 第 3 步：运输（路线各点 → 放货点）──
        self.on_state()
        log.info(f'[搬运] 运输到放货点「{drop}」'
                 + (f'（经路线「{route}」，{len(mid)} 个途经点）' if mid else ''))
        if mid and not self.nav.sequence(mid):      # use 先跑路线各点
            if self._aborted():
                self._abort_stop('搬运')
                return
            return self.task_fail('运输中途经点导航失败')
        if not self._go(drop, drop_pose):
            if self._aborted():
                self._abort_stop('搬运')
                return
            return self.task_fail('运输到放货点失败')
        self.manager.step_success()      # TRANSPORT → LOWERING

        # ── 第 4 步：下降放货 ──
        self.on_state()
        log.info(f'[下降] 开始（{drop}）')
        if not self.cargo.step('lower', drop):
            if self._aborted():
                self._abort_stop('搬运')
                return
            return self.task_fail('下降失败（搬货服务返回失败）')
        self.manager.step_success()      # LOWERING → DONE

        self.on_state()
        # ★ 任务结束是"事件"，必须显眼：用 [任务完成] 前缀，
        #   和中间过程日志区分开，回看时一眼能找到（用户明确要求）。
        log.info(f'[任务完成] {pick} → {drop}')
        self.nav.sleep(0.5)
        self.manager.step_success()      # DONE → IDLE（可接下一单）
        self.on_state()

    # ══════════════ 预检与失败收敛 ══════════════

    # 点位预检：图外 / 压障碍 / 净空不足的点直接拒绝
    # 参数：name - 点名（日志用）；pose - {'x','y'}
    # 返回：True=通过
    def precheck(self, name, pose):      # out 对外接口
        if self.validator is None:       # # 没配判据：不拦
            return True
        ok, reason = self.validator.check(pose['x'], pose['y'])
        if not ok:
            self._log().error(f'点位「{name}」预检未通过：{reason}')
        return bool(ok)

    # 任务失败：状态机置 ERROR + 上报 + 停车
    # 参数：reason - 中文原因
    # 返回：False（便于 return self.task_fail(...) 写法）
    def task_fail(self, reason):         # out 对外接口
        self._log().error(f'任务失败：{reason}')
        self.manager.fail()
        self.publish_zero()              # io 停车
        self.on_state()
        return False
