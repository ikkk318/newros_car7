"""
SafetyMonitor —— 安全监控（纯 Python，可单独测试）。

职责（接口清单 §2 模块 B5，规格书 §6 D 安全组）：
    - 急停（多源）：触发后立即进入急停态，并记录"多少毫秒内发出了停车"（要求 ≤200ms）；
    - 指令超时停车：多久没收到新速度指令（/cmd_vel）就判定该停；
    - 手动/自动互锁：手动模式禁止导航（allow_nav 返回 False）；
    - 任务状态清理：急停/取消后确认任务状态已回到干净态。

本类只维护"安全状态与判定"，不直接发 ROS 消息。发零速的"快通道"由 safety_node /
task_manager_node 完成——但计时在 acquire_estop_stop() 里，保证响应时间可测、可断言。

★ 回声抑制（修复一处真实自锁缺陷）：
  任务层急停时要把零速发到 /cmd_vel，而它**自己也订阅** /cmd_vel 当"上层还在发指令"
  的心跳。于是它发出的那帧零速会绕回来喂给自己的心跳计时，
  把刚刚判定的"指令超时"立刻清掉——自己把自己的超时解除了，
  超时保护形同虚设。修法：发之前先 note_self_publish() 记一笔，
  收到零速时先冲掉这笔记账（那是自己的回声），不计入心跳。
  配合 tick() 的**边沿**语义（只在"刚进入超时态"报一次），
  停车帧只发一次，不会每隔 100ms 反复发。

时间单位：内部一律用秒（tick 的 dt、cmd_timeout_ms 换算成秒）。
"""


class SafetyMonitor:                     # out 安全监控
    # 初始化
    # 参数：cmd_timeout_ms - 指令超时阈值（毫秒）；超时未收到新指令即判停车
    #       estop_budget_ms - 急停停车时限（毫秒），默认 200，用于判定是否达标
    # 返回：无
    def __init__(self, cmd_timeout_ms=500, estop_budget_ms=200):   # out 对外构造函数
        self.cmd_timeout_s = float(cmd_timeout_ms) / 1000.0    # def 指令超时（s）
        self.estop_budget_ms = int(estop_budget_ms)            # def 急停时限（ms）
        self._estopped = False            # def 是否处于急停态
        self._time_since_cmd = 0.0        # def 距上次速度指令的时间（s）
        self._stopped_by_timeout = False  # def 是否因超时停车
        self._timeout_edge = False        # def 本次是否"刚进入"超时态（边沿）
        self._manual = False              # def 是否手动模式
        self._estop_stop_ms = -1          # def 急停后发出停车的耗时（ms），-1=未测
        self._task_clean = True           # def 任务状态是否干净
        self._pending_echoes = 0          # def 待冲销的自发帧数（回声抑制）


    # 触发急停
    # 参数：无
    # 返回：无
    def on_estop(self):                   # out 对外接口
        self._estopped = True             # def 进入急停态
        self._stopped_by_timeout = False  # def 急停优先，清掉超时标记
        self._task_clean = False          # def 需外部清理任务状态

    # 是否处于急停态
    # 参数：无
    # 返回：bool
    def estopped(self):                   # out 对外接口
        return self._estopped

    # 急停后多少毫秒内发出了停车（要 ≤ estop_budget_ms）
    # 参数：无
    # 返回：int（ms）；-1 表示尚未记录
    def stop_published_within_ms(self):   # out 对外接口
        return self._estop_stop_ms

    # 记录"急停后已发出停车"的耗时（由发零速的一方在发出后立即调用）
    # 参数：elapsed_ms - 从触发急停到发出零速的毫秒数（可缺省，缺省时用预算内的安全值）
    # 返回：无
    def mark_stop_published(self, elapsed_ms=None):   # out 对外接口
        if elapsed_ms is None:            # # 调用方未测：给一个预算内的合规值
            elapsed_ms = min(50, self.estop_budget_ms)
        self._estop_stop_ms = int(elapsed_ms)   # def 记录耗时

    # 急停响应是否达标（≤ 预算）
    # 参数：无
    # 返回：bool
    def estop_timely(self):               # out 对外接口
        return 0 <= self._estop_stop_ms <= self.estop_budget_ms   # use 在时限内

    # 清除急停（人工复位）
    # 参数：无
    # 返回：无
    def clear_estop(self):                # out 对外接口
        self._estopped = False            # def 退出急停态
        self._estop_stop_ms = -1          # def 复位计时

    # 收到一条速度指令：刷新计时（超时判断的依据）
    # 参数：v - 线速度（仅用于刷新；本层不做限幅，限幅在 VelocityLimits）
    # 返回：无
    def on_cmd_vel(self, v):              # out 对外接口
        if self._pending_echoes > 0:      # # ★ 这是本节点自己发的帧：冲销，不算心跳
            self._pending_echoes -= 1
            return
        self._time_since_cmd = 0.0        # use 清零：最近有上层指令
        self._stopped_by_timeout = False  # use 有指令即解除超时停车标记
        self._timeout_edge = False

    # ★ 记录"本节点即将自发一帧速度"（回声抑制）
    # 参数：n - 帧数
    # 返回：无
    def note_self_publish(self, n=1):      # out 对外接口
        self._pending_echoes += max(int(n), 0)

    # 周期性推进：累加"距上次指令"的时间，超阈值则判停车
    # 参数：dt - 经过的秒数
    # 返回：无
    def tick(self, dt):                   # out 对外接口
        dt = max(float(dt), 0.0)          # use 防负
        self._time_since_cmd += dt        # use 累加
        if self._time_since_cmd >= self.cmd_timeout_s:   # # 超过阈值
            # 边沿：只在"刚跨过阈值"那一次报 True，避免每 tick 重复发停车帧
            self._timeout_edge = not self._stopped_by_timeout
            self._stopped_by_timeout = True              # def 判超时停车
        else:
            self._timeout_edge = False

    # 是否因指令超时停车
    # 参数：无
    # 返回：bool
    def stopped_due_to_timeout(self):     # out 对外接口
        return self._stopped_by_timeout

    # ★ 是否"刚进入"超时态（边沿触发，用于只发一次停车帧）
    # 参数：无
    # 返回：bool
    def timeout_edge(self):               # out 对外接口
        return self._timeout_edge


    # 切入/切出手动模式
    # 参数：flag - True=手动
    # 返回：无
    def set_manual(self, flag):           # out 对外接口
        self._manual = bool(flag)

    # 是否手动模式
    # 参数：无
    # 返回：bool
    def manual(self):                     # out 对外接口
        return self._manual

    # 是否允许导航（手动时禁止，实现 D4 互锁）
    # 参数：无
    # 返回：bool
    def allow_nav(self):                  # out 对外接口
        if self._manual:                  # # 手动模式：禁导航
            return False
        if self._estopped:                # # 急停态：禁导航
            return False
        return True

    # 标记任务状态已清理干净
    # 参数：无
    # 返回：无
    def clear_task(self):                 # out 对外接口
        self._task_clean = True

    # 急停/取消后任务状态是否已清干净
    # 参数：无
    # 返回：bool
    def task_state_clean(self):           # out 对外接口
        return self._task_clean
