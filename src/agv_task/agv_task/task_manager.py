"""
TaskManager —— 完整搬运状态机（纯 Python 逻辑，可单独测试）。

职责：把搬运全流程串起来——去取货 → 顶升 → 运输 → 下降 → 完成。

状态流转（step_success() 每调一次走一格）：
    IDLE → GO_LOAD → LIFTING → TRANSPORT → LOWERING → DONE → (自动回 IDLE)
    任一步出错 → ERROR；急停 → PAUSED → resume() 回到原状态。

本类只管"状态怎么走"，不亲自导航/顶升——真正动作由 task_manager_node 执行，
每完成一步就调 step_success()；失败调 on_nav_failed()/fail()；急停调 on_estop()。
因此本类是纯逻辑、可单测。
"""

# 正向状态序列（step_success 沿此推进）
SEQUENCE = (                             # def 搬运主序列
    'GO_LOAD',          # 前往取货点
    'LIFTING',          # 顶升（调 /agv/cargo 服务）
    'TRANSPORT',        # 运输到放货点
    'LOWERING',         # 下降放货
    'DONE',             # 完成
)

# 每个状态对应的人类可读进度与提示（发布 TaskState 用）
STATE_PROGRESS = {                       # def 状态 → (进度, 中文提示)
    'IDLE': (0.0, '空闲，等待任务'),
    'GO_LOAD': (0.15, '前往取货点'),
    'LIFTING': (0.40, '顶升中'),
    'TRANSPORT': (0.60, '运输中'),
    'LOWERING': (0.85, '下降放货中'),
    'DONE': (1.0, '搬运完成'),
    'PAUSED': (0.0, '已暂停（急停/停止）'),
    'ERROR': (0.0, '任务异常，需人工复位'),
}


class TaskManager:                       # out 搬运任务状态机
    # 初始化
    # 参数：无
    # 返回：无
    def __init__(self):                  # out 对外构造函数
        self._state = 'IDLE'             # def 当前状态
        self._pick = ''                  # def 取货点名
        self._drop = ''                  # def 放货点名
        self._task_id = ''               # def 任务号
        self._seq = 0                    # def 任务序号（生成 task_id）

    # 开始一次搬运
    # 参数：pick - 取货点名；drop - 放货点名；task_id - 可选任务号（不给则自动生成）
    # 返回：(ok, reason)
    def start(self, pick, drop, task_id=None):    # out 对外接口
        if self._state not in ('IDLE', 'DONE'):   # # 非空闲：拒绝（串行执行）
            return False, f'当前状态 {self._state}，无法开始新任务'
        if not pick or not drop:         # # 点位名不可空
            return False, '取货点与放货点名不能为空'
        self._pick = str(pick)           # def 取货点
        self._drop = str(drop)           # def 放货点
        self._seq += 1                   # use 序号自增
        self._task_id = task_id or f'task-{self._seq:04d}'   # def 任务号
        self._state = 'GO_LOAD'          # def 进入第一格
        return True, f'任务 {self._task_id} 开始：{self._pick} → {self._drop}'

    # 当前状态
    # 参数：无
    # 返回：str
    def state(self):                     # out 对外接口
        return self._state

    # 任务号
    # 参数：无
    # 返回：str
    def task_id(self):                   # out 对外接口
        return self._task_id

    # 当前目标点名（取货/放货按阶段给）
    # 参数：无
    # 返回：str
    def target_location(self):           # out 对外接口
        if self._state in ('GO_LOAD', 'LIFTING'):
            return self._pick            # # 取货阶段
        if self._state in ('TRANSPORT', 'LOWERING'):
            return self._drop            # # 放货阶段
        return ''                        # # 其他阶段无目标

    # 取货点名
    # 参数：无
    # 返回：str
    def pick_location(self):             # out 对外接口
        return self._pick

    # 放货点名
    # 参数：无
    # 返回：str
    def drop_location(self):             # out 对外接口
        return self._drop

    # 进度与提示（发布 TaskState 用）
    # 参数：无
    # 返回：(progress: float, message: str)
    def progress_info(self):             # out 对外接口
        return STATE_PROGRESS.get(self._state, (0.0, self._state))

    # 当前步成功 → 推进一步
    # 参数：无
    # 返回：(ok, reason, new_state)
    def step_success(self):              # out 对外接口
        if self._state in ('IDLE', 'ERROR', 'PAUSED'):    # # 这些态不推进
            return False, f'状态 {self._state} 不可推进', self._state
        if self._state == 'DONE':        # # 已完成：自动回 IDLE
            self._state = 'IDLE'
            return True, '任务完成，回到空闲', self._state
        idx = SEQUENCE.index(self._state)     # use 找当前格
        if idx + 1 < len(SEQUENCE):           # # 还有下一格
            self._state = SEQUENCE[idx + 1]
            return True, f'进入 {self._state}', self._state
        return False, '未知状态', self._state

    # 导航失败 → ERROR
    # 参数：无
    # 返回：无
    def on_nav_failed(self):             # out 对外接口
        self._state = 'ERROR'

    # 通用失败 → ERROR
    # 参数：无
    # 返回：无
    def fail(self):                      # out 对外接口
        self._state = 'ERROR'

    # 取消 → 回 IDLE（清空目标）
    # 参数：无
    # 返回：无
    def cancel(self):                    # out 对外接口
        self._state = 'IDLE'
        self._pick = ''
        self._drop = ''

    # 急停 → PAUSED
    # ★ 这里只记"已暂停"，**不记**暂停前那一格：急停会取消 Nav2 目标并中止后台线程，
    #   当前这一段导航/搬货已经作废，续跑那一格没有意义（resume 一律回 IDLE 重配任务）。
    # 参数：无
    # 返回：无
    def on_estop(self):                  # out 对外接口
        if self._state not in ('IDLE', 'ERROR', 'PAUSED'):   # # 有活动任务才需暂停
            self._state = 'PAUSED'

    # 人工复位：ERROR/PAUSED → IDLE（清空任务）
    # 参数：无
    # 返回：无
    def reset(self):                     # out 对外接口
        self._state = 'IDLE'
        self._pick = ''
        self._drop = ''

    # 是否处于可接单的干净态
    # 参数：无
    # 返回：bool
    def is_idle(self):                   # out 对外接口
        return self._state in ('IDLE', 'DONE')
