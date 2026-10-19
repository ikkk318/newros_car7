"""
GotoController —— 单点导航状态机（纯 Python 逻辑，可单独测试）。

职责（接口清单 §2 模块 B4）：把已有的 goto_pose() 包一层状态机，
让外面能问"现在到哪一步了"。本类只维护状态迁移，不亲自跑导航——
真正的导航由 task_manager_node 调 goto_pose()，再把成功/失败回调进来。

状态（与 TaskState.state 对齐，单点导航只用到 4 个）：
    IDLE → (start) → GO_LOAD → (on_nav_succeeded) → DONE
                           ↘ (on_nav_failed) → ERROR
    cancel() 任意状态 → IDLE

坐标换算（点名 → 坐标）由调用方通过 waypoint 服务完成，本类只收 name 与最终坐标，
保持"纯逻辑、可测"。
"""


class GotoController:                    # out 单点导航状态机
    # 初始化
    # 参数：无
    # 返回：无
    def __init__(self):                  # out 对外构造函数
        self._state = 'IDLE'             # def 当前状态
        self._target = ''                # def 目标点名
        self._pose = None                # def 目标坐标字典 {'x','y','yaw'} 或 None

    # 开始一次导航
    # 参数：name - 目标点名；pose - 可选，已知的坐标 dict（给了就免去再查服务）
    # 返回：(ok, reason) —— 已在导航中会拒绝
    def start(self, name, pose=None):    # out 对外接口
        if self._state == 'GO_LOAD':     # # 已在导航：拒绝并发
            return False, f'已在前往「{self._target}」，请勿重复下发'
        self._target = str(name)         # def 记目标点名
        self._pose = dict(pose) if pose else None   # def 记坐标（可能为空）
        self._state = 'GO_LOAD'          # def 迁移到"前往中"
        return True, f'开始前往「{self._target}」'

    # 当前状态
    # 参数：无
    # 返回：str
    def state(self):                     # out 对外接口
        return self._state

    # 当前目标点名
    # 参数：无
    # 返回：str
    def target(self):                    # out 对外接口
        return self._target

    # 目标坐标
    # 参数：无
    # 返回：dict 或 None
    def target_pose(self):               # out 对外接口
        return dict(self._pose) if self._pose else None

    # Nav2 报成功时调
    # 参数：无
    # 返回：无
    def on_nav_succeeded(self):          # out 对外接口
        if self._state == 'GO_LOAD':     # # 只有导航中才接受结果
            self._state = 'DONE'

    # Nav2 报失败时调
    # 参数：无
    # 返回：无
    def on_nav_failed(self):             # out 对外接口
        if self._state == 'GO_LOAD':     # # 只有导航中才接受结果
            self._state = 'ERROR'

    # 取消（回到 IDLE，清目标）
    # 参数：无
    # 返回：无
    def cancel(self):                    # out 对外接口
        self._state = 'IDLE'
        self._target = ''
        self._pose = None

    # 任务完成后归位（DONE → IDLE，供状态机接下一单）
    # 参数：无
    # 返回：无
    def reset(self):                     # out 对外接口
        self._state = 'IDLE'
        self._target = ''
        self._pose = None
