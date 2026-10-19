"""
state_reporter —— 任务状态组装与发布。

把"当前该报什么状态"从节点里抽出来：它要同时看搬运状态机、单点导航状态机
和安全层（急停优先），三者的优先级关系是本模块唯一的逻辑，
原本埋在原节点的 publish_state() 里，与 ROS 发布混在一起不便单独看。

优先级：急停(PAUSED) > 搬运任务 > 单点导航 > 空闲(IDLE)
"""

from agv_task.task_manager import STATE_PROGRESS   # use 搬运状态 → (进度, 提示)

# 单点导航状态 → (进度, 中文)
GOTO_PROGRESS = {                        # def 导航状态表
    'GO_LOAD': (0.5, '导航中'),
    'DONE': (1.0, '导航完成'),
    'ERROR': (0.0, '导航失败'),
}


# 组装一条状态（纯函数，不碰 ROS，便于单测）
# 参数：manager - TaskManager；goto - GotoController；safety - SafetyMonitor
# 返回：dict {'task_id','state','target','progress','message'}
def compose(manager, goto, safety):      # out 对外接口
    mgr_state = manager.state()          # def 搬运态
    goto_state = goto.state()            # def 导航态

    if mgr_state != 'IDLE':              # # 有搬运任务：报它
        task_id = manager.task_id()
        target = manager.target_location()
        progress, message = manager.progress_info()
        state = mgr_state
    elif goto_state != 'IDLE':           # # 没有搬运任务，但单点导航有动静
        task_id = ''
        target = goto.target()
        progress, message = GOTO_PROGRESS.get(goto_state, (0.0, goto_state))
        state = goto_state
    else:                                # # 都空闲
        task_id, target, progress, message, state = \
            '', '', 0.0, '空闲，等待任务', 'IDLE'

    if safety.estopped():                # # 急停显示优先
        state, message, progress = 'PAUSED', '已急停（安全停止）', 0.0
    return {'task_id': task_id, 'state': state, 'target': target,
            'progress': float(progress), 'message': message}


# 搬运状态 → 进度与提示（供查询服务复用）
# 参数：state - 状态串
# 返回：(progress, message)
def progress_of(state):                  # out 对外接口
    return STATE_PROGRESS.get(state, (0.0, state))
