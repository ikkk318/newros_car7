"""
state_reporter 单元测试：状态组装优先级（急停 > 搬运 > 导航 > 空闲）。

原先这段优先级判断埋在原 task_manager_node 的 publish_state() 里，
只有起 ROS 才能验。抽成纯函数后可以逐条断言。
"""

from agv_task.goto_controller import GotoController
from agv_task.safety_monitor import SafetyMonitor
from agv_task.state_reporter import compose, progress_of
from agv_task.task_manager import TaskManager


def make():                              # # 造一套干净组件
    return TaskManager(), GotoController(), SafetyMonitor()


def test_all_idle():                     # # 都空闲
    m, g, s = make()
    info = compose(m, g, s)
    assert info['state'] == 'IDLE'
    assert info['task_id'] == '' and info['target'] == ''
    assert info['progress'] == 0.0


def test_transport_task_takes_priority():      # # 搬运任务优先于单点导航
    m, g, s = make()
    m.start('取货', '放货')
    g.start('某个点')
    info = compose(m, g, s)
    assert info['state'] == 'GO_LOAD'
    assert info['target'] == '取货'
    assert info['task_id'].startswith('task-')
    assert info['progress'] == 0.15


def test_goto_reported_when_no_task():   # # 没有搬运任务时报单点导航
    m, g, s = make()
    g.start('巡检点')
    info = compose(m, g, s)
    assert info['state'] == 'GO_LOAD'
    assert info['target'] == '巡检点'
    assert info['task_id'] == ''
    assert info['progress'] == 0.5


def test_goto_done_and_error():          # # 导航完成 / 失败
    m, g, s = make()
    g.start('A')
    g.on_nav_succeeded()
    assert compose(m, g, s)['state'] == 'DONE'
    g2 = GotoController()
    g2.start('A')
    g2.on_nav_failed()
    assert compose(m, g2, s)['state'] == 'ERROR'


def test_estop_overrides_everything():   # # ★ 急停显示优先于一切
    m, g, s = make()
    m.start('取货', '放货')
    g.start('巡检点')
    s.on_estop()
    info = compose(m, g, s)
    assert info['state'] == 'PAUSED'
    assert '急停' in info['message']
    assert info['progress'] == 0.0


def test_estop_cleared_returns_to_task():    # # 解除急停后回到真实状态
    m, g, s = make()
    m.start('取货', '放货')
    s.on_estop()
    assert compose(m, g, s)['state'] == 'PAUSED'
    s.clear_estop()
    assert compose(m, g, s)['state'] == 'GO_LOAD'


def test_transport_progress_advances():  # # 搬运过程进度随状态推进
    m, g, s = make()
    m.start('取货', '放货')
    seen = [compose(m, g, s)['progress']]
    for _ in range(3):                   # GO_LOAD → LIFTING → TRANSPORT → LOWERING
        m.step_success()
        seen.append(compose(m, g, s)['progress'])
    assert seen == [0.15, 0.40, 0.60, 0.85]


def test_progress_of_unknown_state():    # # 未知状态不崩，回落到原串
    assert progress_of('WHATEVER') == (0.0, 'WHATEVER')
