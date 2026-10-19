"""
TaskManager 单元测试：搬运主序列、失败/取消/急停/恢复、目标点名。
"""

from agv_task.task_manager import SEQUENCE, TaskManager


def test_sequence_has_no_docking_states():   # # 已去精定位：序列里不该有 ALIGN_*
    assert 'ALIGN_LOAD' not in SEQUENCE
    assert 'ALIGN_UNLOAD' not in SEQUENCE
    assert SEQUENCE == ('GO_LOAD', 'LIFTING', 'TRANSPORT', 'LOWERING', 'DONE')


def test_normal_flow():                  # # 正常一走到底，最后自动回 IDLE
    m = TaskManager()
    assert m.state() == 'IDLE'
    ok, reason = m.start('取货点', '放货点')
    assert ok is True
    assert m.state() == 'GO_LOAD'
    for expect in ('LIFTING', 'TRANSPORT', 'LOWERING', 'DONE'):
        m.step_success()
        assert m.state() == expect
    m.step_success()                     # # DONE → IDLE
    assert m.state() == 'IDLE'
    assert m.is_idle() is True


def test_target_location_by_phase():     # # 取货阶段报取货点，放货阶段报放货点
    m = TaskManager()
    m.start('A', 'B')
    assert m.target_location() == 'A'    # GO_LOAD
    m.step_success()
    assert m.target_location() == 'A'    # LIFTING
    m.step_success()
    assert m.target_location() == 'B'    # TRANSPORT
    m.step_success()
    assert m.target_location() == 'B'    # LOWERING
    m.step_success()
    assert m.target_location() == ''     # DONE


def test_start_rejected_when_busy():     # # 串行：执行中不能开新任务
    m = TaskManager()
    m.start('A', 'B')
    ok, reason = m.start('C', 'D')
    assert ok is False and '无法开始新任务' in reason


def test_start_needs_both_names():       # # 缺点名拒绝
    m = TaskManager()
    ok, reason = m.start('A', '')
    assert ok is False and '不能为空' in reason


def test_fail_then_reset():              # # 失败进 ERROR，reset 回 IDLE
    m = TaskManager()
    m.start('A', 'B')
    m.fail()
    assert m.state() == 'ERROR'
    m.reset()
    assert m.state() == 'IDLE'
    assert m.pick_location() == ''


def test_estop_pauses_then_reset_clears():   # # 急停进 PAUSED，人工复位回 IDLE（不续跑）
    m = TaskManager()
    m.start('A', 'B')
    m.step_success()                     # → LIFTING
    m.on_estop()
    assert m.state() == 'PAUSED'
    # ★ 急停已取消导航并中止线程，不允许"续跑暂停前那一格"；恢复=清零重配
    assert not hasattr(m, 'resume')
    m.reset()
    assert m.state() == 'IDLE'
    assert m.is_idle() is True
    assert m.pick_location() == '' and m.drop_location() == ''


def test_estop_when_idle_keeps_idle():   # # 空闲时急停不产生 PAUSED
    m = TaskManager()
    m.on_estop()
    assert m.state() == 'IDLE'


def test_cancel_clears():                # # 取消：回 IDLE 且清目标
    m = TaskManager()
    m.start('A', 'B')
    m.cancel()
    assert m.state() == 'IDLE'
    assert m.pick_location() == '' and m.drop_location() == ''


def test_step_success_in_error_noop():   # # ERROR 态不可推进
    m = TaskManager()
    m.start('A', 'B')
    m.fail()
    ok, reason, state = m.step_success()
    assert ok is False and state == 'ERROR'
