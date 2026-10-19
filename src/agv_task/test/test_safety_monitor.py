"""
SafetyMonitor 单元测试：急停达标、超时停车、手动互锁、任务清理。
对应规格书验收：急停响应 ≤200ms；隐患 3：任务中断后状态要清干净。
"""

from agv_task.safety_monitor import SafetyMonitor


def test_estop_sets_state():             # # 急停进入急停态
    s = SafetyMonitor()
    assert s.estopped() is False
    s.on_estop()
    assert s.estopped() is True
    assert s.allow_nav() is False        # # 急停时禁导航


def test_estop_timely_within_budget():   # # 急停停车 ≤200ms 达标
    s = SafetyMonitor(estop_budget_ms=200)
    s.on_estop()
    s.mark_stop_published(30)            # # 记录 30ms 发出停车
    assert s.stop_published_within_ms() == 30
    assert s.estop_timely() is True


def test_estop_over_budget_fails():      # # 超过 200ms 判不达标
    s = SafetyMonitor(estop_budget_ms=200)
    s.on_estop()
    s.mark_stop_published(350)
    assert s.estop_timely() is False


def test_cmd_timeout_stop():             # # 指令超时停车
    s = SafetyMonitor(cmd_timeout_ms=500)
    s.on_cmd_vel(0.3)                    # # 收到指令
    s.tick(0.3)                          # # 累计 0.3s < 0.5s
    assert s.stopped_due_to_timeout() is False
    s.tick(0.3)                          # # 累计 0.6s ≥ 0.5s
    assert s.stopped_due_to_timeout() is True


def test_new_cmd_clears_timeout():       # # 新指令解除超时
    s = SafetyMonitor(cmd_timeout_ms=500)
    s.tick(1.0)
    assert s.stopped_due_to_timeout() is True
    s.on_cmd_vel(0.5)                    # # 来了新指令
    assert s.stopped_due_to_timeout() is False


def test_manual_locks_nav():             # # 手动模式禁导航（D4 互锁）
    s = SafetyMonitor()
    assert s.allow_nav() is True
    s.set_manual(True)
    assert s.manual() is True
    assert s.allow_nav() is False
    s.set_manual(False)
    assert s.allow_nav() is True


def test_task_clean_after_estop_then_clear():   # # 急停后状态需清理（隐患 3）
    s = SafetyMonitor()
    s.on_estop()
    assert s.task_state_clean() is False     # # 急停后不干净
    s.clear_task()                           # # 外部清理任务状态
    assert s.task_state_clean() is True


def test_clear_estop():                  # # 人工复位
    s = SafetyMonitor()
    s.on_estop()
    s.clear_estop()
    assert s.estopped() is False
    assert s.stop_published_within_ms() == -1


# ══════════════════ 回归：自发零速的回声不得清掉自己的超时 ══════════════════

def test_self_published_zero_does_not_clear_timeout():   # # ★ 节点发停车帧后不得"自解锁"
    # 场景：上层停发速度 → 安全层判超时 → 节点补发零速到 /cmd_vel。
    # 节点自己也订阅 /cmd_vel，这帧零速会绕回来。若不冲销，
    # 它就把 _stopped_by_timeout 清成 False——自己把自己的超时解除了，
    # 超时保护失效（缺陷 3）。
    s = SafetyMonitor(cmd_timeout_ms=500)
    s.tick(1.0)                          # # 超时
    assert s.stopped_due_to_timeout() is True
    s.note_self_publish()                # # 声明"我即将发一帧"
    s.on_cmd_vel(0.0)                    # # 回声到达
    assert s.stopped_due_to_timeout() is True     # # 仍然停车


def test_genuine_cmd_still_clears_timeout():             # # 真上层指令仍能解除超时
    s = SafetyMonitor(cmd_timeout_ms=500)
    s.tick(1.0)
    assert s.stopped_due_to_timeout() is True
    s.on_cmd_vel(0.3)                    # # 没有记账 → 是真指令
    assert s.stopped_due_to_timeout() is False


def test_echo_accounting_is_one_shot():  # # 记账只冲销一帧，不误吞后续真指令
    s = SafetyMonitor(cmd_timeout_ms=500)
    s.note_self_publish()
    s.on_cmd_vel(0.0)                    # # 冲销掉
    s.tick(1.0)
    assert s.stopped_due_to_timeout() is True
    s.on_cmd_vel(0.3)                    # # 这一帧是真指令：应解除超时
    assert s.stopped_due_to_timeout() is False


def test_timeout_edge_fires_once():      # # 边沿触发：超时帧只报一次，不反复发
    s = SafetyMonitor(cmd_timeout_ms=500)
    s.tick(0.6)
    assert s.timeout_edge() is True      # # 刚跨过阈值
    s.tick(0.1)
    assert s.timeout_edge() is False     # # 仍在超时态，但不再报边沿
    s.on_cmd_vel(0.2)                    # # 上层恢复
    assert s.timeout_edge() is False
    s.tick(0.6)                          # # 再次跨过阈值
    assert s.timeout_edge() is True

