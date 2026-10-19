"""
槽内点位进槽策略测试（纯逻辑，不需要 ROS）。

★ 这些判据守着一个真实的连锁故障：
    凵型槽里车转不动 → Nav2 差 0.2 rad 永远不判到达 → 原地磨到超时 →
    顶升/下降不执行。所以：
      ① 槽内点必须先到槽口对位（_legs 拆两腿）；
      ② 槽内到点以位置为主，角度放宽到 0.35 rad，且要稳定 0.6 s；
      ③ 角度差大且长期无改善要快速判死，不能磨 140 秒。
"""

import threading

from agv_task.slot_approach import (
    SlotArrive, is_slot_point, mouth_name, wrap_angle)
from agv_task.task_flow import TaskFlow
from agv_task.task_manager import TaskManager


# ── 命名约定 ────────────────────────────────────────────────

def test_slot_suffix_detected():
    assert is_slot_point('1号槽内')
    assert is_slot_point('5号槽内')
    assert not is_slot_point('1号货架')
    assert not is_slot_point('起始点')
    assert not is_slot_point('')
    assert not is_slot_point(None)


def test_mouth_name_derived_from_slot():
    """「X号槽内」→「X号货架」，其余点位不推导。"""
    assert mouth_name('1号槽内') == '1号货架'
    assert mouth_name('12号槽内') == '12号货架'
    assert mouth_name('1号货架') is None
    assert mouth_name('槽内') is None          # 只有后缀、没有前缀：推不出
    assert mouth_name('') is None


def test_wrap_angle_handles_pi_boundary():
    """跨 ±π 的航向差必须归一化，否则 0.2 rad 会被算成 6.08 rad。"""
    assert abs(wrap_angle(0.2) - 0.2) < 1e-9
    diff = wrap_angle((3.0416) - (-3.0416))     # 目标 π-0.1，实际 -π+0.1
    assert abs(abs(diff) - 0.2) < 1e-3


# ── 到点判据 ────────────────────────────────────────────────

TARGET = (1.0, 2.0, 0.0)


def test_far_away_is_wait():
    w = SlotArrive()
    verdict, dist, dyaw = w.feed(0.0, 0.0, 0.0, TARGET, now=0.0)
    assert verdict == 'wait'
    assert dist > 0.12


def test_position_ok_but_yaw_too_far_is_wait():
    """位置到了、角度差 0.5 rad(28°)：不判到达（会顶偏货物）。"""
    w = SlotArrive()
    verdict, _, dyaw = w.feed(1.0, 2.0, 0.5, TARGET, now=0.0)
    assert verdict == 'wait'
    assert dyaw > 0.35


def test_arrive_requires_hold_time():
    """必须连续满足 0.6 s 才算到：防止 0.5 m/s 路过时误判。"""
    w = SlotArrive(hold=0.6)
    assert w.feed(1.0, 2.0, 0.10, TARGET, now=0.0)[0] == 'wait'
    assert w.feed(1.01, 2.0, 0.10, TARGET, now=0.3)[0] == 'wait'
    assert w.feed(1.01, 2.0, 0.10, TARGET, now=0.7)[0] == 'arrive'


def test_leaving_circle_resets_hold():
    """中途离开容差圈，计时必须清零（不能累计）。"""
    w = SlotArrive(hold=0.6)
    assert w.feed(1.0, 2.0, 0.0, TARGET, now=0.0)[0] == 'wait'
    assert w.feed(1.5, 2.0, 0.0, TARGET, now=0.4)[0] == 'wait'
    assert w.feed(1.0, 2.0, 0.0, TARGET, now=0.8)[0] == 'wait'
    assert w.feed(1.0, 2.0, 0.0, TARGET, now=1.5)[0] == 'arrive'


def test_stuck_when_yaw_huge_and_no_improvement():
    """位置到点但角度差 2.4 rad（实测 138°）且 8 秒无改善 → 快速判死。"""
    w = SlotArrive(stuck_sec=8.0)
    assert w.feed(1.0, 2.0, 2.4, TARGET, now=0.0)[0] == 'wait'
    assert w.feed(1.0, 2.0, 2.4, TARGET, now=3.0)[0] == 'wait'
    assert w.feed(1.0, 2.0, 2.4, TARGET, now=8.1)[0] == 'stuck'


def test_stuck_not_triggered_while_yaw_is_converging():
    """角度在收敛（每次改善 >0.05）时不能判死 —— 慢转也算在努力。"""
    w = SlotArrive(stuck_sec=8.0)
    w.feed(1.0, 2.0, 2.4, TARGET, now=0.0)
    w.feed(1.0, 2.0, 1.8, TARGET, now=5.0)      # 改善 0.6 rad
    assert w.feed(1.0, 2.0, 1.4, TARGET, now=10.0)[0] == 'wait'
    assert w.feed(1.0, 2.0, 1.0, TARGET, now=15.0)[0] == 'wait'


def test_small_yaw_error_never_reports_stuck():
    """角度差 0.4 rad 不触发 stuck（只比 arrive 容差大一点，值得再等等）。"""
    w = SlotArrive(stuck_sec=1.0)
    for t in (0.0, 2.0, 5.0):
        assert w.feed(1.0, 2.0, 0.4, TARGET, now=t)[0] == 'wait'


# ── TaskFlow：槽内点拆成"槽口对位 + 进槽"两条腿 ────────────────

class FakeLog:
    def __init__(self):
        self.lines = []

    def _rec(self, level):
        def fn(text):
            self.lines.append((level, str(text)))
        return fn

    def info(self, text):
        self._rec('info')(text)

    def warn(self, text):
        self._rec('warn')(text)

    def error(self, text):
        self._rec('error')(text)


class FakeNode:
    def __init__(self):
        self.log = FakeLog()

    def get_logger(self):
        return self.log


class FakeNav:
    def __init__(self):
        self.steps = []

    def step(self, pose):
        self.steps.append(dict(pose))
        return True

    def sequence(self, points):
        return True

    def sleep(self, seconds):
        pass


class FakeCargo:
    def step(self, cmd, tag=''):
        return True


class FakeStore:
    def __init__(self, points=None, routes=None):
        self.points = points or {}
        self.routes = routes or {}

    def waypoint(self, name):
        return self.points.get(name)

    def route(self, name):
        return self.routes.get(name)


class FakeGotoSM:
    def start(self, name, pose):
        return True, f'导航到 {name}'

    def on_nav_succeeded(self):
        pass

    def on_nav_failed(self):
        pass

    def cancel(self):
        pass


def _flow(points, slot_approach=True):
    flow = TaskFlow(FakeNode(), TaskManager(), FakeGotoSM(), FakeNav(),
                    FakeCargo(), FakeStore(points=points), None,
                    threading.Event(), lambda: None,
                    slot_approach=slot_approach, slot_soft={'yaw': 0.35})
    flow.on_state = lambda: None
    return flow


POINTS = {
    '1号槽内': {'x': 3.0, 'y': 2.692, 'yaw': 0.0},
    '1号货架': {'x': 1.63, 'y': 2.66, 'yaw': 0.0},
    '2号货架': {'x': -7.57, 'y': -7.41, 'yaw': -3.08},
}


def test_slot_pick_goes_via_mouth_first():
    """取货点是槽内 → 先到槽口对位，再进槽；只有进槽那条带 soft。"""
    flow = _flow(POINTS)
    flow.run_transport('1号槽内', '2号货架', '')
    names = [s['name'] for s in flow.nav.steps]
    assert names[0].startswith('1号货架'), names
    assert names[1] == '1号槽内', names
    assert 'soft' not in flow.nav.steps[0]
    assert flow.nav.steps[1].get('soft'), '槽内那条必须带软到点容差'


def test_slot_drop_goes_via_mouth_first():
    """放货点是槽内 → 同样先对位再进槽。"""
    flow = _flow(POINTS)
    flow.run_transport('2号货架', '1号槽内', '')
    names = [s['name'] for s in flow.nav.steps]
    assert names[-2].startswith('1号货架'), names
    assert names[-1] == '1号槽内', names


def test_normal_point_is_not_split():
    """普通点位（不符合命名约定）保持一条腿，不改既有行为。"""
    flow = _flow(POINTS)
    flow.run_transport('2号货架', '1号货架', '')
    assert len(flow.nav.steps) == 2
    assert all('soft' not in s for s in flow.nav.steps)


def test_missing_mouth_falls_back_to_direct_entry():
    """对位点被删/改名 → 直接进槽，但仍带软到点，不能整单报错。"""
    flow = _flow({'1号槽内': POINTS['1号槽内'], '2号货架': POINTS['2号货架']})
    flow.run_transport('1号槽内', '2号货架', '')
    entry = [s for s in flow.nav.steps if s['name'] == '1号槽内']
    assert len(entry) == 1
    assert entry[0].get('soft')


def test_switch_off_keeps_old_behaviour():
    """slot_approach=False 时不拆腿（留给现场快速回退）。"""
    flow = _flow(POINTS, slot_approach=False)
    flow.run_transport('1号槽内', '2号货架', '')
    names = [s['name'] for s in flow.nav.steps]
    assert names[0] == '1号槽内', names
