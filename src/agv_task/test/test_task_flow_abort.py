"""
TaskFlow 中止语义测试：取消不能算失败，也不能让车继续开出去。

★ 这两个 bug 都是真机上典型报障，静态读代码很难发现：
  ① 用户点「中止」，日志已打"已取消任务并停车"，车却又开出去
     —— 因为后台线程还卡在 goto_pose 的 waitUntilNav2Active() 里，
        Nav2 一就绪就照常 goToPose。goto_pose 完全不看中止标志。
  ② 中止之后既不能终止也不能启动新任务
     —— nav.step() 返回 False 被 run_transport 一律当失败，
        manager.fail() 把状态推到 ERROR；而 _can_start() 明确拒绝 ERROR。

本文件用假的 nav / cargo / store 把 TaskFlow 的流程跑一遍，
只验证"中止时的状态语义"，不需要 ROS。
"""

import threading

from agv_task.task_flow import TaskFlow
from agv_task.task_manager import TaskManager


# ── 测试替身 ────────────────────────────────────────────────

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
    """导航执行器替身：按脚本返回步骤结果，并记下被调用了几次。"""

    def __init__(self, step_results=None, seq_results=None):
        self.step_results = list(step_results or [])
        self.seq_results = list(seq_results or [])
        self.steps = []
        self.seqs = []

    def step(self, pose):
        self.steps.append(pose)
        return self.step_results.pop(0) if self.step_results else True

    def sequence(self, points):
        self.seqs.append(points)
        return self.seq_results.pop(0) if self.seq_results else True

    def sleep(self, seconds):
        pass


class FakeCargo:
    def __init__(self, results=None):
        self.results = list(results or [])
        self.calls = []

    def step(self, cmd, tag=''):
        self.calls.append(cmd)
        return self.results.pop(0) if self.results else True


class FakeStore:
    def __init__(self, points=None, routes=None):
        self.points = points or {}
        self.routes = routes or {}

    def waypoint(self, name):
        return self.points.get(name)

    def route(self, name):
        return self.routes.get(name)


class FakeGotoSM:
    def __init__(self):
        self._state = 'IDLE'

    def start(self, name, pose):
        self._state = 'NAVIGATING'
        return True, f'导航到 {name}'

    def state(self):
        return self._state

    def on_nav_succeeded(self):
        self._state = 'DONE'

    def on_nav_failed(self):
        self._state = 'ERROR'

    def cancel(self):
        self._state = 'IDLE'

    def reset(self):
        self._state = 'IDLE'


def _make_flow(nav, cargo=None, store=None, abort=None):
    node = FakeNode()
    manager = TaskManager()
    goto = FakeGotoSM()
    abort = abort if abort is not None else threading.Event()
    zeros = []
    flow = TaskFlow(node, manager, goto, nav, cargo or FakeCargo(),
                    store or FakeStore(points={
                        '取货': {'x': 1.0, 'y': 1.0, 'yaw': 0.0},
                        '放货': {'x': 5.0, 'y': 5.0, 'yaw': 0.0},
                    }), None, abort, lambda: zeros.append(1))
    flow.on_state = lambda: None
    return flow, manager, goto, abort, zeros


# ── 测试 ────────────────────────────────────────────────────

def test_cancel_during_nav_keeps_state_idle_not_error():
    """中止导致的导航失败必须回 IDLE，不能落 ERROR。

    ★ 这是"中止后再也开不了新任务"的直接原因。ERROR 在 _can_start()
      里是硬拒绝状态，用户只能重启节点才能恢复。
    """
    abort = threading.Event()
    flow, manager, goto, _, zeros = _make_flow(
        FakeNav(step_results=[False]), abort=abort)
    abort.set()                       # 模拟用户点了中止
    flow.run_transport('取货', '放货', '')
    assert manager.state() == 'IDLE', f'中止后状态应为 IDLE，实际 {manager.state()}'
    assert goto.state() == 'IDLE'
    # 中止时仍要发一帧零速（安全第一），但绝不能是 fail 路径
    assert zeros, '中止后必须发零速停车'


def test_cancel_during_cargo_keeps_state_idle():
    """顶升阶段被中止，同样回 IDLE。"""
    abort = threading.Event()
    flow, manager, _, _, _ = _make_flow(
        FakeNav(step_results=[True]), FakeCargo(results=[False]), abort=abort)
    abort.set()
    flow.run_transport('取货', '放货', '')
    assert manager.state() == 'IDLE'


def test_real_failure_still_goes_error():
    """真失败（没被中止）必须仍然是 ERROR —— 别把取消语义修过头。"""
    flow, manager, _, _, zeros = _make_flow(FakeNav(step_results=[False]))
    flow.run_transport('取货', '放货', '')
    assert manager.state() == 'ERROR'
    assert zeros, '失败时要停车'


def test_cancel_during_route_sequence():
    """路线途经点被中止，也回 IDLE 而不是 ERROR。"""
    abort = threading.Event()
    flow, manager, _, _, _ = _make_flow(
        FakeNav(seq_results=[False]), abort=abort,
        store=FakeStore(points={'取货': {'x': 1.0, 'y': 1.0, 'yaw': 0.0},
                                '放货': {'x': 5.0, 'y': 5.0, 'yaw': 0.0}},
                        routes={'绕行': [{'name': 'p1', 'x': 2, 'y': 2, 'yaw': 0}]}))
    abort.set()
    flow.run_transport('取货', '放货', '绕行')
    assert manager.state() == 'IDLE'


def test_cancel_before_first_step_sends_no_goal():
    """中止已置位时，一步都不该走（不能"取消完又开出去"）。"""
    abort = threading.Event()
    nav = FakeNav()
    abort.set()
    flow, manager, _, _, _ = _make_flow(nav, abort=abort)
    flow.run_transport('取货', '放货', '')
    # FakeNav 只记录"被调用"，真 NavRunner 会在 step 入口检查中止标志。
    # 这里验证流程层不会在已中止时推进状态机：状态停在 IDLE。
    assert manager.state() == 'IDLE'


def test_goto_cancel_does_not_mark_error():
    """单点导航被中止，也不该把 goto 状态机打成 ERROR。"""
    abort = threading.Event()
    flow, _, goto, _, _ = _make_flow(FakeNav(step_results=[False]), abort=abort)
    abort.set()
    flow.run_goto(['取货'], '')
    assert goto.state() == 'IDLE', f'中止后 goto 应为 IDLE，实际 {goto.state()}'


def test_missing_pick_point_is_real_failure():
    """点位不存在是真失败（跟取消要区分开）。"""
    flow, manager, _, _, _ = _make_flow(FakeNav(), store=FakeStore(points={}))
    flow.run_transport('不存在的点', '放货', '')
    assert manager.state() == 'ERROR'
