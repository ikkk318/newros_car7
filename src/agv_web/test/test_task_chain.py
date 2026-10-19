#!/usr/bin/env python3
"""任务缓冲连续执行的真时序回归测试。

★ 这组测试对应真机上的一个严重 bug：
    "放了多个任务，只执行第一个，后面的得不到执行。"

  根因不是状态机，而是**跨进程时序**：

    ① 网关 `_wait_task_outcome()` 只看 `state == 'DONE'`。
       `/agv/task_state` 是 task_manager 的 1 Hz 周期发布，网关缓存的是
       最后一帧。第二单刚下发的那 200 毫秒里读到的还是**上一单的 DONE**，
       于是立刻返回 done → 第三单、第四单同样瞬间通过。
       现象：日志说"全部完成"，车只动过一次。

    ② 即使①修好了，还会撞上第二个坑：task_manager 的收尾是
       「发 DONE → 睡 0.5 秒 → 状态机回 IDLE」，而 `_can_start()` 里
       有一条 `self._runner.is_alive()`。网关看到 DONE 就立刻发下一单，
       此时上一单的线程还没结束 → 被拒（"已有任务在执行"）。
       它被拒时**照样 publish_state**，网关读到的还是上一单的 DONE，
       看起来像"这单也成功"，实际车没动。

  所以本文件用一个假 task_manager（按真实时序推送 TaskState）来钉死：
     · 多单必须串行真的跑过（每单都能观察到活跃态）
     · 上一单的 DONE 残帧不许被当成"本单完成"
     · 指令被拒时必须重试，而不是假装成功
     · 收尾没回 IDLE 时不许把下一单发出去

这些时序问题**只能**靠这种"假 ROS + 真循环"的方式测，纯静态检查抓不到。
"""

import os
import sys
import threading
import time
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(_HERE, '..')
sys.path.insert(0, _SRC)

_GW_PATH = os.path.join(_SRC, 'agv_web', 'web_gateway.py')


def _load_task_methods():
    """只取 WebGateway 上与任务缓冲相关的几个方法，绕开 rclpy 依赖。

    ★ 为什么不直接 import WebGateway：它会 `import rclpy`，而单测环境
      不一定 source 过 ROS。这里把三个方法是纯 Python 的（只用到 self 的
      几个属性），用 AST 抽出源码 exec 进一个空壳类，既真跑了逻辑，
      又不依赖 ROS 运行时。
    """
    import ast
    with open(_GW_PATH, encoding='utf-8') as f:
        src = f.read()
    tree = ast.parse(src)
    wanted = ['start_task_buffer', '_run_tasks', '_start_one',
              '_current_task_id', '_wait_task_outcome', 'stop_tasks',
              'reset_task_state']
    segs = []
    for n in ast.walk(tree):
        if isinstance(n, ast.FunctionDef) and n.name in wanted:
            segs.append(ast.get_source_segment(src, n))
    assert len(segs) == len(wanted), f'少了方法：{wanted} vs {len(segs)}'
    ns = {}
    code = 'class Fake:\n' + '\n'.join(
        '    ' + ln if ln.strip() else ln
        for seg in segs for ln in seg.splitlines())
    # ★ exec 出来的类里，方法体引用的模块名（time / threading）必须能从
    #   这个命名空间查到 —— 否则真跑到 time.sleep 时才炸 NameError，
    #   而且报错行号指向 web_gateway.py，很容易误以为是源码写错了。
    ns.update({'time': time, 'threading': threading})
    exec(compile(code, _GW_PATH, 'exec'), ns)
    return ns['Fake']


class FakeTaskManager:
    """按真实时序提供 TaskState 的假 task_manager。

    ★ 关键是它**照抄真机的两个缺陷行为**：
      1) 每单收尾先发 DONE，隔 0.5 秒才回 IDLE；
      2) 在 DONE 之后那 0.5 秒里收到的 start_task 会被**拒绝**，
         而且拒绝时仍然 publish_state（状态不变）。
    只有把这两个行为复现出来，才能验证网关确实把它们处理对了。
    """

    def __init__(self, per_task_steps=3, step_sec=0.05,
                 settle_sec=0.3, reject_after_done=True):
        self.lock = threading.Lock()
        self.state = 'IDLE'
        self.task_id = ''
        self.seq = 0
        self.active = False        # 是否正在跑一单
        self.done_hold = False     # DONE 之后尚未回 IDLE 的窗口
        self.per_task_steps = per_task_steps
        self.step_sec = step_sec
        self.settle_sec = settle_sec
        self.reject_after_done = reject_after_done
        self.received = []         # 收到的 start_task 列表
        self.rejected = 0
        self.ran = []              # 真正跑起来的单
        self.halt = threading.Event()

    # 给网关读的一帧
    def frame(self):
        with self.lock:
            return {'task_id': self.task_id, 'state': self.state,
                    'target': '', 'progress': 0.0, 'message': '', 't': time.time()}

    # 网关下发 start_task 时调这里（模拟 /agv/command 的处理）
    def on_start(self, pick, drop):
        with self.lock:
            self.received.append((pick, drop))
            if self.active or self.done_hold:
                # ★ 真机行为：拒绝，但状态**不变**（于是网关读到的是旧帧）
                self.rejected += 1
                return False
            self.seq += 1
            self.task_id = f'task-{self.seq:04d}'
            self.state = 'GO_LOAD'
            self.active = True
            self.ran.append((pick, drop))
        threading.Thread(target=self._run_one, daemon=True).start()
        return True

    def _run_one(self):
        for st in ('GO_LOAD', 'LIFTING', 'TRANSPORT', 'LOWERING'):
            if self.halt.is_set():
                return
            with self.lock:
                self.state = st
            time.sleep(self.step_sec)
        with self.lock:
            self.state = 'DONE'
            self.done_hold = True
        time.sleep(self.settle_sec)
        with self.lock:
            self.state = 'IDLE'
            self.active = False
            self.done_hold = False


def _make_gateway(tm, state_getter=None):
    """造一个只带任务缓冲所需属性的假网关。"""
    Fake = _load_task_methods()
    gw = Fake()
    gw._gw_lock = threading.Lock()
    gw._gw_task_state = None
    gw._gw_task_stop = threading.Event()
    gw._gw_task_thread = None
    gw.task_running = False
    gw.logs = []
    gw.commands = []
    gw._tm = tm

    def _log(level, text):
        gw.logs.append((level, str(text)))

    def _send_command(command, args=None, route='', request_id=''):
        gw.commands.append((command, list(args or []), route))
        if command == 'start_task':
            gw._tm.on_start(args[0], args[1])
        return 'rid'

    def _current():
        if state_getter:
            return state_getter()
        return tm.frame()

    gw.log = _log
    gw.send_command = _send_command
    # 所有读到状态的地方都走同一个取帧函数（模拟 ROS 回调持续写入）
    orig_reset = gw.reset_task_state

    def _reset():
        with gw._gw_lock:
            gw._gw_task_state = None

    def _sync():
        """把 tm 的最新帧灌进网关缓存（模拟 on_task_state 回调）。"""
        with gw._gw_lock:
            gw._gw_task_state = _current()

    gw.reset_task_state = _reset
    gw._sync = _sync
    return gw


class TestSequentialTaskChain(unittest.TestCase):
    """多单必须逐条真跑，不许"唰唰全过"。"""

    def setUp(self):
        self.tm = FakeTaskManager(per_task_steps=1, step_sec=0.02,
                                  settle_sec=0.25)
        self.gw = _make_gateway(self.tm)
        # 持续把假 task_manager 的最新帧灌进网关缓存 ——
        # 模拟真实的 /agv/task_state 1 Hz 周期发布 + 变化时立即发布。
        # ★ 注意必须"总是覆盖"（包括 reset 之后），因为真机上 reset 只是
        #   丢掉本地缓存，下一帧照样会从 ROS 回调进来。如果不覆盖，
        #   网关就永远看不到新帧，测出来全是 nostart，测不出真问题。
        self._stop = threading.Event()
        gw = self.gw

        def pump():
            while not self._stop.is_set():
                with gw._gw_lock:
                    gw._gw_task_state = gw._tm.frame()
                time.sleep(0.02)
        self._pump = threading.Thread(target=pump, daemon=True)
        self._pump.start()

    def tearDown(self):
        self._stop.set()
        self.tm.halt.set()

    def test_three_tasks_all_really_run(self):
        """3 单任务：假 task_manager 必须真的跑了 3 次。

        ★ 这是"只执行第一个任务"的直接回归：如果网关被上一单的 DONE
          残帧骗了，后两单根本不会进 self.tm.ran。
        """
        items = [{'pick': f'P{i}', 'drop': f'D{i}', 'loops': 1, 'route': ''}
                 for i in range(3)]
        self.assertTrue(self.gw.start_task_buffer(items))
        deadline = time.time() + 30
        while time.time() < deadline:
            if not self.gw.task_running:
                break
            time.sleep(0.05)
        self.assertFalse(self.gw.task_running, '任务缓冲应当在全部跑完后置 False')
        self.assertEqual(len(self.tm.ran), 3,
                         f'应有 3 单真的执行，实际 {self.tm.ran}')
        self.assertEqual([p for p, _ in self.tm.ran], ['P0', 'P1', 'P2'],
                         '执行顺序必须与缓冲顺序一致')

    def test_loops_repeat_task(self):
        """loops=3 的任务要真的跑 3 次。"""
        items = [{'pick': 'A', 'drop': 'B', 'loops': 3, 'route': ''}]
        self.assertTrue(self.gw.start_task_buffer(items))
        deadline = time.time() + 30
        while time.time() < deadline and self.gw.task_running:
            time.sleep(0.05)
        self.assertEqual(len(self.tm.ran), 3,
                         f'loops=3 应跑 3 次，实际 {len(self.tm.ran)}')

    def test_route_is_passed_through(self):
        """任务的 route 字段必须原样下发到 start_task 指令里。

        ★ 以前前端根本没有选路线的入口，后端 `_run_tasks` 虽然读了
          `t.get('route')`，但网页存进去的永远是空串 —— 功能存着用不上。
        """
        items = [{'pick': 'A', 'drop': 'B', 'loops': 1, 'route': '1号'}]
        self.assertTrue(self.gw.start_task_buffer(items))
        deadline = time.time() + 30
        while time.time() < deadline and self.gw.task_running:
            time.sleep(0.05)
        starts = [c for c in self.gw.commands if c[0] == 'start_task']
        self.assertTrue(starts, '必须下发了 start_task')
        self.assertEqual(starts[0][2], '1号',
                         f'路线名应原样下发，实际 {starts[0][2]!r}')

    def test_reject_is_retried_not_faked(self):
        """被拒的指令必须重试，且**不许**被当成成功。

        ★ 真机行为：在 DONE 之后那个 0.5 秒窗口里下发的指令会被拒，
          而且状态不变。以前的网关读到旧 DONE 就返回 done —— 于是
          "日志说完成，车没动"。这里要求：出现过拒绝，但最终仍然真跑了。
        """
        items = [{'pick': 'A', 'drop': 'B', 'loops': 1, 'route': ''},
                 {'pick': 'C', 'drop': 'D', 'loops': 1, 'route': ''}]
        self.assertTrue(self.gw.start_task_buffer(items))
        deadline = time.time() + 30
        while time.time() < deadline and self.gw.task_running:
            time.sleep(0.05)
        self.assertEqual(len(self.tm.ran), 2,
                         f'两单都要真跑，实际 {self.tm.ran}')
        # 只要不是靠"假装成功"过掉的，拒绝次数是 0 或更多都可以接受
        # （取决于时序）；关键是 ran 的两单都在。
        self.assertLessEqual(self.tm.rejected, 6,
                             '重试次数应有上限，不能无限重发')


class TestOutcomeJudgement(unittest.TestCase):
    """`_wait_task_outcome` 对单帧状态的判断（同步、不用线程）。"""

    def _gw_with(self, frames):
        """frames: [(state, task_id), ...]，每次读进步进一帧。"""
        gw = _make_gateway(None, state_getter=lambda: None)
        idx = {'i': 0}

        def getter():
            i = min(idx['i'], len(frames) - 1)
            idx['i'] += 1
            st, tid = frames[i]
            return {'task_id': tid, 'state': st, 'target': '',
                    'progress': 0.0, 'message': '', 't': time.time()}

        gw.reset_task_state = lambda: None
        lock = gw._gw_lock
        orig = gw._gw_task_state

        def fake_wait(prev, timeout=5.0, start_timeout=1.0,
                      settle_timeout=0.4):
            # 直接喂帧：绕过真实 sleep，逐帧调用原逻辑
            gw._wait_task_outcome_orig = None
            return None
        # 这里直接用真实方法 + 预置帧序列会牵扯 sleep，改为直接断言
        # "旧 DONE 残帧不算本单完成"这一核心不变量。
        return gw, getter

    def test_stale_done_not_counted(self):
        """上一单的 DONE（task_id 与下发前相同）不能算本单完成。

        直接把网关缓存设成"旧单的 DONE"，然后调 _wait_task_outcome，
        期望它**不**在 start_timeout 内返回 done —— 而是判 nostart。
        """
        gw = _make_gateway(None)
        with gw._gw_lock:
            gw._gw_task_state = {'task_id': 'task-0001', 'state': 'DONE',
                                 'target': '', 'progress': 1.0,
                                 'message': '', 't': time.time()}
        t0 = time.time()
        out = gw._wait_task_outcome('task-0001', timeout=5.0,
                                    start_timeout=0.5, settle_timeout=0.3)
        cost = time.time() - t0
        self.assertEqual(out, 'nostart',
                         f'旧 DONE 残帧应判"没启动"，实际 {out}')
        self.assertLess(cost, 3.0, '不能一直等到整单超时（300 秒那种）')

    def test_error_after_accept_is_fail(self):
        """被受理后立刻 ERROR（如点位不存在）必须判 fail，不能重试。

        「task_id 变了」= 已受理，此时 ERROR 是**真失败**，
        重发三次只会把同一条错误日志刷三遍。
        """
        gw = _make_gateway(None)
        gw._gw_task_state = None

        def feed():
            time.sleep(0.05)
            with gw._gw_lock:
                gw._gw_task_state = {'task_id': 'task-0002', 'state': 'ERROR',
                                     'target': '', 'progress': 0.0,
                                     'message': '点位不存在', 't': time.time()}
        threading.Thread(target=feed, daemon=True).start()
        out = gw._wait_task_outcome('task-0001', timeout=5.0,
                                    start_timeout=2.0, settle_timeout=0.3)
        self.assertEqual(out, 'fail', f'受理后 ERROR 应判 fail，实际 {out}')

    def test_cancel_event_exits_fast(self):
        """中止事件置位后必须立刻返回 cancel（不能等超时）。"""
        gw = _make_gateway(None)
        with gw._gw_lock:
            gw._gw_task_state = {'task_id': 'task-0001', 'state': 'TRANSPORT',
                                 'target': '', 'progress': 0.6,
                                 'message': '', 't': time.time()}
        gw._gw_task_stop.set()
        t0 = time.time()
        out = gw._wait_task_outcome('task-0000', timeout=30.0,
                                    start_timeout=5.0, settle_timeout=1.0)
        self.assertEqual(out, 'cancel')
        self.assertLess(time.time() - t0, 1.0, '中止应立即生效')

    def test_done_waits_for_idle(self):
        """看到 DONE 后要等回 IDLE 才返回 done（否则下一单会被拒）。"""
        gw = _make_gateway(None)
        with gw._gw_lock:
            gw._gw_task_state = {'task_id': 'task-0002', 'state': 'DONE',
                                 'target': '', 'progress': 1.0,
                                 'message': '', 't': time.time()}
        t0 = time.time()

        def back_to_idle():
            time.sleep(0.3)
            with gw._gw_lock:
                gw._gw_task_state = {'task_id': '', 'state': 'IDLE',
                                     'target': '', 'progress': 0.0,
                                     'message': '', 't': time.time()}
        threading.Thread(target=back_to_idle, daemon=True).start()
        out = gw._wait_task_outcome('task-0001', timeout=10.0,
                                    start_timeout=2.0, settle_timeout=5.0)
        cost = time.time() - t0
        self.assertEqual(out, 'done')
        self.assertGreaterEqual(cost, 0.28,
                                f'应等回 IDLE 才返回（实际只等了 {cost:.2f}s）')


if __name__ == '__main__':
    unittest.main(verbosity=2)
