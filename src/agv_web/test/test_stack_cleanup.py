#!/usr/bin/env python3
"""换栈"彻底清理"的回归测试（停旧栈 → 清残留 SHM → 等端口 → 才起新栈）。

★ 对应真机上报的现象（用户原话）：

    "导航切换到建图模式地图黑屏怎么回事，节点也不启动"
    "自检说 slam_toolbox / robot_state_publisher 未发现，/scan /map 都没收到"
    "导航与建图切换要完全杀死或清理之前的遗留再重新启动，不然老是有这种 bug"

  查下来不是建图代码的问题，是**换栈时旧的没收拾干净**：

    ① `proc.wait()` 只等到 `ros2 launch` 父进程，gzserver 是它的兄弟进程，
       还要好几秒才释放 11345。新 gzserver bind 失败是**静默**的 →
       日志照打、世界里没有数据 → "地图一片黑"。

    ② 被 Ctrl-C 打断的节点死在 destroy_node() 里（日志里那串
       KeyboardInterrupt traceback），FastDDS 的 /dev/shm 段来不及 unlink。
       下一个栈的参与者撞上残留：`Failed init_port fastrtps_port74xx`
       → 同机优先走共享内存，SHM 一废就谁也发现不到谁 →
       网页"节点未发现 / 收不到 /scan"，而终端里节点其实都活着。

  本文件用 AST 把纯逻辑函数抽出来真跑（不 import rclpy），
  再加两条 AST 契约，防止以后有人把这几步又从 stop()/start() 里删掉。
"""

import ast
import os
import socket
import sys
import tempfile
import time
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(_HERE, '..')
sys.path.insert(0, _SRC)

_GW_PATH = os.path.join(_SRC, 'agv_web', 'web_gateway.py')


def _src():
    with open(_GW_PATH, encoding='utf-8') as f:
        return f.read()


def _load_funcs(names):
    """把 web_gateway 里的**模块级**函数抽出来放进一个命名空间执行。

    ★ 不直接 import：web_gateway 会 import rclpy，单测环境不一定 source 过 ROS。
      这几个函数只用到 os / time / socket，抽出来就是普通 Python。
    """
    src = _src()
    tree = ast.parse(src)
    segs = []
    for n in tree.body:                      # 只认模块级函数，不碰类方法
        if isinstance(n, ast.FunctionDef) and n.name in names:
            segs.append(ast.get_source_segment(src, n))
    assert len(segs) == len(names), \
        f'少抽到函数：想要 {sorted(names)}，实际 {len(segs)} 个'
    code = '\n\n'.join(segs)
    ns = {'os': os, 'time': time, 'socket': socket,
          'GAZEBO_MASTER_PORT': 11345,
          '_SHM_STALE_PREFIXES': ('fastrtps_', 'sem.fastrtps_')}
    exec(compile(code, _GW_PATH, 'exec'), ns)
    return ns


_NS = _load_funcs(['_group_members', 'wait_group_dead', '_mapped_shm_names',
                   'purge_stale_shm', 'wait_port_free'])


class _Silent:
    """日志回调替身：把消息收起来，断言时看有没有说该说的话。"""

    def __init__(self):
        self.lines = []

    def __call__(self, msg):
        self.lines.append(msg)


class TestGroupWait(unittest.TestCase):
    """② 等整组死透 —— 不能只看 launch 父进程。"""

    def test_bogus_group_has_no_members(self):
        self.assertEqual(_NS['_group_members'](999999), [])

    def test_wait_returns_true_for_dead_group(self):
        self.assertTrue(_NS['wait_group_dead'](999999, timeout=0.3))

    def test_wait_returns_true_when_no_pgid(self):
        # # 没有 pgid 时不能傻等 6 秒（proc 没起来时就是这种情况）
        t0 = time.time()
        self.assertTrue(_NS['wait_group_dead'](None, timeout=6.0))
        self.assertLess(time.time() - t0, 0.5)

    def test_wait_returns_false_and_reports_when_stuck(self):
        """自己起一个进程组，别让它退 —— 必须超时返回 False 并报 PID。"""
        import subprocess
        proc = subprocess.Popen(['sleep', '5'], start_new_session=True)
        try:
            say = _Silent()
            t0 = time.time()
            ok = _NS['wait_group_dead'](proc.pid, timeout=0.6, log=say)
            self.assertFalse(ok)
            self.assertLess(time.time() - t0, 3.0)
            self.assertTrue(any('没退' in s for s in say.lines), say.lines)
            self.assertIn(proc.pid, _NS['_group_members'](proc.pid))
        finally:
            proc.kill()
            proc.wait()


class TestPurgeStaleShm(unittest.TestCase):
    """④ 只删"无主的 FastDDS 段"，别的碰都不许碰。"""

    def _mk(self, d, name, age=60.0):
        path = os.path.join(d, name)
        with open(path, 'wb') as f:
            f.write(b'x' * 16)
        old = time.time() - age
        os.utime(path, (old, old))
        return path

    def test_only_unmapped_stale_fastrtps_removed(self):
        with tempfile.TemporaryDirectory() as d:
            mapped = self._mk(d, 'fastrtps_port7411_alive')    # 有主 → 留
            stale = self._mk(d, 'sem.fastrtps_port7411_dead')  # 无主 → 删
            other = self._mk(d, 'my_own_shm_segment')          # 不匹配 → 留
            purge = self._with_mapped({os.path.basename(mapped)})
            say = _Silent()
            n = purge(d, log=say)
            self.assertEqual(n, 1)
            self.assertTrue(os.path.exists(mapped), '有主的段被删了！会直接搞崩在用它的进程')
            self.assertFalse(os.path.exists(stale))
            self.assertTrue(os.path.exists(other), '不属于 FastDDS 的段不许碰')
            self.assertTrue(any('清理残留共享内存' in s for s in say.lines))

    def test_fresh_files_are_kept(self):
        """刚生成的（< grace）不删：可能是正在启动的参与者刚建的。"""
        with tempfile.TemporaryDirectory() as d:
            fresh = self._mk(d, 'fastrtps_port7411_new', age=0.0)
            purge = self._with_mapped(set())
            self.assertEqual(purge(d, log=_Silent()), 0)
            self.assertTrue(os.path.exists(fresh))

    def test_missing_dir_is_not_fatal(self):
        purge = self._with_mapped(set())
        self.assertEqual(purge('/dev/shm_不存在', log=_Silent()), 0)

    def _with_mapped(self, names):
        """临时替换 _mapped_shm_names。

        ★ 注意：函数对象的 __globals__ 就是 _NS 本身，`dict(_NS)` 复制一份
          再改**没用** —— 里面调的还是原来的函数。必须改原命名空间，
          跑完再还原。
        """
        old = _NS['_mapped_shm_names']
        _NS['_mapped_shm_names'] = lambda: set(names)
        purge = _NS['purge_stale_shm']

        def run(*a, **kw):
            try:
                return purge(*a, **kw)
            finally:
                _NS['_mapped_shm_names'] = old
        return run

    def test_real_mapped_names_helper_does_not_crash(self):
        ref = _NS['_mapped_shm_names']()
        self.assertIsInstance(ref, set)


class TestWaitPortFree(unittest.TestCase):
    """⑤ gzserver 端口：bind 失败是静默的，必须先等它空出来。"""

    def test_free_port_returns_immediately(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind(('', 0))
        port = sock.getsockname()[1]
        sock.close()
        t0 = time.time()
        self.assertTrue(_NS['wait_port_free'](port, timeout=2.0))
        self.assertLess(time.time() - t0, 0.5)

    def test_busy_port_times_out_with_warning(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('', 0))
        srv.listen(1)
        port = srv.getsockname()[1]
        try:
            say = _Silent()
            t0 = time.time()
            ok = _NS['wait_port_free'](port, timeout=0.5, log=say)
            self.assertFalse(ok)
            self.assertLess(time.time() - t0, 3.0)
            self.assertTrue(any('仍被占用' in s for s in say.lines), say.lines)
            self.assertTrue(any('静默' in s for s in say.lines),
                            '必须提醒 bind 失败不报错，否则下次还是找不到原因')
        finally:
            srv.close()


class TestStopStartContract(unittest.TestCase):
    """AST 契约：这几步不能从 stop()/start() 里被悄悄删掉。"""

    def _method_src(self, cls, name):
        src = _src()
        tree = ast.parse(src)
        for n in ast.walk(tree):
            if isinstance(n, ast.ClassDef) and n.name == cls:
                for m in n.body:
                    if isinstance(m, ast.FunctionDef) and m.name == name:
                        return ast.get_source_segment(src, m)
        self.fail(f'{cls}.{name} 不见了')

    def test_stop_waits_group_then_purges_then_port(self):
        s = self._method_src('StackRunner', 'stop')
        # ★ 用带参数的调用串定位，别用裸函数名：stop() 开头那条
        #   "本来就没在跑" 的早退分支里也有一句 purge_stale_shm。
        i_wait = s.find('wait_group_dead(pgid')
        i_purge = s.rfind('purge_stale_shm(log=say)')
        i_port = s.find('wait_port_free(log=say)')
        self.assertGreater(i_wait, -1, 'stop() 必须等整组死透')
        self.assertGreater(i_purge, -1, 'stop() 必须清残留 SHM')
        self.assertGreater(i_port, -1, 'stop() 必须等 Gazebo 端口')
        self.assertLess(i_wait, i_purge, '顺序：先等进程死，再清 SHM')
        self.assertLess(i_purge, i_port, '顺序：先清 SHM，再等端口')
        self.assertIn('os.killpg', s, '等不动时要能升级成 SIGKILL')

    def test_start_cleans_before_popen(self):
        s = self._method_src('StackRunner', 'start')
        i_purge = s.find('purge_stale_shm')
        i_port = s.find('wait_port_free')
        i_popen = s.find('subprocess.Popen')
        self.assertGreater(i_purge, -1, 'start() 起之前也要兜一层清理')
        self.assertLess(i_purge, i_popen, '清理必须在 Popen 之前')
        self.assertLess(i_port, i_popen, '等端口必须在 Popen 之前')

    def test_switch_is_serialized(self):
        s = self._method_src('StackRunner', 'switch')
        self.assertIn('_op_lock', s, '换栈必须整段串行，不能被连点插进来')

    def test_stop_is_serialized_but_does_not_block_state(self):
        """停栈要等十几秒：要串行（_op_lock），但不能攥着 _gw_lock 不放。

        _gw_lock 是 /api/state 每秒都在抢的锁；如果整段等待都写在
        `with self._gw_lock:` 里面，网页在切换期间会整片卡住（转圈）。
        这里只做结构检查：stop() 用了 _op_lock，且 _gw_lock 块在
        “发信号/等待”之前就已经结束（靠缩进判断，简单但不脆弱）。
        """
        s = self._method_src('StackRunner', 'stop')
        self.assertIn('self._op_lock', s, '停栈必须串行，不能被连点插进来')
        lines = s.splitlines()
        # 找出 `with self._gw_lock:` 那一行的缩进，块内不许出现 proc.wait
        gw_indent = None
        for ln in lines:
            if 'with self._gw_lock' in ln:
                gw_indent = len(ln) - len(ln.lstrip())
                continue
            if gw_indent is None:
                continue
            if not ln.strip():
                continue
            indent = len(ln) - len(ln.lstrip())
            if indent <= gw_indent:          # # 已经退出这个 with 块
                gw_indent = None
                continue
            self.assertNotIn('proc.wait', ln,
                             '长时间的等待不能写在 _gw_lock 里（会堵住状态接口）')


if __name__ == '__main__':
    unittest.main(verbosity=2)
