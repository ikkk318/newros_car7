#!/usr/bin/env python3
"""切模式/换栈时的"运行态作废"回归测试。

★ 对应真机上报的一串现象：

    "从导航模式与建图模式相互切换时有遗留，导致网页地图显示有重影"
    "切换之后小车的位置显示依旧保留之前切换前的最后显示，
     导致切换后小车的实际位置与显示的位置不符"
    "重新定位使用不了"

  三条其实是同一个根：**网关的缓存只被新数据覆盖，从不主动作废。**

    ① 地图：`_gw_map_png / _gw_map_meta` 一直留着上一套栈的图。新栈的
       /map 要等 slam_toolbox 或 map_server 激活（几秒到几十秒）。
       这段空窗期前端每一帧都收到旧 PNG + 可能是新几何 —— 屏幕上就是
       两张图错位叠着（重影）。

    ② 位姿：`_gw_pose` 同样是"最后一帧有效值"。新栈还没定位时 TF 查不到，
       `refresh_pose()` 里 `if pose is not None: 覆盖`，查不到就什么都不做，
       于是旧位姿被无限期沿用 —— 小车图标稳稳停在切换前的位置上，
       用户看到的是"定位正常"，这比不显示危险得多。

    ③ 重新定位：`set_initial_pose()` 是哑发，publish 完就回 ok=True。
       建图模式没有 AMCL、导航栈没起时也没有订阅者，消息发出去石沉大海，
       界面照样显示"已重新定位"。用户点几次都没反应，自然认为按钮坏了。

  本文件用 AST 抽出这几个纯逻辑方法（不 import rclpy），逐个钉死。
"""

import ast
import math
import os
import sys
import threading
import time
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(_HERE, '..')
sys.path.insert(0, _SRC)

_GW_PATH = os.path.join(_SRC, 'agv_web', 'web_gateway.py')


def _src():
    with open(_GW_PATH, encoding='utf-8') as f:
        return f.read()


# ── 从源码里抽出指定方法，塞进一个空壳类里执行 ──
# ★ 为什么不直接 import WebGateway：它会 import rclpy，单测环境不一定
#   source 过 ROS。这些方法的逻辑只用到 self 上的几个普通属性，
#   抽出来就能真跑，属于"测逻辑而不是测框架"。
def _load(wanted, extra_ns=None):
    src = _src()
    tree = ast.parse(src)
    segs = []
    for n in ast.walk(tree):
        if isinstance(n, ast.FunctionDef) and n.name in wanted:
            segs.append(ast.get_source_segment(src, n))
    assert len(segs) == len(wanted), \
        f'少了方法：{wanted}，实际抽到 {len(segs)} 个'
    code = 'class Fake:\n' + '\n'.join(
        '    ' + ln if ln.strip() else ln
        for seg in segs for ln in seg.splitlines())
    ns = {'math': math, 'time': time, 'threading': threading}
    ns.update(extra_ns or {})
    exec(compile(code, _GW_PATH, 'exec'), ns)
    return ns['Fake']


class _FakePose:
    """够用的 PoseWithCovarianceStamped 替身：只记字段，不做校验。"""
    class _Header:
        def __init__(self):
            self.frame_id = ''
            self.stamp = None

    class _P:
        def __init__(self):
            self.position = type('Q', (), {'x': 0.0, 'y': 0.0})()
            self.orientation = type('Q', (), {'z': 0.0, 'w': 1.0})()

    class _Pose:
        def __init__(self):
            self.pose = _FakePose._P()
            self.covariance = [0.0] * 36

    def __init__(self):
        self.header = _FakePose._Header()
        self.pose = _FakePose._Pose()


class _FakeClock:
    class _T:
        def __init__(self):
            self.sec = 0

    def now(self):
        t = _FakeClock._T()
        return type('N', (), {'to_msg': lambda self=None: None})()


class _FakePub:
    """假 publisher：能配置订阅者数量，记录发布了什么。"""

    def __init__(self, subscribers=1):
        self._n = subscribers
        self.sent = []

    def get_subscription_count(self):
        # 老版本 rclpy 可能没有这个方法：用 raise 分支验证代码的容错
        if self._n < 0:
            raise AttributeError('get_subscription_count')
        return self._n

    def publish(self, msg):
        self.sent.append(msg)


# ══════════════════════════════════════════════════════════
#  ① 换栈必须把上一套栈的运行态全部作废
# ══════════════════════════════════════════════════════════
class TestResetRuntimeCache(unittest.TestCase):

    def setUp(self):
        Fake = _load(['reset_runtime_cache'], {'MODE_TEXT': {}})
        self.node = Fake()
        self.node._gw_lock = threading.Lock()
        self.node._gw_map_png = b'PNG-OLD'
        self.node._gw_map_meta = {'width': 10, 'height': 10, 'resolution': 0.05,
                                  'origin_x': 0.0, 'origin_y': 0.0,
                                  'origin_yaw': 0.0}
        self.node._gw_map_epoch = 1
        self.node._gw_pose = (1.0, 2.0, 0.5, time.time())
        self.node._gw_scan = {'ranges': [1.0]}
        self.node._gw_plan = [(0.0, 0.0), (1.0, 1.0)]
        self.node._gw_task_state = {'state': 'DONE'}
        self.node._gw_last_pose_ok = time.monotonic()
        self.node._gw_last_map_encode = time.monotonic()
        # ★ 假 tf buffer：记录 clear 有没有被调过。
        #   这是本轮最隐蔽的一层 —— 见 test_clears_tf_buffer 的注释。
        self.tf_cleared = []
        node = self

        class _Buf:
            def clear(self):
                node.tf_cleared.append(True)

        self.node.tf_buffer = _Buf()
        self.logs = []
        self.node.log = lambda lvl, txt: self.logs.append((lvl, txt))

    def test_all_runtime_state_cleared(self):
        """五样缓存一样都不能剩：漏一样就是用户看到的那种残影。"""
        self.node.reset_runtime_cache('测试')
        self.assertIsNone(self.node._gw_map_png, '旧地图 PNG 必须清掉（防重影）')
        self.assertIsNone(self.node._gw_map_meta, '旧地图几何必须清掉')
        self.assertIsNone(self.node._gw_pose,
                          '旧位姿必须清掉（否则小车停在切换前的位置）')
        self.assertIsNone(self.node._gw_scan, '旧激光点属于另一套坐标系')
        self.assertIsNone(self.node._gw_plan, '旧全局路径同理')
        self.assertIsNone(self.node._gw_task_state, '旧任务状态不该跨栈显示')

    def test_epoch_increases_and_timers_reset(self):
        self.node.reset_runtime_cache('测试')
        self.assertEqual(self.node._gw_map_epoch, 2, '地图代次要 +1')
        self.node.reset_runtime_cache('再切一次')
        self.assertEqual(self.node._gw_map_epoch, 3, '连续换栈要单调递增')
        self.assertEqual(self.node._gw_last_pose_ok, 0.0,
                         '保鲜计时器归零，否则下一次 tick 立刻判"超时"')
        self.assertEqual(self.node._gw_last_map_encode, 0.0)

    def test_idempotent(self):
        """重复清理不能报错、也不能把 epoch 重置（前端靠它单调判断）。"""
        self.node.reset_runtime_cache('一')
        self.node.reset_runtime_cache('二')
        self.assertIsNone(self.node._gw_map_png)
        self.assertEqual(self.node._gw_map_epoch, 3)

    def test_epoch_starts_at_one(self):
        """★ 代次从 1 起：前端初始值是 -1，用 0 当首值会让
        "后端第一个值"和"前端未初始化"在 == 比较里混淆。"""
        src = _src()
        self.assertIn('self._gw_map_epoch = 1', src)

    def test_clears_tf_buffer(self):
        """★★ 换栈必须清 tf2 的历史缓存，否则位姿会被"复活"。

        这是本轮最隐蔽的一层，值得单独钉死：
          Buffer 默认缓存 10 秒的变换。旧栈节点被杀之后，那段历史仍然躺在
          buffer 里，`lookup_transform(map, base, Time())` 会继续返回
          "最后一个已知变换"。于是 reset 刚把 _gw_pose 设成 None，
          下一个 0.2 秒的 tick 就被旧 TF 重新填满。

        现象：清理日志明明打了，界面上的小车却纹丝不动停在切换前的位置，
        而且还"有条有理"地跟着旧数据动 —— 这比完全不显示更难怀疑。
        """
        self.node.reset_runtime_cache('测试')
        self.assertTrue(self.tf_cleared, 'tf_buffer.clear() 必须被调用')

    def test_tf_clear_failure_does_not_crash(self):
        """个别 tf2 版本没有 clear()：要退化成日志，不能把整个切换搞挂。"""
        class _Bad:
            def clear(self):
                raise AttributeError('no clear')
        self.node.tf_buffer = _Bad()
        self.node.reset_runtime_cache('测试')      # 不能抛
        text = ' '.join(t for _l, t in self.logs)
        self.assertIn('tf', text.lower(), '失败要留痕，别静默')

    def test_logs_reason(self):
        self.node.reset_runtime_cache('切到建图')
        text = ' '.join(t for _l, t in self.logs)
        self.assertIn('切到建图', text, '日志要写清是谁触发的清理（对时间线用）')


# ══════════════════════════════════════════════════════════
#  ② set_mode 只在"模式真的变了"时清缓存
# ══════════════════════════════════════════════════════════
class TestSetModeResetsOnlyOnChange(unittest.TestCase):

    def _mk(self):
        # ★ set_mode 依赖 stack_manager 里的三个模块级名字，必须注入。
        #   直接 import stack_manager 也可以，但它会连带引入 subprocess /
        #   build_launch_command 等一堆东西；这里只需要这三个符号，
        #   注入进去测的是 set_mode 自己的逻辑，最干净。
        Fake = _load(['set_mode', 'reset_runtime_cache'], {
            'MODES': ('nav', 'mapping'),
            'MODE_TEXT': {'nav': '导航模式', 'mapping': '建图模式'},
            'teleop_topic_for': lambda m: ('/cmd_vel_raw' if m == 'mapping'
                                           else '/agv/web_teleop'),
        })
        node = Fake()
        node._gw_lock = threading.Lock()
        node.mode = 'nav'
        node.teleop_topic = '/agv/web_teleop'
        node.cmd_pub = 'PUB_NAV'
        node.pub_by_mode = {'nav': 'PUB_NAV', 'mapping': 'PUB_MAPPING'}
        node._gw_map_png = b'PNG'
        node._gw_map_meta = {'width': 1, 'height': 1, 'resolution': 0.05,
                             'origin_x': 0.0, 'origin_y': 0.0, 'origin_yaw': 0.0}
        node._gw_map_epoch = 0
        node._gw_pose = (0.0, 0.0, 0.0, 0.0)
        node._gw_scan = None
        node._gw_plan = None
        node._gw_task_state = None
        node._gw_last_pose_ok = 0.0
        node._gw_last_map_encode = 0.0
        node.tf_buffer = type('B', (), {'clear': lambda self=None: None})()
        node.logs = []
        node.log = lambda lvl, txt: node.logs.append((lvl, txt))
        return node

    def test_switch_topic_and_clear(self):
        node = self._mk()
        self.assertTrue(node.set_mode('mapping'))
        self.assertEqual(node.mode, 'mapping')
        self.assertEqual(node.cmd_pub, 'PUB_MAPPING', '遥控必须切到建图通道')
        self.assertEqual(node.teleop_topic, '/cmd_vel_raw')
        self.assertIsNone(node._gw_map_png, '换模式必须清掉旧地图')
        self.assertEqual(node._gw_map_epoch, 1)

    def test_same_mode_does_not_clear(self):
        """★ 幂等性：同一模式重复设置不能把正在用的地图清掉。

        stack 每 4 秒轮询一次状态，若不判变化就清缓存，地图会周期性地
        闪没一下 —— 那比"残留"更烦人。
        """
        node = self._mk()
        node.set_mode('nav')
        self.assertEqual(node._gw_map_png, b'PNG', '模式没变不该清地图')
        self.assertEqual(node._gw_map_epoch, 0)

    def test_invalid_mode_rejected(self):
        node = self._mk()
        self.assertFalse(node.set_mode('飞'))
        self.assertEqual(node.mode, 'nav', '非法模式不能改状态')
        self.assertEqual(node._gw_map_png, b'PNG')


# ══════════════════════════════════════════════════════════
#  ③ 位姿保鲜期：查不到 TF 不能无限沿用最后一帧
# ══════════════════════════════════════════════════════════
class _TfRaiser:
    """lookup_transform 一律失败，模拟"新栈还没定位"。"""

    def __init__(self, exc):
        self._exc = exc

    def lookup_transform(self, *a, **k):
        raise self._exc


class _TfStep:
    """lookup_transform 返回一个**旧时间戳**的变换。

    ★ 这是"tf2 缓存会复活位姿"的复现器：旧栈节点停了，buffer 里还留着
      10 秒历史，lookup 照样成功、不报错，只是 stamp 很旧。
      只看"查没查到"的实现会把它当成有效位姿。

    ★ `sim_base` 用来模拟 **Gazebo 仿真时钟**：stamp 从 0 开始算，
      而网关自己的 clock 走 wall time（1.79e9 秒）。两者相减几十亿秒，
      这正是"两个模式都看不到小车"的真实环境。
    """

    def __init__(self, age_sec=30.0, sim_base=None, stamp=None):
        self.age = age_sec
        self.sim_base = sim_base        # None = 用 wall time 做 stamp
        self.fixed_stamp = stamp        # 给定时用固定 stamp（模拟"卡住不动"）

    def lookup_transform(self, *a, **k):
        class _Ns:
            def __init__(self, v):
                self.sec = int(v)
                self.nanosec = int((v - int(v)) * 1e9)

        class _Header:
            def __init__(self, t):
                self.stamp = _Ns(t)
                self.frame_id = 'map'

        class _T:
            def __init__(self):
                self.x = 7.0
                self.y = 8.0

        class _R:
            def __init__(self):
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0
                self.w = 1.0

        class _Tr:
            def __init__(self):
                self.translation = _T()
                self.rotation = _R()

        class _Msg:
            def __init__(self, t):
                self.header = _Header(t)
                self.transform = _Tr()

        if self.fixed_stamp is not None:
            return _Msg(self.fixed_stamp)
        if self.sim_base is not None:
            # 仿真时钟：启动后 sim_base 秒 + 正常抖动，与 wall time 无关
            return _Msg(self.sim_base + (time.time() % 1.0) * 0.01)
        return _Msg(time.time() - self.age)


class TestPoseHold(unittest.TestCase):

    def _mk(self, hold=2.0, tf=None):
        class _Lookup(Exception):
            pass

        class _Conn(Exception):
            pass

        class _Extra(Exception):
            pass

        class _Ns:
            def __init__(self, v):
                self.sec = int(v)
                self.nanosec = int((v - int(v)) * 1e9)

        class _Clock:
            def now(self):
                return type('N', (), {'to_msg': lambda self=None: _Ns(time.time())})()

        Fake = _load(['refresh_pose', '_tf_age', '_tf_stamp', '_tf_is_fresh',
                      '_clock_seconds'], {
            'LookupException': _Lookup,
            'ConnectivityException': _Conn,
            'ExtrapolationException': _Extra,
            'rclpy': type('R', (), {'time': type('T', (), {
                'Time': staticmethod(lambda *a: None)})})(),
            'quaternion_to_yaw': staticmethod(
                lambda x, y, z, w: 0.0),
        })
        node = Fake()
        node._gw_lock = threading.Lock()
        node.map_frame = 'map'
        node.base_frame = 'base_footprint'
        node.pose_hold_sec = hold
        node._gw_pose = (9.0, 9.0, 1.0, time.time())     # 上一套栈留下的位姿
        node._gw_last_pose_ok = time.monotonic()
        node._gw_last_pose_err = 0.0
        node._gw_last_pose_lost_log = 0.0
        node._gw_pose_stamp = 0.0
        node._gw_pose_stamp_at = 0.0
        node._gw_pose_clock = None
        node._gw_pose_fresh = False
        node._gw_pose_stale = False
        node.tf_buffer = tf if tf is not None else _TfRaiser(_Lookup())
        node.get_clock = lambda: _Clock()
        node.logs = []
        node.log = lambda lvl, txt: node.logs.append((lvl, txt))
        node.query_pose_by_service = lambda: None
        node.get_logger = lambda: type('L', (), {
            'error': lambda self=None, m=None: None,
            'warn': lambda self=None, m=None: None,
            'info': lambda self=None, m=None: None})()
        return node, _Conn

    def test_keeps_pose_within_hold_window(self):
        """刚切完的第 1 秒内保留旧位姿：避免界面闪一下"定位未就绪"。"""
        node, _ = self._mk(hold=2.0)
        node.refresh_pose()
        self.assertIsNotNone(node._gw_pose, '保鲜期内应该还留着')

    def test_drops_pose_after_hold_window(self):
        """★ 核心断言：超过保鲜期必须报 None，不能再显示旧位置。

        这就是用户看到的"切换后小车位置与显示不符"的根 ——
        旧代码在这里什么都不做，旧位姿永远有效。
        """
        node, _ = self._mk(hold=2.0)
        node._gw_last_pose_ok = time.monotonic() - 5.0      # 5 秒没拿到位姿
        node.refresh_pose()
        self.assertIsNone(node._gw_pose,
                          '超时后必须置 None，否则小车永远停在旧位置')

    def test_drops_stale_tf_even_when_lookup_succeeds(self):
        """★★ 本轮最深的一层：tf2 缓存会把旧栈的位姿"复活"。

        Buffer 默认缓存 10 秒历史，旧栈节点被杀后 lookup_transform 仍然
        **查得到、不报错**，只是 stamp 很旧。只看"查没查到"的实现会把它
        当成有效位姿 —— 于是 reset 刚清掉的 _gw_pose 在 0.2 秒后又被
        填满，界面上的小车停在切换前的位置，而且看起来"有条有理"。

        所以必须校验变换本身的时间戳，不能只看有没有异常。
        """
        node, _ = self._mk(hold=2.0, tf=_TfStep(age_sec=30.0))
        node._gw_pose = (1.0, 1.0, 0.0, time.time())     # 假设刚才还有值
        node.refresh_pose()
        self.assertIsNone(node._gw_pose,
                          '变换时间戳太旧时必须作废，否则旧栈位姿会复活')

    def test_accepts_fresh_tf(self):
        """新鲜变换要正常用：不能为了防腐把正常数据也拒了。"""
        node, _ = self._mk(hold=2.0, tf=_TfStep(age_sec=0.05))
        node._gw_pose = None
        node.refresh_pose()
        self.assertIsNotNone(node._gw_pose, '新鲜变换必须被接受')
        self.assertAlmostEqual(node._gw_pose[0], 7.0)
        self.assertAlmostEqual(node._gw_pose[1], 8.0)

    def test_tf_age_helper(self):
        """_tf_age 必须用节点自己的 clock（仿真 sim time / 真机 wall time 都对）。"""
        node, _ = self._mk(hold=2.0, tf=_TfStep(age_sec=5.0))
        msg = node.tf_buffer.lookup_transform()
        age = node._tf_age(msg)
        self.assertIsNotNone(age)
        self.assertAlmostEqual(age, 5.0, delta=1.0)

    # ── 时基不一致（本轮真实故障）────────────────────────────
    def test_sim_time_tf_is_not_rejected(self):
        """★★ 真实故障回归锁：Gazebo 栈用 sim time，网关用 wall time。

        stamp 是"启动后 5 秒"，网关 clock 是 wall time（1.79e9 秒），
        相减 ≈ 57 年。第一版实现直接拿这个差值去比保鲜期，
        于是**每一帧都被判成过期**，位姿永远是 None ——
        现象就是"地图、路径、激光都正常，唯独看不到小车"，
        而且建图和导航两个模式全中（两套栈都带 Gazebo）。

        正确行为：时基不可比时不能判死，必须退化为"时间戳是否在更新"。
        """
        node, _ = self._mk(hold=2.0, tf=_TfStep(sim_base=5.0))
        node._gw_pose = None
        node.refresh_pose()
        self.assertIsNotNone(
            node._gw_pose,
            '仿真时基下位姿必须能用，否则两个模式都看不到小车（本次故障）')
        self.assertAlmostEqual(node._gw_pose[0], 7.0)
        self.assertAlmostEqual(node._gw_pose[1], 8.0)

    def test_tf_age_returns_none_on_incomparable_bases(self):
        """时基不可比时 _tf_age 必须返回 None（而不是几十亿秒）。

        返回值会被调用方用来判过期，返回荒谬的大数就等于"永远过期"。
        """
        node, _ = self._mk(hold=2.0, tf=_TfStep(sim_base=5.0))
        msg = node.tf_buffer.lookup_transform()
        self.assertIsNone(
            node._tf_age(msg),
            'sim stamp 与 wall clock 相减无意义，必须返回 None 交给调用方降级处理')

    def test_frozen_sim_tf_eventually_expires(self):
        """仿真时基下上游**停了**，仍然要能判过期（不能因为降级就永远放行）。

        stamp 卡在 5.0 不动超过保鲜期 → 说明没人在发 TF 了 → 必须作废，
        否则又回到"旧栈位姿复活"的老毛病。
        """
        node, _ = self._mk(hold=2.0, tf=_TfStep(sim_base=5.0, stamp=5.0))
        node._gw_pose = (1.0, 1.0, 0.0, time.time())
        node.refresh_pose()                    # 第一次见：接受，并记录时刻
        self.assertIsNotNone(node._gw_pose, '第一次见该接受（否则一上来就丢）')
        # 把"上次看到 stamp 变化"的时刻往前推 5 秒，模拟 stamp 一直没变
        node._gw_pose_stamp_at = time.monotonic() - 5.0
        node.refresh_pose()
        self.assertIsNone(node._gw_pose,
                          '仿真时基下 stamp 卡住超时也必须作废，否则旧位姿复活')

    def test_sim_time_stamp_advancing_keeps_pose(self):
        """时基不可比但 stamp 在前进 → 上游活着 → 位姿保留。"""
        node, _ = self._mk(hold=2.0, tf=_TfStep(sim_base=5.0, stamp=5.0))
        node.refresh_pose()
        self.assertIsNotNone(node._gw_pose)
        node.tf_buffer.fixed_stamp = 5.02       # stamp 前进了一点
        node._gw_pose_stamp_at = time.monotonic() - 30.0   # 即使"很久没变过"
        node.refresh_pose()
        self.assertIsNotNone(node._gw_pose,
                             'stamp 前进了就代表上游在发，必须保留')

    def test_refresh_pose_survives_lookup_exception(self):
        """lookup 抛异常时不能炸（fresh 曾只在 try 内赋值 → UnboundLocalError）。

        这个 bug 的表现同样是"小车永远不显示"，而且日志里只会看到
        位姿解析异常的限流打印，很容易被误认为"TF 没起来"。

        ★ 注意断言的是"不崩 + 保鲜期内保留旧位姿"：
          偶发一次 lookup 失败不该让界面上的车立刻消失（会闪），
          真正的作废交给保鲜期。
        """
        node, _Conn = self._mk(hold=2.0)
        node.tf_buffer = _TfRaiser(_Conn('boom'))
        node.refresh_pose()                     # ★ 不抛 UnboundLocalError 即通过
        self.assertIsNotNone(node._gw_pose,
                             '保鲜期内的偶发失败应保留旧位姿，不该立刻清空（会闪）')
        # 保鲜期过后必须作废
        node._gw_last_pose_ok = time.monotonic() - 5.0
        node.refresh_pose()
        self.assertIsNone(node._gw_pose, '保鲜期过后必须作废')

    def test_declares_pose_hold_param(self):
        src = _src()
        self.assertIn("declare_parameter('pose_hold_sec'", src,
                      '保鲜期必须是可调参数（真机上这 2 秒可能要放大）')
        self.assertIn('self.pose_hold_sec = float(', src)


# ══════════════════════════════════════════════════════════
#  ④ 重新定位：没订阅者必须如实报失败
# ══════════════════════════════════════════════════════════
class TestRelocalizeReportsReality(unittest.TestCase):

    def _mk(self, subscribers=1):
        Fake = _load(['set_initial_pose'], {
            'PoseWithCovarianceStamped': _FakePose,
        })
        node = Fake()
        node._gw_lock = threading.Lock()
        node.map_frame = 'map'
        node.initial_pub = _FakePub(subscribers)
        node.get_clock = lambda: _FakeClock()
        node._gw_pose = (1.0, 1.0, 0.0, 0.0)
        node._gw_last_pose_ok = 0.0
        return node

    def test_ok_when_subscribed(self):
        node = self._mk(1)
        ok, msg = node.set_initial_pose(2.5, -1.5, math.pi / 2)
        self.assertTrue(ok, msg)
        self.assertEqual(len(node.initial_pub.sent), 1, '必须真的 publish 一条')
        m = node.initial_pub.sent[0]
        self.assertAlmostEqual(m.pose.pose.position.x, 2.5)
        self.assertAlmostEqual(m.pose.pose.position.y, -1.5)
        # yaw=π/2 → 四元数 z=sin(π/4), w=cos(π/4)
        self.assertAlmostEqual(m.pose.pose.orientation.z, math.sin(math.pi / 4))
        self.assertAlmostEqual(m.pose.pose.orientation.w, math.cos(math.pi / 4))
        self.assertEqual(m.header.frame_id, 'map')

    def test_fails_when_no_subscriber(self):
        """★ 建图模式没有 AMCL，导航栈没起也没有 AMCL —— 这条路必须报失败。

        旧实现无条件 return {'ok': True}，界面显示"已重新定位"，
        实际消息没人接。用户点几次都一样，于是认为"这个按钮没用"。
        """
        node = self._mk(0)
        ok, msg = node.set_initial_pose(0.0, 0.0, 0.0)
        self.assertFalse(ok, '没有订阅者时必须报失败')
        self.assertIn('没有', msg)

    def test_tolerates_old_rclpy_without_count(self):
        """老版本 rclpy 没有 get_subscription_count，不能因此 500。"""
        node = self._mk(-1)
        ok, _msg = node.set_initial_pose(1.0, 1.0, 0.0)
        self.assertTrue(ok, '拿不到订阅数时不能阻塞定位（宁发放过）')

    def test_cache_pose_follows_target(self):
        """发布后本地缓存立刻跟上：AMCL 收敛要 1~3 秒，
        这期间界面停在旧位置会让用户以为没生效又点一次。"""
        node = self._mk(1)
        node.set_initial_pose(3.0, 4.0, 0.5)
        self.assertIsNotNone(node._gw_pose)
        self.assertAlmostEqual(node._gw_pose[0], 3.0)
        self.assertAlmostEqual(node._gw_pose[1], 4.0)
        self.assertAlmostEqual(node._gw_pose[2], 0.5)
        self.assertGreater(node._gw_last_pose_ok, 0.0,
                           '要刷新保鲜计时，否则下一次 tick 会立刻把它清掉')

    def test_api_returns_message(self):
        """接口层要把真实结果传出去，不能永远 True。"""
        src = _src()
        seg = src[src.index('async def api_relocalize'):]
        seg = seg[:seg.index('async def api_maps')]
        self.assertIn('set_initial_pose(x, y, yaw)', seg)
        self.assertIn("'ok': bool(ok)", seg,
                      '必须把 ok 透传出去（旧版写死 True）')
        self.assertIn("'message': msg", seg)


# ══════════════════════════════════════════════════════════
#  ⑤ WS 事件顺序：epoch 必须在 map_changed 之前发
# ══════════════════════════════════════════════════════════
class TestWsEpochEvent(unittest.TestCase):

    def test_epoch_emitted_before_version(self):
        src = _src()
        seg = src[src.index('async def ws_handler'):]
        seg = seg[:seg.index('app.router.add_')]
        i_epoch = seg.index("'type': 'map_epoch'")
        i_ver = seg.index("'type': 'map_changed'")
        self.assertLess(i_epoch, i_ver,
                        '先发 map_epoch（作废一切）再发 map_changed（重载），'
                        '顺序反了前端会用旧 meta 重载一次')

    def test_epoch_resets_version_cursor(self):
        """代次变了要把版本游标重置，否则新栈的 version 恰好等于旧值时不发事件。"""
        src = _src()
        seg = src[src.index('async def ws_handler'):]
        i = seg.index("if snap['map_epoch'] != last_epoch:")
        tail = seg[i:i + 400]
        self.assertIn('last_ver = -1', tail,
                      '代次变化后必须 last_ver = -1，强制下一帧重发 map_changed')

    def test_snapshot_carries_epoch(self):
        src = _src()
        seg = src[src.index('def snapshot'):]
        seg = seg[:seg.index('# ══════════════════════════ 二、')]
        self.assertIn("'map_epoch': epoch", seg,
                      '快照必须带 map_epoch，否则前端收不到换栈信号')


# ══════════════════════════════════════════════════════════
#  ⑥ manage_stack=false：切模式不能 503
# ══════════════════════════════════════════════════════════
class TestManualStackModeSwitch(unittest.TestCase):
    """手工起栈（manage_stack=false）时也必须能切模式。

    ★ 这是本轮发现的真实缺口，不是测试凑数：
      以前 api_stack_start 第一句就是 `if node.stack is None: 503`。
      而 manage_stack=false 正是"用户自己在终端起栈、网页只当遥控器"，
      那种情况下「启动导航」「开始建图」两个按钮全都 503 ——
      可**切遥控通道**这件事必须由网关做（建图栈没有 arbiter，
      遥控不发 /cmd_vel_raw 车一动都不动）。于是手工起栈的用户
      在网页上永远切不过去，一按就被拒。
    """

    def _fn_src(self):
        src = _src()
        seg = src[src.index('async def api_stack_start'):]
        return seg[:seg.index('async def api_stack_stop')]

    def test_no_early_503(self):
        seg = self._fn_src()
        # 老代码是函数体第一条 if + 直接 return 503
        head = seg[:seg.index('data = await request.json()')]
        self.assertNotIn('status=503', head,
                         '不能让 manage_stack=false 在解析请求前就直接 503')

    def test_degrades_to_mode_switch(self):
        seg = self._fn_src()
        self.assertIn('node.set_mode(mode)', seg,
                      '降到"只切遥控通道"这条路必须调 set_mode')
        self.assertIn('reset_runtime_cache', seg,
                      '降级切换同样要清旧栈缓存（否则还是重影）')
        self.assertIn('manage_stack=false', seg,
                      '要如实告诉用户"栈得你自己起"，不能假装启动了')

    def test_reports_teleop_topic(self):
        seg = self._fn_src()
        self.assertIn('teleop_topic', seg,
                      '要把切换后的遥控话题回给前端（排查时一眼看出对不对）')


if __name__ == '__main__':
    unittest.main()
