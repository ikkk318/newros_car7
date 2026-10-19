#!/usr/bin/env python3
"""web_gateway 的集成型单测（真机 bug 回归）。

这里每一条都对应一个"静态检查抓不到、真机上才暴露"的缺陷：

① `quaternion_to_yaw(rot)` 少传三个参数
   → 被 `except Exception: pose = None` 静默吞掉
   → 位姿永远 None → 小车、朝向箭头、激光散点全不显示
② `api_check` 里写了 `self.mode`（闭包里没有 self）
   → NameError → 自检接口整个 500 → 前端"一键自检"不可用
③ 位姿异常被完全静默
   → 出问题毫无痕迹，只能靠肉眼猜

这些测试不需要真起 HTTP / Gazebo：用 importlib 解析源码 + 直接调函数，
把"签名叫不叫得通"和"闭包变量名对不对"钉死。
"""

import ast
import importlib.util
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(_HERE, '..')                       # src/agv_web
sys.path.insert(0, _SRC)
sys.path.insert(0, os.path.join(_HERE, '..', '..'))    # src

_GW_PATH = os.path.join(_SRC, 'agv_web', 'web_gateway.py')


def _parse_source():
    with open(_GW_PATH, encoding='utf-8') as f:
        return ast.parse(f.read(), filename=_GW_PATH)


class TestQuaternionCallSignature(unittest.TestCase):
    """① 四元数转换必须按 4 个标量传参。"""

    def test_quaternion_to_yaw_takes_four_scalars(self):
        """被调函数的真实签名是 (x, y, z, w) 四个位置参数。

        ★ 这里直接解析 agv_common 的源码，不 import 它：
          agv_common 是另一个 ROS 包，只有 source 过 install/setup.bash 才能
          import。而这条测试的核心是"签名长什么样"，读 AST 更稳、也不受
          环境是否 source 影响（之前就因为这个在未 source 时误报失败）。
        """
        geo = os.path.join(_HERE, '..', '..', 'agv_common',
                           'agv_common', 'geometry.py')
        self.assertTrue(os.path.isfile(geo), f'找不到 {geo}')
        with open(geo, encoding='utf-8') as f:
            tree = ast.parse(f.read(), filename=geo)
        sig = None
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and node.name == 'quaternion_to_yaw':
                sig = [a.arg for a in node.args.args]
                break
        self.assertIsNotNone(sig, 'geometry.py 里应有 quaternion_to_yaw')
        self.assertEqual(sig, ['x', 'y', 'z', 'w'])

    def test_call_site_passes_four_args(self):
        """★ 核心回归：web_gateway 里调用 quaternion_to_yaw 必须传 4 个参数。

        之前的 bug 是 `quaternion_to_yaw(rot)`，只传了 1 个（整个四元数对象），
        运行时抛 TypeError，又被上层 `except Exception` 吞成 pose=None。
        """
        tree = _parse_source()
        calls = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            f = node.func
            name = getattr(f, 'id', None) or getattr(f, 'attr', None)
            if name == 'quaternion_to_yaw':
                calls.append(node)
        self.assertTrue(calls, '源码里应至少有一处 quaternion_to_yaw 调用')
        for node in calls:
            self.assertEqual(
                len(node.args), 4,
                f'第 {node.lineno} 行 quaternion_to_yaw 传了 {len(node.args)} 个参数，'
                f'应为 4 个 (x, y, z, w)')

    def test_quaternion_fields_are_unpacked(self):
        """传进去的应该是 rot.x / rot.y / rot.z / rot.w，而不是 rot 本身。"""
        tree = _parse_source()
        found = False
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                f = node.func
                name = getattr(f, 'id', None) or getattr(f, 'attr', None)
                if name != 'quaternion_to_yaw':
                    continue
                attrs = []
                for a in node.args:
                    attrs.append(getattr(a, 'attr', None))
                if attrs == ['x', 'y', 'z', 'w']:
                    found = True
        self.assertTrue(found, '应存在 quaternion_to_yaw(rot.x, rot.y, rot.z, rot.w)')


class TestNoSelfInClosures(unittest.TestCase):
    """② make_app() 内部的闭包不能用 self（作用域里只有 node）。"""

    def test_no_bare_self_in_make_app(self):
        tree = _parse_source()
        make_app = None
        for node in tree.body:
            if isinstance(node, ast.FunctionDef) and node.name == 'make_app':
                make_app = node
                break
        self.assertIsNotNone(make_app, '应能找到 make_app 函数')

        offenders = []
        for node in ast.walk(make_app):
            if isinstance(node, ast.Attribute):
                v = node.value
                if isinstance(v, ast.Name) and v.id == 'self':
                    offenders.append((node.lineno, node.attr))
        self.assertEqual(
            offenders, [],
            f'make_app() 内部闭包不应引用 self，发现：{offenders}')

    def test_api_check_uses_node_mode(self):
        """自检接口判模式必须用 node.mode。"""
        with open(_GW_PATH, encoding='utf-8') as f:
            src = f.read()
        self.assertIn("if node.mode == 'mapping'", src)


class TestPoseErrorNotSilent(unittest.TestCase):
    """③ 位姿异常不能静默吞掉。"""

    def test_refresh_pose_logs_unexpected_exception(self):
        with open(_GW_PATH, encoding='utf-8') as f:
            src = f.read()
        self.assertIn('_gw_last_pose_err', src,
                      '位姿异常应有日志限流字段')
        self.assertIn('位姿解析异常', src,
                      '位姿解析异常应打日志，不能静默 pose=None')


class TestGridRenderSanity(unittest.TestCase):
    """地图 PNG 的真实像素：验证方向没画反。"""

    def test_wall_row_lands_on_correct_edge(self):
        """★ 地图上下翻转回归：底部的墙必须出现在 PNG 的最后一行。

        造一张 4x3 的图，只在第 0 行（y 最小，即真实世界的底部）放占据格，
        编码后该行必须对应 PNG 的最后一行。写反了就会出现在最上面。
        """
        from agv_web.grid_image import occupancy_to_png
        import struct
        import zlib
        W, H = 4, 3
        data = [0] * (W * H)
        for x in range(W):
            data[0 * W + x] = 100          # 第 0 行 = 世界底部，整行是墙
        png = occupancy_to_png(W, H, data)
        # 解出像素
        i = png.find(b'IDAT')
        ln = struct.unpack('>I', png[i - 4:i])[0]
        raw = zlib.decompress(png[i + 4:i + 4 + ln])
        first = raw[1:1 + W * 3]                       # PNG 第一行
        last_off = (H - 1) * (1 + W * 3)
        last = raw[last_off + 1:last_off + 1 + W * 3]  # PNG 最后一行
        black = bytes((0, 0, 0)) * W
        self.assertEqual(last, black, '底部的墙应出现在 PNG 最后一行')
        self.assertNotEqual(first, black, '顶部不应是墙（说明方向画反了）')


class TestRouteMethodCompatibility(unittest.TestCase):
    """④ 前端 api() 固定发 POST，后端路由必须能收 POST。

    ★ 这条是用户实际踩到的坑：`/api/maps`、`/api/stack`、`/api/check`
      原来只注册了 add_get，前端 POST 过去吃 405；前端 catch 后把 maps
      当成空数组，用户看到的现象是"地图库是空的 → 必须先建图 → 建完还
      是不行"，完全无法进入导航模式。

    这类 bug 静态检查、单元测试都发现不了（函数本身没错，错在注册方法），
    只有把"前端发的动词"和"后端注册的动词"对齐才能防住。
    """

    # 前端 app.js 里通过 api('/api/xxx') 调用、且 api() 用 POST 的接口
    _POST_FROM_FRONTEND = (
        '/api/maps', '/api/stack', '/api/check', '/api/tasks',
        '/api/lease', '/api/teleop', '/api/stop', '/api/command',
        '/api/store', '/api/cargo', '/api/stack_start', '/api/stack_stop',
        '/api/map_switch', '/api/map_delete', '/api/map_save',
        '/api/relocalize', '/api/routes', '/api/resume', '/api/login',
        '/api/shutdown',
    )

    def _registered(self):
        """解析出每个路径允许的方法集合。

        ★ 注意 add_route 的签名叫 `add_route(method, path, handler)`，
          路径是**第二个**参数；只有 add_get/add_post 才是 `(path, handler)`，
          路径是第一个。一开始按 args[-1] 取会把 handler 当路径，
          于是整张路由表都解析成空 —— 测试全红但代码其实是对的。
        """
        tree = _parse_source()
        routes = {}
        for n in ast.walk(tree):
            if not isinstance(n, ast.Call):
                continue
            fn = n.func
            if not isinstance(fn, ast.Attribute):
                continue
            if fn.attr not in ('add_get', 'add_post', 'add_route'):
                continue
            # 取"字符串字面量参数里最靠后的那个"作为路径：
            # add_get/add_post → 只有 path；add_route → ('*', path)
            path = None
            for a in n.args:
                if isinstance(a, ast.Constant) and isinstance(a.value, str):
                    path = a.value
            if not path:
                continue
            if fn.attr == 'add_get':
                routes.setdefault(path, set()).add('GET')
            elif fn.attr == 'add_post':
                routes.setdefault(path, set()).add('POST')
            else:
                routes.setdefault(path, set()).update({'GET', 'POST'})
        return routes

    def test_frontend_paths_accept_post(self):
        routes = self._registered()
        missing = []
        for path in self._POST_FROM_FRONTEND:
            allowed = routes.get(path)
            if allowed is None:
                missing.append(f'{path}（后端根本没注册）')
            elif 'POST' not in allowed:
                missing.append(f'{path}（只允许 {"".join(sorted(allowed))}）')
        self.assertEqual([], missing,
                         '这些接口前端用 POST 调，后端必须收 POST：\n  '
                         + '\n  '.join(missing))

    def test_maps_route_keeps_get_too(self):
        """同时保留 GET：浏览器直接敲 URL、curl 调试都靠它。"""
        routes = self._registered()
        self.assertIn('GET', routes.get('/api/maps', set()))
        self.assertIn('GET', routes.get('/api/stack', set()))

    def test_readonly_queries_accept_get(self):
        """纯查询类接口必须 GET/POST 都能收。

        ★ /api/routes 原来只注册了 add_post。前端恰好都发 POST，所以
          一直没暴露；但这就意味着浏览器敲 URL、curl、或者别的客户端
          一律 405 —— 和当初 /api/maps 的 bug 是同一类，只是还藏在
          没人走的分支里。纯查询接口（不改状态）没有理由拒绝 GET。
        """
        routes = self._registered()
        for path in ('/api/maps', '/api/stack', '/api/check',
                     '/api/tasks', '/api/routes'):
            allowed = routes.get(path, set())
            self.assertIn('GET', allowed, f'{path} 应支持 GET（现在只允许 {allowed}）')
            self.assertIn('POST', allowed, f'{path} 应支持 POST（现在只允许 {allowed}）')

    def test_readonly_handlers_tolerate_empty_body(self):
        """GET 没有 body，处理器不能直接 await request.json()。

        ★ aiohttp 里 GET 请求 body 为空，`await request.json()` 会抛
          "Expecting value" → 500。改了注册方法却没改处理器，等于把
          405 换成了 500，更难看。这里只能静态检查"有没有 try 包住"，
          真正的行为由 api_routes 的注释与端到端测试保障。
        """
        src = open(_GW_PATH, encoding='utf-8').read()
        i = src.index('async def api_routes')
        seg = src[i:i + 900]
        self.assertIn('request.json()', seg)
        self.assertIn('except Exception', seg,
                      'api_routes 必须容忍空 body（GET 请求）')

    def test_shutdown_route_exists(self):
        """网页「退出」按钮依赖这个接口，缺了就等于退不干净。"""
        self.assertIn('/api/shutdown', self._registered())


class TestLeaseRenewExists(unittest.TestCase):
    """⑤ 控制权必须有 renew 动作，且超时要够长。

    ★ 用户实际抱怨："获取控制权后不点又会丢失，手动控制时要不停地点"。
      根因：lease_idle_sec 默认 3 秒，且只有发速度时才续期。用户拿到锁后
      想一下往哪走就过期了。修法有三处，缺一不可：
        ① 超时放宽（3 → 120 秒）
        ② 后端 /api/lease 支持 action=renew
        ③ 前端定时轻量续期
      这里把 ①② 钉死，③ 交给前端测试。
    """

    def test_lease_idle_default_is_generous(self):
        tree = _parse_source()
        found = None
        for n in ast.walk(tree):
            if not isinstance(n, ast.Call):
                continue
            fn = n.func
            if not (isinstance(fn, ast.Attribute)
                    and fn.attr == 'declare_parameter'):
                continue
            if len(n.args) < 2:
                continue
            if not (isinstance(n.args[0], ast.Constant)
                    and n.args[0].value == 'lease_idle_sec'):
                continue
            if isinstance(n.args[1], ast.Constant):
                found = n.args[1].value
        self.assertIsNotNone(found, '找不到 lease_idle_sec 的声明')
        self.assertGreaterEqual(
            float(found), 30.0,
            f'控制权闲置超时只有 {found}s，用户思考几秒就掉锁，'
            '实操会变成"每走一步重新点一次"')

    def test_lease_api_supports_renew(self):
        with open(_GW_PATH, encoding='utf-8') as f:
            src = f.read()
        self.assertIn("action == 'renew'", src,
                      '/api/lease 必须支持 renew，否则只有发速度时才续期')
        self.assertIn('node.lease.renew(', src)


class TestStartupCleanup(unittest.TestCase):
    """⑥ 启动要清上次残留，但绝不能误杀。

    用户原话："像终端的启动版那样，启动的时候自动清理上次没有正常关闭的
    进程"。终端版 agv_console.py 早就在启动时 pkill 了，网页版一直漏。

    同时必须守住安全边界：
      · 不许用 python3/node 这类宽泛词（会杀掉网关自己、WorkBuddy 本体）
      · manage_stack=false（用户自己在别的终端起了栈）时不许清
    """

    def test_cleanup_function_exists(self):
        with open(_GW_PATH, encoding='utf-8') as f:
            src = f.read()
        self.assertIn('def cleanup_residual(', src)
        self.assertIn('_CLEANUP_PATTERNS', src)

    def test_cleanup_patterns_are_narrow(self):
        """模式必须足够特别，不能是 python3 这种。"""
        tree = _parse_source()
        pats = None
        for n in ast.walk(tree):
            if isinstance(n, ast.Assign):
                for t in n.targets:
                    if isinstance(t, ast.Name) and t.id == '_CLEANUP_PATTERNS':
                        if isinstance(n.value, (ast.Tuple, ast.List)):
                            pats = [e.value for e in n.value.elts
                                    if isinstance(e, ast.Constant)]
        self.assertIsNotNone(pats, '找不到 _CLEANUP_PATTERNS')
        dangerous = ('python3', 'python', 'node', 'ros2', 'bash', 'sh')
        for p in pats:
            self.assertNotIn(
                p, dangerous,
                f'清理模式 {p!r} 太宽泛，会误杀无关进程（包括网关自己）')

    def test_cleanup_respects_manage_stack(self):
        """manage_stack=false 时不能再清理用户自己的栈。"""
        with open(_GW_PATH, encoding='utf-8') as f:
            src = f.read()
        self.assertIn('node.stack is not None', src)
        self.assertIn('AGV_NO_CLEANUP', src,
                      '要留一个环境变量逃生口，便于排查时禁用清理')

    def test_cleanup_never_kills_itself(self):
        """★ 清理绝不能杀掉拉起自己的 launch。

        真实事故：最初用 'ros2 launch agv_' 匹配所有本项目 launch，
        而网关自己就是 `ros2 launch agv_web web_console.launch.py` 拉起来的，
        于是启动瞬间把自己杀了：
            SIGINT 34 (ros2 launch agv_)
            34 已杀死  ros2 launch agv_web web_console.launch.py ...
        网页根本没起来。所以必须同时具备：
          ① _ancestor_pids() 排除祖先链
          ② _CLEANUP_LAUNCH_EXCLUDE 排除 agv_web 自身
        """
        with open(_GW_PATH, encoding='utf-8') as f:
            src = f.read()
        self.assertIn('def _ancestor_pids(', src, '必须有祖先链排除')
        self.assertIn('_CLEANUP_LAUNCH_EXCLUDE', src, '必须排除 agv_web 自身')
        self.assertIn('pid in protect', src, '匹配结果要过祖先链过滤')

    def test_ancestor_pids_includes_self(self):
        """_ancestor_pids 至少包含当前进程自己。"""
        import importlib.util
        # 只取这个函数，避开模块级 ROS import
        with open(_GW_PATH, encoding='utf-8') as f:
            src = f.read()
        tree = ast.parse(src)
        func = None
        for n in tree.body:
            if isinstance(n, ast.FunctionDef) and n.name == '_ancestor_pids':
                func = n
                break
        self.assertIsNotNone(func, '找不到 _ancestor_pids')
        mod = ast.Module(body=[func], type_ignores=[])
        ns = {'os': os}
        exec(compile(mod, '<anc>', 'exec'), ns)
        pids = ns['_ancestor_pids']()
        self.assertIn(os.getpid(), pids, '祖先集合必须含自己')


class TestMapYamlPath(unittest.TestCase):
    """map_yaml 必须是"真实存在的 .yaml 文件路径"。

    ★ 这条测试是给一个很隐蔽的 bug 立的碑：
      原来写的是
          kwargs['map_yaml'] = paths[0] if isinstance(paths, (list, tuple)) \\
              else str(paths)
      而 MapManager.map_paths() 返回的是 **dict**，两个分支都不匹配，
      于是 map_yaml 变成了一整个 dict 的 str()：
          "{'dir': '/...', 'pgm': '/...', 'yaml': '/...', ...}"
      map_server 拿这个"路径"当然打不开文件，导航栈能起来但地图加载失败。
      症状和"没地图"几乎一样，所以用户会以为是"要先建图"。
    """

    def test_gateway_uses_yaml_key_not_str_dict(self):
        with open(_GW_PATH, encoding='utf-8') as f:
            src = f.read()
        # ★ 必须先把注释剔掉再查：修复时在同一行上方留了"以前的错误写法是什么"
        #   的说明注释，直接 assertNotIn 会把注释里的示例当成真代码抓出来，
        #   测试自己误报（第一次就是这么挂的）。
        code = '\n'.join(ln.split('#', 1)[0] for ln in src.splitlines())
        # 不许再出现"猜类型"的写法
        self.assertNotIn('isinstance(paths, (list, tuple))', code,
                         'map_paths() 返回 dict，不能再按 list/tuple 猜类型')
        self.assertNotIn("map_yaml'] = paths[0]", code)
        self.assertNotIn('else str(paths)', code,
                         'dict 直接 str() 会变成一整串花括号，map_server 打不开')
        # 必须用键取
        self.assertIn('mm.map_yaml_file(', code,
                      '应当走 map_yaml_file() 辅助方法')

    def test_gateway_uses_launch_alias_for_map_server(self):
        """传给 launch 的地图路径必须走 launch_map_yaml()。

        ★ 另一个更隐蔽的坑：中文地图名（控制台自动生成的「地图10月19日16时」）
          会被 Nav2 的 RewrittenYaml（yaml.dump 默认 allow_unicode=False）
          转义成 \\uXXXX，map_server 收到 "u5730u56FE…" 直接 bad file，
          整个 bringup 中止 —— 表现是"地图全黑、只剩起始点、小车也不见"。
          launch_map_yaml() 对非 ASCII 名返回纯 ASCII 别名，对 ASCII 名原样返回。
        """
        with open(_GW_PATH, encoding='utf-8') as f:
            src = f.read()
        code = '\n'.join(ln.split('#', 1)[0] for ln in src.splitlines())
        self.assertIn('mm.launch_map_yaml(', code,
                      '导航启动必须用 launch_map_yaml()，否则中文地图名会加载失败')
        # 不许再直接把真实路径塞给 launch
        self.assertNotIn("kwargs['map_yaml'] = mm.map_yaml_file(", code,
                         'map_yaml 是给 map_server 的，必须过 ASCII 别名')

    def test_map_manager_has_yaml_helper(self):
        """MapManager 要提供 map_yaml_file()，避免调用方各自拼路径。"""
        from agv_common.map_manager import MapManager
        self.assertTrue(hasattr(MapManager, 'map_yaml_file'),
                        '缺少 map_yaml_file()，调用方又会开始猜路径')

    def test_map_yaml_file_really_exists(self):
        """用真实地图库验证：取出来的必须是存在的文件，不是 dict 字符串。"""
        import tempfile
        import shutil
        from agv_common.map_manager import MapManager
        root = tempfile.mkdtemp(prefix='agvmaps_')
        try:
            d = os.path.join(root, 'm1')
            os.makedirs(d)
            for ext in ('pgm', 'yaml'):
                with open(os.path.join(d, f'm1.{ext}'), 'w', encoding='utf-8') as f:
                    f.write('x')
            mm = MapManager(root)
            got = mm.map_yaml_file('m1')
            self.assertIsInstance(got, str)
            self.assertNotIn('{', got, '不能是 dict 的 str()')
            self.assertTrue(os.path.isfile(got), f'map_yaml 指向的文件不存在：{got}')
            self.assertTrue(got.endswith('m1.yaml'))
        finally:
            shutil.rmtree(root, ignore_errors=True)

    def test_map_info_fields(self):
        """map_info 必须给出前端副标题要用的字段。"""
        import tempfile
        import shutil
        from agv_common.map_manager import MapManager
        root = tempfile.mkdtemp(prefix='agvmaps_')
        try:
            d = os.path.join(root, 'm1')
            os.makedirs(d)
            for ext in ('pgm', 'yaml'):
                with open(os.path.join(d, f'm1.{ext}'), 'w', encoding='utf-8') as f:
                    f.write('x')
            with open(os.path.join(d, 'waypoints.yaml'), 'w', encoding='utf-8') as f:
                f.write('frame_id: map\nwaypoints:\n'
                        '  起点: {x: 0.0, y: 0.0, yaw: 0.0}\n'
                        '  终点:\n    x: 1.0\n    y: 2.0\n    yaw: 0.5\n')
            mm = MapManager(root)
            info = mm.map_info('m1')
            for k in ('name', 'saved_at', 'saved_ts', 'points', 'has_routes'):
                self.assertIn(k, info)
            self.assertEqual(info['points'], 2, f'点位数应 2，实际 {info["points"]}')
            self.assertTrue(info['saved_at'], '应能取到保存时间')
            self.assertFalse(info['has_routes'])
            # 不存在的地图不能抛异常，只能返回零值
            empty = mm.map_info('does_not_exist')
            self.assertEqual(empty['points'], 0)
            self.assertEqual(empty['saved_at'], '')
        finally:
            shutil.rmtree(root, ignore_errors=True)


class TestTaskApiActions(unittest.TestCase):
    """任务接口必须支持前端真正会发的每个 action。

    ★ 用户报的 bug："执行完任务后还是显示执行中，且手动点击中断任务后
      也依旧显示执行中，导致之后不能终止任务也不能启动新任务"

      根因不是状态机，是**接口缺分支**：
      前端 `api()` 固定发 POST，`refreshTasks()` 每 5 秒 POST 一次
      `/api/tasks`（body 空 → action 默认 'list'），但 `api_tasks` 的
      POST 分支只处理 add/delete/clear/start，落到最后一行
      `未知 action` 400。于是一整条恶性链：

        读取失败 → catch 里 S.taskRunning 不更新
        → 界面保持上一次的 true → 按钮永远"执行中…"
        → 再点走"中止"分支 → 永远开不了新任务

      更糟的是中止只发了一条裸 `cancel_task`，网关的 `_run_tasks`
      线程靠 `_gw_task_stop` 事件才退出，那个事件从来没被置位过
      （`stop_tasks()` 写了却没有任何调用点），所以线程要等 300 秒超时。
    """

    def _actions_in(self, handler_name):
        """取出某个 handler 里比较过的 action 字面量集合。"""
        tree = _parse_source()
        for n in ast.walk(tree):
            if isinstance(n, ast.AsyncFunctionDef) and n.name == handler_name:
                found = set()
                for c in ast.walk(n):
                    if isinstance(c, ast.Compare):
                        for comp in [c.left] + list(c.comparators):
                            if isinstance(comp, ast.Constant) and \
                                    isinstance(comp.value, str):
                                found.add(comp.value)
                    if isinstance(c, ast.Constant) and isinstance(c.value, str):
                        found.add(c.value)
                return found
        return set()

    def test_tasks_post_supports_list(self):
        acts = self._actions_in('api_tasks')
        self.assertIn('list', acts,
                      'POST /api/tasks 必须支持 list，否则 refreshTasks 永远 400')

    def test_tasks_supports_stop(self):
        acts = self._actions_in('api_tasks')
        self.assertIn('stop', acts,
                      '必须提供 stop 动作，中止时才能置 _gw_task_stop 事件')

    def test_stop_tasks_is_actually_called(self):
        """★ stop_tasks() 以前是死代码 —— 定义了没有任何调用点。

        这属于最隐蔽的一类 bug：静态检查全绿、函数体看着也没错，
        但功能就是不生效。所以专门钉一条"必须有人调用它"。
        """
        with open(_GW_PATH, encoding='utf-8') as f:
            src = f.read()
        code = '\n'.join(ln.split('#', 1)[0] for ln in src.splitlines())
        # 除定义行之外的调用点
        calls = [ln for ln in code.splitlines()
                 if 'stop_tasks(' in ln and 'def stop_tasks' not in ln]
        self.assertTrue(calls, 'stop_tasks() 没有调用点，中止任务时事件永远不会置位')

    def test_cancel_command_stops_buffer(self):
        """cancel_task / stop 命令必须走 stop_tasks，而不是只发裸命令。"""
        tree = _parse_source()
        for n in ast.walk(tree):
            if isinstance(n, ast.AsyncFunctionDef) and n.name == 'api_command':
                src = ast.get_source_segment(
                    open(_GW_PATH, encoding='utf-8').read(), n) or ''
                code = '\n'.join(ln.split('#', 1)[0] for ln in src.splitlines())
                self.assertIn('stop_tasks', code,
                              'api_command 收到取消命令时必须同时停任务缓冲')
                return
        self.fail('找不到 api_command')

    def test_stop_tasks_clears_running_flag(self):
        """stop_tasks 必须立刻把 task_running 置 False。

        否则界面要等后台线程的下一轮循环才变回"开始执行"，
        用户会觉得"点了没反应"。
        """
        src = open(_GW_PATH, encoding='utf-8').read()
        tree = ast.parse(src)
        for n in ast.walk(tree):
            if isinstance(n, ast.FunctionDef) and n.name == 'stop_tasks':
                seg = ast.get_source_segment(src, n) or ''
                code = '\n'.join(ln.split('#', 1)[0] for ln in seg.splitlines())
                self.assertIn('task_running = False', code,
                              'stop_tasks 要立刻清掉 task_running')
                self.assertIn('_gw_task_stop.set()', code,
                              '必须置停止事件，否则后台线程要等到超时')
                return
        self.fail('找不到 stop_tasks')


class TestTaskManagerPublishesAfterCancel(unittest.TestCase):
    """task_manager 的 handle_cancel 必须发一次状态。

    ★ 以前漏了 `publish_state()`：状态机内部回了 IDLE，
      但 /agv/task_state 最后一帧还停在 GO_LOAD，
      网页显示"执行中"，车其实早停了。
      这类"改了状态却没上报"的漏点很容易再犯，所以钉住。
    """

    # _HERE = <ws>/src/agv_web/test。别数层数（数错过一次，
    # 结果是"文件找不到 → 静默跳过 → 测试永远绿"），
    # 直接沿父链找名为 src 的那一级，最稳。
    def _find_ws_src(self):
        d = _HERE
        for _ in range(8):
            if os.path.basename(d) == 'src' and os.path.isdir(
                    os.path.join(d, 'agv_task')):
                return d
            nd = os.path.dirname(d)
            if nd == d:
                break
            d = nd
        return ''

    @property
    def _tm_path(self):
        src = self._find_ws_src()
        return (os.path.join(src, 'agv_task', 'agv_task', 'task_manager_node.py')
                if src else '')

    def test_handle_cancel_publishes_state(self):
        # ★ 这里原来是 skipTest，结果路径算错时测试"静默通过"了。
        #   静默跳过比失败更危险：报告里显示绿色，你以为测过了。
        #   所以找不到文件直接 fail。
        self.assertTrue(os.path.isfile(self._tm_path),
                        f'找不到 task_manager_node.py：{self._tm_path}')
        src = open(self._tm_path, encoding='utf-8').read()
        tree = ast.parse(src)
        for n in ast.walk(tree):
            if isinstance(n, ast.FunctionDef) and n.name == 'handle_cancel':
                seg = ast.get_source_segment(src, n) or ''
                code = '\n'.join(ln.split('#', 1)[0] for ln in seg.splitlines())
                self.assertIn('publish_state', code,
                              'handle_cancel 改变了状态机，必须紧跟 publish_state()')
                return
        self.fail('找不到 handle_cancel')


if __name__ == '__main__':
    unittest.main()