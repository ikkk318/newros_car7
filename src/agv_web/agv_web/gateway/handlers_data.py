"""
handlers_data.py —— 读写"数据资产"的接口：地图库、任务缓冲、点位/路线、自检。

**为什么和 ops 分开**
这些接口的失败方式完全不同：ops 失败意味着"车没动 / 栈没起"，用户立刻
看得见；而这里失败往往是**静默**的 —— 地图列表空、任务存不上、自检少一项，
页面照样能点，只是行为莫名其妙。所以它们集中在一个文件里，方便对照
`docs/INTERFACES.md` 的文件格式逐条核。
"""

import asyncio                             # use 线程池里跑磁盘/子进程
import time                                # use 自检里的耗时统计

from aiohttp import web


class ApiData:
    """数据类接口：地图库、任务缓冲、点位/路线、自检。"""

    def __init__(self, node):
        self.node = node


    # ── 地图库 ──

    async def api_maps(self, request):
        # 地图库是纯逻辑模块，直接复用（不依赖 ROS）
        mm = self.node.get_map_manager()
        if mm is None:
            return web.json_response({'ok': False, 'message': '地图库不可用'}, status=503)
        try:
            maps = mm.list_maps()
        except Exception as exc:
            return web.json_response({'ok': False, 'message': f'读地图库失败：{exc}'},
                                     status=500)
        # info：每张图的保存时间/点位数，给前端"选地图"列表当副标题。
        # ★ maps 本身**必须保持纯字符串数组**：前端 renderMaps 和旧调用方
        #   都按字符串遍历，改成对象会连带把地图库列表页打坏。
        info = {}
        for nm in maps:
            try:
                info[nm] = mm.map_info(nm)
            except Exception:
                info[nm] = {'name': nm, 'points': 0, 'saved_at': ''}
        return web.json_response({'ok': True, 'maps': maps, 'current': self.node.current_map,
                                  'info': info})

    async def api_map_delete(self, request):
        mm = self.node.get_map_manager()
        if mm is None:
            return web.json_response({'ok': False, 'message': '地图库不可用'}, status=503)
        data = await request.json()
        name = str(data.get('name', '')).strip()
        if not name:
            return web.json_response({'ok': False, 'message': '缺少地图名'}, status=400)
        if name == self.node.current_map:
            return web.json_response({'ok': False,
                                      'message': '不能删除正在使用的地图'}, status=409)
        try:
            ok, msg = mm.delete_map(name)
        except Exception as exc:
            return web.json_response({'ok': False, 'message': f'删除失败：{exc}'},
                                     status=500)
        self.node.log('warn' if ok else 'error', f'网页删除地图 {name}：{msg}')
        return web.json_response({'ok': bool(ok), 'message': msg})

    async def api_map_switch(self, request):
        """切换地图：写回选择并提示需要重启导航栈（ROS 2 里换图必须重启 map_server）。

        ★ 这里不尝试热切换：Nav2 的 map_server 生命周期虽可重载地图，
          但本项目的点位/路线文件是按地图名注入 launch 参数的，热切会不一致。
          如实告诉用户"要重启栈"比悄悄半生效安全。
        """
        data = await request.json()
        name = str(data.get('name', '')).strip()
        mm = self.node.get_map_manager()
        if mm is None:
            return web.json_response({'ok': False, 'message': '地图库不可用'}, status=503)
        if not name:
            return web.json_response({'ok': False, 'message': '缺少地图名'}, status=400)
        if not mm.map_exists(name):
            return web.json_response({'ok': False, 'message': f'地图 {name} 不存在'},
                                     status=404)
        self.node.current_map = name
        self.node.log('warn', f'网页选择地图 {name}；换图需重启导航栈才能生效')
        return web.json_response({'ok': True, 'current': name,
                                  'message': f'已选择 {name}；需重启导航栈生效'})

    # ── 任务缓冲 ──

    async def api_tasks(self, request):
        mm = self.node.get_task_store()
        if mm is None:
            return web.json_response({'ok': False, 'message': '任务缓冲不可用'}, status=503)
        if request.method == 'GET':
            return web.json_response({'ok': True, 'tasks': mm['load'](),
                                      'running': self.node.task_running})
        data = await request.json()
        action = str(data.get('action', 'list'))
        try:
            # ★ POST 也必须支持 list。
            #   前端 api() 助手固定发 POST，refreshTasks 每 5 秒调一次
            #   `/api/tasks`，以前这里没有 list 分支，直接落到最后
            #   "未知 action" 400。后果是一条完整的恶性链：
            #     刷新失败 → catch 里只打一行 warn，S.taskRunning 不更新
            #     → 界面永远保持上一次的值（true）→ 按钮永远显示"执行中…"
            #     → 再点按钮走的是"中止"分支，永远开不了新任务。
            #   这就是"读取任务失败"刷屏 + "一直显示执行中"的同一个根。
            if action in ('list', ''):
                return web.json_response({'ok': True, 'tasks': mm['load'](),
                                          'running': self.node.task_running})
            if action == 'add':
                items = mm['load']()
                items.append({'pick': str(data.get('pick', '')),
                              'drop': str(data.get('drop', '')),
                              'loops': max(1, int(data.get('loops', 1))),
                              'route': str(data.get('route', ''))})
                mm['save'](items)
                self.node.log('info', f"网页新增搬运任务：{data.get('pick')} ⇒ {data.get('drop')}")
                return web.json_response({'ok': True, 'tasks': items})
            if action == 'delete':
                idx = int(data.get('index', -1))
                items = mm['load']()
                if 0 <= idx < len(items):
                    gone = items.pop(idx)
                    mm['save'](items)
                    return web.json_response({'ok': True, 'tasks': items,
                                              'message': f"已删除 {gone.get('pick')}"})
                return web.json_response({'ok': False, 'message': '序号超出范围'}, status=400)
            if action == 'replace':
                # ★ 整表覆盖：给某一单改路线用。
                #   以前想改路线只能"删了重新加"——顺序会被打到最后，
                #   用户精心排的执行序列就散了。整表回写既保顺序又只有一次 IO。
                raw = data.get('tasks')
                if not isinstance(raw, list):
                    return web.json_response({'ok': False, 'message': 'tasks 必须是数组'},
                                             status=400)
                if self.node.task_running:
                    return web.json_response({'ok': False,
                                              'message': '任务正在执行，先中止再改'}, status=409)
                items = [{'pick': str(t.get('pick', '')),
                          'drop': str(t.get('drop', '')),
                          'loops': max(1, int(t.get('loops', 1) or 1)),
                          'route': str(t.get('route', '') or '')}
                         for t in raw if isinstance(t, dict)]
                mm['save'](items)
                return web.json_response({'ok': True, 'tasks': items,
                                          'message': '任务序列已更新'})
            if action == 'clear':
                mm['save']([])
                return web.json_response({'ok': True, 'tasks': []})
            if action == 'start':
                if self.node.task_running:
                    return web.json_response({'ok': False,
                                              'message': '已有任务在执行'}, status=409)
                items = mm['load']()
                if not items:
                    return web.json_response({'ok': False, 'message': '任务缓冲为空'},
                                             status=400)
                self.node.start_task_buffer(items)      # use 后台线程逐条执行
                return web.json_response({'ok': True, 'message': '已开始执行任务'})
            if action == 'stop':
                # ★ 中止任务必须走这里，不能只发一条裸的 cancel_task 命令。
                #   只发命令的话：task_manager 那边确实取消了，但网关的
                #   _run_tasks 后台线程还在 _wait_task_outcome 里等，它靠的是
                #   _gw_task_stop 这个 Event 才会退出循环 →
                #   task_running 永远不置 False → 界面永远"执行中"。
                #   stop_tasks() 同时做两件事：置 Event + 发 cancel_task。
                was = self.node.task_running
                self.node.stop_tasks()
                return web.json_response(
                    {'ok': True, 'running': False,
                     'message': '已中止任务' if was else '当前没有正在执行的任务'})
            return web.json_response({'ok': False, 'message': '未知 action'}, status=400)
        except Exception as exc:
            return web.json_response({'ok': False, 'message': f'操作失败：{exc}'},
                                     status=500)

    # ── 点位 / 路线 ──

    async def api_routes(self, request):
        # ★ 兼容 GET：GET 没有 body，直接 await request.json() 会抛
        #   "Expecting value"，然后变 500。GET 时从查询串取 action/name，
        #   默认 list —— 这样浏览器直接打开 /api/routes?action=list 就能看。
        try:
            data = await request.json()
        except Exception:
            data = {}
        if not isinstance(data, dict):
            data = {}
        if not data:
            data = {'action': request.query.get('action', 'list'),
                    'name': request.query.get('name', '')}
        action = str(data.get('action', 'list'))
        name = str(data.get('name', ''))
        if action == 'list':
            res, err = self.node.store_request('route', 'list')
        elif action == 'get':
            res, err = self.node.store_request('route', 'get', name=name)
        elif action == 'delete':
            res, err = self.node.store_request('route', 'delete', name=name)
        elif action == 'save':
            res, err = self.node.store_request('route', 'save', name=name,
                                          point_names=data.get('point_names') or [])
        else:
            return web.json_response({'ok': False, 'message': '未知 action'}, status=400)
        if res is None:
            return web.json_response({'ok': False, 'message': err}, status=503)
        items = [{'name': i.name, 'x': i.x, 'y': i.y, 'yaw': i.yaw, 'kind': i.kind}
                 for i in res.items]
        return web.json_response({'ok': bool(res.ok), 'message': res.message,
                                  'items': items, 'names': list(res.names),
                                  'counts': list(res.counts), 'count': int(res.count)})

    # ── 自检 ──

    async def api_check(self, request):
        """逐项检查运行环境，返回结构化结果（全绿才好导航）。

        与终端控制台的 `check` 子命令对齐：节点是否在、服务是否就绪、
        地图/点位/路线文件是否齐全。
        """
        report = []
        # ① 关键节点
        # ★ rclpy 的 Node 没有 get_node_names；要用 get_node_names_and_namespaces
        try:
            pairs = self.node.get_node_names_and_namespaces()
            nodes = [n for n, _ns in pairs]
        except Exception:
            nodes = []
        want_nodes = ['task_manager_node', 'velocity_arbiter', 'amcl',
                      'controller_server', 'planner_server']
        # ★ 这里必须用 self.node.mode，不能写 self.mode：handler 是 ApiData 的
        #   方法，作用域里只有 self（ApiData）和 self.node（网关节点），
        #   mode 在后者身上。写成 self.mode 会 AttributeError，整个自检接口 500，
        #   前端"一键自检"直接失败。
        if self.node.mode == 'mapping':
            # 建图栈里没有 arbiter / task_manager / amcl / nav2，硬查会全红；
            # 改成查建图真正依赖的东西，否则"自检全红"会误导用户。
            # 服务同理：建图栈只有 waypoint_node（点位/路线），
            # /agv/query 与 /agv/cargo 本来就不该存在 —— 以前也硬查，
            # 于是建图模式自检必然挂三个红项，看着像"节点都没起"。
            want_nodes = ['slam_toolbox', 'robot_state_publisher', 'waypoint_node']
            want_services = [('/agv/store', self.node.store_cli)]
        else:
            want_services = [('/agv/query', self.node.query_cli),
                             ('/agv/store', self.node.store_cli),
                             ('/agv/cargo', self.node.cargo_cli)]
        for n in want_nodes:
            hit = any(n in x for x in nodes)
            report.append({'item': f'节点 {n}', 'ok': hit,
                           'message': '在线' if hit else '未发现'})
        # ② 服务就绪
        for label, cli in want_services:
            ok = cli.service_is_ready()
            report.append({'item': f'服务 {label}', 'ok': ok,
                           'message': '就绪' if ok else '未就绪'})
        # ③ 数据
        with self.node._gw_lock:
            has_map = self.node._gw_map_png is not None
            has_pose = self.node._gw_pose is not None
            has_scan = self.node._gw_scan is not None
        # ★ 地图没收到时，如果有 map_server 的失败原文就如实转述。
        #   以前这里只会写"未收到 /map"，用户看到的是"全黑 + 未收到"，
        #   完全猜不到是地图文件/路径的问题（实际踩坑：中文地图名被转义）。
        map_msg = '已收到 /map'
        if not has_map:
            map_msg = '未收到 /map'
            rec = getattr(self.node, '_gw_map_error', None)   # getattr：测试用 AST 抽方法
            if rec and (time.monotonic() - rec[0]) <= 180.0:
                map_msg = f'地图加载失败：{rec[1][:80]}'
        report.append({'item': '地图已加载', 'ok': has_map, 'message': map_msg})
        report.append({'item': '定位可用', 'ok': has_pose,
                       'message': 'map→base 可查' if has_pose else '查不到 TF'})
        report.append({'item': '激光可用', 'ok': has_scan,
                       'message': '已收到 /scan' if has_scan else '未收到 /scan'})
        # ④ 地图库
        mm = self.node.get_map_manager()
        if mm is not None:
            try:
                n_maps = len(mm.list_maps())
                report.append({'item': '地图库', 'ok': n_maps > 0,
                               'message': f'{n_maps} 张地图'})
            except Exception as exc:
                report.append({'item': '地图库', 'ok': False, 'message': str(exc)})
        all_ok = all(r['ok'] for r in report)
        self.node.log('info' if all_ok else 'warn',
                 f"网页自检：{'全部通过' if all_ok else '有项目未通过'}")
        return web.json_response({'ok': all_ok, 'items': report})

    # ── 保存建图结果（等价控制台建图模式的 [m]）──

    async def api_map_save(self, request):
        """把当前 SLAM 地图存进地图库。

        流程与控制台一致：调 `map_saver_cli` 生成 .pgm/.yaml → MapManager 归位。
        ★ 必须放到线程里执行：map_saver_cli 要 1~3 秒，直接在事件循环里
          subprocess.run 会把整个网页卡住（连状态推送都停）。
        """
        data = await request.json()
        name = str(data.get('name', '')).strip()
        if not name:
            return web.json_response({'ok': False, 'message': '请给地图起个名字'},
                                     status=400)
        try:
            from agv_common.map_manager import valid_name
        except Exception:
            valid_name = None
        if valid_name is not None:
            good, reason = valid_name(name)
            if not good:
                return web.json_response({'ok': False, 'message': reason}, status=400)
        mm = self.node.get_map_manager()
        if mm is None:
            return web.json_response({'ok': False, 'message': '地图库不可用'}, status=503)
        loop = asyncio.get_event_loop()
        try:
            ok, msg = await loop.run_in_executor(None, self.node.save_map_now, name, mm)
        except Exception as exc:
            self.node.log('error', f'保存地图失败：{exc}')
            return web.json_response({'ok': False, 'message': f'保存失败：{exc}'},
                                     status=500)
        self.node.log('info' if ok else 'error', f'网页保存地图 {name}：{msg}')
        return web.json_response({'ok': bool(ok), 'message': msg, 'name': name})