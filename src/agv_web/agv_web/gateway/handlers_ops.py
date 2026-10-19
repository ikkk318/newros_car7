"""
handlers_ops.py —— 会"让车动 / 让进程动"的接口。

包含：控制权租约、遥控、急停/停下、指令下发、搬货、重定位、恢复、
栈的查询/启动/切换/停止、以及网页退出。

**这些接口的共同点：都有副作用，都要过门卫**
  · 遥控先验控制权（`ControlLease`），再按模式选话题（导航走
    `/agv/web_teleop` 由 task_manager 转发，建图直发 `/cmd_vel_raw`）；
  · 指令不直接调节点，统一走 `/agv/command`，由 task_manager 判定；
  · 栈操作要 15~40 秒，必须丢线程池 —— 直接在事件循环里做会把网页卡死。

★ 阻塞动作一律 `loop.run_in_executor`：只占 executor，不占事件循环。
"""

import asyncio                             # use 线程池里跑阻塞动作
import os                                  # use 向自己发 SIGINT
import signal                              # use 同上

from aiohttp import web

from agv_common.web_guard import LOCKED_BY_OTHER

from ..stack_manager import (
    MODE_MAPPING, MODE_NAV, MODE_TEXT, MODES, validate_map_name,
)
from .util import _with_where


class ApiOps:
    """动作类接口：每个都会改变车、栈或进程的状态。"""

    def __init__(self, node):
        self.node = node


    # ── 控制权 ──

    async def api_lease(self, request):
        """控制权操作：acquire / renew / release。

        ★ 为什么单独立一个 renew 动作（而不是只靠 api_teleop 顺带续期）：
          旧实现只有发速度时才续期。可用户拿到控制权后往往先"想一下往哪走"、
          或者按一下停一下，中间不发速度，锁就按闲置超时被释放了。
          前端下一次按方向键才发现自己没锁，必须再点一次「取得控制权」，
          连起来就是"每走一步点一次锁"，根本没法用。
          加了 renew 之后，前端可以在"持有控制权期间"定时轻量续期，
          锁的语义变成"我还在用这台车"，而不是"我这几毫秒在发速度"。
        """
        data = await request.json()
        action = data.get('action', 'acquire')
        who = str(data.get('who', ''))
        if action == 'acquire':
            ok, code, owner = self.node.lease.acquire(who)
            if not ok:
                return web.json_response({'ok': False, 'code': code,
                                          'owner': owner,
                                          'message': f'控制权在另一台设备（{owner}）手上'},
                                         status=409)
            return web.json_response({'ok': True, 'owner': owner})
        if action == 'renew':
            # 续期失败不返回 409：这是后台定时调用，报错会刷满前端日志。
            # 用 200 + ok:false 让前端自己安静地把 hasLease 置回 false。
            ok = self.node.lease.renew(who)
            if ok:
                return web.json_response({'ok': True, 'owner': self.node.lease.owner()})
            return web.json_response({'ok': False, 'code': LOCKED_BY_OTHER,
                                      'owner': self.node.lease.owner(),
                                      'message': '控制权已不是你的'})
        if action == 'release':
            self.node.lease.release(who)
            self.node.publish_twist(0.0, 0.0)
            self.node.watchdog.idle()
            return web.json_response({'ok': True})
        return web.json_response({'ok': False, 'message': '未知 action'}, status=400)

    # ── 遥控 / 急停 ──

    async def api_teleop(self, request):
        data = await request.json()
        who = str(data.get('who', ''))
        if self.node.lease.owner() != who:
            return web.json_response({'ok': False, 'code': LOCKED_BY_OTHER,
                                      'owner': self.node.lease.owner(),
                                      'message': '没有控制权，无法遥控'}, status=409)
        self.node.lease.renew(who)                  # # 手还按着：续期
        v = float(data.get('v', 0.0))
        w = float(data.get('w', 0.0))
        # ★ 松手与掉线要分开：前端松手时会主动补发一帧 (0,0) 让车停，
        #   那是**正常操作**，不是"前端失联"。把它也 feed() 进看门狗，
        #   0.35 秒后就会误报一次"遥控心跳超时"（日志噪音，且以前还会
        #   顺手把控制权收走）。零速直接 idle 掉看门狗，车已经停了，
        #   没有什么需要超时兜底的。
        if abs(v) < 1e-6 and abs(w) < 1e-6:
            self.node.watchdog.idle()               # # 主动松手：收手，不判掉线
        else:
            self.node.watchdog.feed()               # # 真在开：刷新掉线计时
        v, w = self.node.publish_twist(v, w)
        return web.json_response({'ok': True, 'v': v, 'w': w})

    async def api_stop(self, request):
        self.node.publish_twist(0.0, 0.0)
        self.node.watchdog.idle()
        self.node.lease.force_release()
        self.node._gw_estop = True
        self.node.send_command('stop')
        self.node.log('warn', '网页触发急停')
        return web.json_response({'ok': True})

    # ── 指令总线 ──

    async def api_command(self, request):
        data = await request.json()
        cmd = str(data.get('command', '')).strip()
        args = data.get('args') or []
        route = data.get('route', '')
        if not cmd:
            return web.json_response({'ok': False, 'message': '缺少 command'}, status=400)
        # ★ 取消类命令必须同时把网关自己的任务缓冲停掉。
        #   以前这里只发一条裸命令，于是出现"点了取消，网页还是执行中"：
        #   task_manager 那边停止了，网关的 _run_tasks 线程却还在等状态。
        #   这不是 task_manager 的锅，是网关漏了配对动作。
        if cmd in ('cancel_task', 'stop'):
            # 'stop' 是急停：只置网关事件，取消动作由 stop 命令自己完成
            self.node.stop_tasks(notify=(cmd == 'cancel_task'))
            if cmd == 'stop':
                self.node.send_command('stop')
            self.node.log('warn', f'网页下达取消：{cmd}，任务缓冲已停止')
            return web.json_response({'ok': True, 'running': False,
                                      'message': '已取消并停车'})
        rid = self.node.send_command(cmd, args, route)
        self.node.log('info', f'网页下发指令：{cmd} {args}（{rid}）')
        return web.json_response({'ok': True, 'request_id': rid})

    async def api_store(self, request):
        data = await request.json()
        res, err = self.node.store_request(
            data.get('resource', 'waypoint'), data.get('action', 'list'),
            name=data.get('name', ''), x=data.get('x', 0.0), y=data.get('y', 0.0),
            yaw=data.get('yaw', 0.0), kind=data.get('kind', ''),
            point_names=data.get('point_names') or [])
        if res is None:
            return web.json_response({'ok': False, 'message': err}, status=503)
        items = [{'name': i.name, 'x': i.x, 'y': i.y, 'yaw': i.yaw, 'kind': i.kind}
                 for i in res.items]
        return web.json_response({'ok': bool(res.ok), 'message': res.message,
                                  'items': items, 'names': list(res.names),
                                  'counts': list(res.counts), 'count': int(res.count)})

    # ── 搬货 ──

    async def api_cargo(self, request):
        data = await request.json()
        cmd = str(data.get('command', 'lift'))
        # ★ 顶升/下降是**事件**（用户明确要求它们出现在运行日志里）：
        #   手动点按钮触发，天然稀疏，且每次都有业务意义 ——
        #   "顶升开始 / 顶升完成 / 顶升失败"必须留得住、翻得回。
        #   注意后端只记"下发"和"结果"两条：中间不会刷屏。
        zh = '顶升' if cmd == 'lift' else '下降'
        self.node.log('info', f'网页触发{zh}{_with_where(data)}')
        res, err = self.node.cargo_request(cmd)
        if res is None:
            self.node.log('error', f'{zh}失败：{err}')
            return web.json_response({'ok': False, 'message': err}, status=503)
        ok = bool(res.ok)
        if ok:
            self.node.log('ok', f'{zh}完成' + (f'：{res.message}' if res.message else ''))
        else:
            self.node.log('error', f'{zh}失败：{res.message or "搬货服务拒绝"}')
        return web.json_response({'ok': ok, 'busy': bool(res.busy),
                                  'message': res.message})

    # ── 重定位 / 恢复 ──

    async def api_relocalize(self, request):
        data = await request.json()
        x, y = float(data.get('x', 0.0)), float(data.get('y', 0.0))
        yaw = float(data.get('yaw', 0.0))
        ok, msg = self.node.set_initial_pose(x, y, yaw)
        self.node.log('warn' if ok else 'error', f'网页下发重新定位：{msg}')
        return web.json_response({'ok': bool(ok), 'message': msg})

    async def api_resume(self, request):
        self.node._gw_estop = False
        rid = self.node.send_command('resume')
        self.node.log('warn', '网页请求恢复（解除急停）')
        return web.json_response({'ok': True, 'request_id': rid})

    # ── 栈状态 / 起停 / 切模式（单命令体验的核心）──

    async def api_stack(self, request):
        """查栈状态。"""
        st = self.node.stack.state() if self.node.stack is not None else {
            'mode': self.node.mode, 'mode_text': MODE_TEXT.get(self.node.mode, self.node.mode),
            'running': None, 'pid': None, 'since': 0.0,
            'error': '' if self.node.stack is not None else '本进程不管理栈（manage_stack=false）'}
        st['manage'] = self.node.stack is not None
        st['teleop_topic'] = self.node.teleop_topic
        return web.json_response({'ok': True, 'stack': st})

    async def api_stack_start(self, request):
        """启动或切换栈。

        请求体：{mode: 'nav'|'mapping', map: 地图名(可选), name: 地图名}
        ★ 整个过程要 15~40 秒（Nav2 激活），必须放线程里，
          否则会把 aiohttp 事件循环堵死，连进度日志都推不出去。

        ★★ manage_stack=false 的降级路径（本轮补的真实缺口）
        以前这里第一句就是 `if self.node.stack is None: 503`。可 manage_stack=false
        正是"用户自己在别的终端起栈、网页只当遥控器"的场景 —— 那种情况下
        「启动导航 / 开始建图」两个按钮全都会 503，而**切换遥控通道**这件事
        恰恰必须由网关做（建图栈没有 arbiter，遥控要直发 /cmd_vel_raw，
        不然车一动都不动）。少了这条路径，手工起栈的用户在网页上永远
        切不过去，一按就被拒。

        所以：没有 StackRunner 时只做"网关自己能做的那半件事"——
        切遥控通道 + 作废旧栈缓存，并如实告诉用户"栈得你自己起"。
        """
        data = await request.json()
        mode = str(data.get('mode', '')).strip().lower()
        if mode not in MODES:
            return web.json_response(
                {'ok': False, 'message': f'未知模式 {mode!r}（只支持 {MODES}）'},
                status=400)
        if self.node.stack is None:
            changed = self.node.mode != mode
            # ★ 只清一次：set_mode 内部在"模式真的变了"时已经调了
            #   reset_runtime_cache（那里还顺手清 tf 缓存）。
            #   这里再清一遍会让 epoch 连跳两格，前端收到两个 map_epoch
            #   事件、连做两次复位（视图被二次 autoCenter，屏幕会跳一下）。
            #   幂等指的是"重复清理不出错"，不代表可以重复推进代次。
            self.node.set_mode(mode)
            if not changed:
                self.node.reset_runtime_cache(f'切到{MODE_TEXT.get(mode, mode)}')
            msg = (f'本进程不管理栈（manage_stack=false），已把网关切到'
                   f'{MODE_TEXT.get(mode, mode)}、遥控发往 '
                   f'{self.node.teleop_topic}。ROS 栈请你在终端自行启动/停止。')
            self.node.log('warn', msg)
            return web.json_response({'ok': True, 'message': msg, 'mode': mode,
                                      'manage': False,
                                      'teleop_topic': self.node.teleop_topic})
        # 导航模式需要地图：可以传地图名，也可以不传（让用户后面自己选）
        kwargs = {}
        # ★ 建图：点位/路线先落到固定草稿目录。
        #   建图时用户会顺手存点，但地图名要等"保存"那一刻才定，
        #   没法直接写进某张图。保存时 MapManager.save_map 会调
        #   promote_draft 把草稿并进新地图（只补空缺，不覆盖）。
        if mode == MODE_MAPPING:
            from agv_common.map_manager import MAPPING_DRAFT   # use 草稿目录名
            mm = self.node.get_map_manager()
            if mm is not None:
                kwargs['waypoint_file'] = mm.waypoints_file(MAPPING_DRAFT)
                kwargs['routes_file'] = mm.routes_file(MAPPING_DRAFT)
        map_name = str(data.get('map') or data.get('name') or '').strip()
        if map_name:
            mm = self.node.get_map_manager()
            if mm is None:
                return web.json_response({'ok': False, 'message': '地图库不可用'},
                                         status=503)
            ok, reason = validate_map_name(mode, map_name)
            if not ok:
                return web.json_response({'ok': False, 'message': reason}, status=400)
            if mode == MODE_NAV:
                if not mm.map_exists(map_name):
                    return web.json_response(
                        {'ok': False, 'message': f'地图 {map_name} 不存在'}, status=404)
                # ★ 这里曾经是 `paths[0] if isinstance(paths, (list, tuple))
                #   else str(paths)`，而 map_paths() 返回的是 **dict**，
                #   于是 map_yaml 变成了一整个 dict 的 str()：
                #     "{'dir': '/...', 'pgm': '/...', 'yaml': '/...', ...}"
                #   map_server 拿到这个"路径"当然找不到文件，导航栈起来了
                #   但地图加载失败 —— 表现就是"有图却进不了导航"。
                #   字典就用键取，别猜类型。
                #
                # ★ 用 launch_map_yaml() 而不是 map_yaml_file()：
                #   地图名是中文时（控制台按时间自动生成「地图10月19日16时」），
                #   这个值走完 agv_stack → navigation → nav2_bringup 之后，
                #   map_server 拿到的是 "u5730u56FE…" 这种被转义过的串（实测），
                #   直接 Failed processing YAML … bad file → 导航栈起不来、
                #   网页地图全黑。launch_map_yaml() 对非 ASCII 名返回一份
                #   纯 ASCII 别名；ASCII 名（world666map）原样返回，行为不变。
                kwargs['map_yaml'] = mm.launch_map_yaml(map_name)
                kwargs['waypoint_file'] = mm.waypoints_file(map_name)
                kwargs['routes_file'] = mm.routes_file(map_name)
                self.node.current_map = map_name
        elif mode == MODE_NAV:
            # 没指定地图：如果之前选过就用之前的，否则明确报错（导航没图起不来）
            mm = self.node.get_map_manager()
            if self.node.current_map and mm is not None and mm.map_exists(self.node.current_map):
                kwargs['map_yaml'] = mm.launch_map_yaml(self.node.current_map)
                kwargs['waypoint_file'] = mm.waypoints_file(self.node.current_map)
                kwargs['routes_file'] = mm.routes_file(self.node.current_map)
            else:
                return web.json_response(
                    {'ok': False,
                     'message': '导航模式需要先选一张地图（在建图模式里建，或从地图库选）'},
                    status=400)

        loop = asyncio.get_event_loop()
        try:
            ok, msg = await loop.run_in_executor(
                None, lambda: self._do_switch(mode, kwargs))
        except Exception as exc:
            self.node.log('error', f'切换栈失败：{exc}')
            return web.json_response({'ok': False, 'message': f'切换失败：{exc}'},
                                     status=500)
        self.node.log('info' if ok else 'error', f'切到{MODE_TEXT[mode]}：{msg}')
        return web.json_response({'ok': bool(ok), 'message': msg,
                                  'mode': mode, 'stack': self.node.stack.state()})

    def _do_switch(self, mode, kwargs):
        """线程里执行：先停旧的，再起新的，最后把网关遥控通道切过去。"""
        ok, msg = self.node.stack.switch(mode, **kwargs)
        if ok:
            # ★ 必须先确认开关成功再切遥控通道：建图栈没起来就切到
            #   /cmd_vel_raw 会让遥控在导航栈下绕过仲裁，很危险。
            # ★ 换栈必须清缓存（重影 / 小车停在旧位置），但只能清一次：
            #   set_mode 在"模式变了"时内部已经清过，这里只补"模式没变、
            #   但栈被原地重启"那种情况。重复推进 epoch 会让前端连做
            #   两次复位，视图被二次 autoCenter，屏幕会跳一下。
            prev_mode = self.node.mode
            self.node.set_mode(mode)
            if prev_mode == mode:
                self.node.reset_runtime_cache(f'重启{MODE_TEXT[mode]}栈')
        return ok, msg

    async def api_stack_stop(self, request):
        if self.node.stack is None:
            return web.json_response(
                {'ok': False, 'message': '本进程不管理栈'}, status=503)
        loop = asyncio.get_event_loop()
        self.node.publish_twist(0.0, 0.0)
        ok, msg = await loop.run_in_executor(None, self.node.stack.stop)
        self.node.log('info' if ok else 'error', f'停止栈：{msg}')
        return web.json_response({'ok': bool(ok), 'message': msg})

    # ── 退出 ──

    async def api_shutdown(self, request):
        """网页上点「退出」：把整套栈停掉，然后把网关自己也退掉。

        **这个接口为什么存在**
        用户的实际抱怨是："关了网页，车还在跑/进程还在后台，下次启动一堆残留"。
        关浏览器窗口时浏览器不会等我们做完收尾，所以必须由页面主动发这个请求。

        **区分"刷新"和"退出"（关键）**
        不能一断 WS 就关机 —— 用户按 F5 刷新页面、手机切到别的 App 再回来，
        都会短暂断开连接。如果那样就停机，用起来会非常烦。
        所以：
          · 页面刷新 / 切后台 → 只断 WS，不调用本接口 → 栈继续跑；
          · 用户点「退出」按钮 / 关标签页（beforeunload 时用 sendBeacon 发）
            → 调用本接口 → 停栈 + 退出网关。
        另外 handle_signals=False 是因为我们自己在请求里调 web.run_app 的退出，
        让 aiohttp 不用再装自己的信号处理器（它会和 rclpy 抢）。

        请求体：{who: 设备标识, stop_stack: true/false}
        ★ 只允许"控制权持有者"或"没有别人持有"时关机，避免一台设备把
          另一台正在操作的车关掉。
        """
        data = {}
        try:
            data = await request.json()
        except Exception:
            pass
        who = str(data.get('who', ''))
        owner = self.node.lease.owner()
        # 别人正握着方向盘时，不许关机
        if owner and who and owner != who:
            return web.json_response(
                {'ok': False, 'message': f'控制权在「{owner}」手上，不能从这里关机'},
                status=409)

        stop_stack = data.get('stop_stack', True)
        self.node.log('warn', '收到网页退出请求：正在停止 ROS 栈并关闭网关…')
        self.node.publish_twist(0.0, 0.0)
        self.node.watchdog.idle()

        # 起个后台任务去做真正的收尾，先把 HTTP 响应发回去。
        # ★ 不能在请求处理里直接 await stop()（要 12 秒），浏览器会以为超时。
        async def _do_shutdown():
            await asyncio.sleep(0.4)        # 让响应先发出去
            if stop_stack and self.node.stack is not None and self.node.stack.is_running():
                loop = asyncio.get_running_loop()
                try:
                    await loop.run_in_executor(None, lambda: self.node.stack.stop(10.0))
                except Exception as exc:
                    self.node.log('error', f'退出时停栈失败：{exc}')
            self.node.publish_twist(0.0, 0.0)
            self.node.log('warn', '网关即将退出')

            # ★ 退出方式踩了两次坑，记下来别重犯：
            #   ① `raise SystemExit(0)` 在 Task 里只会让这个任务以异常结束，
            #      run_app 的 run_forever 照旧转圈 —— 完全没用。
            #   ② `loop.stop()` 也不行：aiohttp 的 run_app 内部有 try/finally
            #      要 await runner.cleanup()，一旦循环被硬停，那些 await 永远
            #      等不到，进程就卡在退出路径上（实测 30 秒都退不掉，日志里
            #      "网关即将退出"打了但进程还在）。
            #   最终用"向自己发 SIGINT"：Python 的默认处理器把它变成
            #   KeyboardInterrupt，能从协程里一路穿出去，run_app 的 finally
            #   正常执行 runner.cleanup()，再落到 main() 的 finally 停栈 +
            #   rclpy.shutdown()。这条链是 aiohttp 官方支持的退出路径。
            #   用 SIGTERM 不行 —— 它的默认动作是直接终止，不跑 finally。
            await asyncio.sleep(0.1)         # 让上面的日志先刷出去
            os.kill(os.getpid(), signal.SIGINT)

        asyncio.ensure_future(_do_shutdown())
        return web.json_response({'ok': True, 'message': '正在停止并退出…'})