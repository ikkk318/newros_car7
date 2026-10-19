"""
web_app.py —— 把 handler 装成 aiohttp 应用：中间件、路由表、`make_app()`。

**结构**
    guard_middleware   口令 + IP 白名单（唯一中间件）
    ApiBase            页面/登录/状态快照/地图 PNG/日志/WS —— 不碰业务动作
    ApiOps             （handlers_ops.py）遥控、指令、栈起停、退出
    ApiData            （handlers_data.py）地图库、任务、路线、自检
    ROUTE_SPECS        26 条路由，`(方法, 路径, owner, handler 名)`
    make_app(node)     建 app → 实例化三个 handler → 按表注册 → 挂静态目录

**为什么路由要单独列一张表**
以前是 26 条 `app.router.add_xxx(...)` 散在 800 行函数末尾，改一个路径要
滚很久才找得到；更要紧的是没法一眼数清"到底几个接口"。现在表就是清单，
`len(ROUTE_SPECS)` 可以直接断言，文档也能照着核。

**`*` 而不是 GET/POST 的那几条**
`/api/maps`、`/api/tasks`、`/api/routes`、`/api/check`、`/api/stack` 原来
只注册了一半方法，而前端 `api()` 助手默认发 POST —— 于是 POST /api/maps
吃 405，前端 catch 后当成"地图库是空的"，用户看到的现象是
"必须先建图 → 建完还是不行"，极难查。所以这几个统一 GET/POST 都收。
"""

from aiohttp import web

from agv_common.web_guard import BAD_TOKEN

from .util import STATIC_DIR
from .handlers_data import ApiData
from .handlers_ops import ApiOps


class ApiBase:
    """基础接口：页面、登录、状态快照、地图 PNG、日志、WebSocket。

    ★ 这类 handler 只"读状态"或"发页面"，不发起任何业务动作 ——
      动作类全在 `handlers_ops.py` / `handlers_data.py`，
      这样"打开网页会不会把车动起来"这种问题一眼能答。
    """

    def __init__(self, node):
        self.node = node


    async def index(self, request):
        return web.FileResponse(str(STATIC_DIR / 'index.html'))

    async def login_page(self, request):
        return web.FileResponse(str(STATIC_DIR / 'index.html'))

    async def api_login(self, request):
        data = await request.json()
        ok, code = self.node.auth.check(request.remote or '', data.get('token', ''))
        if not ok:
            return web.json_response({'ok': False, 'code': code,
                                      'message': '口令错误' if code == BAD_TOKEN else 'IP 不允许'},
                                     status=403)
        resp = web.json_response({'ok': True})
        resp.set_cookie('agv_token', self.node.auth.token or '', httponly=True,
                        samesite='Lax', max_age=86400)
        return resp

    async def api_state(self, request):
        return web.json_response(self.node.snapshot())

    async def api_map_png(self, request):
        with self.node._gw_lock:
            png = self.node._gw_map_png
            version = self.node._gw_map_version
        if png is None:
            return web.Response(status=404, text='地图尚未收到')
        return web.Response(body=png, content_type='image/png',
                            headers={'X-Map-Version': str(version),
                                     'Cache-Control': 'no-store'})

    async def api_logs(self, request):
        with self.node._gw_lock:
            logs = list(self.node._gw_logs)
        return web.json_response({'logs': logs})


    async def ws_handler(self, request):
        ws = web.WebSocketResponse(heartbeat=15.0)
        await ws.prepare(request)
        self.node._gw_ws_clients.add(ws)
        try:
            last_ver = -1
            last_epoch = -1
            while not ws.closed:
                snap = self.node.snapshot()
                await ws.send_json({'type': 'state', 'data': snap})
                # ★ 代次（换栈）优先于版本（同一套栈里的地图变化）。
                #   代次变了要让前端"作废一切"，所以单独发一个事件，
                #   不能只靠 map_changed —— 那个的语义是"图更新了，去重载"。
                if snap['map_epoch'] != last_epoch:
                    last_epoch = snap['map_epoch']
                    last_ver = -1               # 代次变了，版本比较要重来
                    await ws.send_json({'type': 'map_epoch', 'epoch': last_epoch})
                if snap['map_version'] != last_ver:
                    last_ver = snap['map_version']
                    await ws.send_json({'type': 'map_changed',
                                        'version': last_ver})
                with self.node._gw_lock:
                    logs = list(self.node._gw_logs)[-40:]
                await ws.send_json({'type': 'logs', 'data': logs})
                await asyncio.sleep(0.25)      # # 4 Hz 推送，手机流量也够省
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            self.node.log('warn', f'WebSocket 断开：{exc}')
        finally:
            self.node._gw_ws_clients.discard(ws)
        return ws


# ══════════════════ 准入中间件：口令 + IP 白名单 ══════════════════
@web.middleware
async def guard_middleware(request, handler):
    node = request.app['node']

    ip = request.remote or ''
    # token 可以放在 query 或 Cookie 里（登录页写 Cookie，之后自动带上）
    token = request.query.get('token') or request.cookies.get('agv_token') or ''
    # ★ 静态资源、登录页、【以及登录接口本身】都要放行。
    #   把 /api/login 也放行是关键：用户第一次打开页面时浏览器还没有 Cookie，
    #   若连登录接口都要先有口令才放行，就形成死锁——
    #   永远 403「口令错误」，那个口令明明是对的，人也登不进来。
    #   （这是真机上踩到的死锁 bug：登录接口被自己的认证中间件拦住了。）
    if (request.path in ('/', '/login', '/api/login')
            or request.path.startswith('/static/')):
        ok, code = node.auth.check_ip(ip)
        if not ok:
            return web.json_response({'ok': False, 'code': code,
                                      'message': f'来源 IP {ip} 不在白名单'}, status=403)
        return await handler(request)
    ok, code = node.auth.check(ip, token)
    if not ok:
        status = 403
        return web.json_response({'ok': False, 'code': code,
                                  'message': '口令错误' if code == BAD_TOKEN
                                             else f'来源 IP {ip} 不在白名单'},
                                 status=status)
    return await handler(request)


# ══════════════════════════ 路由表：26 条 + 静态目录 ══════════════════════════
# (HTTP 方法, 路径, 归属 handler 对象, 方法名)
#   '*' = GET/POST 都收（原因见文件头）
ROUTE_SPECS = (
    ('GET',  '/',                'base', 'index'),
    ('GET',  '/login',           'base', 'login_page'),
    ('POST', '/api/login',       'base', 'api_login'),
    ('GET',  '/api/state',       'base', 'api_state'),
    ('GET',  '/api/map.png',     'base', 'api_map_png'),
    ('GET',  '/api/logs',        'base', 'api_logs'),
    ('POST', '/api/lease',       'ops',  'api_lease'),
    ('POST', '/api/teleop',      'ops',  'api_teleop'),
    ('POST', '/api/stop',        'ops',  'api_stop'),
    ('POST', '/api/command',     'ops',  'api_command'),
    ('POST', '/api/store',       'ops',  'api_store'),
    ('POST', '/api/cargo',       'ops',  'api_cargo'),
    ('POST', '/api/relocalize',  'ops',  'api_relocalize'),
    ('*',    '/api/maps',        'data', 'api_maps'),
    ('POST', '/api/map_switch',  'data', 'api_map_switch'),
    ('POST', '/api/map_delete',  'data', 'api_map_delete'),
    ('*',    '/api/tasks',       'data', 'api_tasks'),
    ('*',    '/api/routes',      'data', 'api_routes'),
    ('*',    '/api/check',       'data', 'api_check'),
    ('POST', '/api/resume',      'ops',  'api_resume'),
    ('POST', '/api/map_save',    'data', 'api_map_save'),
    ('*',    '/api/stack',       'ops',  'api_stack'),
    ('POST', '/api/stack_start', 'ops',  'api_stack_start'),
    ('POST', '/api/stack_stop',  'ops',  'api_stack_stop'),
    ('POST', '/api/shutdown',    'ops',  'api_shutdown'),
    ('GET',  '/ws',              'base', 'ws_handler'),
)


def make_app(node):
    """构造 aiohttp 应用。路由与 node 分离，便于只核对路由而不必真起 HTTP。"""
    app = web.Application(middlewares=[guard_middleware])
    app['node'] = node

    owners = {'base': ApiBase(node), 'ops': ApiOps(node), 'data': ApiData(node)}
    for http_method, path, owner, name in ROUTE_SPECS:
        app.router.add_route(http_method, path, getattr(owners[owner], name))
    app.router.add_static('/static/', STATIC_DIR)
    return app
