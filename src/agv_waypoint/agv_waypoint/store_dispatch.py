"""
store_dispatch —— 点位/路线存储的统一调度（纯 Python，不碰 ROS，可单独测试）。

对应接口 AgvStore.srv：把「resource + action」翻译成对 WaypointStore / RouteStore
的具体调用，并生成中文回执。

为什么把这段逻辑从 ROS 节点里拉出来：
    原先 8 个 CRUD 服务各自在 waypoint_node 里写一遍「校验 → 调存储 → 拼回执」，
    既重复又只能靠起 ROS 才能验证。现在调度逻辑是纯函数，
    waypoint_node 只做「srv 字段 ↔ dict」的搬运，逻辑本身可单测。

约定：本模块只认 dict，不认 ROS 消息。
    请求 dict：{'resource','action','name','x','y','yaw','kind','point_names'}
    响应 dict：{'ok','message','items','names','counts','count'}
    items 里每项是 {'name','x','y','yaw','kind'}
"""

# 合法资源与动作
RESOURCES = ('waypoint', 'route')          # def 资源类型
ACTIONS = ('save', 'get', 'list', 'delete')  # def 动作


# 构造一条空响应（字段齐全，避免调用方 KeyError）
# 参数：无
# 返回：dict
def _empty():                              # in 内部方法
    return {'ok': False, 'message': '', 'items': [],
            'names': [], 'counts': [], 'count': 0}


# 调度一次请求
# 参数：req - 请求 dict；wp - WaypointStore；rt - RouteStore
# 返回：响应 dict（ok/message/items/names/counts/count）
def dispatch(req, wp, rt):                 # out 对外接口
    res = str((req or {}).get('resource', '') or '').strip().lower()
    act = str((req or {}).get('action', '') or '').strip().lower()
    name = str((req or {}).get('name', '') or '').strip()

    # ① 资源/动作合法性
    if res not in RESOURCES:
        out = _empty()
        out['message'] = f'未知资源「{res}」，应为 waypoint 或 route'
        return out
    if act not in ACTIONS:
        out = _empty()
        out['message'] = f'未知动作「{act}」，应为 save / get / list / delete'
        return out

    if res == 'waypoint':                  # # 点位：1 个位姿的资源
        return _waypoint(act, name, req, wp)
    return _route(act, name, req, wp, rt)  # # 路线：N 个位姿的资源


# 点位类动作
# 参数：act - 动作；name - 点名；req - 请求 dict；wp - WaypointStore
# 返回：响应 dict
def _waypoint(act, name, req, wp):         # in 内部方法
    out = _empty()

    if act == 'save':                      # # 存/覆盖
        if not name:
            out['message'] = '点名不能为空'
            return out
        ok = wp.save(name, req.get('x', 0.0), req.get('y', 0.0),
                     req.get('yaw', 0.0), req.get('kind', ''))
        out['ok'] = bool(ok)
        out['message'] = (f'已保存点位「{name}」（{wp.kind(name)}）'
                          if ok else '保存失败：坐标非法或无法写入文件')
        return out

    if act == 'get':                       # # 按名查一个
        pose = wp.get(name)
        if pose is None:
            out['message'] = f'点位「{name}」不存在'
            return out
        out['ok'] = True
        out['count'] = 1
        out['items'] = [{'name': name, 'x': pose['x'], 'y': pose['y'],
                         'yaw': pose['yaw'], 'kind': pose['kind']}]
        out['message'] = f'点位「{name}」已找到'
        return out

    if act == 'list':                      # # 列全部（按名排序，顺序稳定）
        names = wp.list_names()
        out['ok'] = True
        out['count'] = len(names)
        out['names'] = names
        out['counts'] = [1] * len(names)   # # 点位恒为 1 个位姿
        out['items'] = []
        for n in names:
            pose = wp.get(n)
            out['items'].append({'name': n, 'x': pose['x'], 'y': pose['y'],
                                 'yaw': pose['yaw'], 'kind': pose['kind']})
        out['message'] = f'共 {len(names)} 个点位'
        return out

    # act == 'delete'
    if not name:
        out['message'] = '点名不能为空'
        return out
    ok = wp.delete(name)                   # # 不存在返回 False，不报错
    out['ok'] = bool(ok)
    out['message'] = (f'已删除点位「{name}」' if ok else f'点位「{name}」不存在')
    return out


# 路线类动作
# 参数：act - 动作；name - 路线名；req - 请求 dict；wp - 点位存储；rt - 路线存储
# 返回：响应 dict
def _route(act, name, req, wp, rt):        # in 内部方法
    out = _empty()

    if act == 'save':                      # # 存/覆盖：按点名从点位表复制位姿快照
        wanted = [str(p).strip() for p in (req.get('point_names') or [])
                  if str(p).strip()]
        if not name:
            out['message'] = '路线名不能为空'
            return out
        if not wanted:                     # # 空路线：拒绝，避免留下空壳
            out['message'] = '路线至少要有一个途经点'
            return out
        missing = [p for p in wanted if wp.get(p) is None]   # use 校验点名都存在
        if missing:                        # # 有缺失：整条拒绝，不写半条
            out['message'] = '这些点位不存在：' + '、'.join(missing)
            return out
        points = []                        # def 途经点快照
        for pname in wanted:
            pose = wp.get(pname)
            points.append({'name': pname, 'x': pose['x'],
                           'y': pose['y'], 'yaw': pose['yaw']})
        ok = rt.save(name, points)
        out['ok'] = bool(ok)
        out['count'] = len(points) if ok else 0
        out['message'] = (f'已保存路线「{name}」，共 {len(points)} 个途经点'
                          if ok else '保存失败：无法写入文件')
        return out

    if act == 'get':                       # # 查一条路线的途经点
        pts = rt.get(name)
        if not pts:
            out['message'] = f'路线「{name}」不存在'
            return out
        out['ok'] = True
        out['count'] = len(pts)
        out['items'] = [{'name': p['name'], 'x': p['x'], 'y': p['y'],
                         'yaw': p['yaw'], 'kind': ''} for p in pts]
        out['message'] = f'路线「{name}」共 {len(pts)} 个途经点'
        return out

    if act == 'list':                      # # 列全部路线
        names = rt.list_names()
        counts = rt.counts()
        out['ok'] = True
        out['count'] = len(names)
        out['names'] = names
        out['counts'] = [int(counts.get(n, 0)) for n in names]
        out['message'] = f'共 {len(names)} 条路线'
        return out

    # act == 'delete'
    if not name:
        out['message'] = '路线名不能为空'
        return out
    ok = rt.delete(name)
    out['ok'] = bool(ok)
    out['message'] = (f'已删除路线「{name}」' if ok else f'路线「{name}」不存在')
    return out
