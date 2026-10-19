"""
store_dispatch 单元测试：统一存储服务的调度逻辑（点位 / 路线 × save/get/list/delete）。

对应接口 AgvStore.srv。这些测试取代了原先散在 8 个服务里的重复校验，
重点验证"两资源共用一套回执规则"，避免再出现"这个服务有 ok、那个没有"。
"""

from agv_waypoint.route_store import RouteStore
from agv_waypoint.store_dispatch import dispatch
from agv_waypoint.waypoint_store import WaypointStore


def make_stores(tmp_path):               # # 造一对空存储（用临时目录，不碰真实文件）
    wp = WaypointStore(str(tmp_path / 'wp.yaml'), 'map')
    rt = RouteStore(str(tmp_path / 'rt.yaml'), 'map')
    return wp, rt


def req(resource, action, **kw):         # # 造一个请求 dict
    base = {'resource': resource, 'action': action, 'name': '',
            'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'kind': '', 'point_names': []}
    base.update(kw)
    return base


# ══════════════════ 资源 / 动作合法性 ══════════════════

def test_unknown_resource_rejected(tmp_path):    # # 未知资源：拒绝且给出提示
    wp, rt = make_stores(tmp_path)
    out = dispatch(req('shelf', 'list'), wp, rt)
    assert out['ok'] is False and '未知资源' in out['message']


def test_unknown_action_rejected(tmp_path):      # # 未知动作：拒绝
    wp, rt = make_stores(tmp_path)
    out = dispatch(req('waypoint', 'rename'), wp, rt)
    assert out['ok'] is False and '未知动作' in out['message']


def test_response_always_has_ok(tmp_path):       # # ★ 每个动作都带 ok（含 list）
    wp, rt = make_stores(tmp_path)
    for action in ('save', 'get', 'list', 'delete'):
        out = dispatch(req('waypoint', action, name='A'), wp, rt)
        assert 'ok' in out and isinstance(out['ok'], bool)
    for action in ('save', 'get', 'list', 'delete'):
        out = dispatch(req('route', action, name='R'), wp, rt)
        assert 'ok' in out and isinstance(out['ok'], bool)


# ══════════════════ 点位 ══════════════════

def test_waypoint_save_then_get(tmp_path):       # # 存了能取回来
    wp, rt = make_stores(tmp_path)
    out = dispatch(req('waypoint', 'save', name='A', x=1.0, y=2.0, yaw=0.5,
                       kind='cargo'), wp, rt)
    assert out['ok'] is True
    got = dispatch(req('waypoint', 'get', name='A'), wp, rt)
    assert got['ok'] is True and got['count'] == 1
    p = got['items'][0]
    assert p['name'] == 'A' and p['x'] == 1.0 and p['y'] == 2.0
    assert p['kind'] == 'cargo'


def test_waypoint_get_missing_not_error(tmp_path):   # # 不存在：ok=false，不抛异常
    wp, rt = make_stores(tmp_path)
    out = dispatch(req('waypoint', 'get', name='没有这个点'), wp, rt)
    assert out['ok'] is False and '不存在' in out['message']


def test_waypoint_save_requires_name(tmp_path):  # # 空名字拒绝
    wp, rt = make_stores(tmp_path)
    out = dispatch(req('waypoint', 'save', name=''), wp, rt)
    assert out['ok'] is False


def test_waypoint_list_sorted_and_complete(tmp_path):   # # 列表稳定有序、字段齐
    wp, rt = make_stores(tmp_path)
    for n in ('C', 'A', 'B'):
        dispatch(req('waypoint', 'save', name=n, x=1.0, y=1.0), wp, rt)
    out = dispatch(req('waypoint', 'list'), wp, rt)
    assert out['ok'] is True and out['count'] == 3
    assert out['names'] == ['A', 'B', 'C']       # # 按名排序
    assert len(out['items']) == 3
    assert all(set(p) == {'name', 'x', 'y', 'yaw', 'kind'} for p in out['items'])


def test_waypoint_delete(tmp_path):              # # 删除；再删同一点 ok=false
    wp, rt = make_stores(tmp_path)
    dispatch(req('waypoint', 'save', name='A'), wp, rt)
    out = dispatch(req('waypoint', 'delete', name='A'), wp, rt)
    assert out['ok'] is True
    again = dispatch(req('waypoint', 'delete', name='A'), wp, rt)
    assert again['ok'] is False


# ══════════════════ 路线 ══════════════════

def test_route_save_snapshots_poses(tmp_path):   # # 路线存的是位姿快照
    wp, rt = make_stores(tmp_path)
    dispatch(req('waypoint', 'save', name='A', x=1.0, y=1.0, yaw=0.1), wp, rt)
    dispatch(req('waypoint', 'save', name='B', x=2.0, y=2.0, yaw=0.2), wp, rt)
    out = dispatch(req('route', 'save', name='R', point_names=['A', 'B']), wp, rt)
    assert out['ok'] is True and out['count'] == 2

    # 改点位不应影响已存路线（快照语义）
    dispatch(req('waypoint', 'save', name='A', x=9.0, y=9.0), wp, rt)
    got = dispatch(req('route', 'get', name='R'), wp, rt)
    assert [p['x'] for p in got['items']] == [1.0, 2.0]


def test_route_empty_rejected(tmp_path):         # # 空路线拒绝，不留空壳
    wp, rt = make_stores(tmp_path)
    out = dispatch(req('route', 'save', name='R', point_names=[]), wp, rt)
    assert out['ok'] is False and '至少' in out['message']
    assert dispatch(req('route', 'get', name='R'), wp, rt)['ok'] is False


def test_route_missing_point_rejects_whole(tmp_path):   # # 有点名不存在：整条拒绝，不写半条
    wp, rt = make_stores(tmp_path)
    dispatch(req('waypoint', 'save', name='A'), wp, rt)
    out = dispatch(req('route', 'save', name='R', point_names=['A', '缺的点']), wp, rt)
    assert out['ok'] is False and '不存在' in out['message']
    assert dispatch(req('route', 'get', name='R'), wp, rt)['ok'] is False


def test_route_save_requires_name(tmp_path):     # # 路线名不能空
    wp, rt = make_stores(tmp_path)
    dispatch(req('waypoint', 'save', name='A'), wp, rt)
    out = dispatch(req('route', 'save', name='', point_names=['A']), wp, rt)
    assert out['ok'] is False


def test_route_list_names_and_counts(tmp_path):  # # 列表给出名字与各条点数（同序）
    wp, rt = make_stores(tmp_path)
    for n in ('A', 'B', 'C'):
        dispatch(req('waypoint', 'save', name=n), wp, rt)
    dispatch(req('route', 'save', name='R1', point_names=['A']), wp, rt)
    dispatch(req('route', 'save', name='R2', point_names=['A', 'B', 'C']), wp, rt)
    out = dispatch(req('route', 'list'), wp, rt)
    assert out['ok'] is True and out['count'] == 2
    assert out['names'] == ['R1', 'R2']
    assert out['counts'] == [1, 3]               # # 与 names 同序


def test_route_delete(tmp_path):                 # # 删除路线
    wp, rt = make_stores(tmp_path)
    dispatch(req('waypoint', 'save', name='A'), wp, rt)
    dispatch(req('route', 'save', name='R', point_names=['A']), wp, rt)
    assert dispatch(req('route', 'delete', name='R'), wp, rt)['ok'] is True
    assert dispatch(req('route', 'delete', name='R'), wp, rt)['ok'] is False


def test_route_get_items_have_empty_kind(tmp_path):   # # 途经点 kind 留空（不是点位）
    wp, rt = make_stores(tmp_path)
    dispatch(req('waypoint', 'save', name='A', x=1.0, y=1.0, kind='nav'), wp, rt)
    dispatch(req('route', 'save', name='R', point_names=['A']), wp, rt)
    got = dispatch(req('route', 'get', name='R'), wp, rt)
    assert got['items'][0]['kind'] == ''


def test_waypoint_and_route_do_not_collide(tmp_path):   # # 同名点位与路线互不干扰
    wp, rt = make_stores(tmp_path)
    dispatch(req('waypoint', 'save', name='同名', x=1.0, y=1.0), wp, rt)
    dispatch(req('route', 'save', name='同名', point_names=['同名']), wp, rt)
    assert dispatch(req('waypoint', 'get', name='同名'), wp, rt)['ok'] is True
    route = dispatch(req('route', 'get', name='同名'), wp, rt)
    assert route['ok'] is True and route['count'] == 1
    assert dispatch(req('waypoint', 'delete', name='同名'), wp, rt)['ok'] is True
    assert dispatch(req('route', 'get', name='同名'), wp, rt)['ok'] is True   # # 路线还在


# ══════════════════ 响应字段取值矩阵（防契约漂移） ══════════════════
# AgvStore.srv 的注释里写了一张 (resource, action) × 字段 的矩阵；
# 下面几条把它钉住，免得实现悄悄改了而文档/调用方还以为有值。

def _seed(tmp_path):                     # # 造一份有两个点位、一条路线的数据
    wp, rt = make_stores(tmp_path)
    dispatch(req('waypoint', 'save', name='A', x=1.0, y=1.0), wp, rt)
    dispatch(req('waypoint', 'save', name='B', x=2.0, y=2.0), wp, rt)
    dispatch(req('route', 'save', name='R', point_names=['A', 'B']), wp, rt)
    return wp, rt


def test_route_list_items_is_empty_by_design(tmp_path):   # # ★ route+list 不填 items
    # items 里每项的 name 是"途经点名"，无法表达"哪条路线的哪些点"这层归属；
    # 硬填会让调用方误以为 name 是路线名。要看途经点请用 route+get。
    wp, rt = _seed(tmp_path)
    out = dispatch(req('route', 'list'), wp, rt)
    assert out['ok'] is True
    assert out['items'] == []                 # # 故意为空
    assert out['names'] == ['R']              # # 名字与个数走 names/counts
    assert out['counts'] == [2]
    assert out['count'] == 1


def test_waypoint_list_fills_items(tmp_path):   # # waypoint+list 填 items（与 route 不一致，是刻意的）
    wp, rt = _seed(tmp_path)
    out = dispatch(req('waypoint', 'list'), wp, rt)
    assert out['ok'] is True
    assert [p['name'] for p in out['items']] == ['A', 'B']
    assert out['counts'] == [1, 1]            # # 点位恒为 1 个位姿


def test_get_fills_items_and_count(tmp_path):   # # get 填 items 与 count
    wp, rt = _seed(tmp_path)
    w = dispatch(req('waypoint', 'get', name='A'), wp, rt)
    assert w['count'] == 1 and len(w['items']) == 1
    r = dispatch(req('route', 'get', name='R'), wp, rt)
    assert r['count'] == 2 and len(r['items']) == 2


def test_save_and_delete_leave_items_empty(tmp_path):   # # save/delete 不填 items
    wp, rt = _seed(tmp_path)
    for resource, kw in (('waypoint', {'x': 3.0, 'y': 3.0}), ('route', {'point_names': ['A']})):
        out = dispatch(req(resource, 'save', name='新', **kw), wp, rt)
        assert out['ok'] is True and out['items'] == [], f'{resource}+save 不该填 items'
    for resource in ('waypoint', 'route'):
        out = dispatch(req(resource, 'delete', name='新'), wp, rt)
        assert out['ok'] is True and out['items'] == [], f'{resource}+delete 不该填 items'


def test_every_action_reports_ok_bool(tmp_path):   # # ok 恒为 bool，且失败也为 False 而非 None
    wp, rt = _seed(tmp_path)
    cases = [
        req('waypoint', 'save', name='C', x=0.0, y=0.0),
        req('waypoint', 'get', name='C'),
        req('waypoint', 'list'),
        req('waypoint', 'delete', name='不存在的点'),
        req('route', 'save', name='R2', point_names=['A']),
        req('route', 'get', name='不存在的路线'),
        req('route', 'list'),
        req('route', 'delete', name='不存在的路线'),
        req('bogus', 'list'),
        req('waypoint', 'bogus'),
    ]
    for c in cases:
        out = dispatch(c, wp, rt)
        assert isinstance(out['ok'], bool), f'{c["resource"]}/{c["action"]} 的 ok 不是 bool'
        assert isinstance(out['message'], str) and out['message']

