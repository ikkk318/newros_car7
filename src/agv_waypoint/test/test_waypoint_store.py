"""
WaypointStore 单元测试：增删查、类型、重名覆盖、持久化与坏文件容错。
"""

import os

from agv_waypoint.waypoint_store import KIND_CARGO, KIND_NAV, WaypointStore


def test_save_get(tmp_path):             # # 存了能查到（含类型）
    store = WaypointStore(str(tmp_path / 'wp.yaml'))
    assert store.save('货架A', 3.0, 1.2, 0.0) is True
    pose = store.get('货架A')
    assert pose == {'x': 3.0, 'y': 1.2, 'yaw': 0.0, 'kind': KIND_CARGO}


def test_save_nav_kind(tmp_path):        # # 导航点类型能存能取
    store = WaypointStore(str(tmp_path / 'wp.yaml'))
    store.save('走廊1', 1.0, 2.0, 0.5, kind=KIND_NAV)
    assert store.kind('走廊1') == KIND_NAV
    assert store.names_of_kind(KIND_NAV) == ['走廊1']
    assert store.names_of_kind(KIND_CARGO) == []


def test_bad_kind_falls_back(tmp_path):  # # 非法类型按搬货点处理，不报错
    store = WaypointStore(str(tmp_path / 'wp.yaml'))
    store.save('X', 0.0, 0.0, 0.0, kind='乱写')
    assert store.kind('X') == KIND_CARGO


def test_save_bad_coords_rejected(tmp_path):   # # 坐标非法：不写盘、返回 False
    store = WaypointStore(str(tmp_path / 'wp.yaml'))
    assert store.save('X', 'a', 0.0, 0.0) is False
    assert store.count() == 0


def test_save_overwrite(tmp_path):       # # 重名直接覆盖
    store = WaypointStore(str(tmp_path / 'wp.yaml'))
    store.save('A', 1.0, 1.0, 0.0)
    store.save('A', 9.0, 9.0, 1.5, kind=KIND_NAV)
    assert store.get('A')['x'] == 9.0
    assert store.kind('A') == KIND_NAV
    assert store.count() == 1


def test_delete_missing_is_false(tmp_path):   # # 删不存在返回 False，不抛异常
    store = WaypointStore(str(tmp_path / 'wp.yaml'))
    assert store.delete('没有这个点') is False


def test_get_missing_is_none(tmp_path):  # # 查不存在返回 None
    store = WaypointStore(str(tmp_path / 'wp.yaml'))
    assert store.get('nope') is None


def test_list_empty(tmp_path):           # # 空表列点返回空列表
    store = WaypointStore(str(tmp_path / 'wp.yaml'))
    assert store.list_names() == []
    assert store.all() == {}


def test_persistence(tmp_path):          # # 重启后点位与类型都还在
    path = str(tmp_path / 'wp.yaml')
    s1 = WaypointStore(path)
    s1.save('取货点', 0.5, -1.0, 1.57)
    s1.save('走廊A', -2.0, 3.0, -0.5, kind=KIND_NAV)

    s2 = WaypointStore(path)             # # 重新构造：模拟重启
    assert s2.count() == 2
    assert s2.get('走廊A') == {'x': -2.0, 'y': 3.0, 'yaw': -0.5, 'kind': KIND_NAV}
    assert s2.list_names() == ['取货点', '走廊A']   # # 排序稳定


def test_get_returns_copy(tmp_path):     # # 外部改返回值不污染内存
    store = WaypointStore(str(tmp_path / 'wp.yaml'))
    store.save('A', 1.0, 2.0, 3.0)
    pose = store.get('A')
    pose['x'] = 999.0
    assert store.get('A')['x'] == 1.0


def test_corrupt_file_tolerated(tmp_path):   # # 坏文件不崩，降级为空表
    p = tmp_path / 'wp.yaml'
    p.write_text('这不是: [合法的 yaml: 结构\n', encoding='utf-8')
    store = WaypointStore(str(p))        # # 构造不应抛异常
    assert store.list_names() == []


def test_missing_file_ok(tmp_path):      # # 文件不存在也能构造
    store = WaypointStore(str(tmp_path / 'not_exist.yaml'))
    assert store.count() == 0


def test_old_file_without_kind_compatible(tmp_path):   # # 老点位文件（无 kind）仍可用
    p = tmp_path / 'wp.yaml'
    p.write_text('frame_id: map\nwaypoints:\n  旧点: {x: 1.0, y: 2.0, yaw: 0.0}\n',
                 encoding='utf-8')
    store = WaypointStore(str(p))
    assert store.kind('旧点') == KIND_CARGO


# ══════════════════════════════════════════════════════════════
#  外部改动自动重载（和 RouteStore 同源，见 test_route_store.py 的说明）
# ══════════════════════════════════════════════════════════════

def _bump_mtime(path):                   # # 把 mtime 拨后，规避 1 秒精度
    import time
    st = os.stat(path)
    os.utime(path, (st.st_atime, time.time() + 2))


def test_reloads_external_waypoint(tmp_path):
    """终端新建的点位，网页这边要能立刻看到。"""
    p = str(tmp_path / 'wp.yaml')
    web = WaypointStore(p)
    web.save('网页点', 1.0, 1.0, 0.0)

    console = WaypointStore(p)           # 扮演终端控制台
    console.save('终端点', 2.0, 2.0, 0.0)
    _bump_mtime(p)

    assert set(web.list_names()) == {'网页点', '终端点'}


def test_save_does_not_wipe_external_points(tmp_path):
    """网页存点时不能把终端刚加的点冲掉（数据丢失级 bug）。"""
    p = str(tmp_path / 'wp.yaml')
    web = WaypointStore(p)
    web.save('网页点', 1.0, 1.0, 0.0)

    console = WaypointStore(p)
    console.save('终端点', 2.0, 2.0, 0.0)
    _bump_mtime(p)

    web.save('网页又一点', 3.0, 3.0, 0.0)

    fresh = WaypointStore(p)
    assert set(fresh.list_names()) == {'网页点', '终端点', '网页又一点'}
    assert fresh.get('终端点')['x'] == 2.0


def test_get_sees_external_move_of_same_point(tmp_path):
    """同名点被别人挪了位置，要取到新坐标。"""
    p = str(tmp_path / 'wp.yaml')
    web = WaypointStore(p)
    web.save('货架A', 1.0, 1.0, 0.0)

    console = WaypointStore(p)
    console.save('货架A', 9.0, 9.0, 1.57)
    _bump_mtime(p)

    assert web.get('货架A')['x'] == 9.0


def test_delete_keeps_external_points(tmp_path):
    """删一个点时，别人新增的点必须留着。"""
    p = str(tmp_path / 'wp.yaml')
    web = WaypointStore(p)
    web.save('A', 0.0, 0.0, 0.0)

    console = WaypointStore(p)
    console.save('B', 5.0, 5.0, 0.0)
    _bump_mtime(p)

    assert web.delete('A') is True
    fresh = WaypointStore(p)
    assert fresh.list_names() == ['B']
