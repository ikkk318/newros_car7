"""
RouteStore 单元测试：存/查/删、顺序保持、覆盖、持久化与坏文件容错。
"""

from agv_waypoint.route_store import RouteStore


def pts(*names):                         # # 造一串途经点
    return [{'name': n, 'x': float(i), 'y': float(i * 2), 'yaw': 0.0}
            for i, n in enumerate(names)]


def test_save_get_order(tmp_path):       # # 存了能查到，且顺序保持
    s = RouteStore(str(tmp_path / 'rt.yaml'))
    assert s.save('巡检A', pts('走廊1', '走廊2', '走廊3')) is True
    got = s.get('巡检A')
    assert [p['name'] for p in got] == ['走廊1', '走廊2', '走廊3']


def test_overwrite(tmp_path):            # # 重名覆盖
    s = RouteStore(str(tmp_path / 'rt.yaml'))
    s.save('A', pts('p1', 'p2'))
    s.save('A', pts('p9'))
    assert [p['name'] for p in s.get('A')] == ['p9']
    assert s.count() == 1


def test_empty_rejected(tmp_path):       # # 空名/空路线拒绝
    s = RouteStore(str(tmp_path / 'rt.yaml'))
    assert s.save('', pts('p1')) is False
    assert s.save('A', []) is False
    assert s.count() == 0


def test_delete_missing_false(tmp_path):  # # 删不存在返回 False
    s = RouteStore(str(tmp_path / 'rt.yaml'))
    assert s.delete('没有') is False


def test_get_missing_none(tmp_path):     # # 查不存在返回 None
    s = RouteStore(str(tmp_path / 'rt.yaml'))
    assert s.get('没有') is None


def test_list_and_counts(tmp_path):      # # 列名与计数
    s = RouteStore(str(tmp_path / 'rt.yaml'))
    s.save('B', pts('p1', 'p2'))
    s.save('A', pts('p1'))
    assert s.list_names() == ['A', 'B']
    assert s.counts() == {'A': 1, 'B': 2}


def test_persistence(tmp_path):          # # 重启后路线还在
    path = str(tmp_path / 'rt.yaml')
    s1 = RouteStore(path)
    s1.save('巡检A', pts('走廊1', '走廊2'))

    s2 = RouteStore(path)
    assert s2.count() == 1
    assert [p['name'] for p in s2.get('巡检A')] == ['走廊1', '走廊2']
    assert s2.get('巡检A')[1]['x'] == 1.0


def test_get_returns_copy(tmp_path):     # # 外部改返回值不污染内存
    s = RouteStore(str(tmp_path / 'rt.yaml'))
    s.save('A', pts('p1'))
    got = s.get('A')
    got[0]['x'] = 999.0
    assert s.get('A')[0]['x'] == 0.0


def test_corrupt_file_tolerated(tmp_path):   # # 坏文件不崩
    p = tmp_path / 'rt.yaml'
    p.write_text('这不是: [合法 yaml\n', encoding='utf-8')
    s = RouteStore(str(p))
    assert s.list_names() == []


def test_missing_file_ok(tmp_path):      # # 文件不存在也能构造
    s = RouteStore(str(tmp_path / 'none.yaml'))
    assert s.count() == 0


# ══════════════════════════════════════════════════════════════
#  外部改动自动重载（真机 bug："保存的路线加载进网页不显示"）
# ══════════════════════════════════════════════════════════════
#
# ★ 背景：waypoint_node 启动时把 routes.yaml 读进内存，之后就不再碰文件。
#   终端控制台新建一条路线后，网页那边永远看不到；更糟的是——网页再存
#   一条路线时，会用**旧内存**整表覆盖文件，把终端刚加的那条直接抹掉。
#   修法：每次对外读写前比对文件 mtime，变了就重读。
#
# 测试手法：用**第二个 RouteStore 实例**扮演"终端控制台"去写同一个文件 ——
#   这比手写 YAML 字符串更贴近真实（真实入口也只有这一条写路径），
#   而且不会因为手写格式少一层 `routes:` 而测出假结果。


def _bump_mtime(path):
    """把文件 mtime 往后拨，确保"别人刚改过"能被检测到。

    ★ 有些文件系统 mtime 精度只有 1 秒，同一秒内两次写会被判成"没变"。
      显式拨快 2 秒，让测试稳定反映"外部改动"。
    """
    import os
    import time
    st = os.stat(path)
    os.utime(path, (st.st_atime, time.time() + 2))


def test_reloads_when_file_changed_externally(tmp_path):
    """外部往文件里加了路线，内存里必须能看到（不用重启节点）。"""
    p = str(tmp_path / 'rt.yaml')
    web = RouteStore(p)
    web.save('网页建的', pts('p1'))

    # 扮演"终端控制台"：另起一个实例写同一个文件
    console = RouteStore(p)
    console.save('终端建的', pts('p2'))
    _bump_mtime(p)                       # use 保证 mtime 一定变了

    assert set(web.list_names()) == {'网页建的', '终端建的'}


def test_save_after_external_change_does_not_lose_data(tmp_path):
    """网页在外人改过文件之后再存一条，外人的那条不能被覆盖掉。

    ★ 这是数据丢失级 bug：旧实现保存时直接 dump 内存整表，
      终端刚加的路线会被**静默删除**，用户以为"路线自己不见了"。
    """
    p = str(tmp_path / 'rt.yaml')
    web = RouteStore(p)
    web.save('网页建的', pts('p1'))

    console = RouteStore(p)
    console.save('终端建的', pts('p2'))
    _bump_mtime(p)

    web.save('网页又建的', pts('p3'))    # 这一刻最容易把别人的数据冲掉

    fresh = RouteStore(p)                # 从磁盘重新读，看真实结果
    assert set(fresh.list_names()) == {'网页建的', '终端建的', '网页又建的'}
    assert fresh.get('终端建的') is not None


def test_delete_after_external_change_keeps_other_routes(tmp_path):
    """外部改动后删一条，其它路线（含外部的）必须还在。"""
    p = str(tmp_path / 'rt.yaml')
    web = RouteStore(p)
    web.save('A', pts('p1'))

    console = RouteStore(p)
    console.save('B', pts('p2'))
    _bump_mtime(p)

    assert web.delete('A') is True
    fresh = RouteStore(p)
    assert fresh.list_names() == ['B']


def test_get_sees_external_update_of_same_route(tmp_path):
    """别人改的是**同名**路线，也要看到新内容，不能拿旧缓存。"""
    p = str(tmp_path / 'rt.yaml')
    web = RouteStore(p)
    web.save('巡检', pts('老点'))

    console = RouteStore(p)
    console.save('巡检', pts('新点1', '新点2'))
    _bump_mtime(p)

    got = web.get('巡检')
    assert [x['name'] for x in got] == ['新点1', '新点2']


def test_no_reload_when_file_untouched(tmp_path):
    """文件没变就不重读（避免每次查询都打 IO）。"""
    p = str(tmp_path / 'rt.yaml')
    s = RouteStore(p)
    s.save('A', pts('p1'))
    mtime_after_save = s._mtime
    for _ in range(5):
        s.list_names()
        s.count()
    assert s._mtime == mtime_after_save, '文件没变不该重新 load'
