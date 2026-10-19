"""map_check 单元测试：点位合法性预检（用合成小地图，不依赖真实地图）。"""
import os

from agv_common.map_check import MapChecker, load_pgm, load_map_meta


def _write_map(tmp_path, rows):
    """把二维 0/255 网格写成 P5 pgm + yaml。

    rows[0] 是图像第一行（对应 y 最大侧）；0=墙，255=空。
    resolution=0.1，origin=(0,0)，于是地图覆盖 x∈[0, W*0.1]、y∈[0, H*0.1]。
    """
    H = len(rows)
    W = len(rows[0])
    pgm = os.path.join(tmp_path, 'test.pgm')
    with open(pgm, 'wb') as f:
        f.write(b'P5\n')
        f.write(f'{W} {H}\n'.encode())
        f.write(b'255\n')
        for row in rows:
            f.write(bytes(row))
    yaml_path = os.path.join(tmp_path, 'test.yaml')
    with open(yaml_path, 'w') as f:
        f.write('image: test.pgm\n')
        f.write('resolution: 0.1\n')
        f.write('origin: [0.0, 0.0, 0.0]\n')
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.25\n')
    return yaml_path


def test_load_pgm_dimensions(tmp_path):
    """PGM 尺寸与内容正确解析。"""
    y = _write_map(tmp_path, [[255, 0], [0, 255]])
    data, w, h = load_pgm(os.path.join(tmp_path, 'test.pgm'))
    assert (w, h) == (2, 2)
    assert list(data) == [255, 0, 0, 255]


def test_load_map_meta(tmp_path):
    """yaml 元数据解析。"""
    y = _write_map(tmp_path, [[255, 255], [255, 255]])
    meta = load_map_meta(y)
    assert meta['resolution'] == 0.1
    assert meta['origin'] == (0.0, 0.0)


def test_open_point_accepted(tmp_path):
    """空地中央的点：合法。"""
    rows = [[255] * 20 for _ in range(20)]
    # 四周加墙
    rows[0] = [0] * 20
    rows[-1] = [0] * 20
    for r in rows:
        r[0] = 0
        r[-1] = 0
    mc = MapChecker(_write_map(tmp_path, rows), robot_clearance=0.3)
    ok, reason = mc.check_point(1.0, 1.0)
    assert ok, reason


def test_point_on_wall_rejected(tmp_path):
    """落在墙上的点：拒绝。"""
    rows = [[255] * 20 for _ in range(20)]
    rows[0] = [0] * 20
    rows[-1] = [0] * 20
    for r in rows:
        r[0] = 0
        r[-1] = 0
    mc = MapChecker(_write_map(tmp_path, rows), robot_clearance=0.3)
    # x=0 附近就是左墙
    ok, reason = mc.check_point(0.0, 1.0)
    assert not ok and '障碍' in reason


def test_point_outside_map_rejected(tmp_path):
    """图外点：拒绝且说明范围。"""
    rows = [[255] * 20 for _ in range(20)]
    rows[0] = [0] * 20
    rows[-1] = [0] * 20
    for r in rows:
        r[0] = 0
        r[-1] = 0
    mc = MapChecker(_write_map(tmp_path, rows), robot_clearance=0.3)
    ok, reason = mc.check_point(100.0, 100.0)
    assert not ok and '范围外' in reason


def test_too_close_to_wall_rejected(tmp_path):
    """离墙太近（净空不足）：拒绝，且提示净空数值。"""
    rows = [[255] * 20 for _ in range(20)]
    rows[0] = [0] * 20
    rows[-1] = [0] * 20
    for r in rows:
        r[0] = 0
        r[-1] = 0
    mc = MapChecker(_write_map(tmp_path, rows), robot_clearance=0.3)
    # 贴着左墙内一格：净空仅 0.1m < 0.3m
    ok, reason = mc.check_point(0.1, 1.0)
    assert not ok and '净空' in reason


def test_clearance_increases_away_from_wall(tmp_path):
    """离开墙越远，净空越大（距离场方向正确）。"""
    rows = [[255] * 20 for _ in range(20)]
    rows[0] = [0] * 20
    rows[-1] = [0] * 20
    for r in rows:
        r[0] = 0
        r[-1] = 0
    mc = MapChecker(_write_map(tmp_path, rows), robot_clearance=0.3)
    c1 = mc.clearance_at(0.1, 1.0)
    c5 = mc.clearance_at(0.5, 1.0)
    assert c5 > c1


def test_bottleneck_open_corridor(tmp_path):
    """全开场地：起点到目标瓶颈很大。"""
    rows = [[255] * 40 for _ in range(40)]
    rows[0] = [0] * 40
    rows[-1] = [0] * 40
    for r in rows:
        r[0] = 0
        r[-1] = 0
    mc = MapChecker(_write_map(tmp_path, rows), robot_clearance=0.3)
    b = mc.bottleneck(0.5, 0.5, 3.0, 3.0)
    assert b is not None and b >= 0.5


def test_bottleneck_narrow_gap(tmp_path):
    """一堵带窄缝的墙：瓶颈等于缝的净空，远小于开阔处。"""
    rows = [[255] * 40 for _ in range(40)]
    rows[0] = [0] * 40
    rows[-1] = [0] * 40
    for r in rows:
        r[0] = 0
        r[-1] = 0
    # 在第 20 列竖一道墙，只在中间留 1 格缝
    for r in range(1, 39):
        rows[r][20] = 0
    rows[20][20] = 255   # 缝
    mc = MapChecker(_write_map(tmp_path, rows), robot_clearance=0.3)
    b = mc.bottleneck(0.5, 2.0, 3.0, 2.0)
    assert b is not None and b < 0.3      # 缝太窄，瓶颈很小


def test_bottleneck_disconnected(tmp_path):
    """目标在完全封闭的墙后：不连通，返回 None。"""
    rows = [[255] * 40 for _ in range(40)]
    rows[0] = [0] * 40
    rows[-1] = [0] * 40
    for r in rows:
        r[0] = 0
        r[-1] = 0
    for r in range(1, 39):
        rows[r][20] = 0     # 无缝隙的整墙
    mc = MapChecker(_write_map(tmp_path, rows), robot_clearance=0.3)
    assert mc.bottleneck(0.5, 2.0, 3.0, 2.0) is None
