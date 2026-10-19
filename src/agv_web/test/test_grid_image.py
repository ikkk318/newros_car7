"""
grid_image 单元测试：OccupancyGrid → PNG、世界坐标 ↔ 像素、激光点转换。

★ 这里最值得钉死的是"地图上下翻转"：ROS 的 row 0 在底部，PNG 第 0 行在顶部。
  写错了图会上下颠倒，而地图常常近似上下对称，肉眼很难发现，表现却是
  "小车显示的实时位置跟画面里的墙对不上"——会被误判成定位漂移。
"""

import struct
import zlib
import math

from agv_web.grid_image import (
    encode_png, occupancy_to_png, grid_meta, world_to_pixel,
    scan_points_to_map, UNKNOWN)


class FakeVec:                           # def 位姿替身
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class FakeQuat:                          # def 四元数替身（默认单位四元数）
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class FakePose:                          # def 位姿替身
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.position = FakeVec(x, y, 0.0)
        self.orientation = FakeQuat(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class FakeInfo:                          # def 地图信息替身
    def __init__(self, width, height, resolution, ox=0.0, oy=0.0, oyaw=0.0):
        self.width, self.height = width, height
        self.resolution = resolution
        self.origin = FakePose(ox, oy, oyaw)


class FakeGrid:                          # def OccupancyGrid 替身
    def __init__(self, width, height, data, resolution, ox=0.0, oy=0.0, oyaw=0.0):
        self.info = FakeInfo(width, height, resolution, ox, oy, oyaw)
        self.data = data


def parse_png(png):
    """极简 PNG 解析：校验签名、IHDR、解出原始像素行。"""
    assert png[:8] == b'\x89PNG\r\n\x1a\n', 'PNG 签名不对'
    pos = 8
    w = h = depth = ctype = None
    idat = b''
    while pos < len(png):
        (length,) = struct.unpack('>I', png[pos:pos + 4])
        tag = png[pos + 4:pos + 8]
        data = png[pos + 8:pos + 8 + length]
        crc = struct.unpack('>I', png[pos + 8 + length:pos + 12 + length])[0]
        assert crc == (zlib.crc32(tag + data) & 0xFFFFFFFF), f'{tag} 的 CRC 不对'
        if tag == b'IHDR':
            w, h, depth, ctype = struct.unpack('>IIBB', data[:10])
        elif tag == b'IDAT':
            idat += data
        pos += 12 + length
    raw = zlib.decompress(idat)
    stride = w * 3 + 1
    rows = []
    for y in range(h):
        line = raw[y * stride:(y + 1) * stride]
        assert line[0] == 0, '本实现只用 filter=0'
        rows.append(line[1:])
    return {'width': w, 'height': h, 'depth': depth, 'ctype': ctype, 'rows': rows}


# ─────────────────────────── PNG 编码 ───────────────────────────

def test_encode_png_header_and_size():
    rows = [bytes([255, 0, 0] * 4)] * 3        # # 4x3 全红
    png = encode_png(4, 3, rows)
    out = parse_png(png)
    assert out['width'] == 4 and out['height'] == 3
    assert out['depth'] == 8 and out['ctype'] == 2   # # 8 位真彩色
    assert out['rows'][0] == bytes([255, 0, 0] * 4)


def test_encode_png_rejects_bad_dimensions():
    for w, h in ((0, 5), (5, 0), (-1, 3)):
        try:
            encode_png(w, h, [])
            assert False, '非法尺寸应抛异常'
        except ValueError:
            pass


def test_encode_png_rejects_wrong_row_length():
    try:
        encode_png(4, 1, [bytes(3)])           # # 应为 12 字节
        assert False, '行长度不符应抛异常'
    except ValueError:
        pass


# ─────────────────────── OccupancyGrid → PNG ───────────────────────

def test_occupancy_colors():
    # 2x1：未知、占据
    g = FakeGrid(2, 1, [UNKNOWN, 100], 0.05)
    out = parse_png(occupancy_to_png(2, 1, g.data))
    row = out['rows'][0]
    assert row[0:3] == bytes((128, 128, 128)), '未知应为中灰'
    assert row[3:6] == bytes((0, 0, 0)), '占据应为黑'


def test_occupancy_free_is_near_white():
    out = parse_png(occupancy_to_png(1, 1, [0]))
    assert out['rows'][0] == bytes((254, 254, 254))


def test_occupancy_threshold_matches_rviz_habit():
    # 65 及以上算占据，64 及以下算空闲
    out = parse_png(occupancy_to_png(2, 1, [64, 65]))
    assert out['rows'][0][0:3] == bytes((254, 254, 254))
    assert out['rows'][0][3:6] == bytes((0, 0, 0))


def test_occupancy_rejects_size_mismatch():
    try:
        occupancy_to_png(3, 3, [0, 0, 0])      # # 数据不足 9 个
        assert False, '尺寸不符应抛异常'
    except ValueError:
        pass


def test_occupancy_flips_vertically():
    """★ 关键回归：ROS 的 row 0 是地图最底部，PNG 第 0 行是顶部。

    造一张 1 宽 2 高的图：ROS 里 row0=占据(底部)、row1=空闲(顶部)。
    画出来应当 PNG 第 0 行（顶部）=空闲，第 1 行（底部）=占据。
    """
    g = FakeGrid(1, 2, [100, 0], 0.05)         # # row0 占据(下), row1 空闲(上)
    out = parse_png(occupancy_to_png(1, 2, g.data))
    assert out['rows'][0] == bytes((254, 254, 254)), 'PNG 顶行应是 ROS 的最后一行（空闲）'
    assert out['rows'][1] == bytes((0, 0, 0)), 'PNG 底行应是 ROS 的第 0 行（占据）'


def test_occupancy_custom_unknown_color():
    out = parse_png(occupancy_to_png(1, 1, [UNKNOWN], unknown_rgba=(1, 2, 3)))
    assert out['rows'][0] == bytes((1, 2, 3))


# ─────────────────────── 地图元信息 ───────────────────────

def test_grid_meta_extracts_geometry():
    g = FakeGrid(594, 762, [0] * (594 * 762), 0.025, ox=-10.70, oy=-9.83)
    m = grid_meta(g)
    assert m['width'] == 594 and m['height'] == 762
    assert abs(m['resolution'] - 0.025) < 1e-12
    assert abs(m['origin_x'] + 10.70) < 1e-9
    assert abs(m['origin_y'] + 9.83) < 1e-9
    assert abs(m['origin_yaw']) < 1e-9


def test_grid_meta_reads_origin_yaw():
    g = FakeGrid(10, 10, [0] * 100, 0.05, oyaw=math.pi / 2)
    m = grid_meta(g)
    assert abs(m['origin_yaw'] - math.pi / 2) < 1e-9


# ─────────────────────── 世界坐标 → 像素 ───────────────────────

def test_world_to_pixel_origin_is_bottom_left():
    """origin 是地图左下角：世界坐标取 origin 时，像素应落在左下角。

    PNG 坐标里 y 轴向下，所以左下角 = (px=0, py=height)。
    """
    meta = {'width': 100, 'height': 200, 'resolution': 0.1,
            'origin_x': -5.0, 'origin_y': -10.0, 'origin_yaw': 0.0}
    px, py = world_to_pixel(-5.0, -10.0, meta)
    assert abs(px - 0.0) < 1e-9
    assert abs(py - 200.0) < 1e-9, '左下角应换算到 PNG 的最底行'


def test_world_to_pixel_center():
    meta = {'width': 100, 'height': 100, 'resolution': 0.1,
            'origin_x': 0.0, 'origin_y': 0.0, 'origin_yaw': 0.0}
    # 地图 10m x 10m，世界 (5,5) 是正中心 → 像素 (50,50)
    px, py = world_to_pixel(5.0, 5.0, meta)
    assert abs(px - 50.0) < 1e-9
    assert abs(py - 50.0) < 1e-9


def test_world_to_pixel_up_is_smaller_y():
    """世界 y 增大 → PNG 的 py 应减小（图像往上）。"""
    meta = {'width': 100, 'height': 100, 'resolution': 0.1,
            'origin_x': 0.0, 'origin_y': 0.0, 'origin_yaw': 0.0}
    _, py_low = world_to_pixel(1.0, 1.0, meta)
    _, py_high = world_to_pixel(1.0, 9.0, meta)
    assert py_high < py_low, '世界 y 越大，PNG 行号越小'


def test_world_to_pixel_handles_rotated_map():
    """地图 origin 带 yaw 时，世界点要先转进栅格系再除以分辨率。

    构造：origin=(0,0)、yaw=+90°、10m×10m、0.1 m/格。
    此时地图栅格 x 轴指向世界 +y，栅格 y 轴指向世界 -x，
    即地图覆盖世界区域 x∈[-10,0]、y∈[0,10]。

    取世界点 (0, 5)：栅格系里 dx=5, dy=0 → px=50、py=height（正好贴 PNG 底边）。
    若忘了减 origin_yaw，这个点会被算到 px=0 去，整体错位 5 m。
    """
    meta = {'width': 100, 'height': 100, 'resolution': 0.1,
            'origin_x': 0.0, 'origin_y': 0.0, 'origin_yaw': math.pi / 2}
    px, py = world_to_pixel(0.0, 5.0, meta)
    assert abs(px - 50.0) < 1e-6, f'旋转后应落到横向中线，实际 px={px}'
    assert abs(py - 100.0) < 1e-6, f'该点在地图下边缘，py 应为 height，实际 {py}'
    # 反向核对：世界点 (1,0) 不在该地图覆盖范围内，换算后应落到图外
    px_out, py_out = world_to_pixel(1.0, 0.0, meta)
    assert px_out < 0.0 or py_out > 100.0, '图外的点应换算到画布范围之外'


# ─────────────────────── 激光点 → 地图坐标 ───────────────────────

def test_scan_points_at_identity_pose():
    # 车在原点朝 +x；一条 2 m 的正前方读数 → 点应在 (2, 0)
    pts = scan_points_to_map([2.0], 0.0, 0.0, 0.0, 0.0, 0.0)
    assert len(pts) == 1
    assert abs(pts[0][0] - 2.0) < 1e-9 and abs(pts[0][1]) < 1e-9


def test_scan_points_respect_robot_yaw():
    # 车朝 +y（yaw=pi/2）、正前方读数 2 m → 点应在 (0, 2)
    pts = scan_points_to_map([2.0], 0.0, 0.0, 0.0, 0.0, math.pi / 2)
    assert abs(pts[0][0]) < 1e-9 and abs(pts[0][1] - 2.0) < 1e-9


def test_scan_points_respect_robot_translation():
    pts = scan_points_to_map([1.0], 0.0, 0.0, 10.0, 20.0, 0.0)
    assert abs(pts[0][0] - 11.0) < 1e-9 and abs(pts[0][1] - 20.0) < 1e-9


def test_scan_points_drop_invalid_readings():
    """无效读数必须丢掉，否则 nan 会让前端 canvas 整帧画不出来。"""
    ranges = [float('inf'), float('nan'), None, 0.0, -1.0, 2.0]
    pts = scan_points_to_map(ranges, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert len(pts) == 1, f'应只保留 1 个有效点，实际 {len(pts)}'
    assert abs(pts[0][0] - 2.0) < 1e-9


def test_scan_points_respect_range_limits():
    pts = scan_points_to_map([0.01, 2.0, 9.0], 0.0, 0.0, 0.0, 0.0, 0.0,
                             range_min=0.05, range_max=8.0)
    assert len(pts) == 1 and abs(pts[0][0] - 2.0) < 1e-9


def test_scan_points_multiple_beams_sweep():
    # 3 束、-90°~+90°，各 1 m：应得到 (-1,0) 附近、(0 后面) 和 (0,1)
    pts = scan_points_to_map([1.0, 1.0, 1.0], -math.pi / 2, math.pi / 2,
                             0.0, 0.0, 0.0)
    assert len(pts) == 3
    assert abs(pts[0][0]) < 1e-9 and abs(pts[0][1] + 1.0) < 1e-9   # 朝 -y
    assert abs(pts[2][0]) < 1e-9 and abs(pts[2][1] - 1.0) < 1e-9   # 朝 +y
