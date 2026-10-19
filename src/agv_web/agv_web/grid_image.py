"""
grid_image —— 把 ROS 的 OccupancyGrid 转成网页能直接显示的 PNG（纯逻辑，不依赖 ROS）。

**为什么自己写 PNG 编码而不用 Pillow**
车机（Ubuntu 22.04 + ROS 2 Humble）上不保证装了 Pillow，但 `zlib` 和 `struct`
是 Python 标准库。少一个 pip 依赖，就少一个"部署到车上才发现缺包"的坑。

**地图方向为什么容易画反**
ROS 的 OccupancyGrid 数据是**从下往上**存的（row 0 是 y 最小那一行），
而 PNG 是**从上往下**存的（第 0 行在图像顶部）。直接顺序写会得到上下颠倒的图，
表现为"小车在地图里的位置跟画面对不上"，而且因为地图常常上下近似对称，
肉眼很难一眼看出——所以这里显式翻转，并写测试钉死。
"""

import struct                            # use 打包 PNG 块头
import zlib                              # use PNG 压缩


# OccupancyGrid 的取值语义（rosidl 定义）
UNKNOWN = -1                             # def 未知区域
FREE = 0                                 # def 空闲
OCCUPIED = 100                           # def 占据


def _chunk(tag, data):                   # use 生成一个 PNG 块
    """PNG 块 = 长度(4) + 类型(4) + 数据 + CRC(4)。"""
    return (struct.pack('>I', len(data)) + tag + data
            + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF))


def encode_png(width, height, rgb_rows):  # out 把 RGB 行数据编码成 PNG
    """把像素数据编码成 PNG 字节串。

    参数：width, height - 图像尺寸（像素）
          rgb_rows     - 长度为 height 的序列，每项是长度 width*3 的 bytes
    返回：PNG 文件的 bytes
    """
    if width <= 0 or height <= 0:
        raise ValueError('图像尺寸必须为正')
    raw = bytearray()
    for row in rgb_rows:                 # use 每行前加 filter 字节 0（None）
        if len(row) != width * 3:
            raise ValueError('每行字节数应为 width*3')
        raw.append(0)
        raw.extend(row)
    ihdr = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)  # 8 位、真彩色
    return (b'\x89PNG\r\n\x1a\n'
            + _chunk(b'IHDR', ihdr)
            + _chunk(b'IDAT', zlib.compress(bytes(raw), 6))
            + _chunk(b'IEND', b''))


def occupancy_to_png(width, height, data, unknown_rgba=None):
    """把 OccupancyGrid 转成三色 PNG：空闲=浅灰白、占据=黑、未知=蓝灰。

    参数：width, height - 网格尺寸（单元格数）
          data         - 一维整数序列（长度 width*height），-1/0~100
          unknown_rgba - 可选，(r,g,b) 三元组，覆盖未知区域颜色
    返回：PNG 字节串

    ★ 必须上下翻转：ROS 的 row 0 在 y 最小处（底部），PNG 第 0 行在顶部。
    """
    if len(data) != width * height:
        raise ValueError(f'数据长度 {len(data)} 与尺寸 {width}x{height} 不符')
    if unknown_rgba is None:
        unknown_rgba = (128, 128, 128)   # def 未知：中灰（与 RViz 默认接近）
    unk = bytes(unknown_rgba)
    free = bytes((254, 254, 254))        # def 空闲：近白
    occ = bytes((0, 0, 0))               # def 占据：黑

    rows = []
    for py in range(height - 1, -1, -1):          # use 从最后一行往上取 = 翻转
        base = py * width
        buf = bytearray()
        for px in range(width):
            v = data[base + px]
            if v < 0:                             # # 未知
                buf += unk
            elif v >= 65:                         # # 占据（门槛同 RViz 习惯）
                buf += occ
            else:                                 # # 空闲
                buf += free
        rows.append(bytes(buf))
    return encode_png(width, height, rows)


def grid_meta(msg):                      # out 从 OccupancyGrid 提取网页需要的元信息
    """提取显示地图必须的几何信息。

    参数：msg - 带 info(width/height/resolution/origin) 的对象
    返回：dict，键为 width/height/resolution/origin_x/origin_y/origin_yaw

    ★ origin 是地图**左下角**在世界坐标里的位置。前端定位像素时必须用它，
      漏了就会整体错位（而且错得很像"定位漂了"，容易误判）。
    """
    info = msg.info
    o = info.origin
    q = o.orientation
    # 由四元数取 yaw（地图一般只绕 z 旋转；这里只用 z/w 两分量足够）
    yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
    return {'width': int(info.width),            # def 宽（格）
            'height': int(info.height),          # def 高（格）
            'resolution': float(info.resolution),  # def 分辨率(m/格)
            'origin_x': float(o.position.x),     # def 左下角 x(m)
            'origin_y': float(o.position.y),     # def 左下角 y(m)
            'origin_yaw': yaw}                   # def 地图朝向(rad)


def _yaw_from_quat(x, y, z, w):          # in 内部工具
    """四元数 → yaw（只用得上绕 z 的旋转）。"""
    import math
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def world_to_pixel(wx, wy, meta):        # out 世界坐标 → 地图像素（左上原点）
    """把地图坐标系下的 (x,y) 转成 PNG 上的像素坐标。

    参数：wx, wy - 世界坐标(m)；meta - grid_meta() 的返回值
    返回：(px, py) 浮点像素坐标，py 已按 PNG 的"从上往下"翻转
    """
    import math
    res = meta['resolution']
    dx = wx - meta['origin_x']
    dy = wy - meta['origin_y']
    yaw = meta.get('origin_yaw', 0.0)
    if yaw:                              # # 地图本身有旋转：先把点转到地图栅格系
        c, s = math.cos(-yaw), math.sin(-yaw)
        dx, dy = dx * c - dy * s, dx * s + dy * c
    px = dx / res
    py = (meta['height']) - (dy / res)   # use 翻转成从上往下
    return px, py


def scan_points_to_map(ranges, angle_min, angle_increment,
                       robot_x, robot_y, robot_yaw,
                       range_min=0.0, range_max=float('inf')):
    """把一帧 LaserScan 转成地图坐标系下的点列表（给网页画散点）。

    参数：ranges/angle_min/angle_increment - 激光数据与角度定义
          robot_x/y/yaw                   - 激光在 map 下的位姿
          range_min/range_max             - 有效量程（越界的点丢掉）
    返回：[(x, y), ...]（map 坐标系，米）

    ★ 无效读数（None / inf / nan / 0 / 越界）必须丢掉：直接参与三角函数会得到
      nan，前端 canvas 一旦拿到 nan 整条扫描就画不出来（静默失败，很难查）。
    """
    import math
    pts = []
    for i, r in enumerate(ranges):
        if r is None:
            continue
        try:
            rv = float(r)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(rv) or rv <= 0.0:
            continue
        if rv < range_min or rv > range_max:
            continue
        a = robot_yaw + angle_min + i * angle_increment
        pts.append((robot_x + rv * math.cos(a), robot_y + rv * math.sin(a)))
    return pts
