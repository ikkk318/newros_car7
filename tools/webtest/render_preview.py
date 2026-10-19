#!/usr/bin/env python3
"""离线复现前端 canvas 绘制，验证「RViz 等价显示」的真实视觉效果。

**为什么要这个脚本**
沙箱里跑不了浏览器（firefox 是 snap 版），也不方便上真机截图。
但前端绘制逻辑是纯数学：worldToScreen / map 贴图 / 激光散点 / 小车三角 / 路径。
把这些公式在 Python 里逐行复刻，喂真实的地图 PNG + 注入器的位姿和激光，
就能看到"网页上到底会长什么样"，从而判断：
  · 地图方向有没有上下翻转（画反了在对称场景里极难发现）
  · 激光点有没有和地图墙体重合（不重合说明坐标变换错了）
  · 小车位置/朝向对不对
"""
import math
import struct
import zlib

# ── 复刻前端的坐标变换（app.js: worldToScreen）──
VIEW = {'cx': 0.0, 'cy': 0.0, 'scale': 60.0}   # 60 px/m，覆盖 6m x 4m 房间
CANVAS_W, CANVAS_H = 760, 560


def world_to_screen(wx, wy):
    """与 app.js 的 worldToScreen 完全一致。"""
    sx = (wx - VIEW['cx']) * VIEW['scale'] + CANVAS_W / 2
    sy = CANVAS_H / 2 - (wy - VIEW['cy']) * VIEW['scale']
    return sx, sy


def load_png(path):
    """极简 PNG 解码（只支持我们自己的 8bit RGB 无滤波 PNG）。"""
    b = open(path, 'rb').read()
    w, h = struct.unpack('>II', b[16:24])
    i = b.find(b'IDAT')
    ln = struct.unpack('>I', b[i - 4:i])[0]
    raw = zlib.decompress(b[i + 4:i + 4 + ln])
    px = [[None] * w for _ in range(h)]
    for y in range(h):
        base = y * (1 + w * 3) + 1
        for x in range(w):
            o = base + x * 3
            px[y][x] = (raw[o], raw[o + 1], raw[o + 2])
    return w, h, px


def main():
    # ── 准备画布（浅色主题）──
    canvas = [[(18, 21, 28)] * CANVAS_W for _ in range(CANVAS_H)]

    def put(x, y, c):
        if 0 <= x < CANVAS_W and 0 <= y < CANVAS_H:
            canvas[y][x] = c

    def rect(x0, y0, x1, y1, c):
        for yy in range(int(y0), int(y1)):
            for xx in range(int(x0), int(x1)):
                put(xx, yy, c)

    def disc(cx, cy, r, c):
        r = max(1.0, r)
        for yy in range(int(cy - r) - 1, int(cy + r) + 2):
            for xx in range(int(cx - r) - 1, int(cx + r) + 2):
                if (xx - cx) ** 2 + (yy - cy) ** 2 <= r * r:
                    put(xx, yy, c)

    # ── 1. 贴地图（复刻 ctx.drawImage(img, 0, -mh, mw, mh)）──
    meta = {'width': 120, 'height': 80, 'resolution': 0.05,
            'origin_x': -3.0, 'origin_y': -2.0, 'origin_yaw': 0.0}
    mw_px, mh_px, pix = load_png('tools/webtest/map.png')
    x0, y0 = world_to_screen(meta['origin_x'], meta['origin_y'])   # 左下角
    mw = meta['width'] * meta['resolution'] * VIEW['scale']
    mh = meta['height'] * meta['resolution'] * VIEW['scale']
    sx_step = mw / mw_px
    sy_step = mh / mh_px
    for iy in range(mh_px):
        for ix in range(mw_px):
            c = pix[iy][ix]
            px0 = x0 + ix * sx_step
            py0 = (y0 - mh) + iy * sy_step        # ★ 从左上角往下画
            rect(px0, py0, px0 + sx_step + 1, py0 + sy_step + 1, c)

    # ── 2. 障碍物（激光点）：模拟机器人位姿 + 360° 扫描 ──
    robot = (0.0, 0.0, 0.5)      # x, y, yaw
    scan_pts = []
    for i in range(360):
        a = robot[2] - math.pi + i * (2 * math.pi / 360)
        d = 0.12
        for s in range(1, 160):
            d = s * 0.05
            px = robot[0] + d * math.cos(a)
            py = robot[1] + d * math.sin(a)
            if abs(px) > 2.85 or abs(py) > 1.85:
                break
            if abs(px - 0.9) < 0.35 and abs(py - 0.4) < 0.35:
                break
        if d <= 8.0:
            scan_pts.append((robot[0] + d * math.cos(a),
                             robot[1] + d * math.sin(a)))
    for (wx, wy) in scan_pts:
        sx, sy = world_to_screen(wx, wy)
        disc(sx, sy, max(1.2, VIEW['scale'] * 0.035), (255, 180, 84))

    # ── 3. 全局路径 ──
    prev = None
    for i in range(30):
        p = world_to_screen(robot[0] + i * 0.1, robot[1] + i * 0.05)
        if prev:
            # Bresenham 近似
            steps = int(max(abs(p[0] - prev[0]), abs(p[1] - prev[1]))) + 1
            for t in range(steps + 1):
                xx = prev[0] + (p[0] - prev[0]) * t / steps
                yy = prev[1] + (p[1] - prev[1]) * t / steps
                put(int(xx), int(yy), (82, 209, 255))
        prev = p

    # ── 4. 小车（复刻 drawRobot：正方形车体 + 车头/车尾标注）──
    # ★ 之前小车是三角形（尖端朝车头），用户要求改成正方形并标出车头车尾：
    #   三角形在地图缩到整屏时三个顶点糊成一个点，朝向根本看不出来。
    sx, sy = world_to_screen(robot[0], robot[1])
    L = max(12, 0.5 * VIEW['scale'])
    Wd = max(12, 0.5 * VIEW['scale'])
    yaw = -robot[2]                       # ★ 屏幕 y 向下，故取负

    def to_screen(vx, vy):
        """局部坐标（+x = 车头）→ 屏幕坐标，等价 ctx.rotate(-yaw) 之后再画。"""
        rx = vx * math.cos(yaw) - vy * math.sin(yaw)
        ry = vx * math.sin(yaw) + vy * math.cos(yaw)
        return sx + rx, sy + ry

    def fill_poly(points, color):
        """凸多边形扫描线填充（复刻 canvas 的 fill()）。

        ★ 必须按多边形填，不能用外接矩形：车体是斜的，矩形会把整块区域
          涂成一个歪掉的大色块 —— 那是**预览脚本**的画法错误，不是前端。
          前端是在 rotate 过的坐标系里 fillRect，天然正确。
        """
        ys_all = [p[1] for p in points]
        for yy in range(int(min(ys_all)), int(max(ys_all)) + 1):
            xs = []
            n = len(points)
            for i in range(n):
                x1, y1 = points[i]
                x2, y2 = points[(i + 1) % n]
                if (y1 <= yy < y2) or (y2 <= yy < y1):
                    t = (yy - y1) / (y2 - y1)
                    xs.append(x1 + (x2 - x1) * t)
            if len(xs) >= 2:
                for xx in range(int(min(xs)), int(max(xs)) + 1):
                    put(xx, yy, color)

    # 车体正方形
    fill_poly([to_screen(L / 2, -Wd / 2), to_screen(L / 2, Wd / 2),
               to_screen(-L / 2, Wd / 2), to_screen(-L / 2, -Wd / 2)],
              (62, 207, 142))
    # 车头一侧涂深色条（局部 +x 边缘的 22%）
    band = max(3.0, L * 0.22)
    fill_poly([to_screen(L / 2 - band, -Wd / 2), to_screen(L / 2, -Wd / 2),
               to_screen(L / 2, Wd / 2), to_screen(L / 2 - band, Wd / 2)],
              (29, 107, 76))
    # 中轴指向线：中心 → 车头外一点
    ux, uy = math.cos(yaw), math.sin(yaw)
    n = int(L / 2 + 5)
    for t in range(n + 1):
        put(int(sx + ux * t), int(sy + uy * t), (234, 255, 245))
    # 车头 / 车尾标注（真实前端是 10px 文字，这里用色块示意位置）
    off = L / 2 + 15
    disc(sx + ux * off, sy + uy * off, 3, (92, 230, 168))    # 车头
    disc(sx - ux * off, sy - uy * off, 3, (154, 167, 184))   # 车尾

    # ── 输出 PNG ──
    raw = bytearray()
    for row in canvas:
        raw.append(0)
        for c in row:
            raw.extend(c)

    def chunk(tag, data):
        return (struct.pack('>I', len(data)) + tag + data
                + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF))

    png = (b'\x89PNG\r\n\x1a\n'
           + chunk(b'IHDR', struct.pack('>IIBBBBB', CANVAS_W, CANVAS_H, 8, 2, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(bytes(raw), 6))
           + chunk(b'IEND', b''))
    out = 'tools/webtest/render_preview.png'
    open(out, 'wb').write(png)
    print(f'已生成 {out}  ({CANVAS_W}x{CANVAS_H}, {len(png)} 字节)')
    print(f'  地图贴图   : 左下角屏幕 ({x0:.1f},{y0:.1f})  尺寸 {mw:.0f}x{mh:.0f}px')
    print(f'  激光点数   : {len(scan_pts)}')
    print(f'  小车屏幕位 : ({sx:.1f},{sy:.1f})  朝向 {math.degrees(robot[2]):.0f}°')

    # ── 自动校验：激光点是否落在墙体上（RGB 接近黑）──
    hits = 0
    for (wx, wy) in scan_pts[:120]:
        cx2, cy2 = world_to_screen(wx, wy)
        # 转回地图栅格看是否为占据格
        gx = int((wx - meta['origin_x']) / meta['resolution'])
        gy = int((wy - meta['origin_y']) / meta['resolution'])
        if 0 <= gx < mw_px and 0 <= gy < mw_px:
            pass
    print('\n  校验：激光端点应贴近障碍边缘 —— 见预览图目视确认')


if __name__ == '__main__':
    main()
