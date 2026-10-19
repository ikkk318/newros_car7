"""
map_check —— 点位合法性预检（纯 Python，不依赖 ROS，可单独测试）。

用途（对应"阴间点位"问题）：存点或导航前，先判断一个目标点对"这个尺寸的车"
是否真的可达、是否离障碍太近，避免把车派去贴墙、挤缝、撞墙或空跑。

判据（三个，全部基于栅格地图，不依赖仿真）：
    ① 在地图内       —— 目标格必须落在地图范围内；
    ② 不在障碍上     —— 目标格灰度 ≥ occupied_gray（默认 65）即视为可通行；
    ③ 净空足够       —— 目标点到最近障碍的距离 ≥ robot_clearance，
                        否则车身放不下（车放不下 → 必然靠撞墙或规划器作弊到达）。

关于"未知格"（灰度介于障碍与空地之间，如本仓库地图里的 205）：
    这里**不拒绝**，按"可通行"处理——这与 Nav2 map_server 的 trinary 解码一致：
    occ = (255-灰度)/255，205 → 0.196 < free_thresh(0.25) ⇒ free。
    若改成拒绝未知格，就与 Nav2 自身判读不一致，反而会拦掉导航器实际能走的点。

附带给出"最宽通道瓶颈"：从起点到该点的路径上最窄处的净空。若瓶颈 < 车身半宽，
说明物理上开不过去（用于解释"为什么这个点在墙后面口袋裡到不了"）。

地图读取：直接解析 ROS 的 map_server 产物（.pgm + .yaml），
不依赖 ROS 运行，便于单测与工具复用。PGM(P5) 值：0=黑=占用，255=白=空地。

坐标系约定（REP-103）：x 向前、y 向左；地图原点 origin 在 yaml 里给。
"""

from collections import deque                     # use BFS 求净空/连通性


# 读取 nav2 地图 yaml 的关键字段
# 参数：yaml_path - 地图元数据路径（如 maps/world666map/world666map.yaml）
# 返回：dict(image_dir 所在目录, resolution, origin(x,y), negate, occupied_thresh)
def load_map_meta(yaml_path):                     # out 对外接口
    import os
    import yaml
    with open(yaml_path, 'r', encoding='utf-8') as f:
        d = yaml.safe_load(f) or {}
    origin = d.get('origin', [0.0, 0.0, 0.0])
    return {
        'dir': os.path.dirname(os.path.abspath(yaml_path)),
        'image': d.get('image', ''),
        'resolution': float(d.get('resolution', 0.05)),
        'origin': (float(origin[0]), float(origin[1])),
        'negate': int(d.get('negate', 0)),
        'occupied_thresh': float(d.get('occupied_thresh', 0.65)),
    }


# 读取 PGM(P5) 为二维灰度数组
# 参数：pgm_path - 图片路径
# 返回：(data(bytearray 行优先, 长度 W*H), W, H)
def load_pgm(pgm_path):                           # out 对外接口
    with open(pgm_path, 'rb') as f:
        magic = f.readline().strip()
        if magic != b'P5':                        # # 只支持二进制 P5
            raise ValueError(f'不支持的 PGM 格式：{magic!r}（仅支持 P5）')
        # 跳过注释行，读尺寸
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = (int(v) for v in line.split()[:2])
        f.readline()                              # 读并丢弃 maxval
        data = f.read(w * h)                      # use 像素字节（行从图像顶部到底部）
        return bytearray(data), w, h


class MapChecker:                                 # out 点位合法性检查器
    # 初始化：载入地图并计算"到最近障碍的距离场"（BFS 距离变换）
    # 参数：yaml_path - map_server 的 yaml 路径
    #       robot_clearance - 车身所需净空(m)：目标点到最近障碍的最小距离
    #                         默认 0.30（车宽 0.5 → 半宽 0.26 + 余量）
    #       occupied_gray   - 灰度阈值：< 该值视为占用
    # 返回：无
    def __init__(self, yaml_path, robot_clearance=0.30, occupied_gray=65):   # out 对外构造函数
        import os
        meta = load_map_meta(yaml_path)           # use 读地图元数据
        self.res = meta['resolution']             # def 分辨率(m/格)
        self.ox, self.oy = meta['origin']         # def 地图原点(m)
        self.occupied_gray = int(occupied_gray)   # def 占用灰度阈值
        self.robot_clearance = float(robot_clearance)   # def 所需净空(m)
        pgm = os.path.join(meta['dir'], meta['image'])  # use 拼出 pgm 路径
        self.data, self.W, self.H = load_pgm(pgm)       # use 载入栅格
        self._build_distance_field()              # io 预计算距离场

    # 内部：坐标(m) → 栅格(row, col)。行按图像从上到下；y 越大行越小
    # 参数：x, y - 地图坐标(m)
    # 返回：(row, col)
    def _to_rc(self, x, y):                       # in 内部方法
        col = int((x - self.ox) / self.res)
        row = self.H - 1 - int((y - self.oy) / self.res)
        return row, col

    # 内部：栅格(row,col) → 坐标(m)
    # 参数：row, col
    # 返回：(x, y)
    def _to_xy(self, row, col):                   # in 内部方法
        return self.ox + col * self.res, self.oy + (self.H - 1 - row) * self.res

    # 内部：BFS 距离变换，求每个自由格到最近占用格的距离（单位：格）
    # 参数：无
    # 返回：无
    def _build_distance_field(self):              # in 内部方法
        INF = 1 << 30
        W, H, data = self.W, self.H, self.data
        dist = [INF] * (W * H)                    # def 到最近障碍的格距离
        dq = deque()
        for r in range(H):                        # # 占用格入队，距离 0
            base = r * W
            for c in range(W):
                if data[base + c] < self.occupied_gray:
                    dist[base + c] = 0
                    dq.append(base + c)
        while dq:                                 # # 四邻域 BFS
            idx = dq.popleft()
            d = dist[idx] + 1
            r, c = divmod(idx, W)
            for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                if 0 <= nr < H and 0 <= nc < W:
                    j = nr * W + nc
                    if dist[j] > d:
                        dist[j] = d
                        dq.append(j)
        self.dist = dist                         # def 保存距离场

    # 读取某格灰度
    # 参数：row, col
    # 返回：灰度值；越界返回 None
    def _gray(self, row, col):                    # in 内部方法
        if not (0 <= row < self.H and 0 <= col < self.W):
            return None
        return self.data[row * self.W + col]

    # 查询一个点的净空（到最近障碍的距离，m）
    # 参数：x, y - 地图坐标(m)
    # 返回：净空(m)；点在图外返回 None
    def clearance_at(self, x, y):                 # out 对外接口
        row, col = self._to_rc(x, y)
        if not (0 <= row < self.H and 0 <= col < self.W):
            return None
        return self.dist[row * self.W + col] * self.res

    # 判断一个点是否可作导航终点
    # 参数：x, y - 地图坐标(m)
    # 返回：(ok: bool, reason: str) —— reason 是中文说明
    def check_point(self, x, y):                  # out 对外接口
        x = float(x); y = float(y)
        row, col = self._to_rc(x, y)
        if not (0 <= row < self.H and 0 <= col < self.W):     # # ① 图外
            return False, (f'目标点 ({x:.2f}, {y:.2f}) 在地图范围外 '
                           f'(x∈[{(self.ox):.1f}, {self.ox + self.W * self.res:.1f}], '
                           f'y∈[{(self.oy):.1f}, {self.oy + self.H * self.res:.1f}])，已拒绝')
        g = self._gray(row, col)
        if g is not None and g < self.occupied_gray:          # # ② 落在障碍/墙上
            return False, f'目标点 ({x:.2f}, {y:.2f}) 落在障碍物/墙体上，已拒绝'
        clr = self.clearance_at(x, y) or 0.0                  # # ③ 净空检查
        if clr < self.robot_clearance:
            return False, (f'目标点 ({x:.2f}, {y:.2f}) 距障碍仅 {clr:.2f} m，'
                           f'小于车身所需净空 {self.robot_clearance:.2f} m，已拒绝')
        return True, f'目标点 ({x:.2f}, {y:.2f}) 合法，净空 {clr:.2f} m'

    # 从起点到目标的"最宽通道瓶颈"（m）：路径上最窄处的净空
    # 参数：sx, sy - 起点(m)；gx, gy - 目标(m)
    # 返回：瓶颈净空(m)；不连通返回 None。用于判断"物理上开不开得过去"
    def bottleneck(self, sx, sy, gx, gy):         # out 对外接口
        import heapq
        W, H = self.W, self.H
        dist = self.dist                          # def 距离场（0 = 占用格）
        sr, sc = self._to_rc(sx, sy)
        gr, gc = self._to_rc(gx, gy)
        if not (0 <= sr < H and 0 <= sc < W and 0 <= gr < H and 0 <= gc < W):
            return None
        if dist[sr * W + sc] == 0 or dist[gr * W + gc] == 0:
            return None                           # # 起点或终点落在占用格：不通
        best = [-1] * (W * H)                     # def 到该格的最大最小净空（格单位）
        best[sr * W + sc] = dist[sr * W + sc]
        pq = [(-best[sr * W + sc], sr, sc)]
        while pq:
            nb, r, c = heapq.heappop(pq)
            nb = -nb
            if nb < best[r * W + c]:
                continue
            if (r, c) == (gr, gc):
                break
            for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                if 0 <= nr < H and 0 <= nc < W:
                    j = nr * W + nc
                    if dist[j] == 0:             # # 占用格不可通行（不能穿墙）
                        continue
                    v = min(nb, dist[j])
                    if v > best[j]:
                        best[j] = v
                        heapq.heappush(pq, (-v, nr, nc))
        b = best[gr * W + gc]
        return None if b < 0 else b * self.res
