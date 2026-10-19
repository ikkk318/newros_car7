#!/usr/bin/env python3
"""
make_world_map.py —— 按"唯一定义的真源(layout)"同时生成 Gazebo 世界 与 导航栅格地图
================================================================================
为什么要有这个脚本：
    world（Gazebo 里的物理场地）和 栅格地图（Nav2 用来定位/规划的 h 图）必须**一致**，
    否则 AMCL 对不上、车走不动。手写两份必然漂移。这里用**一份参数化布局**同时导出：
      ① world666.world      —— 供 Gazebo 加载的 SDF 世界（墙 + 凵型槽位 + 货物方块）
      ② world666map.pgm/.yaml —— 与之一一对应的二维占用栅格（0=障碍，255=空地）
      ③ waypoints.yaml      —— 5 个"X号货架"（槽口对位，kind: cargo）
                              + 5 个"X号槽内"（槽内顶升，kind: cargo）
                              + 起始点 + 4 个导航点（kind: nav），控制台/任务层可直接用
      ④ routes.yaml         —— 1 号巡检路线（途经 4 个导航点；可在控制台"路线管理"里改）

    ★ 世界只有这一份真源：`src/agv_description/worlds/world666.world` 由本脚本生成，
      改场景就改本脚本再重跑，不要手改 world——否则地图/点位/世界三者又会漂移。
    ★ 栅格地图：控制台"建图模式"跑出来的那份（maps/world666map/）是**实测地图**，
      以它为准；本脚本生成的 pgm 是同布局的参考版，用于对照与重建场景。
      两者尺寸不同是正常的——实测地图是 SLAM 边走边建的，范围由走到哪决定；
      实测那份为 594×762、origin (-10.7,-9.83)，只覆盖 x[-10.70, 4.15]。

场景（依据用户提供的 world666 世界重建）：
    一栋厂房（外墙 + 内部隔墙，沿用用户世界的墙几何，含其轻微旋转）；
    5 个 **凵型槽位**（1~5 号；位姿取自用户世界的存档位姿 = 用户给的"货架位置"表）；
    2 个货物方块坐在 1 号 / 5 号槽的**顶上**，其余 3 个槽为空槽。
    ★ 货物坐在槽顶之上（不占内槽）：内槽恒为空，槽口外留约 0.9 m 停车位，
      车头正对槽口即可到位——当前阶段到点即停，不做插入、不做特征解算。
    0 号槽位（用户世界里悬空 0.29 m 的那个）按用户要求不纳入本场景使用；
    且因它悬在激光扫描面(0.17 m)之上、2D 激光看不见它，本脚本**不把它烧进地图**
    （保持"地图 = 激光能看到的世界"这一致性；车高 0.13 m，可从它底下正常穿过）。

几何契约（来自 agv_description 的真实车模型，详见 搬运场景几何契约.md）：
    车 0.5×0.5 m、车顶 0.13 m、车外接圆 0.354 m；
    雷达在车体几何中心 x=0.00 m、z=0.17 m、360° 全向（无盲区）。
    因此：槽内净宽 ≥0.70（车 0.5 能进）、槽壁高 >0.17（雷达看得见）、
    槽内径长 ≥0.85（车能插进去一段）、主通道 ≥0.90。

用法：
    python3 tools/make_world_map.py                    # 用默认路径生成三件套
    python3 tools/make_world_map.py --out-dir /tmp/x    # 换个输出目录看效果
    python3 tools/make_world_map.py --dry-run           # 只生成不落包
"""
import argparse
import math
import os

# ═══════════════════════════════════════════════════════════════════════
# 一、布局参数（改这里 = 改图纸；world 和地图会一起变）
# ═══════════════════════════════════════════════════════════════════════

WALL_T = 0.15                       # 墙厚（用户世界里的墙厚，沿用）
WALL_H = 2.50                       # 墙高（用户世界里的墙高，沿用）

# ── 墙体清单：世界坐标下"旋转矩形" (cx, cy, size_x, size_y, yaw) ──
#     直接取自用户提供 world666 的墙几何（已把模型/连杆变换折算到世界系）。
WALLS = [
    # ── 外墙（用户世界的四周围墙）──
    ( -0.4248,   9.2560, 20.7500, WALL_T,  0.0000),   # 北（上）外墙
    (  9.8752,  -0.2940, 19.2500, WALL_T, -1.5708),   # 东（右）外墙
    (  0.0752,  -9.8440, 19.7500, WALL_T,  3.1416),   # 南（下）外墙
    (-10.2248,  -0.2940, 19.2762, WALL_T,  1.6231),   # 西（左）外墙
    # ── 内部隔墙（用户世界里的分区墙）──
    ( -7.8988,  -5.3907,  4.5000, WALL_T,  0.0000),
    ( -5.6798,  -8.3618,  3.0000, WALL_T, -1.5708),
    (  3.8406,  -7.8794,  3.5000, WALL_T, -1.5708),
    (  5.3906,  -6.2044,  3.2500, WALL_T,  0.0000),
    ( -8.1052,  -0.8340,  4.2500, WALL_T,  0.0000),
    ( -5.9459,  -4.1295,  2.7113, WALL_T, -1.3965),
    ( -5.4357,   7.3310,  4.0000, WALL_T, -1.5708),
    (  3.5965,   7.1624,  4.5000, WALL_T, -1.5708),
    ( -9.3870,   5.6757,  2.5000, WALL_T,  0.0000),
    (  5.7278,   5.3502,  4.2500, WALL_T,  0.0000),
    (  3.8489,  -6.1017,  2.5000, WALL_T,  1.5708),
    (  3.8489,   0.2483, 10.5000, WALL_T,  1.5708),
]

# ── 凵型槽位几何（与用户 world666 里的槽模型完全一致）──
#     局部坐标：原点在"槽口中心线中点"，+x 指向槽内，+y 为左。
#     三块实体：左壁 / 右壁（两条平行侧壁）/ 远端挡板（槽底）。
#     这些尺寸只用于"把世界建出来"，不再参与任何对接解算。
SLOT_INNER_LENGTH = 1.10            # 内槽径长：槽口→槽底内壁的深度(m)
SLOT_INNER_WIDTH  = 0.80            # 内槽净宽：两壁内侧间距(m)，须 ≥0.70 容车
SLOT_WALL_TH      = 0.08            # 槽壁厚度(m)
SLOT_OUTER_WIDTH  = SLOT_INNER_WIDTH + 2 * SLOT_WALL_TH   # 外槽径宽(m) = 0.96
SLOT_BASE_DEPTH   = 0.08            # 槽底挡板厚度(m)
SLOT_HEIGHT       = 0.40            # 槽高(m)：须 >0.17（雷达高度）才看得见

# ── 货物（坐在槽顶上的大方块；搬运演示的搬运对象）──
#     ★ 截面比槽口大（1.21×1.34 > 外宽 0.96），所以它从不进槽、一直"架"在槽顶上。
#     实现为 **kinematic（运动学）体**：不受重力、不会被物理推挤，位姿可由脚本用
#     /gazebo/set_entity_state 摆放（当前阶段没人摆它，它就一直坐在槽顶当场景道具；
#      接真实搬货后由硬件/仿真插件负责）。
CARGO_SIZE_X = 1.21114              # 货块长(m)（取自用户世界 unit_box 尺寸）
CARGO_SIZE_Y = 1.3404               # 货块宽(m)
CARGO_SIZE_Z = 0.475182             # 货块高(m)
CARGO_MASS   = 1.0                  # 质量(kg)：仅用于惯性，kinematic 下不落重力
CARGO_SEAT_LOCAL_X = 0.50           # "坐姿/放货"中心在槽局部系的 x：= 车停驻点正上方
CARGO_SEAT_Z = SLOT_HEIGHT + CARGO_SIZE_Z / 2.0   # 坐姿中心高(m) = 0.6376：底面正好贴槽壁顶(0.40)

# ── 槽位落点（世界坐标）＋ 开口朝向（取自用户 world666 的存档位姿"货架位置"表）──
#     序号沿用用户编号：SLOT_SPOTS[0] → slot_1 …（0 号不用，见文件头说明）
SLOT_SPOTS = [
    ('slot_1',  2.53198,  2.69159,  0.00000),   # 顶上有货（cargo_box_0）
    ('slot_2', -8.47018, -7.49172, -3.08481),   # 空槽
    ('slot_3',  2.45799, -6.13987,  0.00000),   # 空槽
    ('slot_4', -8.63511,  7.91659,  1.55217),   # 空槽
    ('slot_5', -8.71246, -2.57664, -3.09949),   # 顶上有货（cargo_box_1）
]

# ── 货物落座（用户 world666 里的实际摆放位姿，原样沿用）──
CARGO_SPOTS = [
    {'name': 'cargo_box_0', 'slot': 'slot_1', 'x':  3.02874, 'y':  2.69986, 'yaw': 0.037261},
    {'name': 'cargo_box_1', 'slot': 'slot_5', 'x': -9.34037, 'y': -2.52390, 'yaw': 0.037261},
]

# ── 小车停留点（Nav2 导航终点；取自用户给的"小车停留点"表，车停在槽口外 ~0.9 m、正对槽口）──
#     一行 = (点名, x, y, yaw)；yaw 为车头朝向（正对槽口）。顺序与槽位一一对应。
#     ★ 这些点就是"搬货点"（kind: cargo）：到达后由任务层调 /agv/cargo 触发顶升/下降。
STOP_POINTS = [
    ('1号货架',  1.63,  2.66,  0.00),
    ('2号货架', -7.57, -7.41, -3.08),
    ('3号货架',  1.56, -6.17,  0.00),
    ('4号货架', -8.62,  7.02,  1.55),
    ('5号货架', -7.81, -2.51, -3.10),
]

# ── 槽内顶升点：槽口中心沿开口方向内推 SLOT_INSIDE_OFFSET ──
#     取值 0.468 来自用户给的「1号槽内 x=3.000」− slot_1 槽口中心 x=2.53198；
#     与货物实际摆放中心（CARGO_SPOTS 的 x≈3.02874）相差 <30 mm，落在车顶升盘范围内。
SLOT_INSIDE_OFFSET = 0.468
SLOT_INSIDE_SUFFIX = '号槽内'

# ── 导航点（kind: nav）：只用于组成路线，不触发搬货 ──
NAV_POINTS = [
    ('1点',  0.0,  0.0, 0.0),
    ('2点', -7.0,  2.4, 0.0),
    ('3点', -3.5, -3.3, 0.0),
    ('4点', -0.1,  5.0, 0.0),
]
ROUTE_NAME = '1号'                   # 路线名（routes.yaml）

# ── 地图栅格 ──
RESOLUTION = 0.025                  # m/格
MARGIN = 0.50                       # 地图外扩边距(m)
OCCUPIED_GRAY = 0                   # PGM 里障碍用纯黑

# 颜色（world 视觉用）
COL_WALL  = '0.55 0.55 0.60 1'
COL_SLOT  = '0.45 0.55 0.75 1'      # 槽位蓝灰
COL_CARGO = '0.85 0.65 0.15 1'      # 货物黄色
COL_FLOOR = '0.75 0.75 0.75 1'


# ═══════════════════════════════════════════════════════════════════════
# 二、几何工具（旋转矩形 / 射线净空 / 点位生成）
# ═══════════════════════════════════════════════════════════════════════

def rot_rect_corners(cx, cy, sx, sy, yaw):
    """旋转矩形四角（世界坐标），用于包围盒与点测试。"""
    hx, hy = sx / 2.0, sy / 2.0                    # use 半长半宽
    c, s = math.cos(yaw), math.sin(yaw)            # use 旋转量
    pts = []
    for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
        pts.append((cx + dx * c - dy * s, cy + dx * s + dy * c))
    return pts


def point_in_rect(px, py, cx, cy, sx, sy, yaw, pad=0.0):
    """点是否落在"旋转矩形 + 外扩 pad"内（把点转进矩形局部系再比半长半宽）。"""
    c, s = math.cos(-yaw), math.sin(-yaw)          # use 反向旋转到矩形系
    dx, dy = px - cx, py - cy                      # use 相对矩形中心
    lx = dx * c - dy * s                           # use 局部 x
    ly = dx * s + dy * c                           # use 局部 y
    return abs(lx) <= sx / 2.0 + pad and abs(ly) <= sy / 2.0 + pad


def rects_block(rects, px, py, pad=0.0):
    """点是否被任一方形障碍挡住（rects 每项 (cx,cy,sx,sy,yaw)）。"""
    return any(point_in_rect(px, py, *r, pad=pad) for r in rects)


def ray_clear(rects, cx, cy, ang, dist, step=0.05):
    """从 (cx,cy) 沿 ang 方向打射线，返回撞到障碍前的自由距离（≤dist）。"""
    n = int(dist / step)                            # use 采样步数
    last = 0.0
    for i in range(1, n + 1):
        d = i * step
        if rects_block(rects, cx + math.cos(ang) * d, cy + math.sin(ang) * d):
            return last
        last = d
    return dist


def slot_rects(spot_x, spot_y, open_ang):
    """一个凵型槽位在"开口方向 open_ang"下的三块实体（世界坐标旋转矩形）。

    局部系：原点在槽口中心，+x 指向槽内（=插入方向 = open_ang），+y 为左。
    世界系下：局部 +x 对应角度 open_ang。
    """
    c, s = math.cos(open_ang), math.sin(open_ang)   # use 局部→世界的旋转

    def to_world(lx, ly):                           # in 局部坐标 → 世界坐标
        return (spot_x + lx * c - ly * s, spot_y + lx * s + ly * c)

    il, iw, wt = SLOT_INNER_LENGTH, SLOT_INNER_WIDTH, SLOT_WALL_TH
    ow, bd = SLOT_OUTER_WIDTH, SLOT_BASE_DEPTH
    rects = []
    lx, ly = to_world(il / 2.0, iw / 2.0 + wt / 2.0)          # 左壁
    rects.append((lx, ly, il, wt, open_ang))
    lx, ly = to_world(il / 2.0, -(iw / 2.0 + wt / 2.0))      # 右壁
    rects.append((lx, ly, il, wt, open_ang))
    lx, ly = to_world(il + bd / 2.0, 0.0)                    # 槽底挡板
    rects.append((lx, ly, bd, ow, open_ang))
    return rects


def build_layout():
    """由墙 + 槽位落点（用户给定），算出：槽位 / 货物方块 / 搬货点点位。"""
    walls = [(cx, cy, sx, sy, yaw) for (cx, cy, sx, sy, yaw) in WALLS]

    slots = []                                      # def 槽位清单
    for (name, sx, sy, oa) in SLOT_SPOTS:
        corridor = ray_clear(walls, sx, sy, oa + math.pi, 3.0)   # use 槽口外净距
        slots.append({'name': name, 'x': sx, 'y': sy,
                      'open_ang': oa, 'corridor': corridor})

    # ── 货物方块：坐在槽顶上（位姿沿用用户世界里的实际摆放）──
    cargos = []                                     # def 货物清单
    for cfg in CARGO_SPOTS:
        cargos.append({'name': cfg['name'], 'x': cfg['x'], 'y': cfg['y'],
                       'z': CARGO_SEAT_Z, 'yaw': cfg['yaw'], 'slot': cfg['slot']})

    # ── 搬货点点位（用户给的坐标/朝向 + 内推得到的槽内点）──
    stops = []                                      # def 搬货点清单
    for (name, px, py, yaw), slot in zip(STOP_POINTS, slots):
        sx, sy, oa = slot['x'], slot['y'], slot['open_ang']     # use 槽口中心与开口方向
        stops.append({
            'name': name,
            'x': px, 'y': py, 'yaw': yaw,            # 车头朝 +插入方向 = 正对槽口
            'inside_name': f'{slot["name"][-1]}{SLOT_INSIDE_SUFFIX}',   # use 例：slot_1 → 1号槽内
            'inside_x': sx + SLOT_INSIDE_OFFSET * math.cos(oa),  # use 槽内顶升点
            'inside_y': sy + SLOT_INSIDE_OFFSET * math.sin(oa),
            'inside_yaw': oa,                        # use 槽内朝向取开口方向精确值（须与槽轴对齐）
            'slot': slot['name'],
        })
    return walls, slots, cargos, stops


# ═══════════════════════════════════════════════════════════════════════
# 三、生成 SDF 世界
# ═══════════════════════════════════════════════════════════════════════

def _box_model(name, cx, cy, cz, sx, sy, sz, yaw, color, kinematic=False, mass=1.0):
    """生成一个"单连杆长方体"模型（墙/槽用静态；货物用 kinematic、带惯量，可被脚本摆放）。"""
    if not kinematic:
        return f"""    <model name='{name}'>
      <static>true</static>
      <pose>{cx:.6f} {cy:.6f} {cz:.6f} 0 0 {yaw:.6f}</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>"""
    # kinematic 体：有质量/惯量（"真有重量"），但位姿可由脚本摆放——当前阶段它只是场景道具。
    # 不受重力、不会被物理推挤（以后接仿真搬货插件时，用脚本设位姿表现被顶起/放下）。
    ixx = mass * (sy * sy + sz * sz) / 12.0
    iyy = mass * (sx * sx + sz * sz) / 12.0
    izz = mass * (sx * sx + sy * sy) / 12.0
    return f"""    <model name='{name}'>
      <static>false</static>
      <kinematic>true</kinematic>
      <pose>{cx:.6f} {cy:.6f} {cz:.6f} 0 0 {yaw:.6f}</pose>
      <link name='link'>
        <inertial>
          <mass>{mass}</mass>
          <inertia><ixx>{ixx:.6f}</ixx><ixy>0</ixy><ixz>0</ixz>
                   <iyy>{iyy:.6f}</iyy><iyz>0</iyz><izz>{izz:.6f}</izz></inertia>
        </inertial>
        <collision name='collision'>
          <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>"""


def _slot_model(model_name, spot_x, spot_y, open_ang):
    """凵型槽位：三块实体拼成（左壁 + 右壁 + 槽底挡板），整体绕槽口中心旋转 open_ang。

    做法：把三块实体各自换算到世界系，用 _box_model 生成静态盒（不复用局部坐标，
    避免 SDF 里再套一层 pose 造成旋转叠加出错）。
    """
    parts = []
    for i, (cx, cy, sx, sy, yaw) in enumerate(slot_rects(spot_x, spot_y, open_ang)):
        parts.append(_box_model(f'{model_name}_part{i}', cx, cy, SLOT_HEIGHT / 2.0,
                                sx, sy, SLOT_HEIGHT, yaw, COL_SLOT))
    return '\n'.join(parts)


def build_world(walls, slots, cargos):
    out = ["<?xml version='1.0' ?>",
           "<sdf version='1.7'>",
           "  <world name='default'>",
           "    <gravity>0 0 -9.8</gravity>",
           "    <physics type='ode'>",
           "      <max_step_size>0.001</max_step_size>",
           "      <real_time_factor>1.0</real_time_factor>",
           "      <real_time_update_rate>1000</real_time_update_rate>",
           "    </physics>",
           "    <scene>",
           "      <ambient>0.4 0.4 0.4 1</ambient>",
           "      <background>0.7 0.7 0.7 1</background>",
           "      <shadows>1</shadows>",
           "    </scene>",
           "    <light name='sun' type='directional'>",
           "      <cast_shadows>1</cast_shadows>",
           "      <pose>0 0 10 0 -0 0</pose>",
           "      <diffuse>0.8 0.8 0.8 1</diffuse>",
           "      <specular>0.2 0.2 0.2 1</specular>",
           "      <direction>-0.5 0.1 -0.9</direction>",
           "    </light>",
           "    <model name='ground_plane'>",
           "      <static>true</static>",
           "      <link name='link'>",
           "        <collision name='collision'>",
           "          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>",
           "        </collision>",
           "        <visual name='visual'>",
           "          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>",
           f"          <material><ambient>{COL_FLOOR}</ambient><diffuse>{COL_FLOOR}</diffuse></material>",
           "        </visual>",
           "      </link>",
           "    </model>",
           "",
           "    <!-- ===== 墙体（外墙 + 内部隔墙，沿用用户世界几何）===== -->"]
    for i, (cx, cy, sx, sy, yaw) in enumerate(walls):
        out.append(_box_model(f'wall_{i}', cx, cy, WALL_H / 2, sx, sy, WALL_H, yaw, COL_WALL))
    out.append("")
    out.append("    <!-- ===== 凵型槽位（水平放置，开口朝通道，供 AGV 停到槽口前）===== -->")
    for slot in slots:
        out.append(_slot_model(slot['name'], slot['x'], slot['y'], slot['open_ang']))
    out.append("")
    out.append("    <!-- ===== 货物方块（坐在槽顶上；kinematic 体，位姿可由脚本摆放）=====")
    out.append("         （当前阶段它只是场景道具，搬货动作留给 /agv/cargo 服务）===== -->")
    for cg in cargos:
        out.append(_box_model(cg['name'], cg['x'], cg['y'], cg['z'],
                              CARGO_SIZE_X, CARGO_SIZE_Y, CARGO_SIZE_Z,
                              cg['yaw'], COL_CARGO, kinematic=True, mass=CARGO_MASS))
    out.append("")
    out.append("    <!-- ===== 让 /gazebo/set_entity_state 与 get_entity_state 可用 =====")
    out.append("         （需要脚本摆放实体位姿时用它，例如以后接仿真搬货插件）===== -->")
    out.append("    <plugin name='gazebo_ros_state' filename='libgazebo_ros_state.so'>")
    out.append("      <ros><namespace>/gazebo</namespace></ros>")
    out.append("      <update_rate>50.0</update_rate>")
    out.append("    </plugin>")
    out.append("  </world>")
    out.append("</sdf>")
    return '\n'.join(out) + '\n'


# ═══════════════════════════════════════════════════════════════════════
# 四、生成栅格地图（与上面的世界一一对应）
# ═══════════════════════════════════════════════════════════════════════

def _rasterize_rect(grid, W, H, ox, oy, cx, cy, sx, sy, yaw):
    """把一个旋转矩形"烧"进栅格（0=障碍）。"""
    for (px, py) in rot_rect_corners(cx, cy, sx, sy, yaw):
        pass
    # 用包围盒 + 逐格点测试，简单可靠（格子数不多，几十毫秒级）
    hx = (abs(sx * math.cos(yaw)) + abs(sy * math.sin(yaw))) / 2.0
    hy = (abs(sx * math.sin(yaw)) + abs(sy * math.cos(yaw))) / 2.0
    c0 = max(int((cx - hx - ox) / RESOLUTION) - 1, 0)
    c1 = min(int((cx + hx - ox) / RESOLUTION) + 1, W - 1)
    r0 = max(int((cy - hy - oy) / RESOLUTION) - 1, 0)
    r1 = min(int((cy + hy - oy) / RESOLUTION) + 1, H - 1)
    for row in range(r0, r1 + 1):
        wy = oy + (row + 0.5) * RESOLUTION
        for col in range(c0, c1 + 1):
            wx = ox + (col + 0.5) * RESOLUTION
            if point_in_rect(wx, wy, cx, cy, sx, sy, yaw):
                grid[row * W + col] = OCCUPIED_GRAY


def build_map(walls, slots, ox, oy, W, H):
    """生成占用栅格：255=空地，0=障碍（墙 + 槽位实体）。"""
    grid = bytearray([255] * (W * H))
    for (cx, cy, sx, sy, yaw) in walls:
        _rasterize_rect(grid, W, H, ox, oy, cx, cy, sx, sy, yaw)
    for slot in slots:
        for (cx, cy, sx, sy, yaw) in slot_rects(slot['x'], slot['y'], slot['open_ang']):
            _rasterize_rect(grid, W, H, ox, oy, cx, cy, sx, sy, yaw)
    return grid


def write_pgm(path, grid, W, H):
    """写 P5 二进制 PGM（Nav2 map_server 认这个格式）。"""
    with open(path, 'wb') as f:
        f.write(b'P5\n')
        f.write(b'# AGV world666 map (generated by make_world_map.py)\n')
        f.write(f'{W} {H}\n255\n'.encode('ascii'))
        f.write(bytes(grid))


def write_yaml(path, ox, oy, image_name):
    """写地图元数据（原点/分辨率/阈值）。"""
    with open(path, 'w', encoding='utf-8', newline='\n') as f:   # use 固定 LF：与仓库其它 YAML 一致
        f.write(f"""image: {image_name}
resolution: {RESOLUTION}
origin: [{ox:.3f}, {oy:.3f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
""")


def write_waypoints(path, stops, frame='map'):
    """写出点位文件（5 个槽口对位点 + 5 个槽内顶升点 + 起始点 + 4 个导航点）。

    · 「X号货架」= 槽口对位点（kind: cargo）：停在槽口外约 0.9 m、车头正对槽口。
    · 「X号槽内」= 槽内顶升点（kind: cargo）：钻进槽里、停在货物正下方，
      即槽口中心沿开口方向内推 SLOT_INSIDE_OFFSET（0.468 ≈ 用户给的 3.000 − 2.53198）。
    · 「N点」   = 导航点（kind: nav）：只用来组成 routes.yaml 里的路线。
    """
    lines = ['# 由 make_world_map.py 生成（用户 world666 场景）',
             '# 点位分两类（kind）：',
             '#   cargo = 搬货点：到达后由任务层调用 /agv/cargo 触发顶升 / 下降。',
             '#           「X号货架」停在槽口外约 0.9 m、车头正对槽口（接近/对位点）；',
             '#           「X号槽内」是钻进凵型槽、停在货物正下方的顶升点。',
             '#   nav   = 导航点：只用来组成路线，单独 goto 到它不会触发搬货服务。',
             f'frame_id: {frame}',
             'waypoints:']
    lines.append('  起始点: {x: 0.0, y: 0.0, yaw: 0.0, kind: cargo}')
    lines.append('')
    lines.append('  # ---- 槽内顶升点（车钻进槽里，停在货物正下方）----')
    for p in stops:
        lines.append(f'  {p["inside_name"]}:')
        lines.append(f'    x: {p["inside_x"]:.3f}')
        lines.append(f'    y: {p["inside_y"]:.3f}')
        lines.append(f'    yaw: {p["inside_yaw"]:.3f}')
        lines.append('    kind: cargo')
    lines.append('')
    lines.append('  # ---- 槽口对位点（车停在槽外，车头正对槽口）----')
    for p in stops:
        lines.append(f'  {p["name"]}:')
        lines.append(f'    x: {p["x"]:.3f}')
        lines.append(f'    y: {p["y"]:.3f}')
        lines.append(f'    yaw: {p["yaw"]:.3f}')
        lines.append('    kind: cargo')
    lines.append('')
    lines.append('  # ---- 导航点（供路线经过；不触发搬货）----')
    for name, px, py, yaw in NAV_POINTS:
        lines.append(f'  {name}:')
        lines.append(f'    x: {px:.3f}')
        lines.append(f'    y: {py:.3f}')
        lines.append(f'    yaw: {yaw:.3f}')
        lines.append('    kind: nav')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:   # use 固定 LF：与仓库其它 YAML 一致
        f.write('\n'.join(lines) + '\n')


def write_routes(path, frame='map'):
    """写出 1 号巡检路线（途经 4 个导航点；坐标是存点时的快照）。"""
    lines = ['# 本文件的路线与地图绑定（路线由若干点位的位姿快照组成）',
             '# 每条路线 = 按行进顺序排列的途经点（坐标是存点时的快照，改点位不影响已存路线）。',
             f'frame_id: {frame}',
             'routes:',
             '  # 绕场地一圈的巡检路线，途经 4 个导航点后回到起点附近',
             f'  {ROUTE_NAME}:',
             '    points:']
    for name, px, py, yaw in NAV_POINTS:
        lines.append(f'      - {{name: {name}, x: {px:.3f}, y: {py:.3f}, yaw: {yaw:.3f}}}')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:   # use 固定 LF：与仓库其它 YAML 一致
        f.write('\n'.join(lines) + '\n')


# ═══════════════════════════════════════════════════════════════════════
# 五、自检：用 agv_common.map_check 验证停留点可达、通道够宽
# ═══════════════════════════════════════════════════════════════════════

def selfcheck(yaml_path, stops):
    """用 MapChecker 复核停留点净空 + 起点到各停留点的瓶颈通道宽。"""
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    '..', 'src', 'agv_common'))
    from agv_common.map_check import MapChecker
    mc = MapChecker(yaml_path, robot_clearance=0.30)
    print('\n─── 自检（MapChecker，需净空 ≥0.30 m）───')
    bad = 0
    for p in stops:
        ok, reason = mc.check_point(p['x'], p['y'])
        flag = 'OK ' if ok else 'BAD'
        if not ok:
            bad += 1
        print(f'  [{flag}] {p["name"]:8s} {reason}')
    for p in stops:
        b = mc.bottleneck(0.0, 0.0, p['x'], p['y'])
        bt = 'n/a' if b is None else f'{b:.2f} m'
        print(f'  [瓶颈] 起点→{p["name"]}: {bt}')
    # 槽内顶升点：净空更紧（车要钻进槽里），单独复核
    print('  ── 槽内顶升点（车需钻进槽内）──')
    for p in stops:
        ok, reason = mc.check_point(p['inside_x'], p['inside_y'])
        flag = 'OK ' if ok else 'BAD'
        if not ok:
            bad += 1
        print(f'  [{flag}] {p["inside_name"]:8s} {reason}')
    print(f'  —— 结论：{"全部通过" if bad == 0 else f"{bad} 个点不合格"} ——')
    return bad


# ═══════════════════════════════════════════════════════════════════════
# 主流程
# ═══════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description='生成 AGV 场景 world + 地图 + 点位')
    ap.add_argument('--out-dir', default='', help='输出目录（默认写到各包的正式位置）')
    ap.add_argument('--dry-run', dest='write_pkg', action='store_false',
                    help='只生成不落包（调试用）')
    ap.set_defaults(write_pkg=True)
    args = ap.parse_args()

    ws = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    walls, slots, cargos, stops = build_layout()

    # 地图原点：由"墙 + 槽位实体"的包围盒 + 边距决定
    boxes = list(walls)
    for slot in slots:
        boxes += slot_rects(slot['x'], slot['y'], slot['open_ang'])
    xs, ys = [], []
    for (cx, cy, sx, sy, yaw) in boxes:
        for (px, py) in rot_rect_corners(cx, cy, sx, sy, yaw):
            xs.append(px)
            ys.append(py)
    ox, oy = min(xs) - MARGIN, min(ys) - MARGIN
    x1, y1 = max(xs) + MARGIN, max(ys) + MARGIN
    W = int(round((x1 - ox) / RESOLUTION))
    H = int(round((y1 - oy) / RESOLUTION))

    print(f'场景：{len(walls)} 段墙 + {len(slots)} 个凵型槽位 + {len(cargos)} 个货物方块（坐槽顶）')
    for slot in slots:
        print(f'  {slot["name"]}: 中心 ({slot["x"]:.2f},{slot["y"]:.2f}) '
              f'开口朝 {math.degrees(slot["open_ang"]):6.1f}°  槽口外净距 {slot["corridor"]:.2f} m')
    print(f'地图：{W}×{H} 格 @ {RESOLUTION} m，原点 ({ox:.3f}, {oy:.3f})')
    print(f'场地范围（供 task.yaml field_min/max 用）：'
          f'x[{ox:.2f},{x1:.2f}] y[{oy:.2f},{y1:.2f}]')

    world_txt = build_world(walls, slots, cargos)
    grid = build_map(walls, slots, ox, oy, W, H)

    if args.out_dir:
        out_world = os.path.join(args.out_dir, 'world666.world')
        out_pgm = os.path.join(args.out_dir, 'world666map.pgm')
        out_yaml = os.path.join(args.out_dir, 'world666map.yaml')
        out_wp = os.path.join(args.out_dir, 'waypoints.yaml')
        out_rt = os.path.join(args.out_dir, 'routes.yaml')
        os.makedirs(args.out_dir, exist_ok=True)
    else:
        desc = os.path.join(ws, 'src', 'agv_description', 'worlds')
        navmaps = os.path.join(ws, 'src', 'agv_navigation', 'maps', 'world666map')
        store = os.path.join(ws, 'maps', 'world666map')
        out_world = os.path.join(desc, 'world666.world')
        out_pgm = os.path.join(navmaps, 'world666map_ref.pgm')
        out_yaml = os.path.join(navmaps, 'world666map_ref.yaml')
        out_wp = os.path.join(navmaps, 'waypoints.yaml')
        out_rt = os.path.join(navmaps, 'routes.yaml')

    os.makedirs(os.path.dirname(out_world), exist_ok=True)
    os.makedirs(os.path.dirname(out_pgm), exist_ok=True)
    with open(out_world, 'w', encoding='utf-8') as f:
        f.write(world_txt)
    write_pgm(out_pgm, grid, W, H)
    write_yaml(out_yaml, ox, oy, os.path.basename(out_pgm))
    write_waypoints(out_wp, stops)
    write_routes(out_rt)
    print(f'已写出：\n  {out_world}\n  {out_pgm}\n  {out_yaml}\n  {out_wp}\n  {out_rt}')

    if args.write_pkg and not args.out_dir:         # 正式落包：同步点位/路线到运行时地图库
        os.makedirs(store, exist_ok=True)
        for src, dst in ((out_wp, 'waypoints.yaml'), (out_rt, 'routes.yaml')):
            with open(os.path.join(store, dst), 'w', encoding='utf-8') as f:
                f.write(open(src, encoding='utf-8').read())
        print(f'同时同步点位/路线到运行时地图库：{store}/')
        print('（栅格地图 maps/world666map/world666map.pgm 以控制台建图实测为准，本脚本不覆盖）')

    bad = selfcheck(out_yaml, stops)
    print('\n─── 货物落座（场景道具；搬货动作由 /agv/cargo 服务负责）───')
    for cg in cargos:
        print(f'  {cg["name"]} 坐在 {cg["slot"]} 顶上 ({cg["x"]:.2f}, {cg["y"]:.2f}, z={cg["z"]:.2f})')
    return 1 if bad else 0


if __name__ == '__main__':
    raise SystemExit(main())
