"""
goal_check 单元测试：目标点合法性的唯一判据。

分两部分：
  · 用合成小地图测判据逻辑（与 test_map_check.py 同思路，不依赖真实地图）；
  · 用**真实实测地图**做验收回归——用户明确要求「不在 map 的点或者身处于障碍物的点
    直接拒绝」，但同时「车真能进槽作业，以后是需要的」。所以必须证明：
    收紧后地图外被拒，而**槽内作业点与搬货点仍全部放行**。
    真实地图不存在时自动跳过这部分（保持测试可移植）。
"""

import pathlib

import yaml

from agv_common.goal_check import GoalValidator, rect_validator

# 真实实测地图位置（相对本文件：src/agv_common/test/ → src/agv_navigation/...）
REAL_MAP = (pathlib.Path(__file__).resolve().parents[2]
            / 'agv_navigation' / 'maps' / 'world666map' / 'world666map.yaml')


# ══════════════════ 无地图：矩形兜底 ══════════════════

def test_no_map_uses_rect_fallback():    # # 不给地图 → 用矩形兜底
    v = rect_validator(-1.0, 1.0, -2.0, 2.0)
    assert v.map_based() is False
    ok, why = v.check(0.5, 1.0)
    assert ok is True
    ok, why = v.check(2.0, 0.0)
    assert ok is False and '超出场地范围' in why


def test_no_map_and_no_field_allows_all():   # # 既无地图也无范围 → 不拦
    v = GoalValidator(map_yaml='')
    assert v.check(99999.0, -99999.0)[0] is True


def test_default_field_is_unbounded():   # # 缺省兜底范围极宽（等于不拦）
    v = GoalValidator(map_yaml='', field={'min_x': -1.0})
    assert v.check(0.0, 0.0)[0] is True


def test_bad_map_falls_back_and_records_reason():   # # 地图读不到 → 退化并记原因
    v = GoalValidator(map_yaml='D:/绝对不存在的图.yaml',
                      field={'min_x': -1.0, 'max_x': 1.0,
                             'min_y': -1.0, 'max_y': 1.0})
    assert v.map_based() is False
    assert v.error                      # # 记录了失败原因
    assert '矩形兜底' in v.describe()
    assert v.check(5.0, 0.0)[0] is False


def test_non_numeric_coord_rejected():   # # 坐标不是数字：拒绝而不是抛异常
    v = GoalValidator(map_yaml='')
    ok, why = v.check('不是数字', 0.0)
    assert ok is False and '非法' in why


def test_describe_reports_map_when_loaded():   # # 有图时 describe 说明用的是地图
    if not REAL_MAP.is_file():
        return
    v = GoalValidator(map_yaml=str(REAL_MAP), robot_clearance=0.30)
    assert v.map_based() is True
    assert '实测地图' in v.describe() and '所需净空' in v.describe()


# ══════════════════ 真实地图：验收回归 ══════════════════

def _real_validator():                   # # 造真实地图判据（无图则跳过）
    if not REAL_MAP.is_file():
        return None
    return GoalValidator(map_yaml=str(REAL_MAP), robot_clearance=0.30)


# 五个槽内作业点（来自 world666.world / make_world_map.py 的 SLOT_SPOTS）
SLOT_SPOTS = {                           # def 槽内作业点
    'slot_1': (2.53198, 2.69159),
    'slot_2': (-8.47018, -7.49172),
    'slot_3': (2.45799, -6.13987),
    'slot_4': (-8.63511, 7.91659),
    'slot_5': (-8.71246, -2.57649),
}

# 地图自带的搬货/起始点（maps/world666map/waypoints.yaml）
CARGO_SPOTS = {                          # def 搬货点
    '起始点': (0.0, 0.0),
    '1号货架': (1.630, 2.660),
    '2号货架': (-7.570, -7.410),
    '3号货架': (1.560, -6.170),
    '4号货架': (-8.620, 7.020),
    '5号货架': (-7.810, -2.510),
}


def test_real_map_rejects_off_map_points():    # # ★ 地图外的点必须直接拒绝
    v = _real_validator()
    if v is None:
        return
    for x, y in ((6.0, 0.0), (-20.0, 0.0), (0.0, 15.0), (0.0, -15.0),
                 (9999.0, 9999.0), (-11.0, 0.0)):
        ok, why = v.check(x, y)
        assert ok is False, f'({x}, {y}) 应被拒绝，却放行了'
        assert '地图范围外' in why or '拒绝' in why


def test_real_map_accepts_slot_interior_points():   # # ★ 槽内作业点不得被误拦
    # 这是本次收紧判据的关键约束：用户要"能进槽作业"。
    # 槽内净空约 0.38~0.45 m，都 ≥ 0.30 m 的车身净空要求，应全部放行。
    v = _real_validator()
    if v is None:
        return
    for name, (x, y) in SLOT_SPOTS.items():
        ok, why = v.check(x, y)
        assert ok is True, f'槽内点 {name} ({x:.2f}, {y:.2f}) 被误拦：{why}'


def test_real_map_accepts_cargo_waypoints():   # # ★ 地图自带搬货点不得被误拦
    v = _real_validator()
    if v is None:
        return
    for name, (x, y) in CARGO_SPOTS.items():
        ok, why = v.check(x, y)
        assert ok is True, f'搬货点 {name} ({x:.2f}, {y:.2f}) 被误拦：{why}'


def test_real_map_slot_points_pass_just_above_clearance():   # # 槽内净空确实过线
    # 记录事实：槽内点净空必须真的 ≥ 车身净空，而不是"恰好蒙对"。
    v = _real_validator()
    if v is None:
        return
    for name, (x, y) in SLOT_SPOTS.items():
        clr = v.checker.clearance_at(x, y)
        assert clr is not None and clr >= v.robot_clearance, \
            f'槽内点 {name} 净空 {clr:.2f} < 要求 {v.robot_clearance:.2f}'


def test_real_map_matches_task_yaml_bounds():   # # 兜底矩形与实测地图对齐（配置守卫）
    # field_* 现在只是"没有地图时"的兜底；若它与实测地图不一致，
    # 地图读不到时反而会放行图外坐标。这条守卫防止它再漂移。
    if not REAL_MAP.is_file():
        return
    task_yaml = (pathlib.Path(__file__).resolve().parents[2]
                 / 'agv_bringup' / 'config' / 'default' / 'task.yaml')
    if not task_yaml.is_file():
        return
    cfg = yaml.safe_load(task_yaml.read_text(encoding='utf-8'))
    meta = yaml.safe_load(REAL_MAP.read_text(encoding='utf-8'))
    v = GoalValidator(map_yaml=str(REAL_MAP), robot_clearance=0.30)
    c = v.checker
    assert abs(cfg['field_min'] - c.ox) < 0.05
    assert abs(cfg['field_max'] - (c.ox + c.W * c.res)) < 0.05
    assert abs(cfg['field_min_y'] - c.oy) < 0.05
    assert abs(cfg['field_max_y'] - (c.oy + c.H * c.res)) < 0.05
    assert 'precheck_enabled' not in cfg        # # 旧开关已移除


def test_real_map_boundary_values_are_in_map():   # # 兜底边界本身落在地图内
    v = _real_validator()
    if v is None:
        return
    c = v.checker
    # 左下角最靠近地图边缘：净空可能不足而拒绝，但绝不能是"图外"
    ok, why = v.check(c.ox, c.oy)
    assert '地图范围外' not in why
