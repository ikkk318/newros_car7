"""reactive_avoid 单元测试：确定性避障仲裁的三层逻辑。"""
import math

from agv_common.reactive_avoid import ReactiveAvoider


def _scan(ranges, angle_min=-math.pi / 2, angle_increment=math.pi / 180.0):
    """构造一圈激光：默认 -90°~+90°，1° 一点。返回 (ranges, angle_min, inc)。"""
    return list(ranges), angle_min, angle_increment


def _uniform(n, value):
    return [value] * n


def test_far_obstacle_passes_nav_through():
    """前方很远：线速度不衰减、角速度原样跟随导航。"""
    av = ReactiveAvoider()
    r, amin, inc = _scan(_uniform(181, 5.0))
    v, w, info = av.arbitrate(0.4, 0.3, r, amin, inc)
    assert v == 0.4 and w == 0.3
    assert info['engaged'] is False


def test_close_obstacle_scales_down_speed():
    """前方中距：线速度按距离比例下降（介于全速与刹停之间）。"""
    av = ReactiveAvoider(stop_distance=0.3, slow_distance=0.9, take_over_distance=0.2)
    r, amin, inc = _scan(_uniform(181, 0.6))   # 0.6m，落在 0.3~0.9 过渡段
    v, w, info = av.arbitrate(0.4, 0.0, r, amin, inc)
    assert 0.0 < v < 0.4
    assert info['engaged'] is False            # 未进入择向（阈值 0.2 未触发）


def test_very_close_obstacle_stops():
    """前方太近：线速度直接为 0。"""
    av = ReactiveAvoider(stop_distance=0.3)
    r, amin, inc = _scan(_uniform(181, 0.2))
    v, w, info = av.arbitrate(0.4, 0.0, r, amin, inc)
    assert v == 0.0


def test_zero_nav_speed_is_not_raised():
    """★ 上层要停（v_nav=0）时，减速下限不得把它抬成前进速度。

    回归用：曾经 v_out = max(slow_min_speed, v_nav*ratio) 会把 Nav2 的 0 改成 0.10，
    到点原地对准时被进度检查器判"没进展" → 反复恢复行为 → 到点前原地转。
    """
    av = ReactiveAvoider(stop_distance=0.3, slow_distance=0.9, slow_min_speed=0.1)
    r, amin, inc = _scan(_uniform(181, 0.6))       # 0.6m 落在 0.3~0.9 过渡段
    v, w, _ = av.arbitrate(0.0, 0.4, r, amin, inc)
    assert v == 0.0


def test_slow_floor_never_exceeds_nav_speed():
    """下限只用来"别爬太慢"，结果不得超过上层给的速度。"""
    av = ReactiveAvoider(stop_distance=0.3, slow_distance=0.9, slow_min_speed=0.1)
    r, amin, inc = _scan(_uniform(181, 0.6))
    v, _, _ = av.arbitrate(0.05, 0.0, r, amin, inc)   # 上层只给 0.05
    assert v <= 0.05


def test_takeover_left_when_left_is_clearer():
    """左前方空、右前方堵：择向向左偏（角速度为正/逆时针）。"""
    av = ReactiveAvoider(take_over_distance=0.7, front_half_angle=math.radians(30),
                         side_max_angle=math.radians(85))
    # 默认 1° 一点，181 点；构建左右不对称的扫描
    samples = []
    for i in range(181):
        ang = -math.pi / 2 + i * math.pi / 180.0     # -90°..+90°
        # 右侧（负角）近，左侧（正角）远
        samples.append(0.35 if ang < 0 else 3.0)
    v, w, info = av.arbitrate(0.3, 0.0, samples, -math.pi / 2, math.pi / 180.0)
    assert info['engaged'] is True
    assert w > 0                                   # 向左偏


def test_takeover_right_when_right_is_clearer():
    """右前方空、左前方堵：择向向右偏（角速度为负/顺时针）。"""
    av = ReactiveAvoider(take_over_distance=0.7, front_half_angle=math.radians(30),
                         side_max_angle=math.radians(85))
    samples = []
    for i in range(181):
        ang = -math.pi / 2 + i * math.pi / 180.0
        samples.append(0.35 if ang > 0 else 3.0)   # 左侧近、右侧远
    v, w, info = av.arbitrate(0.3, 0.0, samples, -math.pi / 2, math.pi / 180.0)
    assert info['engaged'] is True
    assert w < 0                                   # 向右偏


def test_handback_is_gradual_not_instant():
    """障碍远离后：角速度分步回归导航值，不会一步跳回去（防蛇形）。"""
    av = ReactiveAvoider(take_over_distance=0.7, yaw_handback_rate=0.2,
                         front_half_angle=math.radians(30), side_max_angle=math.radians(85))
    near = []
    for i in range(181):
        ang = -math.pi / 2 + i * math.pi / 180.0
        near.append(0.35 if ang < 0 else 3.0)
    v, w, _ = av.arbitrate(0.3, 0.0, near, -math.pi / 2, math.pi / 180.0)
    assert w > 0 and av.engaged()
    # 障碍远去：一次 dt 内角速度只走一步，且方向朝导航的 0 收敛
    r_far, amin, inc = _scan(_uniform(181, 5.0))
    v2, w2, info2 = av.arbitrate(0.3, 0.0, r_far, amin, inc, dt=0.1)
    assert 0.0 <= w2 < w                           # 朝 0 走了一步，但没到 0


def test_handback_completes_and_releases():
    """持续远离：若干步后角速度回到导航值，并解除接管。"""
    av = ReactiveAvoider(take_over_distance=0.7, yaw_handback_rate=0.5,
                         front_half_angle=math.radians(30), side_max_angle=math.radians(85))
    near = []
    for i in range(181):
        ang = -math.pi / 2 + i * math.pi / 180.0
        near.append(0.35 if ang < 0 else 3.0)
    av.arbitrate(0.3, 0.0, near, -math.pi / 2, math.pi / 180.0)
    r_far, amin, inc = _scan(_uniform(181, 5.0))
    for _ in range(50):
        _, w, info = av.arbitrate(0.3, 0.0, r_far, amin, inc, dt=0.1)
    assert abs(w - 0.0) < 1e-3
    assert info['engaged'] is False


def test_reset_clears_state():
    """reset 后不残留接管状态。"""
    av = ReactiveAvoider(take_over_distance=0.7, front_half_angle=math.radians(30))
    r, amin, inc = _scan(_uniform(181, 0.3))
    av.arbitrate(0.3, 0.0, r, amin, inc)
    av.reset()
    assert av.engaged() is False


def test_reverse_is_not_hijacked_by_steering():
    """★ 倒车时不得夺走转向：否则车在槽内直退会变成左右摇摆，出不来。

    回归用：车在凵型槽内、前方 0.38 m 就是槽底挡板，`front` 恒小于接管阈值。
    若倒车时仍按"左右哪边空"抢转向，车身横向偏移符号一变、转向命令就整个翻过来
    → 原地左右摇摆。倒车时前向扇区数据无意义（车在往后走），必须豁免。
    """
    av = ReactiveAvoider(take_over_distance=0.7, front_half_angle=math.radians(30),
                         side_max_angle=math.radians(85))
    # 右侧近、左侧远（若被夺走转向，会得到 w>0）
    samples = []
    for i in range(181):
        ang = -math.pi / 2 + i * math.pi / 180.0
        samples.append(0.35 if ang < 0 else 3.0)
    v, w, info = av.arbitrate(-0.10, 0.0, samples, -math.pi / 2, math.pi / 180.0)
    assert v == -0.10                              # 倒车速度原样通过
    assert w == 0.0                                # ★ 转向不被夺走
    assert info['engaged'] is False


def test_reverse_steering_stays_symmetric():
    """倒车时左右对称的两种偏移给出相同转向，不再反向翻动。"""
    av = ReactiveAvoider(take_over_distance=0.7, front_half_angle=math.radians(30),
                         side_max_angle=math.radians(85))

    def one_side(near_left):
        s = []
        for i in range(181):
            ang = -math.pi / 2 + i * math.pi / 180.0
            s.append(0.35 if (ang > 0) == near_left else 3.0)
        return s

    _, w_left, _ = av.arbitrate(-0.10, 0.0, one_side(True), -math.pi / 2, math.pi / 180.0)
    av.reset()
    _, w_right, _ = av.arbitrate(-0.10, 0.0, one_side(False), -math.pi / 2, math.pi / 180.0)
    assert w_left == 0.0 and w_right == 0.0        # 两侧都不夺向


def test_reverse_ignores_front_slowdown():
    """倒车时前方太近也不刹停（前向减速对倒车不适用，改由后方锥负责）。"""
    av = ReactiveAvoider(stop_distance=0.3)
    r, amin, inc = _scan(_uniform(181, 0.2))
    v, _, _ = av.arbitrate(-0.10, 0.0, r, amin, inc)
    assert v == -0.10


def test_goal_alignment_rotation_is_not_hijacked():
    """★ 到点原地对准时不得夺走转向：否则航向永远收敛不了，车「到了却以为没到」。

    回归用：车已到达导航点（槽内），Nav2 只下发角速度做航向对准（v_nav=0）。
    此时前方 0.38 m 恒为槽底挡板、横向稍有偏移，接管条件全部满足，
    仲裁器会把 Nav2 的对准角速度换成自己选的固定 0.5 rad/s —— 车身左右摇摆，
    航向永远进不了 yaw_goal_tolerance(0.20)，goToPose 不返回，任务层等到超时报错。
    """
    av = ReactiveAvoider(take_over_distance=0.7, front_half_angle=math.radians(30),
                         side_max_angle=math.radians(85))
    samples = []
    for i in range(181):
        ang = -math.pi / 2 + i * math.pi / 180.0
        samples.append(0.35 if ang < 0 else 3.0)       # 右侧近、左侧远
    v, w, info = av.arbitrate(0.0, 0.3, samples, -math.pi / 2, math.pi / 180.0)
    assert v == 0.0
    assert w == 0.3, f'原地对准的角速度被改成 {w}，航向无法收敛'
    assert info['engaged'] is False


def test_standstill_near_obstacle_does_not_inject_rotation():
    """★ Nav2 说「停」（v=0, w=0）时不得凭空注入旋转。"""
    av = ReactiveAvoider(take_over_distance=0.7, front_half_angle=math.radians(30),
                         side_max_angle=math.radians(85))
    samples = []
    for i in range(181):
        ang = -math.pi / 2 + i * math.pi / 180.0
        samples.append(0.35 if ang < 0 else 3.0)
    v, w, info = av.arbitrate(0.0, 0.0, samples, -math.pi / 2, math.pi / 180.0)
    assert v == 0.0 and w == 0.0, f'停车指令被改成旋转 w={w}'
    assert info['engaged'] is False


def test_takeover_still_works_while_creeping_forward():
    """对照：真正在往前挪（v_nav>0）时，择向接管仍然有效——别把功能改死。"""
    av = ReactiveAvoider(take_over_distance=0.7, front_half_angle=math.radians(30),
                         side_max_angle=math.radians(85))
    samples = []
    for i in range(181):
        ang = -math.pi / 2 + i * math.pi / 180.0
        samples.append(0.35 if ang < 0 else 3.0)
    v, w, info = av.arbitrate(0.3, 0.0, samples, -math.pi / 2, math.pi / 180.0)
    assert info['engaged'] is True
    assert w > 0                                        # 仍向左偏


def test_invalid_readings_ignored():
    """inf/nan/0 等无效读数不影响判断。"""
    av = ReactiveAvoider()
    samples = [float('inf')] * 181
    samples[90] = float('nan')
    samples[91] = 0.0
    v, w, info = av.arbitrate(0.4, 0.2, samples, -math.pi / 2, math.pi / 180.0)
    assert v == 0.4 and w == 0.2                   # 全无效 → 视为很空，原样通过


# ══════════════════════════════════════════════════════════════════════
# 360° 全向雷达（−π ~ +π，361 点）——车已把雷达移到几何中心并开满一圈
# ══════════════════════════════════════════════════════════════════════

def _scan360(ranges, angle_min=-math.pi, angle_increment=math.pi / 180.0):
    """构造一圈全向激光：默认 -180°~+180°，1° 一点，361 点。"""
    return list(ranges), angle_min, angle_increment


def _uniform360(n, value):
    return [value] * n


def test_full_circle_scan_gives_same_result_as_half_circle():
    """★ 视场角无关：同一场景用 181 点半圈与 361 点全圈扫描，输出必须一致。

    雷达改到车体中心并开满 360° 后，正后方多出的数据不该改变前/左/右三扇区的判定。
    若哪天有人改了扇区边界或写死了点数，这条会立刻红。
    """
    av_half = ReactiveAvoider()
    av_full = ReactiveAvoider()
    # 半圈：前方 5 m 空
    v1, w1, i1 = av_half.arbitrate(0.4, 0.3, _uniform(181, 5.0), -math.pi / 2, math.pi / 180.0)
    # 全圈：同样的前方 5 m 空，但后方 2.0 m 有一堵墙（360° 才看得到）
    full = _uniform360(361, 5.0)
    for i in range(361):
        ang = -math.pi + i * math.pi / 180.0
        if abs(ang) > math.radians(100):           # 正后方一带放个近障碍
            full[i] = 2.0
    v2, w2, i2 = av_full.arbitrate(0.4, 0.3, full, -math.pi, math.pi / 180.0)
    assert (v1, w1) == (v2, w2)                    # 后方数据不参与前向仲裁
    assert i2['engaged'] is False


def test_360_scan_front_obstacle_still_stops():
    """全圈扫描下，正前方 0.2 m 障碍仍然刹停（前方扇区判定没被 360° 改坏）。"""
    av = ReactiveAvoider(stop_distance=0.3)
    full = _uniform360(361, 5.0)
    full[180] = 0.2                                # 索引 180 = 角度 0° = 正前方
    v, _, _ = av.arbitrate(0.4, 0.0, full, -math.pi, math.pi / 180.0)
    assert v == 0.0


def test_360_scan_side_bands_pick_correct_side():
    """全圈扫描下侧向择向仍正确：左侧空、右侧堵 → 向左偏。"""
    av = ReactiveAvoider(take_over_distance=0.7, front_half_angle=math.radians(30),
                         side_max_angle=math.radians(85))
    full = _uniform360(361, 5.0)
    for i in range(361):
        ang = -math.pi + i * math.pi / 180.0
        if math.radians(30) < ang < math.radians(85):      # 左侧带：空（保持 5.0）
            continue
        if -math.radians(85) < ang < -math.radians(30):    # 右侧带：堵
            full[i] = 0.35
        if abs(ang) <= math.radians(30):                   # 前方：近，触发接管
            full[i] = 0.35
    v, w, info = av.arbitrate(0.3, 0.0, full, -math.pi, math.pi / 180.0)
    assert info['engaged'] is True
    assert w > 0                                   # 向左偏


def test_360_scan_rear_obstacle_blocks_reverse():
    """★ 倒车时正后方障碍必须拦住（本轮新增的后方保护）。

    车在往后走时车尾才是"去路"，所以前向数据不参与、后方窄锥参与。
    正后方 0.2 m（< stop 0.30）→ 直接刹停为 0。
    """
    av = ReactiveAvoider(stop_distance=0.30, slow_distance=0.55)
    full = _uniform360(361, 5.0)
    full[0] = 0.2                                  # 索引 0 = 角度 -180° = 正后方
    full[360] = 0.2                                # 索引 360 = 角度 +180° = 正后方（首尾重合）
    v, w, info = av.arbitrate(-0.10, 0.0, full, -math.pi, math.pi / 180.0)
    assert info['rear'] < 0.3, f"后方锥没读到近障（rear={info['rear']}）"
    assert v == 0.0, f'车尾 0.2 m 有障碍却仍在倒车（v={v}）'


def test_rear_obstacle_scales_reverse_speed():
    """车尾处于减速过渡段（stop~slow）时，倒车速度按比例降低但不为 0。"""
    av = ReactiveAvoider(stop_distance=0.30, slow_distance=0.55, slow_min_speed=0.10)
    full = _uniform360(361, 5.0)
    full[0] = 0.40                                 # 0.40 落在 0.30~0.55 之间
    v, _, info = av.arbitrate(-0.20, 0.0, full, -math.pi, math.pi / 180.0)
    assert info['rear'] < 0.55
    assert -0.20 < v < 0.0, f'过渡段倒车速度应被压低但不为 0，实际 {v}'


def test_front_obstacle_does_not_block_reverse():
    """★ 对照：倒车时前方障碍不得拦住（前向数据与去路无关，否则槽内退不出来）。"""
    av = ReactiveAvoider(stop_distance=0.30)
    full = _uniform360(361, 5.0)
    full[180] = 0.2                                # 正前方 0.2 m
    v, _, info = av.arbitrate(-0.10, 0.0, full, -math.pi, math.pi / 180.0)
    assert info['front'] < 0.3
    assert v == -0.10, f'倒车被前方障碍拦住了（v={v}）'


def test_rear_obstacle_does_not_block_forward():
    """★ 对照：前进时后方障碍不得拦住（否则车往前跑却被身后的墙拽住）。"""
    av = ReactiveAvoider(stop_distance=0.30)
    full = _uniform360(361, 5.0)
    full[0] = 0.2                                  # 正后方 0.2 m
    v, _, info = av.arbitrate(0.40, 0.0, full, -math.pi, math.pi / 180.0)
    assert info['rear'] < 0.3
    assert v == 0.40, f'前进被后方障碍拦住了（v={v}）'


def _wrap_local(a):
    """把角度归一化到 (-pi, pi]（测试内部用）。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _slot_side_wall_scan(lateral, length=3.0, step=0.02, fill=5.0):
    """造一帧"槽内两侧平行侧壁"的扫描（车体系）。

    侧壁与车身平行、位于车心横向 ±lateral 处，向车后延伸 length。
    墙上纵向偏移 L 处的一点，其极坐标是：
        角度 = 180° − atan(lateral / L)      （在车后，靠近正后方）
        距离 = hypot(lateral, L)
    所以侧壁会横跨很大的后方角度范围——这正是"后方锥开太宽就会读到侧壁"的原因。
    """
    full = _uniform360(361, fill)
    for i in range(361):
        ang = -math.pi + i * math.pi / 180.0
        # 取该角度对应的墙上点（仅后方 ±180° 附近）
        rel = abs(_wrap_local(ang - math.pi))        # 与正后方的夹角
        if rel < 1e-6:
            continue
        L = lateral / math.tan(rel)                  # 反解纵向偏移
        if L > 0.0 and L <= length:
            d = math.hypot(lateral, L)
            full[i] = min(full[i], d)
        # 另一侧同理（镜像）
        rel2 = abs(_wrap_local(ang + math.pi))
        if rel2 > 1e-6:
            L2 = lateral / math.tan(rel2)
            if 0.0 < L2 <= length:
                full[i] = min(full[i], math.hypot(lateral, L2))
    return full


def test_narrow_rear_cone_ignores_slot_side_walls():
    """★★ 最关键的一条：槽内倒车不得被两侧槽壁拦住（"能进不能出"不许复发）。

    实测地图上的真实几何：槽内净宽 0.722 m，车心离两侧槽壁仅 0.361 m，
    侧壁与车身平行、向车后延伸。若后方锥开得太宽，就会"看"到侧壁，
    倒车被自己的减速逻辑一直压着甚至刹停 → 车卡死在槽里。

    按长直平行壁的几何，锥角 h 在边界上读到的最小距离 = lateral / sin(h)。
    车横向偏到极限时 lateral = 0.361 − 0.10 = 0.261：
        h=20° → 0.261/sin20° = 0.763 m（> slow 0.55，干净）
        h=25° → 0.618 m（仍 > 0.55，但实测地图上已因槽口端壁降到 0.54）
        h=30° → 0.522 m（≤ 0.55，开始减速）
    这里的 lateral 取 0.261（最不利）。
    """
    av = ReactiveAvoider(stop_distance=0.30, slow_distance=0.55,
                         rear_half_angle=math.radians(20.0))
    full = _slot_side_wall_scan(lateral=0.261)
    v, _, info = av.arbitrate(-0.20, 0.0, full, -math.pi, math.pi / 180.0)
    assert info['rear'] >= 0.55, (
        f'后方锥读到了槽侧壁（rear={info["rear"]:.3f}）：锥角开太宽，'
        '槽内倒车会被自己拦住，表现为"能进不能出"')
    assert v == -0.20, f'槽内倒车被压住（v={v}）'


def test_wide_rear_cone_would_trip_on_side_walls():
    """反面证据：后方锥开到 45° 就会擦到槽侧壁 —— 证明"窄锥"不是拍脑袋。

    这条固定"宽锥有害"这一事实，避免日后有人为了"更安全"随手调大锥角。
    """
    av = ReactiveAvoider(stop_distance=0.30, slow_distance=0.55,
                         rear_half_angle=math.radians(45.0))
    full = _slot_side_wall_scan(lateral=0.261)
    v, _, info = av.arbitrate(-0.20, 0.0, full, -math.pi, math.pi / 180.0)
    assert info['rear'] < 0.55, (
        f'宽锥本应读到侧壁（rear={info["rear"]:.3f}）—— 若此断言失败，说明锥角语义变了')
    assert abs(v) < 0.20, f'宽锥下倒车本应被压低，实际 v={v}'


def _two_walls(left_d, right_d, front_d=None, side_deg=90):
    """造一帧"左右平行壁"扫描：左带=left_d、右带=right_d、可另加正前方障碍。

    ⚠ 只在 ±side_deg 处放点，模拟不了真实的平行长壁——真实侧壁横跨很宽的角度。
    需要"侧壁是否进入前锥"这类判断时，请用 `_parallel_walls_scan`。
    """
    full = _uniform360(361, 5.0)
    for i in range(361):
        ang = -math.pi + i * math.pi / 180.0
        if abs(_wrap_local(ang - math.radians(side_deg))) <= math.radians(4):
            full[i] = left_d
        if abs(_wrap_local(ang + math.radians(side_deg))) <= math.radians(4):
            full[i] = right_d
    if front_d is not None:
        full[180] = front_d
    return full


def _parallel_walls_scan(near, far, length=3.0, fill=5.0):
    """造一帧【与车身平行的长直侧壁】扫描（车体系，0°=车头，+y=左）。

    真实槽侧壁就是这样：与车身平行、向车头车尾两侧都延伸，因此横跨很宽的角度范围。
    左壁横向 y = +near、右壁 y = −far（near = 近侧壁，即车偏向的那一侧）。
    某角度的射线与 y = +l 的壁交点参数 t = l / sin(a)（须 sin(a) > 0），
    纵向落点 x = t·cos(a) 须在 [−length, +length] 内才算打中。

    ★ 这个建模很关键：把侧壁放在"±90°"是错的——±90° 不在任何前方锥里，
      于是会得出"宽锥也没事"的错误结论。投影到真实角度上才看得出宽锥会被骗。
    """
    full = _uniform360(361, fill)
    for i in range(361):
        a = -math.pi + i * math.pi / 180.0
        s, c = math.sin(a), math.cos(a)
        for l in (near, far):
            if abs(s) < 1e-9:
                continue
            for sign in (1.0, -1.0):               # 左壁(+l) / 右壁(−l)
                denom = s * sign
                if denom <= 1e-9:
                    continue
                t = l / denom
                if t <= 0.0:
                    continue
                x = t * c
                if -length <= x <= length:
                    full[i] = min(full[i], t)
    return full


def test_takeover_gate_ignores_parallel_slot_walls():
    """★★ 本轮修复：只有平行侧壁、正前方通畅时，不得夺向。

    根因：前扇区 ±45° 的锥边界会扫到平行于车身的槽侧壁。槽内净半宽 0.361、车半宽
    0.25 → 车心最多偏 0.111 m，近侧壁距车心 0.250 m；45° 射线命中 0.250/sin45°
    = 0.354 m ≤ take(0.45) → "前方很近"的假象骗开闸门；左右壁距一比又超过 0.10 的
    偏置阈值 → 接管夺向。可两侧壁平行，往哪偏都是蹭墙。
    实测进槽 36%~47% 采样被夺向；闸门收到 20° 后为 0。

    这里用 `_parallel_walls_scan`（真实的平行长壁），不是 ±90° 放两个点——
    后者落在任何前锥之外，测不出真问题。
    """
    av = ReactiveAvoider(stop_distance=0.30, slow_distance=0.55,
                         take_over_distance=0.45,
                         front_half_angle=math.radians(45.0),
                         takeover_half_angle=math.radians(20.0),
                         side_max_angle=math.radians(90.0))
    for near, far in ((0.26, 0.46), (0.46, 0.26), (0.25, 0.47)):
        full = _parallel_walls_scan(near, far)
        v, w, info = av.arbitrate(0.30, 0.0, full, -math.pi, math.pi / 180.0)
        assert info['front_gate'] > 0.45, (
            f'闸门读到了侧壁（gate={info["front_gate"]:.3f}）：锥角开太宽')
        assert info['engaged'] is False, (
            f'正前方通畅却被夺向（近壁={near} 远壁={far}）—— 槽内会反复蹭墙')
        assert w == 0.0, f'不该给出转向，实际 w={w}'


def test_wide_gate_would_be_fooled_by_parallel_walls():
    """反面证据：闸门 = 前锥(45°) 时，平行侧壁确实会骗开闸门并夺向。

    这条固定"窄闸门必要"这一事实。用同一批平行壁数据，只把闸门放宽到 45°。
    """
    full = _parallel_walls_scan(0.26, 0.46)
    wide = ReactiveAvoider(stop_distance=0.30, slow_distance=0.55,
                           take_over_distance=0.45,
                           front_half_angle=math.radians(45.0),
                           takeover_half_angle=math.radians(45.0),
                           side_max_angle=math.radians(90.0))
    _, _, info_w = wide.arbitrate(0.30, 0.0, full, -math.pi, math.pi / 180.0)
    assert info_w['front_gate'] <= 0.45, (
        f'45° 闸门本应被侧壁骗开（gate={info_w["front_gate"]:.3f}）')
    assert info_w['engaged'] is True, '45° 闸门本应夺向'

    narrow = ReactiveAvoider(stop_distance=0.30, slow_distance=0.55,
                             take_over_distance=0.45,
                             front_half_angle=math.radians(45.0),
                             takeover_half_angle=math.radians(20.0),
                             side_max_angle=math.radians(90.0))
    _, _, info_n = narrow.arbitrate(0.30, 0.0, full, -math.pi, math.pi / 180.0)
    assert info_n['front_gate'] > 0.45, '20° 闸门不该被侧壁骗开'
    assert info_n['engaged'] is False, '20° 闸门不该夺向'


def test_takeover_gate_still_catches_real_front_obstacle():
    """★ 对照：真·正前方障碍必须照旧触发接管 —— 窄闸门不能把功能改废。"""
    av = ReactiveAvoider(stop_distance=0.30, slow_distance=0.55,
                         take_over_distance=0.45,
                         front_half_angle=math.radians(45.0),
                         takeover_half_angle=math.radians(20.0),
                         side_max_angle=math.radians(90.0))
    full = _two_walls(5.0, 0.30, front_d=0.40)     # 正前 0.40，右侧 0.30（不对称）
    v, w, info = av.arbitrate(0.30, 0.0, full, -math.pi, math.pi / 180.0)
    assert info['front_gate'] <= 0.45, '窄闸门没看到正前方障碍'
    assert info['engaged'] is True, '正前方有障碍却不接管了 —— 功能被改废'
    assert w > 0.0, f'右侧更近，应向左偏（w>0），实际 w={w}'


def test_side_front_obstacle_slows_but_does_not_steal_steering():
    """★ 分层设计的关键证据：窄锥外、宽锥内的障碍 → 减速但不夺向。

    40° 方向 0.35 m 的障碍：在 45° 宽锥内（要减速），在 20° 窄锥外（不该夺向）。
    这样槽内仍会慢行，但转向归导航 —— 正是本次改动的意图。
    """
    av = ReactiveAvoider(stop_distance=0.30, slow_distance=0.55,
                         take_over_distance=0.45,
                         front_half_angle=math.radians(45.0),
                         takeover_half_angle=math.radians(20.0),
                         side_max_angle=math.radians(90.0))
    full = _uniform360(361, 5.0)
    full[220] = 0.35                               # 角度 +40°，在宽锥内
    full[270] = 0.25                               # 右带近（-90°）
    v, w, info = av.arbitrate(0.30, 0.0, full, -math.pi, math.pi / 180.0)
    assert info['front'] <= 0.55, '宽锥没读到 40° 障碍'
    assert v < 0.30, f'应当减速，实际 v={v}'
    assert info['engaged'] is False, '不该夺向（障碍不在窄闸门内）'
    assert w == 0.0

    # 对照：闸门放宽到 45° 就会夺向 —— 固定"窄闸门才是对的"这一事实
    av2 = ReactiveAvoider(stop_distance=0.30, slow_distance=0.55,
                          take_over_distance=0.45,
                          front_half_angle=math.radians(45.0),
                          takeover_half_angle=math.radians(45.0),
                          side_max_angle=math.radians(90.0))
    _, w2, info2 = av2.arbitrate(0.30, 0.0, full, -math.pi, math.pi / 180.0)
    assert info2['engaged'] is True, '宽闸门本应夺向'


def test_takeover_gate_is_clamped_to_front_cone():
    """防呆：闸门锥不得宽于前扇区（否则等于没改），应被自动夹到前锥以内。"""
    av = ReactiveAvoider(front_half_angle=math.radians(45.0),
                         takeover_half_angle=math.radians(60.0))
    assert abs(av.takeover_half_angle - math.radians(45.0)) < 1e-9, (
        f'闸门应被夹到 45°，实际 {math.degrees(av.takeover_half_angle):.1f}°')
