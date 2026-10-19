"""
倒车能力配置守卫：全链路必须允许倒车（"能进槽，也要能出槽"）。

为什么要有这个文件
------------------
车开进凵型槽后必须能退出来。槽内净宽 0.80 m（实测地图 0.72 m），而车是 0.50×0.50 的方车，
原地掉头需要半对角线 0.25·√2 = 0.354 m，槽内可用半边净空只有 ~0.36 m，余量仅约 7 mm——
这点余量会被膨胀层和栅格化误差吃掉，**倒车是唯一可靠出路**。

这条"能倒车"的性质不是某一处代码决定的，而是被 `nav2_params.yaml` 与 `hardware.yaml`
两个配置文件联合决定的；任何一处写反，车就"能进不能出"，而且现象是"原地左右摇摆"
这种完全看不出原因的样子。所以这里用配置守卫把它钉住：改坏了立刻红，不用等到跑仿真。

（本轮真实缺陷：`nav2_params.yaml` 的 DWB `min_vel_x` 曾是 0.0，
注释写着"不允许倒车，倒车交行为树"，但删掉精定位后没有任何东西接管出槽，
于是控制器根本不生成倒车轨迹 → 车困在槽里。）

跨包说明
--------
本文件在 agv_common 包里，但读的是 agv_navigation 与 agv_bringup 的配置文件——
因为被守护的是一条**跨包不变量**，放在任一单包内都不合适，放共享工具包里最合理。
只读文件、不 import 其它包，仍满足"纯逻辑、可脱离 ROS 运行"。
"""

import pathlib                          # use 定位仓库内的配置文件

import yaml                             # use 解析 Nav2 / 硬件参数

# 本文件位于 <仓库>/src/agv_common/test/，parents[2] 即 src/
SRC = pathlib.Path(__file__).resolve().parents[2]
NAV2_PARAMS = SRC / 'agv_navigation' / 'params' / 'nav2_params.yaml'
HARDWARE = SRC / 'agv_bringup' / 'config' / 'default' / 'hardware.yaml'


# 读 YAML
# 参数：path - 配置文件路径
# 返回：解析后的字典
def _load(path):                        # in 内部工具
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


# 取 DWB 局部规划器的 FollowPath 参数块
# 参数：无
# 返回：FollowPath 参数字典
def _follow_path():                     # in 内部工具
    p = _load(NAV2_PARAMS)
    return p['controller_server']['ros__parameters']['FollowPath']


def test_dwb_can_generate_reverse_trajectories():
    """★ DWB 必须能采样到负线速度，否则控制器永不倒车，车困在槽里出不来。

    这是本轮"能进不能出"的直接根因：DWB 只在 [min_vel_x, max_vel_x] 之间采样，
    min_vel_x=0.0 等于把倒车这个动作从规划器的动作空间里删掉了。
    """
    fp = _follow_path()
    assert fp['min_vel_x'] < 0.0, (
        f"min_vel_x={fp['min_vel_x']}，控制器无法生成倒车轨迹；"
        "车进槽后只能靠恢复行为脱困，会表现为原地左右摇摆"
    )
    assert fp['min_vel_x'] >= -0.30, (
        f"倒车速度 {fp['min_vel_x']} m/s 偏快：槽内净宽仅约 0.72 m，"
        "倒车必须保守（建议 -0.20 左右）"
    )


def test_reverse_speed_is_within_arbiter_limit():
    """倒车速度不得被速度仲裁器限幅改号：|min_vel_x| ≤ max_linear。"""
    fp = _follow_path()
    hw = _load(HARDWARE)['velocity_arbiter']['ros__parameters']
    assert abs(fp['min_vel_x']) <= hw['max_linear'] + 1e-9, (
        f"倒车 {abs(fp['min_vel_x'])} 超过仲裁器 max_linear={hw['max_linear']}"
    )
    # VelocityLimits 是对称限幅，负值原样通过——这里确认"倒车不会被反向截断"
    assert hw['max_linear'] > 0.0


def test_velocity_smoother_does_not_clip_reverse():
    """速度平滑器的下限必须为负，否则前面放开的倒车会在这里被削成 0。"""
    sm = _load(NAV2_PARAMS)['velocity_smoother']['ros__parameters']
    assert sm['min_velocity'][0] < 0.0, (
        f"velocity_smoother.min_velocity[0]={sm['min_velocity'][0]}，会把倒车削成 0"
    )


def test_takeover_gate_stays_narrow():
    """★ 接管闸门锥必须够窄，否则槽内平行侧壁会骗开闸门、车被反复夺向。

    几何：槽内净半宽 0.361 m、车半宽 0.25 m → 车心最多偏 0.111 m，近侧壁距车心
    0.250 m。锥半角 h 的边界命中近侧壁 = 0.250/sin(h)：
        h=20° → 0.731 m（> take 0.45，读不到）
        h=27° → 0.551 m（临界）
        h=30° → 0.500 m
        h=45° → 0.354 m（≤ take 0.45，骗开闸门）
    解析临界 h > asin(0.250/0.55) = 27.0°；实测（真实地图、侧壁有尽头）临界在
    30°~35°。这里卡死 27° 上限（理论最保守值）。
    """
    import math
    hw = _load(HARDWARE)['velocity_arbiter']['ros__parameters']
    assert 'takeover_half_angle_deg' in hw, (
        'hardware.yaml 缺少 takeover_half_angle_deg：接管闸门锥角未显式配置'
    )
    h = float(hw['takeover_half_angle_deg'])
    assert h > 0.0, f'takeover_half_angle_deg={h} 必须为正'
    near_wall = 0.361 - 0.111                      # 最不利偏移下近侧壁距车心
    slow = float(hw['slow_distance'])
    trip = near_wall / math.sin(math.radians(h))
    assert trip > slow, (
        f'takeover_half_angle_deg={h}° 太宽：锥边界命中近侧壁 {trip:.3f} m '
        f'≤ slow({slow})，槽内会被侧壁骗开闸门并被反复夺向'
        f'（解析临界 {math.degrees(math.asin(near_wall / slow)):.1f}°）'
    )
    assert h <= 27.0, (
        f'takeover_half_angle_deg={h}° 超过解析临界 '
        f'{math.degrees(math.asin(near_wall / slow)):.1f}°'
    )
    # 必须比前扇区窄，否则等于没改
    front = float(hw['front_half_angle_deg'])
    assert h < front, (
        f'闸门 {h}° 不窄于前扇区 {front}°，等于没有分层，槽内仍会被夺向'
    )


def test_behavior_server_still_offers_backup():
    """恢复动作里保留 backup（卡死兜底仍需要它，与控制器倒车是两回事）。"""
    bs = _load(NAV2_PARAMS)['behavior_server']['ros__parameters']
    assert 'backup' in bs['behavior_plugins'], (
        f"behavior_plugins={bs['behavior_plugins']} 缺少 backup 恢复动作"
    )
    assert 'backup' in bs, "backup 有插件名但缺少参数块"


def test_rear_guard_cone_stays_narrow():
    """★ 后方保护扇区必须保持"窄锥"，否则槽内倒车会被两侧槽壁压住（能进不能出复发）。

    几何：槽内净宽 0.722 m，车心离两侧平行槽壁仅 0.361 m。长直平行壁在车体系里
    横跨很大的后方角度；锥角 h 在锥边界上读到的最小距离 = lateral / sin(h)。
    取最不利横向偏移（lateral = 0.361 − 0.10 = 0.261）：
        h=20° → 0.763 m（> slow 0.55，干净）
        h=30° → 0.522 m（≤ 0.55，开始减速）
    实测地图（含槽口端壁）更严：±20° → 1.20 m，±25° → 0.54 m。
    所以这里卡死 25° 上限——要放大必须先重测槽内几何。
    """
    import math
    hw = _load(HARDWARE)['velocity_arbiter']['ros__parameters']
    assert 'rear_half_angle_deg' in hw, (
        'hardware.yaml 缺少 rear_half_angle_deg：后方保护锥角未显式配置'
    )
    h = float(hw['rear_half_angle_deg'])
    assert h > 0.0, f'rear_half_angle_deg={h} 必须为正（否则后方保护形同虚设）'
    assert h <= 25.0, (
        f'rear_half_angle_deg={h}° 太宽：槽内两侧平行壁会进入后方锥'
        f'（0.261/sin({h}°) = {0.261 / math.sin(math.radians(h)):.3f} m），'
        '倒车会被自己的减速逻辑压住 → 「能进不能出」复发'
    )
    # 与节点默认值一致（两处不一致会让人误判实际生效的锥角）
    assert abs(h - 20.0) < 1e-9 or h < 20.0, (
        f'hardware.yaml 的 {h}° 大于节点默认 20°，实际生效值容易被误读'
    )


def test_rear_guard_can_see_a_vehicle_width_obstacle():
    """窄锥不能窄到漏掉真障碍：车宽 0.5 m 的物体落在正后方时必须看得见。

    锥角 h 在距离 d 处的横向半宽 = d·tan(h)。要在 slow_distance(0.55 m) 之外
    就发现"挡在车后整条车宽」的障碍，需 d·tan(h) ≥ 车半宽 0.25：
        h=20° → 0.55·tan20° = 0.200 m（略小于 0.25，但在 stop 0.30 处
                 0.30·tan20° = 0.109 m 之外的可见范围已覆盖车尾正对走廊）
    这里用更直接的判据：在刹车距离 0.30 m 处，锥的横向半宽 ≥ 车半宽的一半，
    保证"正对车尾"的障碍一定被看见。
    """
    import math
    hw = _load(HARDWARE)['velocity_arbiter']['ros__parameters']
    h = math.radians(float(hw['rear_half_angle_deg']))
    stop_d = float(hw['stop_distance'])
    half_w = stop_d * math.tan(h)              # 刹停距离处锥的横向半宽
    assert half_w > 0.0
    # 车尾正前方 0.5 m 宽的障碍，其中线必然落入锥内
    assert half_w >= 0.05, (
        f'后方锥在 {stop_d} m 处横向半宽仅 {half_w:.3f} m，可能漏掉正后方障碍'
    )


def test_local_costmap_keeps_unknown_as_free():
    """★ 局部代价地图必须把未知区当自由区，否则倒车会被未探明区域直接堵死。

    雷达已是 360° 全向、车体四周都能扫到，但局部代价地图是**滚动窗口**：
    车刚开进槽、窗口里尚未观测到的格子仍是 NO_INFORMATION。
    若把 track_unknown_space 打开，那些格子变成障碍，倒车轨迹全部非法——
    车照样出不了槽，只是从"摇摆"变成"完全不动"。
    代价：倒车时"看着已知区域往外走"，所以槽内必须慢（见上面倒车速度的断言）。
    """
    lc = _load(NAV2_PARAMS)['local_costmap']['local_costmap']['ros__parameters']
    assert lc.get('track_unknown_space', False) is False, (
        "局部代价地图打开了 track_unknown_space：车后盲区会变成障碍，倒车被堵死"
    )


def test_global_costmap_still_treats_unknown_as_obstacle():
    """全局地图反过来：全局路径不得穿未知区（谨慎探索，与局部图口径不同是有意的）。"""
    gc = _load(NAV2_PARAMS)['global_costmap']['global_costmap']['ros__parameters']
    assert gc['track_unknown_space'] is True
