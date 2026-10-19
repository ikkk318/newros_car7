#!/usr/bin/env python3
"""
agv_stack.launch.py —— 任务栈总启动：点位/路线服务 + 搬货服务 + 任务层
=====================================================================
把业务节点一次拉起，让「指令 → 路由 → 导航 / 搬货 → 状态上报」整条链可跑：

    waypoint_node       /agv/store      点位与路线的统一增删查列
    cargo_node          /agv/cargo      搬货服务（顶升/下降）+ /agv/cargo/busy
    task_manager_node   收 /agv/command，发 /agv/task_state，
                        提供 /agv/query                   任务层唯一执行者

参数来自参数中心：config/default 打底，config/<档位> 覆盖。
档位参数名为 param_profile，默认空（只用 default）；切实车时 param_profile:=real。

★ 参数命名红线（本文件曾因此踩坑）
  ROS 2 launch 的 IncludeLaunchDescription 会把 SetLaunchConfiguration 写进**同一个
  全局 context**（见 launch/actions/include_launch_description.py 的 execute()），没有
  作用域隔离。而 gazebo_ros 的 gzserver.launch.py 也在这个 context 里取参数，且它的
  DeclareLaunchArgument 遇到「名字已被设置」就不再生效。所以名字一旦撞车就会互相污染：
    - 叫 profile   → 被 gzserver 当成 --profile=<档位>（物理档位）
    - 叫 world     → 本文件的空默认值会顶掉 navigation.launch.py 的默认世界 → gzserver
                     不带 world 启动 → 空世界 → /scan 全 inf
    - 叫 params_file → 被 gzserver 当成 --ros-args --params-file（把 nav2 参数灌给 gazebo）
  故本文件对外参数统一带后缀：param_profile / world_file，避免与 gazebo 参数重名。

运行（先 source install/setup.bash）：
    ros2 launch agv_bringup agv_stack.launch.py                       # 只起业务节点
    ros2 launch agv_bringup agv_stack.launch.py with_sim:=true        # 再带上导航栈
    ros2 launch agv_bringup agv_stack.launch.py with_sim:=true gui:=false rviz:=false
    ros2 launch agv_bringup agv_stack.launch.py with_sim:=true param_profile:=real

一条指令走完全程（另开终端）：
    # 存两个搬货点（kind: cargo）与一个导航点（kind: nav）
    ros2 service call /agv/store agv_interfaces/srv/AgvStore \
        "{resource: 'waypoint', action: 'save', name: '取货点', x: 1.0, y: 0.0, yaw: 0.0, kind: 'cargo'}"
    ros2 service call /agv/store agv_interfaces/srv/AgvStore \
        "{resource: 'waypoint', action: 'save', name: '放货点', x: 3.0, y: 1.0, yaw: 0.0, kind: 'cargo'}"
    # 存一条路线（按点名复制位姿）
    ros2 service call /agv/store agv_interfaces/srv/AgvStore \
        "{resource: 'route', action: 'save', name: '巡检A', point_names: ['取货点', '放货点']}"
    # 点名导航（可选路线：route 字段）
    ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
        "{source: 'web', command: 'goto', args: ['取货点'], route: '', request_id: 'r1'}"
    # 看状态 / 问状态
    ros2 topic echo /agv/task_state
    ros2 service call /agv/query agv_interfaces/srv/AgvQuery "{what: 'state'}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 读取参数中心：default 打底 + profile 覆盖（复用 agv_common 的合并逻辑）
# 参数：config_root - 配置根；profile - 档位目录名（空=只用 default）；name - 文件名
# 返回：该文件的参数字典
def _load_params(config_root, profile, name):     # in 内部辅助
    from agv_common.config_loader import load_config      # use 公共配置加载器
    overrides = [profile] if profile else []      # use 无档位则只读 default
    merged = load_config(config_root, overrides=overrides, names=[name])
    return merged.get(name, {})                   # use 返回该文件的参数字典


# 在启动时（LaunchContext 已就绪）组装实际动作，这样 profile/world 才能真正生效
def _launch_setup(context, *args, **kwargs):      # in 启动期回调
    bringup_share = get_package_share_directory('agv_bringup')
    nav_share = get_package_share_directory('agv_navigation')
    config_root = os.path.join(bringup_share, 'config')   # use 参数中心根目录

    # ★ 一律用带后缀的名字读，避免与 gzserver 的 profile/params_file 撞车（见文件头红线）
    profile = context.launch_configurations.get('param_profile', '')   # use 参数档（空=只用 default）
    with_sim = context.launch_configurations.get('with_sim', 'false')    # use 是否带导航栈
    gui = LaunchConfiguration('gui')              # use Gazebo 窗口
    rviz = LaunchConfiguration('rviz')            # use RViz
    # 注意：这里**不能**用 'world' 这个名字（本文件不声明它，也就不会覆盖导航栈的默认世界）
    world = context.launch_configurations.get('world_file', '')     # use 可选世界路径
    map_yaml_arg = context.launch_configurations.get('map_yaml', '')      # use 可选地图 yaml
    waypoint_file_arg = context.launch_configurations.get('waypoint_file', '')   # use 可选点位文件
    routes_file_arg = context.launch_configurations.get('routes_file', '')       # use 可选路线文件

    # 按 profile 合并出各节点参数（LaunchContext 已就绪，profile 真实生效）
    task_params = _load_params(config_root, profile, 'task')         # def 任务参数
    safety_params = _load_params(config_root, profile, 'safety')     # def 安全参数
    cargo_params = _load_params(config_root, profile, 'cargo')       # def 搬货参数
    waypoint_params = _load_params(config_root, profile, 'waypoint') # def 点位/路线参数
    hw_params = _load_params(config_root, profile, 'hardware')       # def 硬件层参数
    task_params = {**task_params, **safety_params}                   # def 合并安全参数
    # 目标点判据用的地图：优先用显式传入的 map_yaml（控制台按所选地图注入），否则用默认。
    # ★ 这张图就是"目标点能不能去"的唯一判据依据；图读不到时节点会退化为矩形范围并告警。
    map_yaml = map_yaml_arg or os.path.join(nav_share, 'maps', 'world666map', 'world666map.yaml')
    task_params = {**task_params, 'map_yaml': map_yaml}              # def 注入地图路径
    # 点位 / 路线文件：优先按所选地图绑定（每份地图一份），否则用配置默认。
    # 路线文件没显式给时，跟着点位文件走（同一目录下的 routes.yaml）。
    if waypoint_file_arg:
        waypoint_params = {**waypoint_params, 'waypoint_file': waypoint_file_arg}
    routes_file = routes_file_arg or (
        os.path.join(os.path.dirname(waypoint_file_arg), 'routes.yaml')
        if waypoint_file_arg else '')
    if routes_file:
        waypoint_params = {**waypoint_params, 'routes_file': routes_file}

    # 业务节点时钟：带 Gazebo 时用仿真时钟（防 TF 跳变）；否则用系统时钟，
    # 不然没有 /clock，节点定时器（安全推进/周期上报）不会触发。
    use_sim_time = {'use_sim_time': with_sim == 'true'}   # def 时钟开关

    waypoint_node = Node(                        # io 点位 / 路线服务节点
        package='agv_waypoint', executable='waypoint_node', name='waypoint_node',
        output='screen', parameters=[waypoint_params, use_sim_time])

    cargo_node = Node(                           # io 搬货服务节点（顶升/下降）
        package='agv_lift', executable='cargo_node', name='cargo_node',
        output='screen', parameters=[cargo_params, use_sim_time])

    task_node = Node(                            # io 任务层节点
        package='agv_task', executable='task_manager_node', name='task_manager_node',
        output='screen', parameters=[task_params, use_sim_time])

    actions = [waypoint_node, cargo_node, task_node]     # def 动作清单

    # 硬件层（★ 硬件无关边界）：速度仲裁（近障减速/择空）。
    # 仲裁节点把 /cmd_vel 加工后发到 /cmd_vel_raw（Gazebo 差速插件听这个），
    # 是"防撞"的最后一层；只有带仿真时才需要它驱动 Gazebo，但常驻无害。
    arb_params = hw_params.get('velocity_arbiter', {}) if hw_params else {}
    arb_params = arb_params.get('ros__parameters', arb_params)   # def 摊平到节点参数
    arb_node = Node(                             # io 速度仲裁（硬件边界）
        package='agv_hardware', executable='velocity_arbiter', name='velocity_arbiter',
        output='screen', parameters=[arb_params, use_sim_time])
    actions.append(arb_node)

    # 可选：带上 agv_navigation 导航栈（仿真 + AMCL + Nav2 + RViz）
    if with_sim == 'true':
        nav_args = {'gui': gui, 'rviz': rviz}    # def 转发给导航栈的参数
        if world:                                # # 只在显式给了 world_file 时才转发，
            nav_args['world'] = world            # # 否则导航栈用它自己的默认世界（不会被空串顶掉）
        if map_yaml_arg:                         # # 控制台选了地图：把它的 yaml 传给导航栈
            nav_args['map'] = map_yaml_arg
        actions.append(IncludeLaunchDescription(  # io 导航栈
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'navigation.launch.py')),
            launch_arguments=nav_args.items(),
        ))

    return actions                               # use 交回给 launch 系统


def generate_launch_description():
    declare_args = [
        DeclareLaunchArgument('param_profile', default_value='',
                              description='参数中心档位目录名：留空=只用 config/default；'
                                          '实车差异写好 config/real/*.yaml 后传 real'
                                          '（★ 勿改名 profile：会被 gzserver 当成 --profile 吃走）'),
        DeclareLaunchArgument('with_sim', default_value='false',
                              description='是否同时拉起 agv_navigation 导航栈（仿真+Nav2）'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo 窗口'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz 窗口'),
        DeclareLaunchArgument('world_file', default_value='',
                              description='可选：Gazebo world 绝对路径（留空用导航栈默认世界）'
                                          '（★ 勿改名 world：本文件的空默认值会顶掉导航栈的默认世界）'),
        DeclareLaunchArgument('map_yaml', default_value='',
                              description='可选：所选地图的 yaml 路径（控制台按地图库文件注入）'),
        DeclareLaunchArgument('waypoint_file', default_value='',
                              description='可选：与所选地图绑定的点位文件路径'),
        DeclareLaunchArgument('routes_file', default_value='',
                              description='可选：与所选地图绑定的路线文件路径'
                                          '（留空则用点位文件同目录的 routes.yaml）'),
    ]
    return LaunchDescription(declare_args + [OpaqueFunction(function=_launch_setup)])
