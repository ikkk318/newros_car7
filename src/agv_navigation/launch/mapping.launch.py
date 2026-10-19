#!/usr/bin/env python3
"""
mapping.launch.py —— 建图（SLAM）总启动：仿真 + robot_state_publisher + slam_toolbox + RViz
=========================================================================================
一条命令跑起「Gazebo 世界 + 本车 + 在线建图」，用于生成仓库栅格地图。

数据流：
  agv_description/sim.launch.py   起 Gazebo + spawn 机器人（发 /scan、/odom、TF）
  slam_toolbox(async)             /scan + TF → /map + map→odom
  RViz                            Fixed Frame=map，实时看建图过程

运行（先 source install/setup.bash）：
    ros2 launch agv_navigation mapping.launch.py
    ros2 launch agv_navigation mapping.launch.py gui:=false rviz:=false   # 无头建图
    ros2 launch agv_navigation mapping.launch.py world:=/abs/xxx.world    # 换世界

建图操作（另开一终端）：
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # 建图时雷达是全向 360°（未顶升货架，后方无遮挡），按正常路线走一圈即可

建图完成后保存地图：
    ros2 run nav2_map_server map_saver_cli -f src/agv_navigation/maps/world666map/world666map
    # 保存后把 world666map.pgm/.yaml 放进 maps/world666map/，再 colcon build 装进 share
    # （也可以直接用控制台建图模式的 [m] 命名保存，它会自动放好位置）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('agv_navigation')
    desc_share = get_package_share_directory('agv_description')
    slam_share = get_package_share_directory('slam_toolbox')

    # 默认世界：agv_description 内自带的厂房世界 world666.world
    # （★ 必须与 navigation.launch.py 的默认世界一致 —— 地图与场景要同源；
    #   该 world 由 tools/make_world_map.py 生成，换场景请重跑脚本而不是手改 world）
    default_world = os.path.join(desc_share, 'worlds', 'world666.world')
    default_slam_params = os.path.join(nav_share, 'params', 'slam_toolbox.yaml')
    default_rviz = os.path.join(nav_share, 'rviz', 'mapping.rviz')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    waypoint_file = LaunchConfiguration('waypoint_file')
    routes_file = LaunchConfiguration('routes_file')

    declare_args = [
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Gazebo world 文件绝对路径'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否打开 Gazebo 客户端窗口'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='是否启动 RViz2'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='是否使用 /clock 仿真时钟'),
        DeclareLaunchArgument('slam_params_file', default_value=default_slam_params,
                              description='slam_toolbox 参数文件绝对路径'),
        # 建图期间也会存点位/路线。控制台会传 maps/_mapping 草稿目录，
        # 保存地图时再并入正式地图（MapManager.promote_draft）。
        DeclareLaunchArgument('waypoint_file',
                              default_value='/tmp/agv_mapping_waypoints.yaml',
                              description='建图期间的点位文件路径'),
        DeclareLaunchArgument('routes_file',
                              default_value='/tmp/agv_mapping_routes.yaml',
                              description='建图期间的路线文件路径'),
    ]

    # ---------- 1. 仿真环境 + 机器人（复用 agv_description，禁用它自带的 RViz） ----------
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': world,
            'gui': gui,
            'use_rviz': 'false',          # RViz 由本文件统一管
        }.items(),
    )

    # ---------- 2. slam_toolbox 在线建图（异步） ----------
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
    )

    # ---------- 3. 点位 / 路线服务（建图时也要能标点） ----------
    # ★ 为什么建图栈也需要它：建图时最自然的动作就是"车开到货架前顺手存个点"。
    #   原来建图栈里根本没有 waypoint_node，/agv/store 不存在，网页存点直接失败，
    #   自检还硬查这个服务报红 —— 用户看到"服务未就绪"，以为节点没起来。
    #   点位文件走 maps/_mapping 草稿目录，保存地图时再并入正式地图。
    waypoint = Node(
        package='agv_waypoint',
        executable='waypoint_node',
        name='waypoint_node',
        output='screen',
        parameters=[{
            'waypoint_file': waypoint_file,
            'routes_file': routes_file,
            'frame_id': 'map',
            'use_sim_time': use_sim_time,
        }],
    )

    # ---------- 4. RViz（Fixed Frame=map，实时看建图） ----------
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(declare_args + [sim, slam, waypoint, rviz2])
