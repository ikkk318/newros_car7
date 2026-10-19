#!/usr/bin/env python3
"""
navigation.launch.py —— 导航总启动：仿真 + AMCL 定位 + Nav2 导航栈 + RViz
=========================================================================
在已建好的地图上跑自主导航。MAP → AMCL 定位 → Nav2 规划/控制 → /cmd_vel。

数据流：
  agv_description/sim.launch.py   起 Gazebo + spawn 机器人（发 /scan、/odom、TF）
  nav2_bringup(bringup)           map_server 加载地图 + AMCL 定位 + 全套 Nav2
  RViz                            Fixed Frame=map，可点工具发导航目标

★ 参数命名红线
  ROS 2 launch 的 IncludeLaunchDescription 把 SetLaunchConfiguration 写进**同一个全局
  context**（无作用域隔离），而 gazebo_ros 的 gzserver.launch.py 也在这个 context 里取
  参数。名字撞车就会互相污染：本文件的 Nav2 参数若叫 params_file，会被 gzserver 当成
  --ros-args --params-file（把 nav2 参数灌给 gazebo）。故改名 nav2_params_file，
  只在下方 nav2 include 里以官方名 params_file 传入。

运行（先 source install/setup.bash）：
    ros2 launch agv_navigation navigation.launch.py
    ros2 launch agv_navigation navigation.launch.py map:=/abs/other.yaml
    ros2 launch agv_navigation navigation.launch.py gui:=false rviz:=false   # 无头

发一个导航目标（RViz 里用「2D Goal Pose」点，或命令行）：
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
        "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}}"

注意：AMCL 标称初始位姿在 (0,0,0)（见 params/nav2_params.yaml 的 initial_pose），
      与仿真出生点一致；若改出生点，两处要同步。
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
    bringup_share = get_package_share_directory('nav2_bringup')    # use nav2 官方启动包路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup') 

    # 默认世界与地图（地图库：maps/<地图名>/<地图名>.yaml）
    # 默认世界：world666.world（★ 换世界时 mapping.launch.py 两处要一起改，否则地图与场景对不上；
    #   该 world 由 tools/make_world_map.py 生成，栅格地图由控制台建图模式实测得到）
    default_world = os.path.join(desc_share, 'worlds', 'world666.world')
    default_map = os.path.join(nav_share, 'maps', 'world666map', 'world666map.yaml')
    default_params = os.path.join(nav_share, 'params', 'nav2_params.yaml')
    rviz_config_dir = os.path.join(bringup_share, 'rviz', 'nav2_default_view.rviz')

    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    # ★ 不叫 params_file：会被 gzserver 当 --params-file 吃走（见文件头红线）
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')




    declare_args = [
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Gazebo world 文件绝对路径'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='地图 yaml 绝对路径（nav2 map_server 加载）'),
        DeclareLaunchArgument('nav2_params_file', default_value=default_params,
                              description='Nav2 参数文件绝对路径'
                                          '（★ 勿改名 params_file：会被 gzserver 当成 --params-file）'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否打开 Gazebo 客户端窗口'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='是否启动 RViz2'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='是否使用 /clock 仿真时钟'),
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

    # ---------- 2. Nav2 全套：map_server + AMCL 定位 + 规划/控制/行为树 ----------
    #   slam:=False → 走「已建好地图 + AMCL 定位」分支（建图请用 mapping.launch.py）
    #   注意顺序：sim 在前，gzserver 在 params_file 还是空的时候就已定好命令行；
    #   若把 nav2 挪到 sim 前面，gzserver 会白捡一份 --params-file（见文件头红线）。
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,   # nav2 官方名，只在本 include 起作用
            'slam': 'False',
            'use_composition': 'False',   # 用普通节点，便于单独看各节点日志
            'autostart': 'true',          # 自动完成生命周期激活
        }.items(),
    )

    # ---------- 3. RViz（Fixed Frame=map，可用 2D Goal Pose 发目标） ----------
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(declare_args + [sim, nav2, rviz2])
