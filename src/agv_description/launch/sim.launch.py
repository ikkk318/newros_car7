#!/usr/bin/env python3
"""
sim.launch.py —— 起 Gazebo Classic 11 物理仿真（ROS 2 Humble）
==================================================================
启动链：
  1. robot_state_publisher  (发布 /robot_description + TF)
  2. gzserver + gzclient    (加载 world: 空房间 + 高度障碍物)
  3. gazebo_ros spawn_entity (把机器人从 /robot_description 生成到世界里)
  4. robot_localization EKF  (/odom + /imu → odom→base_footprint，可关)
  5. RViz2                  (可选，看 TF / 雷达点)

注意：Classic 版插件（libgazebo_ros_*）直接发布 ROS 话题，不需要话题桥。

odom TF 归属：
  默认 use_ekf:=true  → 由 EKF 发布 odom→base_footprint（差速插件让位）；
         use_ekf:=false → 由 Gazebo 差速插件发布（退回纯轮式里程计）。
  两者绝不会同时发布同一段 TF（否则 TF 抖动 → SLAM 地图跳变）。

运行：
    ros2 launch agv_description sim.launch.py
    ros2 launch agv_description sim.launch.py use_rviz:=false
    ros2 launch agv_description sim.launch.py gui:=false            # 无头（不起 gzclient）
    ros2 launch agv_description sim.launch.py use_ekf:=false        # 关 EKF，纯轮式里程计
    ros2 launch agv_description sim.launch.py world:=/abs/path/xxx.sdf

生成后手动发速度：
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \\
        "{linear: {x: 0.3}, angular: {z: 0.0}}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('agv_description')

    default_xacro = os.path.join(pkg_share, 'urdf', 'agv_500.urdf.xacro')
    default_world = os.path.join(pkg_share, 'worlds', 'world666.world')
    default_rviz = os.path.join(pkg_share, 'rviz', 'agv_500.rviz')
    default_ekf_params = os.path.join(pkg_share, 'config', 'ekf_sim.yaml')

    xacro_file = LaunchConfiguration('xacro_file')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_ekf = LaunchConfiguration('use_ekf')

    # 差速插件是否发布 odom→base_footprint：开了 EKF 就置 false（交给 EKF 发）；
    # 否则两个节点会抢发同一段 TF → TF 抖动 → slam_toolbox 地图跳变。
    publish_odom_tf = PythonExpression(
        ["'false' if '", use_ekf, "' == 'true' else 'true'"])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' publish_odom_tf:=', publish_odom_tf]),
        value_type=str)

    declare_args = [
        DeclareLaunchArgument('xacro_file', default_value=default_xacro,
                              description='机器人 xacro 绝对路径'),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Gazebo world (.sdf) 绝对路径'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否打开 Gazebo 客户端窗口（false = 无头）'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='是否启动 RViz2'),
        DeclareLaunchArgument('use_ekf', default_value='true',
                              description='是否用 robot_localization EKF 融合 /odom + /imu '
                                          '并接管 odom→base_footprint 的 TF'),
    ]

    # ---------- 1. 机器人状态发布（sim time 必须开） ----------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    # ---------- 2. Gazebo Classic（gzserver + 可选 gzclient） ----------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': gui,
        }.items(),
    )

    # ---------- 3. 把机器人生成进世界 ----------
    #   spawn_entity 自己会等 /robot_description 话题有数据、等 Gazebo 服务就绪
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_agv_500',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'agv_500',
            '-x', '0.0', '-y', '0.0', '-z', '0.05',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
            # ★ timeout 默认只有 30s：world 模型多 / 机器负载高时 gzserver 就绪慢，
            #   spawn_entity 会在 /spawn_entity 服务还没起来时就报错退出
            #   （实体其实会迟到生成，但节点已死、日志误导人）。放宽到 120s。
            '-timeout', '120',
        ],
    )

    # ---------- 4. EKF 里程计融合（robot_localization） ----------
    #   /odom(轮式) + /imu(角速率) → TF: odom→base_footprint（此时差速插件已让位）
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[default_ekf_params],
        condition=IfCondition(use_ekf),
    )

    # ---------- 5. RViz ----------
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(declare_args + [
        robot_state_publisher,
        gazebo,
        spawn_robot,
        ekf_node,
        rviz2,
    ])
