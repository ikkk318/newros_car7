#!/usr/bin/env python3
"""
waypoint.launch.py —— 单独起「点位 / 路线管理服务节点」，便于脱离仿真调试点位增删查。

运行（先 source install/setup.bash）：
    ros2 launch agv_waypoint waypoint.launch.py
    ros2 launch agv_waypoint waypoint.launch.py waypoint_file:=/home/yuki/agv_points.yaml

测试服务（另开终端）—— 点位与路线共用一个 /agv/store，用 resource 区分：
    ros2 service call /agv/store agv_interfaces/srv/AgvStore \
        "{resource: 'waypoint', action: 'save', name: '货架A', x: 3.0, y: 1.2, yaw: 0.0}"
    ros2 service call /agv/store agv_interfaces/srv/AgvStore \
        "{resource: 'waypoint', action: 'get', name: '货架A'}"
    ros2 service call /agv/store agv_interfaces/srv/AgvStore \
        "{resource: 'waypoint', action: 'list'}"
    ros2 service call /agv/store agv_interfaces/srv/AgvStore \
        "{resource: 'waypoint', action: 'delete', name: '货架A'}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waypoint_file = LaunchConfiguration('waypoint_file')       # use 点位文件路径参数
    routes_file = LaunchConfiguration('routes_file')           # use 路线文件路径参数
    frame_id = LaunchConfiguration('frame_id')                 # use 坐标系参数
    use_sim_time = LaunchConfiguration('use_sim_time')         # use 仿真时钟开关

    declare_args = [
        DeclareLaunchArgument('waypoint_file', default_value='/tmp/agv_waypoints.yaml',
                              description='点位 YAML 持久化文件路径'),
        DeclareLaunchArgument('routes_file', default_value='/tmp/agv_routes.yaml',
                              description='路线 YAML 持久化文件路径'),
        DeclareLaunchArgument('frame_id', default_value='map',
                              description='点位/路线坐标系'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='是否使用 /clock 仿真时钟（纯服务节点可关）'),
    ]

    node = Node(                                               # io 启动点位/路线服务节点
        package='agv_waypoint',
        executable='waypoint_node',
        name='waypoint_node',
        output='screen',
        parameters=[{
            'waypoint_file': waypoint_file,
            'routes_file': routes_file,
            'frame_id': frame_id,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription(declare_args + [node])
