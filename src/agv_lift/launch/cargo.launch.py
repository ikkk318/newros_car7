#!/usr/bin/env python3
"""
cargo.launch.py —— 单独启动搬货服务节点（整栈启动用 agv_stack.launch.py）。

    ros2 launch agv_lift cargo.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='agv_lift', executable='cargo_node', name='cargo_node',
             output='screen'),
    ])
