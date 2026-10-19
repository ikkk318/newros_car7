"""
AGV 网页建图启动文件。

**用途**
用浏览器完成建图：遥控走一圈 → 网页上实时看到地图在长 → 命名保存。
不需要终端控制台，也不需要 RViz。

    ros2 launch agv_web web_mapping.launch.py token:=agv2024
    # 然后浏览器打开 http://<车机IP>:8080

**与导航模式的关键区别（这是能不能动起来的关键）**
建图栈 `mapping.launch.py` 只起「仿真 + slam_toolbox + RViz」，里面
**没有 velocity_arbiter、也没有 task_manager**。而底盘（URDF 里
`cmd_vel:=cmd_vel_raw` 是写死的）只听 `/cmd_vel_raw`。
所以本文件把网关的 `mode` 设成 `mapping`，遥控直接发 `/cmd_vel_raw`。
如果照导航模式发 `/agv/web_teleop`，车会一动不动（没人转发）。

**参数**
    token      访问口令
    port       端口，默认 8080
    map_name   可选：进去就先建个默认名（不填则在网页上起名）
    gui        是否开 Gazebo 窗口（跑在车机上看不到，建议 false）
    rviz       是否开 RViz（网页已能看建图，默认 false）
    world      Gazebo 世界文件路径
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('agv_navigation')

    default_world = os.path.join(
        get_package_share_directory('agv_description'), 'worlds', 'world666.world')

    token = LaunchConfiguration('token')
    port = LaunchConfiguration('port')
    allow_ips = LaunchConfiguration('allow_ips')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_args = [
        DeclareLaunchArgument('token', default_value='',
                              description='访问口令（留空=不校验，不建议）'),
        DeclareLaunchArgument('port', default_value='8080', description='网页端口'),
        DeclareLaunchArgument('allow_ips', default_value='',
                              description='允许的 IP 前缀，逗号分隔；留空=不限制'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='是否打开 Gazebo 客户端窗口'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='是否启动 RViz（网页已能看建图，默认关）'),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Gazebo world 文件绝对路径'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='是否使用仿真时间'),
    ]

    # ── 建图栈（仿真 + slam_toolbox），复用现有 launch ──
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'world': world,
            'gui': gui,
            'rviz': rviz,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ── 网页网关：mode=mapping 让遥控改发 /cmd_vel_raw ──
    gateway = Node(
        package='agv_web',
        executable='web_gateway',
        name='agv_web_gateway',
        output='screen',
        # ★ 只用 UDP，关掉 FastDDS 共享内存（残留段会让节点互相收不到）
        additional_env={'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4'},
        parameters=[{
            'host': '0.0.0.0',
            'port': port,
            'token': token,
            'allow_ips': allow_ips,
            'use_sim_time': use_sim_time,
            'mode': 'mapping',              # ★ 关键：走建图的遥控话题
            'map_topic': '/map',            # slam_toolbox 的实时地图
            'plan_topic': '/plan',          # 建图时没有路径，订阅着不影响
        }],
    )

    return LaunchDescription(declare_args + [mapping, gateway])
