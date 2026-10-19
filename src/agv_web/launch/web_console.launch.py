"""
AGV 网页控制台 —— 单命令启动（推荐用法）。只起网关，进度在网页里选。

**一条命令**
    ros2 launch agv_web web_console.launch.py token:=agv2024

**然后**
浏览器打开 `http://<车机IP>:8080`（手机同局域网也能开），
在页面上点「启动导航」或「开始建图」，网关会自己把对应的 ROS 栈拉起来。
停止、切换模式同样在网页上点——不用回头敲命令，跟终端控制台的体验一致。

**为什么不在这里直接 include 整套栈**
本文件要做的只是"起一个网关"，剩下的交给网页控制。
如果这里就把 `agv_stack` include 进来，那建图/导航又得改命令重启，
就退回到"两条命令"的老问题了。

**参数**
    token        访问口令（建议设）
    port         端口，默认 8080
    allow_ips    允许的 IP 前缀，逗号分隔（可选）
    manage_stack true（默认）= 允许网页起停 ROS 栈；false = 只做网关代理
    ws_root      工作区根目录（一般不用填，自动探测）
    use_sim_time 网关自己用哪种时钟（★ 必须和它将管理的栈一致）

**★ use_sim_time 为什么重要（踩过的坑）**
网页上的小车位置来自 `map→base` 的 TF。网关判断"这个位姿是不是太旧了"
要拿自己的 clock 去和 TF 的 header.stamp 相减 —— **两者必须同一个时基**。
但 Gazebo 栈（建图 / 带仿真的导航）发布 TF 用的是 `/clock` 的**仿真时间**
（从 0 开始，跑十分钟也才 600 秒），而网关默认走系统时间（约 1.79e9 秒），
相减约 57 年 → 每一帧都被判过期 → **位姿永远是 None，
地图和路径都正常，唯独看不到小车**，两个模式全中。

网关侧已经做了"时基不可比时降级判断"的兜底，所以即便这里漏配也不会瞎，
但配上仍然更准（能真正按秒判新鲜度）。默认 true 与 `agv_stack.launch.py`
的 `with_sim` 默认一致。

**依赖**（只差一个）
    sudo apt install python3-aiohttp
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    token = LaunchConfiguration('token')
    port = LaunchConfiguration('port')
    allow_ips = LaunchConfiguration('allow_ips')
    manage_stack = LaunchConfiguration('manage_stack')
    ws_root = LaunchConfiguration('ws_root')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_args = [
        DeclareLaunchArgument('token', default_value='',
                              description='访问口令（留空=不校验，仅限内网调试）'),
        DeclareLaunchArgument('port', default_value='8080',
                              description='网页端口'),
        DeclareLaunchArgument('allow_ips', default_value='',
                              description='允许的 IP 前缀，逗号分隔；留空=不限'),
        DeclareLaunchArgument('manage_stack', default_value='true',
                              description='是否允许网页起停 ROS 栈（建图/导航切换）'),
        DeclareLaunchArgument('ws_root', default_value='',
                              description='工作区根目录（空=自动探测）'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='★ 网关的时钟基准，必须和它管理的栈一致：'
                                          '带 Gazebo 用 true，纯真车用 false。'
                                          '不一致会导致位姿被判过期、小车不显示'),
    ]

    gateway = Node(
        package='agv_web',
        executable='web_gateway',
        name='agv_web_gateway',
        output='screen',
        # ★ 只用 UDP，关掉 FastDDS 共享内存：/dev/shm 残留段会让"节点都在、
        #   话题都在，就是互相收不到"，表现为自检全红 / 地图黑屏 / 任务下发
        #   没反应。网关自己在 main() 里也会设一遍，这里写一份防止有人
        #   绕过 launch 直接 ros2 run。
        additional_env={'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4'},
        parameters=[{
            'host': '0.0.0.0',          # ★ 必须 0.0.0.0，否则手机访问不到
            'port': port,
            'token': token,
            'allow_ips': allow_ips,
            'manage_stack': manage_stack,
            'ws_root': ws_root,
            # ★ 和上游 TF 用同一个时基，否则位姿年龄算成"57 年"→ 小车不显示
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription(declare_args + [gateway])
