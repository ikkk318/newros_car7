"""
agv_common —— AGV 跨模块公共工具库（纯 Python，无 ROS 节点、无状态）。

设计原则：只放"多个包都要用"的纯逻辑，不放任何 ROS 节点或硬件相关代码。
各功能包（waypoint / lift / task / web / voice）都来 import 这里的函数，
避免同一段坐标换算、限幅、配置合并逻辑到处复制。

子模块：
    geometry.py        角度/距离换算（限幅、归一化到 ±π、距离、四元数↔yaw）
    limits.py          速度限幅（线速度/角速度统一收口，安全约束的唯一入口）
    reactive_avoid.py  反应式避障仲裁（近障减速 + 左右择空 + 缓回导航，确定性）
    map_check.py       点位合法性预检（图内/非障碍/净空够；最宽通道瓶颈）
    config_loader.py   YAML 参数加载 + default→sim/real 覆盖合并

坐标系约定（REP-103）：x 向前、y 向左、z 向上；yaw 逆时针为正，单位弧度。
"""

from agv_common.geometry import (      # use 重导出几何工具，便于 from agv_common import clamp
    angle_diff,
    clamp,
    distance,
    quaternion_to_yaw,
    wrap_to_pi,
    yaw_to_quaternion,
)
from agv_common.limits import VelocityLimits      # use 速度限幅
from agv_common.reactive_avoid import ReactiveAvoider   # use 反应式避障仲裁
from agv_common.map_check import MapChecker       # use 点位合法性预检
from agv_common.config_loader import (            # use 配置加载与合并
    deep_merge,
    load_config,
    load_yaml,
)

__all__ = [                            # def 对外导出清单
    'clamp',
    'wrap_to_pi',
    'angle_diff',
    'distance',
    'yaw_to_quaternion',
    'quaternion_to_yaw',
    'VelocityLimits',
    'ReactiveAvoider',
    'MapChecker',
    'deep_merge',
    'load_config',
    'load_yaml',
]
