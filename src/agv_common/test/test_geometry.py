"""
geometry 模块单元测试：角度归一化、限幅、距离、四元数互转。
"""

import math                              # use 常量 pi 与三角函数

from agv_common.geometry import (        # use 被测函数
    angle_diff, clamp, distance, quaternion_to_yaw, wrap_to_pi, yaw_to_quaternion)


def test_clamp_basic():                  # # 区间内原样
    assert clamp(0.5, 0.0, 1.0) == 0.5
    assert clamp(-2.0, 0.0, 1.0) == 0.0    # # 低于下界
    assert clamp(3.0, 0.0, 1.0) == 1.0     # # 高于上界


def test_clamp_reversed_bounds():        # # 上下界传反也要合理
    assert clamp(0.5, 1.0, 0.0) == 0.5


def test_wrap_to_pi():                   # # 折算到 (-pi, pi]
    assert abs(wrap_to_pi(0.3) - 0.3) < 1e-9
    assert abs(wrap_to_pi(3 * math.pi) - math.pi) < 1e-9   # # 3pi → pi
    assert abs(wrap_to_pi(-3 * math.pi) - math.pi) < 1e-9
    assert abs(wrap_to_pi(2 * math.pi)) < 1e-9             # # 2pi → 0


def test_angle_diff():                   # # 跨 ±pi 求差
    assert abs(angle_diff(3.0, -3.0) - 0.28318) < 1e-4     # # 差应走短边
    assert abs(angle_diff(0.0, 1.0) - 1.0) < 1e-9


def test_distance():                     # # 3-4-5 直角三角形
    assert abs(distance(0, 0, 3, 4) - 5.0) < 1e-9
    assert distance(1, 1, 1, 1) == 0.0


def test_quaternion_roundtrip():         # # yaw→四元数→yaw 应还原
    for yaw in (0.0, 0.5, -1.2, 3.0, -3.0):
        x, y, z, w = yaw_to_quaternion(yaw)
        assert abs(quaternion_to_yaw(x, y, z, w) - yaw) < 1e-6
