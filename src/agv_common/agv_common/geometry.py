"""
geometry —— 几何与角度工具（纯 Python，不依赖 ROS，不依赖 numpy 之外的东西）。

为什么单独一份：角度归一到 ±π、求两点距离、四元数↔yaw 这些活儿，
waypoint（存点）、lift（对位）、task（导航/对位判据）三个包都要用。
集中放这里，避免各包各写一份、行为不一致。

坐标系约定（REP-103）：x 向前、y 向左、z 向上；yaw 逆时针为正，单位弧度。
"""

import math                              # use 三角函数与常量 pi


# 数值限幅：把 value 夹到 [lo, hi] 区间内
# 参数：value - 待限幅的值；lo/hi - 下界/上界（要求 lo <= hi）
# 返回：限幅后的值
def clamp(value, lo, hi):                # out 对外接口
    if lo > hi:                          # # 传反了：等价于交换，避免返回怪异结果
        lo, hi = hi, lo
    if value < lo:                       # # 低于下界
        return lo
    if value > hi:                       # # 高于上界
        return hi
    return value                         # use 区间内原样返回


# 角度归一化：把任意角度折算到 (-pi, pi]
# 参数：angle - 任意弧度角（可正可负、可超过 2pi）
# 返回：归一化后的角度（rad）
def wrap_to_pi(angle):                   # out 对外接口
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi   # use 取模映射到 [-pi, pi)
    # 上面公式把 pi 映射成了 -pi，这里统一成 (-pi, pi]，与常见约定一致
    if wrapped == -math.pi:              # # 边界：-pi 与 pi 是同一方向
        return math.pi
    return wrapped


# 两角之差：b - a，并归一化到 (-pi, pi]
# 参数：a, b - 两个角度（rad）
# 返回：差值（rad），正=逆时针
def angle_diff(a, b):                    # out 对外接口
    return wrap_to_pi(b - a)             # use 先做差再归一化


# 平面两点距离
# 参数：x1, y1 - 第一点；x2, y2 - 第二点
# 返回：欧氏距离（m）
def distance(x1, y1, x2, y2):            # out 对外接口
    return math.hypot(x2 - x1, y2 - y1)  # use hypot 比 sqrt(dx^2+dy^2) 更稳，防溢出


# 偏航角 → 四元数（只绕 z 转，平面小车用）
# 参数：yaw - 偏航角（rad）
# 返回：(x, y, z, w) 四元数元组
def yaw_to_quaternion(yaw):              # out 对外接口
    half = yaw * 0.5                     # use 半角公式
    return (0.0, 0.0, math.sin(half), math.cos(half))   # use 绕 z 轴旋转四元数


# 四元数 → 偏航角（只取绕 z 的分量，忽略 roll/pitch，平地小车够用）
# 参数：x, y, z, w - 四元数分量
# 返回：偏航角（rad，归一到 (-pi, pi]）
def quaternion_to_yaw(x, y, z, w):       # out 对外接口
    siny_cosp = 2.0 * (w * z + x * y)    # use 标准四元数→欧拉 yaw 公式分子
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)   # use 分母
    return math.atan2(siny_cosp, cosy_cosp)   # use atan2 求角，自带象限处理
