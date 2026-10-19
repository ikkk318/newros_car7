"""
limits —— 速度限幅（安全约束的唯一收口点）。

规格书 §7.4：速度限幅 线 ≤ 1.0 m/s，角 ≤ 1.0 rad/s。
所有往外发速度的地方（网页摇杆、语音、调度、导航后处理）都必须过这里，
保证"限速"只有一处实现、一个数值来源，改限制只改一处。

本模块是纯逻辑，不依赖 ROS。默认值可被参数中心（agv_bringup/config）覆盖。
"""

from agv_common.geometry import clamp    # use 复数值限幅


class VelocityLimits:                    # out 速度限幅器（可复用对象）
    # 初始化：读入线/角速度上下限
    # 参数：max_linear - 线速度绝对值上限(m/s)；max_angular - 角速度绝对值上限(rad/s)
    #       min_linear/max_*_forward 等暂不细分，先按对称限幅
    # 返回：无
    def __init__(self, max_linear=1.0, max_angular=1.0):   # out 对外构造函数
        self.max_linear = float(max_linear)     # def 线速度上限(m/s)
        self.max_angular = float(max_angular)   # def 角速度上限(rad/s)

    # 限幅线速度
    # 参数：v - 原始线速度(m/s)
    # 返回：限幅后的线速度(m/s)
    def clamp_linear(self, v):              # out 对外接口
        return clamp(float(v), -self.max_linear, self.max_linear)   # use 对称限幅

    # 限幅角速度
    # 参数：w - 原始角速度(rad/s)
    # 返回：限幅后的角速度(rad/s)
    def clamp_angular(self, w):             # out 对外接口
        return clamp(float(w), -self.max_angular, self.max_angular)   # use 对称限幅

    # 一次性限幅 (v, w)
    # 参数：v - 线速度；w - 角速度
    # 返回：(v_limited, w_limited) 元组
    def clamp_twist(self, v, w):            # out 对外接口
        return (self.clamp_linear(v), self.clamp_angular(w))   # use 分别限幅
