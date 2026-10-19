#!/usr/bin/env python3
"""
agv_hardware —— 硬件抽象层（★ 硬件无关边界）。

设计定位：
    在"软件栈"与"物理硬件"之间划一条接口契约线。线上是任务/导航/搬货/交互逻辑，
    线下是传感器与执行器。仿真阶段线下用 Gazebo 实现；实车阶段只替换本包，
    线上代码一行不改。

本包当前提供：
    velocity_arbiter   差速速度仲裁节点：把"规划器想走的速度"与"反应式避障/安全"
                       叠加后发给底盘（近障减速 + 左右择空 + 缓回导航）。
    纯逻辑模块         reactive_avoid 的避障策略、MapChecker 的点位预检判据、
                       VelocityLimits 的限幅，都来自 agv_common。

实车替换点：把 velocity_arbiter 换成"真实下位机驱动"，对外话题/接口保持不变。
搬货机器不在本包：全项目只有一个搬货服务，见 agv_lift 的 cargo_node。
"""
