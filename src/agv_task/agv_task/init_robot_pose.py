"""
init_robot_pose —— 设置机器人初始位姿（函数库，无 main，供其他代码 import）。
等价于在 RViz 里点 "2D Pose Estimate"。
==================================================================
为什么需要它：
    AMCL 启动时并不知道车在地图的哪个位置，粒子云是散开的。
    必须先告诉它一个大致位姿，它才能把粒子收拢、算出 map -> odom 的变换。
    不做这一步，RViz 里表现为：地图乱飘 / 报 "Fixed Frame [map] does not exist" /
    导航一动不动。

用法（在某个节点里）：
    from agv_task.init_robot_pose import init_robot_pose
    init_robot_pose(navigator, x, y, yaw)

坐标系约定（REP-103）：x 向前、y 向左、z 向上；yaw 逆时针为正，单位弧度。
"""
from agv_task.goto_pose import make_goal_pose        # use 复用唯一的位姿合成实现，避免重复


# 位姿合成：x, y, yaw → 指定坐标系下的 PoseStamped（薄封装，实现见 goto_pose.make_goal_pose）
# 参数：x, y - 位置(m)；yaw - 朝向(rad)；frame_id - 坐标系名（默认 map）
# 返回：PoseStamped
def pose_by_xyyaw(x, y, yaw, frame_id='map'):        # out 对外接口
    return make_goal_pose(x, y, yaw, frame_id)      # use 委托给唯一实现


# 初始化位姿：把 x,y,yaw 发给定位系统（/initialpose），并等 Nav2 就绪
# 参数：navigator - BasicNavigator 实例（已 rclpy.init）；
#       x, y - 初始位置(m)；yaw - 初始朝向(rad)；frame_id - 坐标系（默认 map）
# 返回：True=定位已收到初始位姿并激活；False=超时或被中断
def init_robot_pose(navigator, x, y, yaw=0.0, frame_id='map'):   # out 对外接口
    pose = pose_by_xyyaw(x, y, yaw, frame_id)       # use 合成初始位姿
    navigator.setInitialPose(pose)                 # io 发到 /initialpose
    # waitUntilNav2Active 内部会持续重发初始位姿，直到收到 /amcl_pose，
    # 所以不怕 AMCL 还没起来导致这一帧丢失。
    navigator.waitUntilNav2Active()                # io 阻塞等 amcl + bt_navigator 激活
    navigator.get_logger().info(
        f'初始位姿已设置: x={x}, y={y}, yaw={yaw}')   # io 日志
    return True                                     # use 返回成功
