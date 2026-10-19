"""
robot_pose —— 实时获取小车当前位置（函数库，无 main，供其他代码 import）。

用法（在某个节点里）：
    from agv_task.robot_pose import get_current_pose
    pose = get_current_pose(node)      # 返回 dict 或 None
    if pose:
        print(pose['x'], pose['y'], pose['yaw'])

也可在本节点上直接挂一个周期性打印的节点类：
    from agv_task.robot_pose import RobotPoseNode
    node = RobotPoseNode()             # 起节点后 rclpy.spin(node) 即每秒打印一次位置

原理：小车位置由「定位链」决定——
    map ──(AMCL / slam_toolbox 发布)──► odom ──(Gazebo 差速插件发布)──► base_footprint
所以查询 map→base_footprint 的 TF 就得到全局位置 (x, y, yaw)。

坐标系约定（REP-103）：x 向前、y 向左、z 向上；yaw 逆时针为正，单位弧度。
"""
import math                                             # use 角度换算（rad → 度）

import rclpy                                            # use ROS 2 Python 客户端
from rclpy.node import Node                             # def 节点基类
from tf2_ros import Buffer, TransformListener           # def TF 缓冲区与监听器（自动维护变换树）
from tf_transformations import euler_from_quaternion    # use 四元数 → 欧拉角（取 yaw）


# ─────────────────────────── 函数库（供其他文件 import 调用） ───────────────────────────

# 从 TF 缓冲区查一次位置
# 参数：tf_buffer - TF 缓冲区（有 lookup_transform 方法即可，便于测试替身）
#       target_frame - 目标坐标系（默认 map，全局位置）
#       base_frame   - 机器人本体坐标系（默认 base_footprint，与 URDF 一致）
#       timeout_sec  - 单次查询等待上限（秒），默认 0.2，避免长时间阻塞
# 返回：dict {x, y, z, yaw, frame, stamp}；查不到时返回 None（不抛异常）
def query_pose(tf_buffer, target_frame='map', base_frame='base_footprint', timeout_sec=0.2):   # out 对外接口
    try:
        tf = tf_buffer.lookup_transform(                # io 查询坐标变换
            target_frame, base_frame,
            rclpy.time.Time(seconds=0),                 # use 0 = 取最新可用变换
            rclpy.time.Duration(seconds=timeout_sec))   # use 可用等待上限
    except Exception:                                   # # TF 还没建立/超时：返回 None 不崩
        return None

    t = tf.transform.translation                        # def 平移部分
    r = tf.transform.rotation                           # def 旋转部分（四元数）
    # 只取 yaw：平面小车的朝向（平地时 roll/pitch 近似 0，忽略）
    _, _, yaw = euler_from_quaternion([r.x, r.y, r.z, r.w])   # use 四元数 → 欧拉角
    return {                                            # out 组装结果字典
        'x': t.x,                                       # def 位置 x（m）
        'y': t.y,                                       # def 位置 y（m）
        'z': t.z,                                       # def 位置 z（m）
        'yaw': yaw,                                     # def 朝向（rad）
        'frame': target_frame,                          # def 所属坐标系（map）
        'stamp': tf.header.stamp,                       # def 变换时间戳
    }


# 便捷函数：给节点查一次当前位置（省得调用方自己建 Buffer）
# 参数：node - 任意 rclpy 节点（本函数在它上面挂一套可复用的 TF 监听）
#       target_frame / base_frame - 坐标系名
# 返回：同 query_pose（dict 或 None）
def get_current_pose(node, target_frame='map', base_frame='base_footprint'):   # out 对外接口
    # 若该节点还没建过 TF 监听，则给它建一套并挂在节点属性上复用
    if not hasattr(node, '_pose_tf_buffer'):            # # 首次调用：建监听
        node._pose_tf_buffer = Buffer()                 # def TF 缓冲区（挂节点上复用）
        node._pose_tf_listener = TransformListener(node._pose_tf_buffer, node)   # io 起监听
    return query_pose(node._pose_tf_buffer, target_frame, base_frame)            # use 查询并返回


# ─────────────────────────── 可选：周期性打印位置的节点类（供需要常驻监视的代码用） ───────────────────────────

class RobotPoseNode(Node):                              # out 实时位置查询节点（可复用）
    # 初始化：建 TF 监听、读参数、建定时器周期性查询打印
    # 参数：无
    # 返回：无
    def __init__(self):                                 # out 对外构造函数
        super().__init__('robot_pose')              # use 节点名 robot_pose

        # 参数：坐标系与查询频率
        self.declare_parameter('target_frame', 'map')            # io 目标坐标系（全局位置用 map）
        self.declare_parameter('base_frame', 'base_footprint')   # io 本体坐标系（与 URDF 一致）
        self.declare_parameter('rate', 1.0)                      # io 查询频率 Hz（默认每秒一次）
        self.target_frame = self.get_parameter('target_frame').value    # use 读目标坐标系
        self.base_frame = self.get_parameter('base_frame').value        # use 读本体坐标系
        rate = self.get_parameter('rate').value                         # use 读频率

        # TF 缓冲区 + 监听器：自动订阅 /tf 与 /tf_static，维护变换树
        self.tf_buffer = Buffer()                       # def TF 缓冲区
        self.tf_listener = TransformListener(self.tf_buffer, self)   # io 起监听

        # 定时器：周期性查一次位姿并打印（频率下限 0.1 Hz，防参数填 0 除零）
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.on_timer)   # io 周期回调
        self.get_logger().info(
            f'实时位置查询启动：{self.target_frame} → {self.base_frame}，{rate:.1f} Hz')

    # 定时回调：查一次当前位置并打印
    # 参数：无
    # 返回：无
    def on_timer(self):                                 # in 内部回调
        pose = query_pose(self.tf_buffer, self.target_frame, self.base_frame)   # use 查询位姿
        if pose is None:                                # # 还没收到 TF（刚启动或仿真没起）
            self.get_logger().warn(
                f'暂时取不到 {self.target_frame}→{self.base_frame} 变换，确认仿真/定位已启动…')
            return
        # 打印位置与朝向（yaw 另附角度制，便于人工读数）
        self.get_logger().info(
            f'当前位置 [{pose["frame"]}]：'
            f'x={pose["x"]:.3f} m, y={pose["y"]:.3f} m, '
            f'yaw={pose["yaw"]:.3f} rad ({math.degrees(pose["yaw"]):.1f}°), z={pose["z"]:.3f} m')
