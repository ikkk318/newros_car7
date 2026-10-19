"""
waypoint_node —— 点位 / 路线管理节点（把两个纯 Python 存储包成 1 个 ROS 服务）。

服务接口：
    /agv/store    AgvStore   点位与路线的统一增删查列
                             （resource = waypoint|route，action = save|get|list|delete）

为什么只有 1 个服务（而不是原先的 8 个）：
    原先 Waypoint{Save,Get,List,Delete} 与 Route{Save,Get,List,Delete} 两两同构，
    都是"给个名字，对一个带名字的位姿列表做增删查列"。点位就是只有 1 个位姿的
    资源，路线就是有 N 个位姿的资源——本来就是同一个东西，所以合并。
    校验/回执逻辑集中在纯 Python 的 store_dispatch，本节点只做
    「srv 字段 ↔ dict」的搬运，保持薄、可单测。

参数：
    waypoint_file  点位 YAML 文件路径（默认 /tmp/agv_waypoints.yaml）
    routes_file    路线 YAML 文件路径（默认 /tmp/agv_routes.yaml）
    frame_id       坐标系（默认 map）
"""

import os                                # use 默认路径拼接

import rclpy                             # use ROS 2 Python 客户端
from rclpy.node import Node              # def 节点基类

from agv_interfaces.msg import NamedPose          # use 带名字的位姿
from agv_interfaces.srv import AgvStore           # use 统一存储服务

from agv_waypoint.route_store import RouteStore          # use 纯逻辑路线存储
from agv_waypoint.store_dispatch import dispatch         # use 纯逻辑调度
from agv_waypoint.waypoint_store import WaypointStore    # use 纯逻辑点位存储


class WaypointNode(Node):                # out 点位/路线服务节点
    # 初始化：读参数、建存储、注册服务
    # 参数：无
    # 返回：无
    def __init__(self):                  # out 对外构造函数
        super().__init__('waypoint_node')      # use 节点名 waypoint_node

        # 参数：文件路径与坐标系（都可由 launch/参数中心覆盖）
        self.declare_parameter('waypoint_file', '/tmp/agv_waypoints.yaml')
        self.declare_parameter('routes_file', '/tmp/agv_routes.yaml')
        self.declare_parameter('frame_id', 'map')
        wp_path = self.get_parameter('waypoint_file').value
        rt_path = self.get_parameter('routes_file').value
        frame_id = self.get_parameter('frame_id').value

        self.store = WaypointStore(wp_path, frame_id)   # def 点位存储（构造即载入）
        self.routes = RouteStore(rt_path, frame_id)     # def 路线存储（构造即载入）
        self.get_logger().info(          # io 启动日志
            f'点位/路线管理启动：点位 {os.path.abspath(wp_path)}'
            f'（{self.store.count()} 个）、路线 {os.path.abspath(rt_path)}'
            f'（{self.routes.count()} 条），坐标系 {frame_id}')

        self.create_service(AgvStore, '/agv/store', self.on_store)   # io 统一存储服务

    # 统一存储服务：把 ROS 请求搬成纯逻辑 dict，调度后把结果搬回响应
    # 参数：request - AgvStore.Request；response - 待填 Response
    # 返回：填好的 response
    def on_store(self, request, response):   # in 内部回调
        req = {                              # def 纯逻辑请求
            'resource': request.resource,
            'action': request.action,
            'name': request.name,
            'x': request.x,
            'y': request.y,
            'yaw': request.yaw,
            'kind': request.kind,
            'point_names': list(request.point_names),
        }
        out = dispatch(req, self.store, self.routes)   # use 纯逻辑调度

        response.ok = bool(out['ok'])
        response.message = out['message']
        response.count = int(out['count'])
        response.names = list(out['names'])
        response.counts = [int(c) for c in out['counts']]
        response.items = [_to_msg(p) for p in out['items']]   # use 位姿列表 → 消息

        # 日志：写操作与失败打 info/warn，读操作只打 debug（避免刷屏）
        if not response.ok:
            self.get_logger().warn(response.message)
        elif req['action'] in ('save', 'delete'):
            self.get_logger().info(response.message)
        else:
            self.get_logger().debug(response.message)
        return response


# 纯逻辑位姿 dict → NamedPose 消息
# 参数：p - {'name','x','y','yaw','kind'}
# 返回：NamedPose
def _to_msg(p):                          # in 内部辅助
    m = NamedPose()
    m.name = str(p.get('name', ''))
    m.x = float(p.get('x', 0.0))
    m.y = float(p.get('y', 0.0))
    m.yaw = float(p.get('yaw', 0.0))
    m.kind = str(p.get('kind', '') or '')
    return m


# 程序入口
# 参数：无
# 返回：无
def main(args=None):                     # out 程序入口
    rclpy.init(args=args)                # io 初始化 ROS 2
    node = WaypointNode()                # def 建节点
    try:
        rclpy.spin(node)                 # io 转圈等请求
    except KeyboardInterrupt:            # # Ctrl-C 正常退出
        pass
    finally:
        node.destroy_node()              # io 释放资源
        rclpy.shutdown()                 # io 关闭 ROS 2


if __name__ == '__main__':
    main()                               # use 直接运行时进入主函数
