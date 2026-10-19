"""
store_client —— 点位 / 路线服务客户端（把 AgvStore 的调用与等待收拢到一处）。

背景：任务层要"点名 → 坐标"和"路线名 → 途经点"。这两件事原先各写一遍
「等 3 秒服务 → 建请求 → call_async → 轮询 future → 超时判空」，
一共 4 处重复代码（查点、查路线、列点、列路线），且每处都要自己对
"服务不在、超时、ok=false"三种失败做同样处理。

这里收拢成一个类：任务层只说"我要哪个点位的坐标"，不关心服务怎么调。

★ 与 /agv/query 的分工：本模块只做"取数据给任务层用"，
  控制台的查询展示走 /agv/query + /agv/store，不经过这里。
"""

import time                              # use 超时等待

from agv_interfaces.srv import AgvStore  # use 统一存储服务


class StoreClient:                       # out 点位/路线客户端
    # 初始化
    # 参数：node - ROS 节点（用来建客户端与打日志）；timeout_s - 单次调用等待上限(s)
    # 返回：无
    def __init__(self, node, timeout_s=3.0):    # out 对外构造函数
        self.node = node                 # def 宿主节点
        self.timeout_s = float(timeout_s)     # def 等待上限
        # 一个客户端即可覆盖点位与路线：靠请求里的 resource 区分
        self._cli = node.create_client(AgvStore, '/agv/store')   # io 建客户端

    # 内部：调一次服务，返回响应 dict 或 None（不可用/超时/失败）
    # 参数：resource/action - 资源与动作；**fields - 其余请求字段
    # 返回：dict 或 None
    def _call(self, resource, action, **fields):    # in 内部方法
        if not self._cli.wait_for_service(timeout_sec=self.timeout_s):
            self.node.get_logger().error(f'/agv/store 不可用（{resource}/{action}）')
            return None
        req = AgvStore.Request()         # def 请求
        req.resource = resource
        req.action = action
        for key, value in fields.items():     # # 逐字段填（缺省值由 srv 默认给）
            setattr(req, key, value)
        future = self._cli.call_async(req)    # io 异步调用
        deadline = time.monotonic() + self.timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        res = future.result() if future.done() else None
        if res is None:
            self.node.get_logger().error(f'/agv/store 无应答（{resource}/{action}）')
        return res

    # 查一个点位的坐标
    # 参数：name - 点名
    # 返回：dict{'x','y','yaw','kind'} 或 None
    def waypoint(self, name):            # out 对外接口
        res = self._call('waypoint', 'get', name=str(name))
        if res is None or not res.ok or not res.items:
            return None
        p = res.items[0]                 # use get 只返回 1 条
        return {'x': float(p.x), 'y': float(p.y),
                'yaw': float(p.yaw), 'kind': str(p.kind or '')}

    # 查一条路线的途经点
    # 参数：name - 路线名（空则返回 None，表示"不用路线"）
    # 返回：list[dict] 或 None（取不到）
    def route(self, name):               # out 对外接口
        if not name:
            return None
        res = self._call('route', 'get', name=str(name))
        if res is None or not res.ok:
            return None
        return [{'name': str(p.name), 'x': float(p.x),
                 'y': float(p.y), 'yaw': float(p.yaw)} for p in res.items]
