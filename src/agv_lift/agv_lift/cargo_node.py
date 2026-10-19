"""
cargo_node —— 搬货服务（顶升 / 下降二合一）。

★ 这是全项目唯一与"搬货机器"打交道的地方，也是后续接真实硬件的唯一改动点。
  现阶段：只维护"正在搬货"标志，立即返回成功，不驱动任何机器。
  将来：在本文件的服务实现里加上真正的顶升/下降动作与到位判断即可，
        外部接口（服务名、字段、话题）保持不变。

服务：
    /agv/cargo   CargoCmd   command="lift"  → 顶升：开始搬货（busy=true）
                            command="lower" → 下降：放货完成（busy=false）
话题：
    /agv/cargo/busy  std_msgs/Bool   true=正在搬货；false=空闲/放货完成
                                     变化时立即发；另外每秒补发一次，晚加入的订阅者也能看到

参数：
    service_name    服务名（默认 /agv/cargo）
    busy_topic      "正在搬货"标志话题（默认 /agv/cargo/busy）
    publish_period  标志补发周期(s)（默认 1.0）
"""

import rclpy                                        # use ROS 2 Python 客户端
from rclpy.node import Node                         # def 节点基类
from std_msgs.msg import Bool                       # def 布尔标志消息

from agv_interfaces.srv import CargoCmd             # use 搬货服务定义

LIFT_WORDS = ('lift', 'up', '顶升')                  # def 顶升命令的写法
LOWER_WORDS = ('lower', 'down', '放货', '下降')       # def 下降命令的写法


class CargoNode(Node):                              # out 搬货服务节点
    # 初始化：读参数、建话题与服务
    # 参数：无
    # 返回：无
    def __init__(self):                             # out 对外构造函数
        super().__init__('cargo_node')              # use 节点名 cargo_node

        # 参数
        self.declare_parameter('service_name', '/agv/cargo')
        self.declare_parameter('busy_topic', '/agv/cargo/busy')
        self.declare_parameter('publish_period', 1.0)
        service_name = self.get_parameter('service_name').value
        busy_topic = self.get_parameter('busy_topic').value
        period = max(float(self.get_parameter('publish_period').value), 0.1)

        self.busy = False                           # def 正在搬货？（上电为空闲）
        self.pub_busy = self.create_publisher(Bool, busy_topic, 10)     # io 搬货中标志出
        self.create_service(CargoCmd, service_name, self.on_cargo)      # io 搬货服务
        # 周期补发：状态没变也让后启动的订阅者能立刻知道当前状态
        self.create_timer(period, lambda: self.pub_busy.publish(Bool(data=self.busy)))
        self.pub_busy.publish(Bool(data=self.busy))                     # io 先发一次初值
        self.get_logger().info(
            f'搬货服务启动：{service_name}（lift=顶升 / lower=下降），'
            f'搬货中标志 {busy_topic}')

    # 搬货服务回调：只改标志，立即返回（接硬件时改这里）
    # 参数：request - CargoCmd.Request（command/request_id）；response - 待填 Response
    # 返回：填好的 response
    def on_cargo(self, request, response):          # in 内部回调
        cmd = str(request.command or '').strip().lower()
        if cmd in LIFT_WORDS:                       # # 顶升：开始搬货
            self.busy = True
            response.ok = True
            response.message = '已顶升，开始搬货'
        elif cmd in LOWER_WORDS:                    # # 下降：放货完成，回到空闲
            self.busy = False
            response.ok = True
            response.message = '已下降，放货完成（空闲）'
        else:                                       # # 其它命令：拒绝，标志不动
            response.ok = False
            response.message = f'未知命令「{request.command}」：只支持 lift（顶升）/ lower（下降）'
        response.busy = self.busy                   # def 回带执行后的状态
        self.pub_busy.publish(Bool(data=self.busy))   # io 立即广播新状态
        self.get_logger().info(
            f'{response.message}（request_id={request.request_id or "-"}）')
        return response


# 程序入口
# 参数：无
# 返回：无
def main(args=None):                                # out 程序入口
    rclpy.init(args=args)                           # io 初始化 ROS 2
    node = CargoNode()                              # def 建节点
    try:
        rclpy.spin(node)                            # io 转圈等请求
    except KeyboardInterrupt:                       # # Ctrl-C 正常退出
        pass
    finally:
        node.destroy_node()                         # io 释放资源
        rclpy.shutdown()                            # io 关闭 ROS 2


if __name__ == '__main__':
    main()                                          # use 直接运行时进入主函数
