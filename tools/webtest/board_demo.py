#!/usr/bin/env python3
"""
board_demo —— 起真环境产生**真实的高频日志**，用于肉眼看效果 / 截图。

**它和 board_e2e 的分工**
  board_e2e 断言"分流是否正确"，用可控的行数保证可重复；
  board_demo 负责"看起来对不对"：跑**真的** velocity_arbiter 节点，
  它自己按 throttle_duration_sec=1.0 打避障日志 —— 面板上那条
  "前方 0.69m"是真实的避障决策结果，不是编的。

**为什么必须在进程内起网关**
真机上 arbiter 是 launch 拉起的子进程，它的 stdout 由
`StackRunner._pump_output` 读出来、经 `log_translate` 翻译、再喂给
`node.log()`。如果这里把网关也当独立子进程起，arbiter 的输出就
**只进自己的控制台**，网关根本收不到 —— 面板永远是空的，看起来
像"功能没生效"，实际是演示脚本没复刻这条链路。
所以这里精确复刻：arbiter 子进程 stdout → log_translate → node.log()。

用法：
    python3 tools/webtest/board_demo.py
    然后浏览器打开 http://127.0.0.1:8888
"""

import asyncio
import math
import os
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WS_ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
PORT = 8888


def start_scan_and_cmd():
    """后台节点：发激光（前方有障碍）+ 发导航速度。

    不发速度的话，arbiter 的 cmd_timeout 会让它一直输出 0，避障阈值
    触发不了，就看不到避障日志。

    ★ 不在这里调 rclpy.init()：主线程已经 init 过了，
      再调一次会抛 "Context.init() must only be called once"。
      rclpy 的上下文是进程级的，子线程直接建 Node 就行
      （但 init 必须在主线程先做完，之前踩的坑正好相反：
       主线程不 init、子线程建 Node → NotInitializedException）。
    """
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist

    node = Node('board_demo_src')
    pub_scan = node.create_publisher(LaserScan, '/scan', 10)
    pub_cmd = node.create_publisher(Twist, '/cmd_vel', 10)
    # ★ /cmd_vel_raw 也要发：仪表"速度"格订阅的是这条（底盘实际收到的）。
    #   不发的话那一格永远显示 "——"，演示看起来像功能没做。
    #   真机上这条由 velocity_arbiter 发，我们这里模拟它。
    pub_raw = node.create_publisher(Twist, '/cmd_vel_raw', 10)

    def tick():
        m = LaserScan()
        m.angle_min, m.angle_max = -math.pi, math.pi
        m.angle_increment = math.pi / 180.0
        m.range_min, m.range_max = 0.05, 8.0
        n = int((m.angle_max - m.angle_min) / m.angle_increment) + 1
        # 正前方 0.40~0.70m 来回摆，正好落在"减速/择向接管"区间，
        # 于是 arbiter 每秒会打一条避障日志 —— 真实的高频源。
        t = time.monotonic()
        front = 0.40 + 0.30 * (0.5 + 0.5 * math.sin(t * 0.7))
        m.ranges = [front if abs(m.angle_min + i * m.angle_increment) < 0.35
                    else 3.0 + 1.5 * math.sin((m.angle_min + i
                                               * m.angle_increment) * 3)
                    for i in range(n)]
        pub_scan.publish(m)
        c = Twist()
        c.linear.x = 0.30
        c.angular.z = 0.10 * math.sin(t)      # 轻微摆动，让角速度不是死值
        pub_cmd.publish(c)
        # 模拟仲裁的输出：近障减速（越近越慢）
        raw = Twist()
        raw.linear.x = 0.30 if front > 0.60 else max(0.08, 0.30 * front / 0.60)
        raw.angular.z = c.angular.z
        pub_raw.publish(raw)

    node.create_timer(0.1, tick)
    rclpy.spin(node)


def pump_stdout(proc, node):
    """复刻 StackRunner._pump_output：子进程 stdout → 翻译 → node.log()。

    这一层是演示脚本和真机唯一必须对齐的地方。少了它，arbiter 的
    日志只进自己的终端，网页面板永远空白（我第一版就踩了这个坑）。
    """
    from agv_web import log_translate as LT
    try:
        for line in proc.stdout:
            s = (line or '').rstrip()
            if not s or LT.is_noise(s):
                continue
            out = LT.translate(s)
            if not out:
                continue
            if len(out) > 300:
                out = out[:300] + '…'
            node.log(LT.guess_level(s), out)
    except Exception as exc:
        node.log('warn', f'读仲裁节点输出失败：{exc}')


async def main():
    import rclpy
    from aiohttp import web
    from rclpy.executors import MultiThreadedExecutor
    from agv_web.web_gateway import WebGateway, make_app

    os.environ.setdefault('ROS_DOMAIN_ID', '226')
    rclpy.init()
    node = WebGateway()
    # ★ 参数在构造函数里就赋值了，这里必须改属性再建 app ——
    #   第一版只改了 node.port，但 make_app 用的是 node.port，
    #   而 aiohttp 绑定用的是下面 TCPSite 里显式传的 PORT，两者要一致。
    node.port = PORT
    node.auth.token = ''
    node.auth.allow_ips = []

    ex = MultiThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    threading.Thread(target=start_scan_and_cmd, daemon=True).start()

    proc = subprocess.Popen(
        ['ros2', 'run', 'agv_hardware', 'velocity_arbiter'],
        cwd=WS_ROOT, env=dict(os.environ, PYTHONUNBUFFERED='1'),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    threading.Thread(target=pump_stdout, args=(proc, node), daemon=True).start()
    print(f'已启动 velocity_arbiter（1 Hz 避障日志的真实来源）')

    runner = web.AppRunner(make_app(node))
    await runner.setup()
    await web.TCPSite(runner, '0.0.0.0', PORT).start()
    print(f'网关：http://127.0.0.1:{PORT}')
    print('  侧栏「实时数据」应出现"避障"一格，距离每秒变；')
    print('  「运行日志」里不应有任何一行避障。Ctrl-C 结束。')
    try:
        while True:
            await asyncio.sleep(1.0)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            proc.kill()
        await runner.cleanup()
        try:
            ex.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
