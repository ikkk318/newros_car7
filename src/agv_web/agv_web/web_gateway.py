"""
web_gateway —— 入口：装配一个 rclpy 节点 + 一个 aiohttp 应用，然后把它们跑起来。

**代码都去哪了（以前这里 3236 行）**

    gateway/util.py           叶子工具：日志转发判据、局域网地址、静态目录、tasks.yaml
    gateway/stack.py          栈起停 + 进程组/SHM/端口的清理
    gateway/state.py          StateMixin：ROS 回调、位姿/TF、快照
    gateway/commands.py       CommandMixin：遥控、模式、指令、任务缓冲、地图库
    gateway/node.py           WebGateway：参数 / 共享字段 / ROS 句柄 / 启动横幅
    gateway/web_app.py        ApiBase + ROUTE_SPECS + make_app
    gateway/handlers_ops.py   ApiOps：有副作用的动作（遥控/指令/栈起停/退出）
    gateway/handlers_data.py  ApiData：地图库 / 任务缓冲 / 路线 / 自检

**本文件只剩两件事**：设置通信传输、把节点与 HTTP 服务串起来并守住退出路径。

**线程模型（整个项目最容易写错的地方）**
  · 主线程跑 asyncio 事件循环（aiohttp）；
  · 一个守护线程跑 rclpy.spin（MultiThreadedExecutor）；
  · 两者只通过 `run_coroutine_threadsafe` 与线程安全字段交互。
★ 绝不能在 ROS 回调里 `await`，也不能在 asyncio 任务里 `spin_once`。
"""

import asyncio                             # use 主事件循环 / 退出路径
import os                                  # use 环境变量
import threading                           # use 隔离 rclpy spin

import rclpy                               # use ROS 2 Python 接口
from rclpy.executors import MultiThreadedExecutor

from .gateway.node import WebGateway
from .gateway.stack import cleanup_residual, purge_stale_shm
from .gateway.util import _FASTDDS_ENV, _lan_urls

# ★ aiohttp 只在真正起服务时才需要 —— 缺了它要给一句中文提示，
#   而不是甩一屏 ImportError traceback。所以这里先探一下再决定要不要
#   import web_app（web_app 里 aiohttp 是硬依赖）。
try:
    import aiohttp                          # noqa: F401  (只用于探测)
    HAVE_AIOHTTP = True
except Exception:                           # pragma: no cover - 仅缺依赖时走这里
    HAVE_AIOHTTP = False

if HAVE_AIOHTTP:
    from .gateway.web_app import make_app
else:                                       # pragma: no cover
    make_app = None


def main(args=None):
    """程序入口：起 rclpy spin 线程 + aiohttp 主循环。"""
    if not HAVE_AIOHTTP:
        print('缺少依赖 aiohttp：请执行 pip3 install aiohttp')
        return 1

    # ★ 必须在 rclpy.init() 之前：participant 一创建 transport 就定死了。
    os.environ.update(_FASTDDS_ENV)
    rclpy.init(args=args)
    node = WebGateway()

    # ★ 启动即清理上次残留：顺序很关键，必须在 WebGateway 构造之后，
    #   因为要读 manage_stack 参数才能决定"该不该清"。
    #
    #   安全边界（这条最重要）：
    #     manage_stack=false 意味着用户自己在别的终端起栈、网页只当遥控器。
    #     这种场景下用户的那套栈是"合法在运行"的，清掉它等于把用户正在跑的
    #     任务杀掉 —— 绝对不能做。所以只有 manage_stack=true（栈归本进程管，
    #     也就是说别人不该有栈在跑）时才清理。
    #   另一个逃生口：环境变量 AGV_NO_CLEANUP=1 可跳过。
    if (node.stack is not None
            and os.environ.get('AGV_NO_CLEANUP') not in ('1', 'true', 'yes')):
        cleanup_residual(log=lambda m: node.log('warn', m))
        # ★ 进程清了，它们留下的共享内存段还在。
        #   那两个段不删，新起的参与者照样在 open_and_lock_file 上撞墙：
        #   `[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port74xx`
        #   → 同机优先走共享内存，SHM 一废就发现不到彼此 →
        #     网页"节点未发现 / 收不到 /scan / 地图黑屏"。
        #   网关自己的段在 /proc/*/maps 里挂着，属于"有主"，不会被误删。
        purge_stale_shm(log=lambda m: node.log('info', m))

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = make_app(node)
    # ★ 不打印 http://0.0.0.0:8080 —— 那个地址在浏览器里是**打不开**的
    #   （0.0.0.0 只表示"监听所有网卡"，不是一个可连接的地址）。
    #   以前这里写 0.0.0.0 并附一句"手机与电脑同一局域网即可访问"，
    #   用户照着敲不开，就自己去试 127.0.0.1，然后 ERR_CONNECTION_REFUSED。
    #   现在直接给出可点击的局域网地址，一个都没有时明确提示原因。
    _urls = _lan_urls(node.port)
    if _urls:
        for _u in _urls:
            node.log('info', f'控制台地址：{_u}（手机连同一 WiFi 即可打开）')
    else:
        node.log('warn', '控制台只监听了本机回环，其它设备访问不到：'
                         '检查虚拟机网络是否为「桥接模式」，或手动指定 host')

    # ★ 把当前事件循环存到节点上：/api/shutdown 要从"请求的那个协程"里
    #   让 run_forever 退出，而它和 main() 不在同一个栈帧里，只能靠引用。
    try:
        node._gw_loop = asyncio.get_event_loop()
    except RuntimeError:
        node._gw_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(node._gw_loop)

    from aiohttp import web

    started_ok = False
    try:
        web.run_app(app, host=node.host, port=node.port, print=None)
        started_ok = True
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        # ★ 端口被占是最常见的启动失败，而 aiohttp 抛的是一整屏英文
        #   traceback（到 base_events.py 里 raise OSError 为止），
        #   用户完全看不出"到底哪里出错了"。这里拦下来给中文结论。
        import errno as _errno
        if getattr(exc, 'errno', None) in (_errno.EADDRINUSE, 98):
            node.log('error', f'端口 {node.port} 已被占用 —— 可能上一次的'
                              f'控制台还没退干净。换个端口重试（例如 '
                              f'port:={int(node.port) + 1}），或先执行 '
                              f'bash tools/stop_all.sh 清理。')
        elif getattr(exc, 'errno', None) in (_errno.EACCES, 13):
            node.log('error', f'没有权限绑定端口 {node.port}（1024 以下端口'
                              f'需要 root）。请改用 1024 以上的端口。')
        else:
            node.log('error', f'启动 HTTP 服务失败：{exc}')
    finally:
        node.publish_twist(0.0, 0.0)
        # ★ 退出前必须把栈停干净：留着的 gzserver 会占着共享内存段，
        #   下次启动直接报 "Address already in use"；留下的 Nav2 还会继续
        #   往底盘灌速度，非常危险。
        if node.stack is not None and node.stack.is_running():
            node.log('warn', '网关退出：正在停止 ROS 栈…')
            try:
                node.stack.stop(timeout=10.0)
            except Exception as exc:
                node.get_logger().error(f'停止栈失败：{exc}')
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if started_ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
