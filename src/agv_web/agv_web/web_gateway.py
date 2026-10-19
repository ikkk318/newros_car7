"""
web_gateway —— AGV 网页控制台的后端网关（rclpy 节点 + aiohttp 服务，同进程）。

**它替代了什么**
原来的终端控制台 `tools/agv_console.py` 是一个交互式 Python 程序：靠 `input()`
读键盘、靠 `setsid`/`xterm` 开子进程、靠 `pkill` 清残留。网页控制台不能这么干，
所以这里把"能力"拆出来重做，只保留必要的一层：

    浏览器 ──HTTP/WebSocket──► 本节点 ──ROS 2 话题/服务──► 现有的业务节点

**明确不做的事**
- 不重写任务逻辑：所有指令都走现成的 `/agv/command`（`source='web'`），
  地图/点位/路线走 `/agv/store`，状态走 `/agv/task_state` 与 `/agv/query`。
  这样网页和终端控制台看到的是**同一套状态**，不会各说各话。
- 不让前端直接发 `/cmd_vel`：遥控走 `/agv/web_teleop`，由 task_manager 校验
  手动模式后再转发，避免绕过互锁。

**线程模型（这里最容易写错）**
rclpy 的 spin 和 aiohttp 的 event loop 都是阻塞式的，必须分开：
- 主线程跑 asyncio event loop（aiohttp）；
- 一个守护线程跑 rclpy.spin；
- 两者之间只通过 `asyncio.run_coroutine_threadsafe` / 线程安全队列交互。
★ 绝不能在 ROS 回调里直接 `await`，也不能在 asyncio 任务里直接 `rclpy.spin_once`。
"""

import asyncio                             # use HTTP/WebSocket 事件循环
import math                                # use 角度换算
import os                                  # use 环境变量/路径
import re                                  # use 跨节点日志转发判据
import shlex                                # use 日志里安全打印启动命令
import signal                               # use 优雅停止子进程组
import socket                               # use 探测本机局域网 IP（打印访问地址）
import subprocess                          # use 起停 ROS 栈
import threading                           # use 隔离 rclpy spin 线程
import time                                # use 计时/日志

import rclpy                               # use ROS 2 Python 接口
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

# ★ rosout（rcl_interfaces/Log）：用来把**其它节点**（task_manager / cargo_node）
#   的日志转发到网页运行日志。
#   不订阅它的话，网页日志里只有网关自己产生的条目 —— 用户在网页上永远看不到
#   "[顶升] 结束"这种由任务节点打印的关键事件，只能去终端翻。
#   用 try 包住：老环境若缺 rcl_interfaces，网关仍要能起来（只是不转发）。
try:
    from rcl_interfaces.msg import Log as RosoutLog
except Exception:                           # pragma: no cover - 仅缺依赖时走这里
    RosoutLog = None

from agv_interfaces.msg import AgvCommand, TaskState
from agv_interfaces.srv import AgvQuery, AgvStore, CargoCmd
from agv_common.geometry import quaternion_to_yaw
from agv_common.web_guard import (AuthGate, ControlLease, TeleopWatchdog,
                                 BAD_TOKEN, LOCKED_BY_OTHER)

from .grid_image import occupancy_to_png, grid_meta, scan_points_to_map
from .status_board import LogFilter, build_gauges, front_distance   # use 日志降噪 + 顶部仪表
from .stack_manager import (               # use 栈起停（纯逻辑部分可单测）
    MODE_MAPPING, MODE_NAV, MODE_TEXT, MODES, StackError, build_launch_command,
    plan_switch, teleop_topic_for, validate_map_name,
)
try:                                       # # aiohttp 是可选的运行时依赖
    from aiohttp import web
    HAVE_AIOHTTP = True
except Exception:                          # pragma: no cover - 仅缺依赖时走这里
    web = None
    HAVE_AIOHTTP = False


# ══════════════════════════ 〇、通信传输：只用 UDP ══════════════════════════
# ★ 为什么关掉 FastDDS 的共享内存（这条修的是最贵的一类故障）：
#   同机 Node 默认优先走 SHM，而 SHM 靠 /dev/shm 里的 fastrtps_portXXXX 文件
#   做 rendezvous。被 Ctrl-C 打断的节点经常死在 destroy_node() 里，
#   段来不及 unlink；下一个栈的 participant 撞上残留文件就会刷
#   `[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port74xx`，
#   然后**半死不活**：节点全在、话题全在，彼此就是收不到数据。
#   现象五花八门 —— 自检说节点未发现、地图黑屏、任务发出去 task_manager
#   没反应，但独立 ros2 CLI 看又一切正常。
#
#   本项目单机、数据量小（激光 10Hz、TF 50Hz、地图偶发），UDP 完全够用，
#   而且没有共享内存文件可残留 —— 从根上消掉这一类故障。
#   注意：必须在 rclpy.init() **之前**写进环境变量，participant 一创建
#   就固定了 transport，之后再改没用。
_FASTDDS_ENV = {'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4'}


# ══════════════════════════ 一、节点：ROS 侧 ══════════════════════════

class WebGateway(Node):
    """网关节点：持有全 ROS 侧的订阅/发布/服务客户端，对外只暴露线程安全的方法。"""

    def __init__(self):
        super().__init__('agv_web_gateway')

        # ── 参数（全部可由 launch / yaml 覆盖）──
        self.declare_parameter('host', '0.0.0.0')          # def 监听地址（0.0.0.0=局域网可访问）
        self.declare_parameter('port', 8080)               # def HTTP 端口
        self.declare_parameter('token', '')                # def 访问口令（空=不校验）
        self.declare_parameter('allow_ips', '')            # def IP 白名单前缀，逗号分隔（空=不限制）
        # ★ 控制权到底该多久过期？这里踩过一次大坑，记下来：
        #   原来默认 3.0 秒，本意是"人走了自动把锁还回去"。但用户实操时的
        #   动作节奏是：点「取得控制权」→ 想一下往哪走（3~5 秒）→ 按方向键。
        #   中间这几秒什么都不发，锁就过期了；等他按下去时前端还以为自己有锁，
        #   后端回 409 locked_by_other，于是必须再点一次「取得控制权」。
        #   连起来就是"每走一步都要重新点一次"，完全没法用。
        #
        #   关键认知：「把车停住」和「把锁还回去」是两件不同的事，不该共用一个超时。
        #     · 车停住 —— 由 TeleopWatchdog + ROS 的 cmd_timeout 保证，0.35/0.5 秒量级，
        #                 必须是短超时，这是安全底线；
        #     · 锁释放 —— 只是"腾出操作权给别的设备"，慢一点完全无害。
        #                 没锁的设备本来也发不动车（发了也 409），不存在安全风险。
        #   所以锁的超时大幅放宽到 120 秒，并且按住方向键期间会持续续期、
        #   页面可见时也会定时续期（见 /api/lease action=renew）。
        #   真要"人走了立刻还锁"，交给页面的 beforeunload 主动 release。
        self.declare_parameter('lease_idle_sec', 120.0)    # def 控制权闲置释放(s)
        self.declare_parameter('teleop_stale_sec', 0.35)   # def 遥控判定掉线(s)，须小于 ROS 的 0.5
        self.declare_parameter('max_linear', 0.35)         # def 网页遥控限速(m/s)
        self.declare_parameter('max_angular', 0.9)         # def 网页遥控限角速度(rad/s)
        self.declare_parameter('map_frame', 'map')         # def 地图坐标系
        self.declare_parameter('base_frame', 'base_footprint')  # def 车体坐标系
        self.declare_parameter('scan_topic', '/scan')      # def 激光话题
        self.declare_parameter('map_topic', '/map')        # def 地图话题
        self.declare_parameter('plan_topic', '/plan')      # def 全局路径话题
        self.declare_parameter('vel_topic', '/cmd_vel_raw')  # def 底盘实际速度话题（仪表用）
        self.declare_parameter('cargo_busy_topic', '/agv/cargo/busy')  # def 载货中标志（仪表"动作"格用）
        self.declare_parameter('rosout_topic', '/rosout')    # def 其它节点的日志（转发到网页日志）
        self.declare_parameter('relay_rosout', True)         # def 是否转发其它节点日志
        self.declare_parameter('cmd_topic', '/agv/web_teleop')  # def 遥控速度话题（导航模式）
        self.declare_parameter('log_lines', 300)           # def 内存里保留多少条日志
        # ★ 工作模式：
        #   'nav'     导航模式——有 arbiter，遥控走 /agv/web_teleop 由 task_manager 校验转发
        #   'mapping' 建图模式——没有 arbiter 也没有 task_manager，底盘直接听 /cmd_vel_raw，
        #             遥控必须发到 /cmd_vel_raw（否则车根本不动），地图来自 slam_toolbox
        self.declare_parameter('mode', 'nav')              # def 工作模式
        self.declare_parameter('teleop_topic_mapping', '/cmd_vel_raw')  # def 建图遥控话题
        self.declare_parameter('map_min_interval', 0.8)    # def 地图重编码最小间隔(s)
        # ★ 位姿保鲜期（秒）：TF 连续这么久查不到，就对外报"没有位姿"，
        #   而不是无限期沿用最后一帧。切换模式/换图后旧位姿必须失效，
        #   否则网页上的小车会停在上一套栈的位置上，看着像定位正常。
        self.declare_parameter('pose_hold_sec', 2.0)       # def 位姿保鲜期(s)
        # ★ manage_stack=true 时，本进程自己 Popen 整套 ROS 栈，用户只需一条命令；
        #   false 时回到"网关只做代理"，适合接已经手工起好的栈。
        self.declare_parameter('manage_stack', True)       # def 是否由本进程起停 ROS 栈
        self.declare_parameter('ws_root', '')              # def 工作区根目录（空=自动探测）

        gp = self.get_parameter
        self.host = gp('host').value
        self.port = int(gp('port').value)
        self.map_frame = gp('map_frame').value
        self.base_frame = gp('base_frame').value
        self.max_linear = float(gp('max_linear').value)
        self.max_angular = float(gp('max_angular').value)
        self.log_lines = int(gp('log_lines').value)
        self.mode = str(gp('mode').value or 'nav').strip().lower()
        self.map_min_interval = float(gp('map_min_interval').value)
        self.pose_hold_sec = float(gp('pose_hold_sec').value)
        self.ws_root_override = str(gp('ws_root').value or '').strip()

        # ── 准入与控制权（纯逻辑，见 agv_common.web_guard）──
        # allow_ips 用逗号分隔字符串传入：launch 把 LaunchConfiguration 直接映射到
        # 参数时，若声明成字符串数组会因类型不匹配报错，字符串最稳。
        raw_allow = gp('allow_ips').value
        if isinstance(raw_allow, (list, tuple)):
            allow = [str(s).strip() for s in raw_allow if str(s).strip()]
        else:
            allow = [s.strip() for s in str(raw_allow or '').split(',') if s.strip()]
        self.auth = AuthGate(token=gp('token').value, allow_ips=allow)
        self.lease = ControlLease(idle_timeout=float(gp('lease_idle_sec').value))
        self.watchdog = TeleopWatchdog(stale_timeout=float(gp('teleop_stale_sec').value))
        self.stack = None              # def 栈起停器（在 __init__ 末尾创建，见下）

        # ── 共享状态（被 asyncio 线程读，故用锁保护）──
        self._gw_lock = threading.Lock()
        self.current_map = ''          # def 当前地图名（供 /api/maps 显示）
        self._gw_mm = None                # def 地图库（懒建，纯逻辑模块）
        self.task_running = False      # def 任务缓冲是否在执行
        self._gw_task_thread = None       # def 执行任务缓冲的后台线程
        self._gw_task_stop = threading.Event()   # def 请求中止任务缓冲
        self._gw_task_state = None        # def 最近的 TaskState
        self._gw_pose = None              # def 最近位姿 (x, y, yaw, stamp)
        self._gw_scan = None              # def 最近激光 (ranges, angle_min, inc, stamp)
        self._gw_map_png = None           # def 缓存的地图 PNG 字节
        self._gw_map_meta = None          # def 地图几何信息
        self._gw_map_version = 0          # def 地图版本号（变化则重新编码）
        # ★ map_epoch 只在"整套栈被换掉"时自增，和 map_version 不是一回事：
        #   map_version 建图时每 0.8 秒就 +1（地图在长），前端看到它只是去重载
        #   一张同样是这套栈的图；而 map_epoch 变了代表"这张图已经不是刚才那张
        #   图了"，前端必须把视图、选中点、缓存全作废。两者混用会把建图过程
        #   搞成每 0.8 秒重新适配一次视图，屏幕一直跳。
        #   ★ 从 1 起而不是 0：前端初始 epoch 是 -1，用 0 当首值会让
        #     "后端第一个值"和"前端未初始化"混淆（0 也可能被当成合法代次
        #     参与 == 比较），从 1 起语义更干净。
        self._gw_map_epoch = 1            # def 地图代次（换栈才变；从 1 起）
        self._gw_last_map_encode = 0.0    # def 上次地图编码时刻（建图限流用）
        self._gw_last_pose_err = 0.0      # def 上次位姿解析异常打印时刻（限流用）
        self._gw_last_pose_ok = 0.0       # def 上次成功拿到位姿的 monotonic 时刻
        # ★ 位姿的"数据来源时间戳"和"数据来源时钟"（见 _tf_age 的注释）。
        #   缓存位姿时记下来源 stamp，freshness 判断要用同一时基的 now 去比；
        #   _gw_pose_clock 记住"这个 stamp 是哪套时钟给的"，换时钟时自动重置。
        self._gw_pose_stamp = 0.0         # def 位姿对应 TF 的 stamp（秒，float）
        self._gw_pose_stamp_at = 0.0      # def 上次看到 stamp 变化时的 monotonic
        self._gw_pose_clock = None        # def 该 stamp 所属时钟对象（判时基是否一致）
        self._gw_pose_fresh = False       # def 本次 tick 位姿是否新鲜
        self._gw_pose_stale = False       # def 已知数据旧了（区别于"从没拿到过"）
        self._gw_last_pose_lost_log = 0.0 # def 上次"定位丢失"日志时刻（限流用）
        self._gw_plan = None              # def 最近全局路径点列表
        self._gw_logs = []                # def 环形日志（只放"事件"，见 status_board）
        # 最近记过的"归一化正文 + 时刻"，用于挡掉 stdout / rosout 两条通道
        # 送进来的同一条事件（详见 _norm_log 上面的注释）。
        self._gw_log_recent = []          # def 跨通道去重窗口（最多 32 条）
        # ★ 日志降噪器（避障/代价地图/地图扩张这些高频状态限流）。
        #   第一版这里是"日志 → 面板格子"的分流器，结果面板内容随原文变长、
        #   一格放不下（用户："面板显示的日志是固定死的，日志长的话看不到
        #   后面的信息"）。现在面板的值改为直接取自状态量（见 build_gauges），
        #   日志器只管"这条该不该留"，两者彻底切开。
        self._gw_logf = LogFilter(dup_window=15.0, dup_limit=8)
        # ★ 底盘实际速度（/cmd_vel_raw）。这是"网页发了多少 vs 底盘收到多少"
        #   的唯一可比数据：仲裁减速后网页侧仍是 0.3，只有订阅这里才看得到 0.12。
        self._gw_speed = None             # def (v, w, monotonic时刻) 或 None
        self._gw_task_idx = 0             # def 任务缓冲当前第几单
        self._gw_task_total = 0           # def 任务缓冲总共几单
        # ★ 搬货服务的"正在载货"标志（/agv/cargo/busy）。
        #   为什么要订阅它、而不只靠 TaskState 推：
        #     TaskState 在 ERROR / 任务结束时无法表达"货还在车上"。
        #     cargo_node 自己维护的 busy 才是权威事实。仪表"动作"格用它兜底，
        #     否则中止在运输途中时界面会显示"无动作"，用户开动车才发现压着货。
        self._gw_cargo_busy = False       # def 服务报的载货中
        self._gw_estop = False            # def 急停状态
        # def 最近一次"指令被 task_manager 拒绝"的原文（monotonic 时刻, 文本）。
        #   为什么要有：网关下发任务靠 /agv/command 这个话题，**拿不到返回值**，
        #   只能靠 /agv/task_state 里 task_id 有没有变来猜"受没受理"。猜错时
        #   原来只能报一句"指令被拒绝 / 路线不存在"，全是臆测。把 task_manager
        #   自己打的拒绝原因从 /rosout 抓下来，失败时直接告诉用户真实原因。
        self._gw_last_reject = None
        # def 最近一次"地图加载失败"的原文（monotonic 时刻, 文本）。
        #   为什么要有：map_server 加载失败时整个 Nav2 bringup 会中止，
        #   网页上的表现只是"地图全黑 + 没有小车"，自检只会说"未收到 /map"——
        #   用户根本看不出是地图文件/路径的问题。把 map_server 自己打的
        #   失败原文抓下来，自检那一格直接显示病因。
        self._gw_map_error = None
        self._gw_manual = False           # def 手动模式
        # ★ 名字不能叫 _clients：rclpy 的 Node 内部就用 self._clients 保存
        #   服务客户端列表，并在 create_client() 里对它调 .append()。
        #   我们一旦把它覆盖成 set，create_client 就会抛
        #   AttributeError: 'set' object has no attribute 'append'。
        #   （这是真机上踩到的坑：凡是以 _ 开头的属性都要避开 rclpy 内部名。）
        self._gw_ws_clients = set()       # def 已连接的 WebSocket 客户端（在 asyncio 线程用）
        self._gw_loop = None              # def aiohttp 事件循环（退出时从别的线程 stop 它用）

        # ── tf：位姿来自 map→base ──
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── 订阅 ──
        self.create_subscription(TaskState, '/agv/task_state', self.on_task_state, 10)
        self.create_subscription(Bool, '/agv/manual_mode', self.on_manual, 10)
        # 地图用 transient_local：晚加入也能立刻拿到已发布的静态地图
        map_qos = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, gp('map_topic').value,
                                 self.on_map, map_qos)
        self.create_subscription(LaserScan, gp('scan_topic').value, self.on_scan, 5)
        self.create_subscription(Path, gp('plan_topic').value, self.on_plan, 5)
        # ★ 底盘实际速度：仪表"速度"格用它。
        #   为什么订阅 /cmd_vel_raw 而不是 /cmd_vel：
        #     · /cmd_vel 是"上层想走的速度"（Nav2 或遥控），避障减速前的值；
        #     · /cmd_vel_raw 是仲裁之后真正交给底盘的值。
        #   用户看仪表是想知道"车现在到底走多快"，所以必须是后者。
        #   建图模式没有仲裁节点，/cmd_vel_raw 就是遥控直发的那条，也一样成立。
        # ★ 话题名做成参数：实车若换了底盘话题，不用改代码。
        self.create_subscription(Twist, gp('vel_topic').value, self.on_vel, 10)
        # ★ 搬货服务报的"正在载货"（cargo_node 每秒补发一次）。
        #   仪表"动作"格靠它判"车上有没有货"，见 build_gauges 的 act 分支。
        self.create_subscription(Bool, gp('cargo_busy_topic').value,
                                 self.on_cargo_busy, 10)
        # ★ 其它节点的日志 → 网页运行日志。
        #   /rosout 是 ROS 2 内建的"所有节点日志汇总"，rclpy 自动发布，
        #   不需要任务节点做任何改动。
        #   只转发 _RELAY_KEYS 命中的（顶升/下降/任务边界），不转发全部 ——
        #   全转发会把 Nav2 的 1 Hz 唠叨重新灌回网页，等于白做降噪。
        if RosoutLog is not None and bool(gp('relay_rosout').value):
            self.create_subscription(RosoutLog, gp('rosout_topic').value,
                                     self.on_rosout, 50)

        # ── 发布 ──
        # ★ 遥控话题按模式分流，这是建图能不能动起来的关键：
        #   导航模式：发 /agv/web_teleop → task_manager 校验手动模式 → /cmd_vel
        #             → velocity_arbiter 避障限幅 → /cmd_vel_raw → 底盘
        #   建图模式：建图栈里【没有 arbiter、没有 task_manager】，底盘只听
        #             /cmd_vel_raw，所以必须直接发到那里，否则车轮一动不动。
        # ★ 两个 publisher 都建好、按 self.mode 选着用：模式是运行时切换的
        #   （网页上点按钮），不能像以前那样在 __init__ 里一次定死。
        nav_topic = str(gp('cmd_topic').value)
        map_topic = str(gp('teleop_topic_mapping').value)
        self.pub_by_mode = {
            MODE_NAV: self.create_publisher(Twist, nav_topic, 10),
            MODE_MAPPING: self.create_publisher(Twist, map_topic, 10),
        }
        self.teleop_topic = nav_topic          # def 当前生效的遥控话题（显示用）
        self.cmd_pub = self.pub_by_mode[self.mode] if self.mode in self.pub_by_mode \
            else self.pub_by_mode[MODE_NAV]
        self.cmd_web_pub = self.create_publisher(AgvCommand, '/agv/command', 10)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # ── 服务客户端 ──
        self.query_cli = self.create_client(AgvQuery, '/agv/query')
        self.store_cli = self.create_client(AgvStore, '/agv/store')
        self.cargo_cli = self.create_client(CargoCmd, '/agv/cargo')

        # ── 计时器 ──
        self.create_timer(0.2, self.on_tick)      # 5 Hz：位姿刷新 + 看门狗
        self.create_timer(2.0, self.on_query_tick)  # 0.5 Hz：主动查位姿兜底

        # ── 栈起停器（放在最后：它要用到 get_workspace_root）──
        # 单命令模式下，网页点按钮就能拉/停整套 ROS 栈，像终端控制台一样。
        if bool(gp('manage_stack').value):
            self.stack = StackRunner(self)
        if self.mode == MODE_MAPPING:
            self.teleop_topic = teleop_topic_for(MODE_MAPPING)

        # ★ 打印"该用哪个地址访问"。
        #   踩过的真实故障：手机连同一个 WiFi，在浏览器里敲 127.0.0.1:8080
        #   → ERR_CONNECTION_REFUSED。因为 127.0.0.1 永远指**手机自己**，
        #   不是这台跑网关的机器。用户看到终端只打了 http://0.0.0.0:8080
        #   也不知道该换成什么，就自己去试 127.0.0.1。
        #   这里直接把本机局域网 IP 列出来，让人一眼知道该敲哪个。
        self.log('info', f'网页网关已启动：http://{self.host}:{self.port}')
        for url in _lan_urls(self.port):
            self.log('info', f'手机/平板访问请用：{url}')
        if not _lan_urls(self.port):
            self.log('warn', '没检测到局域网 IP（只有回环口）：'
                             '手机连不进来，检查虚拟机网络是否为「桥接模式」')
        self.log('info', f'当前模式：{MODE_TEXT.get(self.mode, self.mode)}'
                         f'（遥控话题 {self.teleop_topic}）')
        if self.stack is not None:
            self.log('info', '本进程可起停 ROS 栈：网页上切换建图/导航即可')
        if not self.auth.token:
            self.log('warn', '未设置访问口令，局域网内任何人都能打开控制台；'
                             '正式使用请设置 token 参数')
        if self.auth.allow_ips:
            self.log('info', f'IP 白名单已启用：{self.auth.allow_ips}')

    # ── 日志：事件全留，高频状态限流 ──
    def log(self, level, text):                # out 记一条日志（自动降噪）
        """记一条日志 —— 但先问 `status_board.LogFilter` 这条该不该留。

        ★ 为什么要降噪（用户原话："日志打印速度太快根本看不过来"）：
          日志列表以前什么都收，避障/代价地图/重连这类 1 Hz 的运行时状态
          十几分钟就把 300 条环形缓冲冲干净，用户翻回去只看得到"前方
          0.42m"这种没用的历史，"顶升完成"一闪而过再也找不回来。

        ★ 降噪的处置分三种（判定逻辑在 status_board.py，纯逻辑、可单测）：
            · 事件（顶升、完成、换栈、保存地图…）→ 全留
            · 已知高频状态（避障 5s、代价地图 30s…）→ 限流留存，不是全丢
            · 没预料到的高频重复 → 超过阈值折叠，并补一条提示
          注意：**不再向面板写文本**。面板的值来自状态量（build_gauges），
          第一版把日志原文塞进面板格子，才有"格子固定死、长了看不全"的问题。

        ★ 无论如何都仍然转发给 ROS logger：终端里该有的还得有，
          网页日志只是"给人看的那一份"。
        """
        text = str(text)
        keep = True
        note = ''
        try:
            # 纯内存操作、O(规则数)，可以直接在回调线程里调
            verdict = self._gw_logf.feed(level, text)
            keep = verdict.get('claim', 'log') != 'drop'
            note = verdict.get('note') or ''
        except Exception:
            keep = True      # 降噪出任何问题都不能影响正常日志
        if keep:
            # ── 跨通道去重：同一条事件只留一条 ──
            # ★ 为什么必须做（真车日志实测）：同一条顶升/下降既走子进程
            #   stdout、又走 /rosout 转发，网页日志里每条都出现两遍。
            #   用户要的是"简化日志"，一进两遍等于白降噪一半。
            norm = _norm_log(text)
            now_m = time.monotonic()
            with self._gw_lock:
                dup_item = None
                for n, ts, it in self._gw_log_recent:
                    if n == norm and (now_m - ts) <= _DUP_WINDOW:
                        dup_item = it
                        break
                if dup_item is not None:
                    # ★ 重复了也不能直接丢：两条通道能判出的级别不一样。
                    #   stdout 那条只有 guess_level（info/warn/error），
                    #   /rosout 那条才知道"顶升成功 = 绿"（ok）。
                    #   谁先到都有可能，所以按"更醒目"的级别合并 ——
                    #   否则绿字时有时无，用户以为成功失败随机。
                    if _LEVEL_RANK.get(level, 0) > _LEVEL_RANK.get(
                            dup_item.get('level', 'info'), 0):
                        dup_item['level'] = level
                else:
                    item = {'t': round(time.time(), 3), 'level': level,
                            'text': text}
                    self._gw_logs.append(item)
                    self._gw_log_recent.append((norm, now_m, item))
                    if len(self._gw_log_recent) > 32:
                        del self._gw_log_recent[:-32]
                    if note:
                        # 折叠提示跟在被折叠的那条后面，用户才知道"不是我漏看"
                        self._gw_logs.append({'t': round(time.time(), 3),
                                              'level': 'warn', 'text': note})
                    if len(self._gw_logs) > self.log_lines:
                        del self._gw_logs[:len(self._gw_logs) - self.log_lines]
        if level == 'error':
            self.get_logger().error(text)
        elif level == 'warn':
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)


    # ── 回调 ──
    def on_task_state(self, msg):              # in 任务状态
        with self._gw_lock:
            self._gw_task_state = {'task_id': msg.task_id, 'state': msg.state,
                                'target': msg.target_location,
                                'progress': float(msg.progress),
                                'message': msg.message, 't': time.time()}

    def on_manual(self, msg):                  # in 手动模式
        with self._gw_lock:
            self._gw_manual = bool(msg.data)

    def on_map(self, msg):                     # in 地图（只在变化时重新编码）
        """收到 OccupancyGrid → 编码成 PNG。

        ★ 建图模式下 slam_toolbox 每 map_update_interval(0.5s) 就发一次全量 /map，
          而一次 594x762 编码约 0.11 秒。若每条消息都编，会占掉 ~23% 的 CPU 且
          在编码期间阻塞 ROS 回调线程，导致激光/位姿回调堆积。
          所以这里按 map_min_interval 限流：到点才编，中间的地图直接丢掉
          （建图本来就是渐进的，丢几帧中间的图完全没影响）。
        """
        now = time.monotonic()
        if self.mode == 'mapping' and self.map_min_interval > 0.0:
            if (now - self._gw_last_map_encode) < self.map_min_interval:
                return
        try:
            meta = grid_meta(msg)
            png = occupancy_to_png(meta['width'], meta['height'], msg.data)
        except Exception as exc:
            self.log('error', f'地图编码失败：{exc}')
            return
        self._gw_last_map_encode = now
        with self._gw_lock:
            # 建图时尺寸可能变（slam_toolbox 会扩张地图），一并记下来
            grew = (self._gw_map_meta is not None
                    and (meta['width'] != self._gw_map_meta['width']
                         or meta['height'] != self._gw_map_meta['height']))
            self._gw_map_meta = meta
            self._gw_map_png = png
            self._gw_map_version += 1
        if grew:
            self.log('info', f"建图中：地图扩张到 {meta['width']}x{meta['height']} 格")

    def on_scan(self, msg):                    # in 激光
        with self._gw_lock:
            self._gw_scan = {'ranges': list(msg.ranges),
                          'angle_min': float(msg.angle_min),
                          'inc': float(msg.angle_increment),
                          'range_min': float(msg.range_min),
                          'range_max': float(msg.range_max),
                          't': time.time()}

    def on_plan(self, msg):                    # in 全局路径
        pts = [(float(p.pose.position.x), float(p.pose.position.y))
               for p in msg.poses]
        with self._gw_lock:
            self._gw_plan = pts

    def on_vel(self, msg):                     # in 底盘实际速度（/cmd_vel_raw）
        """缓存底盘真正收到的速度，供仪表"速度"格显示。

        ★ 为什么还要记单调时钟：这个值用来判断"底盘链路还活不活"。
          发零速之后仲裁仍会以 publish_rate 继续发 0，所以只要话题还在通，
          值会一直刷新；一旦停更超过 1 秒，说明仲裁/底盘链路断了，
          仪表该显示"——"而不是挂着一个"停止"骗人。
        """
        try:
            v = float(msg.linear.x)
            w = float(msg.angular.z)
        except (AttributeError, TypeError, ValueError):
            return
        with self._gw_lock:
            self._gw_speed = (v, w, time.monotonic())

    def on_cargo_busy(self, msg):              # in 载货中标志（/agv/cargo/busy）
        """缓存"正在载货"，供仪表"动作"格判"车上有没有货"。

        ★ 为什么仪表不自己推、要听这个标志：
          TaskState 在任务结束/出错时说不清货在哪 —— 中止在运输途中时状态机
          回 IDLE，但货还压在车上。cargo_node 的 busy 是唯一权威事实。
          这里不判新鲜度：cargo_node 每秒补发一次，值本身就是持续状态而非
          瞬时事件；它停发只意味着节点没了，保持最后的已知值比假装"没货"
          安全（安全问题宁可多报警）。
        """
        try:
            self._gw_cargo_busy = bool(msg.data)
        except (AttributeError, TypeError):
            return

    def on_rosout(self, msg):                  # in 其它节点日志 → 网页日志
        """把任务节点/搬货节点的关键日志转发到网页运行日志。

        ★ 用户原话："到达搬货点请求服务后打印请求结果出来（成功绿字，失败
          红字），并显示在运行日志"。顶升/下降是 task_manager_node 在**另一个
          进程**里调的，网关的 self.log() 根本收不到 —— 必须走 /rosout。

        ★ 为什么只转发关键事件、不全转发：
          /rosout 汇总了全系统日志（Nav2、slam、AMCL 全在里面）。全转发会把
          刚做好的降噪一次性废掉，网页日志重新变成刷屏。所以这里只挑
          "顶升/下降/任务边界"这几类，其余交给终端看。

        ★ 为什么按 node_name 排除自己：
          本节点也会往 /rosout 写（self.get_logger().info(...)），
          而 self.log() 已经记过一遍。不排除会**每条日志重复两遍**。
        """
        try:
            name = str(getattr(msg, 'name', '') or '')
            text = str(getattr(msg, 'msg', '') or '')
            lv = int(getattr(msg, 'level', 20))
        except (AttributeError, TypeError, ValueError):
            return
        if not text:
            return
        # 排除自己：名字可能是 'agv_web_gateway'、'/agv_web_gateway'
        # 或带命名空间的 '/agv/agv_web_gateway'。判据见 _relay_is_self。
        if _relay_is_self(name, self.get_name()):
            return
        # ★ 记下"指令被拒"的原文，供 _start_one 失败时给出真实原因。
        #   放在过滤之前：即使这条日志因为限流没转到网页，也要能取到。
        if '拒绝' in text and '[指令]' in text:
            self._gw_last_reject = (time.monotonic(), text)
        # ★ 记下 map_server 加载失败的原文（同上，放在过滤之前）。
        #   命中几种典型文案：nav2 的中文包装、以及 map_io 的英文原文。
        if ('地图加载失败' in text
                or 'Failed processing YAML' in text
                or 'Failed to load map' in text
                or 'bad file' in text):
            self._gw_map_error = (time.monotonic(), text)
        # ★ 先判级别：ERROR 以上无条件转发。
        #   理由：严重错误不该因为"文本没命中关键词"被过滤掉 ——
        #   过滤器的目的是压噪音，不是藏故障。
        is_bad = lv >= 40
        if not is_bad and not _relay_wanted(text):
            return
        # ROS 级别数值 → 网页日志级别（前端靠这个上色：ok=绿 error=红）
        #   DEBUG/INFO=10/20 → info；WARN=30 → warn；ERROR=40/FATAL=50 → error
        level = 'info'
        if lv >= 40:
            level = 'error'
        elif lv >= 30:
            level = 'warn'
        # ★ 顶升/下降成功要让用户看到"绿字"，但它本身是 INFO 级。
        #   判据放在文本里而不是级别里：cargo_client 成功时打"结束"、失败打"失败"。
        if level == 'info' and _relay_is_success(text):
            level = 'ok'
        self.log(level, text)

    def on_tick(self):                         # 5 Hz：刷新位姿 + 遥控掉线保护
        self.refresh_pose()
        if self.watchdog.should_stop():        # # 前端停止发速度：立刻发零速
            self.publish_twist(0.0, 0.0)
            self.watchdog.idle()
            # ★ 只停车，**不收回控制权**（本轮修的手感 bug）。
            #   teleop_stale_sec 只有 0.35 秒，而控制权租约的闲置期限是
            #   120 秒 —— 原来在这里 force_release()，等于把 120 秒的锁
            #   降级成 0.35 秒：用户点「取得控制权」后只要手一松，
            #   0.35 秒锁就没了，再按方向键只会得到"请先取得控制权"。
            #   实测日志：15:58:29 取得 → 15:58:33 已被收回。
            #   车的安全由上面那帧零速保证，锁的回收交给 ControlLease
            #   自己的 120 秒闲置超时（前端每 20 秒还会续期一次）。
            self.log('warn', '遥控已停止（0.35 秒没有新指令）→ 已发零速；'
                             '控制权保留，闲置 120 秒自动释放')

    def on_query_tick(self):                   # 0.5 Hz：服务查询兜底
        self.refresh_pose(use_service=True)

    # ── 位姿 ──
    def refresh_pose(self, use_service=False):  # out 刷新位姿缓存
        # ★ 这三个必须先初始化：下面 try 里出异常时会跳过赋值，
        #   而后面 `if pose is not None and fresh:` 无条件读 fresh。
        #   曾经 fresh 只在 try 内定义，一旦 lookup 抛异常就
        #   UnboundLocalError —— 位姿永远刷新不了、小车不显示的另一种死法。
        pose = None
        stamp = 0.0
        fresh = False
        stale_reason = ''      # def 判定过期时的原因（只在日志里用）
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame,
                                                rclpy.time.Time())
            tr = tf.transform.translation
            rot = tf.transform.rotation
            # ★ quaternion_to_yaw 的签名是 (x, y, z, w) 四个标量，不是一整个
            #   四元数对象。以前这里写成 quaternion_to_yaw(rot) 会抛
            #   TypeError: missing 3 required positional arguments，而下面
            #   的 `except Exception: pose = None` 把它静默吞掉，于是位姿
            #   永远是 None —— 前端小车、朝向箭头、激光散点全部不显示，
            #   看起来像"TF 查不到"，实际是这一行传参错了。
            pose = (float(tr.x), float(tr.y),
                    quaternion_to_yaw(rot.x, rot.y, rot.z, rot.w),
                    time.time())
            stamp = self._tf_stamp(tf)                       # def 数据来源时间戳
            # ★★ 关键：判断"这个变换本身是不是过期的"。
            #   这是本轮最深的坑。tf2 Buffer 默认缓存 10 秒历史，而且
            #   lookup_transform(..., Time()) 取的是"时间戳最新的那个变换"。
            #   旧栈的节点被杀之后，它会**继续返回最后一个已知变换**，
            #   查得到、不报错、看起来完全正常 —— 于是：
            #     · 位姿保鲜期（只在"查不到"时才计时）永远不触发；
            #     · reset_runtime_cache 刚清掉的 _gw_pose 在 0.2 秒后
            #       又被这条旧变换填满；
            #     · 界面上的小车稳稳停在切换前的位置，还跟着旧数据动。
            #   所以不能只看"查没查到"，必须看"这帧数据有多新"。
            fresh = self._tf_is_fresh(tf, self.pose_hold_sec)
            if not fresh:
                stale_reason = 'TF 变换时间戳太旧（上游已停止发布）'
        except (LookupException, ConnectivityException, ExtrapolationException):
            pose = None
            stamp = 0.0
        except Exception as exc:
            # ★ 不能静默吞掉：这类错误会让整块显示功能失效却毫无提示。
            #   限流打印，避免 5 Hz 刷屏。
            now = time.monotonic()
            if now - self._gw_last_pose_err >= 5.0:
                self._gw_last_pose_err = now
                self.log('error', f'位姿解析异常（{type(exc).__name__}）：{exc}')
            pose = None
            stamp = 0.0

        if pose is not None and fresh:
            with self._gw_lock:
                self._gw_pose = pose
                self._gw_pose_stamp = stamp
                self._gw_pose_clock = self.get_clock()
                self._gw_pose_fresh = True
                self._gw_pose_stale = False      # 拿到新鲜位姿，清掉"旧了"的标记
            self._gw_last_pose_ok = time.monotonic()
            return

        if pose is None and use_service:
            # ★ 服务兜底（0.5 Hz）只在 TF 真拿不到时用。
            #   TF 拿得到但"太旧"时不能用它补：服务查的也是同一套栈的状态，
            #   旧栈残留一样会答上一个旧值，反而绕过新鲜度检查。
            sp = self.query_pose_by_service()
            if sp is not None:
                with self._gw_lock:
                    self._gw_pose = sp
                    self._gw_pose_stamp = 0.0
                    self._gw_pose_clock = None
                    self._gw_pose_fresh = True
                self._gw_last_pose_ok = time.monotonic()
                return

        # ★ 位姿"保鲜期"：查不到（或查到的太旧）时不代表车在原点，
        #   而是"这套栈还没定位好"。以前这里什么都不做，_gw_pose 就一直
        #   保留上一个模式（或上一张图）的最后一帧 —— 用户切模式后会看到
        #   小车稳稳停在旧位置，实际车可能在别处或者根本没定位，
        #   这是最危险的一种"看着正常"。
        with self._gw_lock:
            had_pose = self._gw_pose is not None
            if pose is not None and not fresh:
                # 数据本身就旧了，不必等保鲜期，立刻作废（旧栈已经停了）
                self._gw_pose = None
                # 记住"已知数据是旧的"，让前端区分"没定位"和"定位丢了"
                self._gw_pose_stale = True
                verdict = 'stale'
            elif self._gw_last_pose_ok and \
                    (time.monotonic() - self._gw_last_pose_ok) > self.pose_hold_sec:
                self._gw_pose = None
                self._gw_pose_stale = True
                verdict = 'hold-expired'
            else:
                verdict = 'keep'
            self._gw_pose_fresh = False
        if verdict != 'keep' and had_pose:
            # ★ 必须限流：refresh_pose 是 5 Hz，不限流会在日志里刷成一片，
            #   把真正有用的信息顶掉。3 秒一次足够看出"什么时候开始丢的"。
            now_m = time.monotonic()
            if now_m - self._gw_last_pose_lost_log >= 3.0:
                self._gw_last_pose_lost_log = now_m
                self.log('warn',
                         f'定位丢失：{stale_reason or "超过保鲜期没拿到新位姿"}'
                         f'（{self.pose_hold_sec:.1f}s）→ 界面改为显示"定位未就绪"')

    def _tf_stamp(self, tf):                   # out 该变换的时间戳（秒，float）
        """把 TF 的 header.stamp 摊成秒。取不到返回 0.0。"""
        try:
            s = tf.header.stamp
            return float(s.sec) + float(s.nanosec) * 1e-9
        except Exception:
            return 0.0

    def _tf_age(self, tf):                     # out 该变换距今多少秒（算不出返回 None）
        """算 tf 的时间戳有多旧。

        ★ 为什么要单独一个方法：仿真用 sim time、真机用 wall time，
          混用会得出"差 50 年"这种荒谬的年龄。两边都用节点自己的 clock，
          保证同一时间基准。算不出来就返回 None（调用方据此不做判断）。

        ★★ 但"用节点自己的 clock"这件事本身有一个致命前提：**网关自己的
          clock 必须和上游发布 TF 的时钟同源**。实际情况是：
            · Gazebo 栈（建图 / 带仿真的导航）上游用 /clock 的 **sim time**
              （启动后才几百秒）；
            · 网关只起了一个 aiohttp 进程，从 launch 里【没有】拿到
              use_sim_time，走的是 **wall time**（1.79e9 秒）。
          两者相减 ≈ 57 年，于是每一帧都被判成"太旧"，位姿永远是 None，
          界面上两个模式都看不到小车 —— 这就是本方法第一版引入的回归。
          所以下面用"同一个 clock 对象"做时基校验，不同源就不做减法。
        """
        stamp = self._tf_stamp(tf)
        if stamp <= 0.0:
            return None
        try:
            now = self._clock_seconds(self.get_clock())
        except Exception:
            return None
        if now is None:
            return None
        age = now - stamp
        # ★ 时基不同源的兜底：真实的时间差不会超过一天（86400s）。
        #   超过就说明"我这边的 now 和上游的 stamp 用的不是同一个时钟"，
        #   此时减法结果毫无意义，退回"是否在更新"的判断（见 _tf_is_fresh），
        #   绝不能拿这个荒谬的差值去判过期 —— 那会让所有位姿都被丢掉。
        if age > 86400.0 or age < -86400.0:
            return None
        # 轻微"未来"的时间戳（<2s）是允许的：TF 插值/广播抖动常见，
        # 别把它当成异常。超过 2 秒的未来数据仍然判为不可信。
        if age < -2.0:
            return None
        return age

    def _clock_seconds(self, clock):           # out clock 当前时刻（秒，float）
        """把 ROS clock 摊成秒。拿不到返回 None。"""
        try:
            msg = clock.now().to_msg()
            return float(msg.sec) + float(msg.nanosec) * 1e-9
        except Exception:
            return None

    def _tf_is_fresh(self, tf, hold):          # out 该 TF 是否可用于显示
        """综合判断一个 TF 变换能不能当作"现在的位置"。

        两条判据按"时基是否可比"分流，缺一不可：

        【A｜时基可比，正常路径】网关 clock 与 TF 的 stamp 同源（真机 wall
            time，或网关也开了 use_sim_time）。直接比年龄，`age <= hold` 即新鲜。
            这是最准的判据，也是首选的。

        【B｜时基不可比，仿真兜底】网关走 wall time、上游 TF 走 /clock 的
            sim time，相减是几十亿秒，纯属没意义的数字。此时改用**"时间戳
            是否还在前进"**：
              · stamp 变了 → 上游还在发 → 新鲜；
              · stamp 一直不变，且已经卡住超过 hold 秒 → 上游停了 → 过期。
            这一条必须保留，否则 Gazebo 场景下（网关没拿到 use_sim_time）
            每一帧都会被判过期，两个模式都看不到小车 —— 也就是这次报的 bug。

        两条路都不做"算不出来就当真"：tf2 缓存复活旧栈位姿这个坑，
        在 B 路里同样会被"stamp 卡住超时"抓住。
        """
        age = self._tf_age(tf)
        if age is not None:                    # A：时基可比，直接比年龄
            return age <= hold
        # B：时基不可比，看时间戳是否在前进
        stamp = self._tf_stamp(tf)
        if stamp <= 0.0:
            return False
        now_m = time.monotonic()
        with self._gw_lock:
            prev = self._gw_pose_stamp
            prev_at = self._gw_pose_stamp_at
            if prev_at <= 0.0 or stamp != prev:
                # 时间戳变了（或第一次见）= 上游还在发。记下时刻供下次比对。
                self._gw_pose_stamp = stamp
                self._gw_pose_stamp_at = now_m
                return True
            # stamp 没变。可能只是这一帧没更新（TF 30 Hz、我们 5 Hz 读），
            # 也可能是上游停了。用"卡住多久"区分，超 hold 才算过期。
            return (now_m - prev_at) <= hold


    def query_pose_by_service(self):           # out 用 /agv/query 兜底取位姿
        if not self.query_cli.service_is_ready():
            return None
        req = AgvQuery.Request()
        req.what = 'pose'
        try:
            fut = self.query_cli.call_async(req)
            # 不阻塞 spin 线程太久；拿不到就下次再说
            deadline = time.time() + 0.15
            while not fut.done() and time.time() < deadline:
                time.sleep(0.01)
            if not fut.done():
                return None
            res = fut.result()
            if res is None or not res.ok:
                return None
            return (float(res.x), float(res.y), float(res.yaw), time.time())
        except Exception:
            return None

    # ── 动作：都由 asyncio 一侧调用 ──
    def publish_twist(self, v, w):             # out 发遥控速度（已限幅）
        v = max(-self.max_linear, min(self.max_linear, float(v)))
        w = max(-self.max_angular, min(self.max_angular, float(w)))
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        # ★ 按当前模式选 publisher：建图栈没有 arbiter，只有 /cmd_vel_raw 能驱动底盘
        pub = self.pub_by_mode.get(self.mode) or self.pub_by_mode[MODE_NAV]
        self.cmd_pub = pub
        pub.publish(msg)
        return v, w

    def set_mode(self, mode):                  # out 切换工作模式并换遥控通道
        """切换模式：只改遥控话题，不负责起停栈（那是 StackRunner 的事）。"""
        m = str(mode or '').strip().lower()
        if m not in MODES:
            return False
        old_topic = self.teleop_topic
        with self._gw_lock:
            changed = self.mode != m
            self.mode = m
            self.cmd_pub = self.pub_by_mode[m]
            self.teleop_topic = teleop_topic_for(m)
        if old_topic != self.teleop_topic:
            self.log('info', f'遥控通道：{old_topic} → {self.teleop_topic}')
        # ★ 模式真的变了 = 换了一套栈，旧栈的地图/位姿/激光/路径全部作废。
        #   见 reset_runtime_cache 的注释：不清理就会出现"重影"和小车停在旧位置。
        if changed:
            self.reset_runtime_cache(f'切到{MODE_TEXT.get(m, m)}')
        return True

    def reset_runtime_cache(self, reason=''):  # out 作废上一套栈的运行态缓存
        """把"上一套栈留下的运行态"全部清掉，并推进地图代次。

        **为什么必须有它（真机问题）**
        网页上的地图、激光点、路径、小车位置全部来自本节点缓存的"最后一帧"。
        切换建图/导航时，旧栈的节点会被 SIGINT 杀掉，但**缓存不会自动消失** ——
        新栈的地图要等 slam/map_server 发布 /map 才来（建图模式要几秒，
        导航模式要等 map_server 激活），位姿要等 AMCL/TF 就绪才来。
        这段空窗期里前端一直在收到 snapshot，里面仍然是旧栈的 PNG + 旧位姿，
        于是画出"两张地图叠在一起（重影）"和"小车停在切换前的最后位置"。
        等新数据到了，如果新地图的 origin/尺寸和旧的接近，前端还会因为
        版本号判断/缓存没变而继续用旧图，重影就一直留着。

        所以换栈时必须**主动**把这几样置空，并让 map_epoch +1，
        前端看到 epoch 变了就会重置视图、清掉选中点、重新拉图。

        参数：reason - 写进日志的原因（方便对时间线）
        返回：无
        """
        with self._gw_lock:
            had_map = self._gw_map_png is not None
            self._gw_map_png = None
            self._gw_map_meta = None
            self._gw_map_epoch += 1
            self._gw_pose = None
            self._gw_scan = None
            self._gw_plan = None
            self._gw_task_state = None
            # ★ 位姿来源的时间戳/时钟也要一起清：新栈的时钟可能不是同一个
            #   （比如改了 use_sim_time），留着旧 stamp 会让"时间戳是否在前进"
            #   的判断拿新值去比旧基准，误判成 "变了" → 把旧位姿当新鲜的。
            self._gw_pose_stamp = 0.0
            self._gw_pose_stamp_at = 0.0
            self._gw_pose_clock = None
            self._gw_pose_fresh = False
            self._gw_pose_stale = False
            # ★ 底盘速度也要清：新栈的 /cmd_vel_raw 还没起来，
            #   留着旧栈的速度会让仪表显示"0.30 · -0.28"，而车根本没动。
            self._gw_speed = None
            # ★ 上一套栈的地图加载错误也要清：新栈还没起来时自检如果照旧
            #   念旧错误，会把人带到上一张图上去查。
            self._gw_map_error = None
            epoch = self._gw_map_epoch
        self._gw_last_pose_ok = 0.0
        self._gw_last_map_encode = 0.0
        # ★ 日志限流状态也要清：不清的话新栈第一条避障会被旧栈的
        #   5 秒窗口吞掉，而那正是最需要确认"新栈避障生效"的一刻。
        try:
            self._gw_logf.reset()
        except Exception:
            pass
        # 跨通道去重窗口也清：新栈的"第一次"不该被旧栈的记录当成重复丢掉。
        with self._gw_lock:
            self._gw_log_recent = []
        # ★★ 必须连 tf2 的历史缓存一起清（这是本轮最隐蔽的一层）。
        #   Buffer 默认缓存 10 秒的变换。旧栈的节点被杀之后，那段历史仍然
        #   躺在 buffer 里，`lookup_transform(map, base, Time())` 会继续
        #   返回"最后一个已知变换" —— 于是我们刚 set 成 None 的 _gw_pose，
        #   在下一个 0.2 秒的 tick 里就被旧 TF 重新填满了。
        #   现象就是：缓存清理日志明明打了，界面上的小车却纹丝不动地
        #   停在切换前的位置，你说它不对、它又"有条有理"地跟着旧数据动。
        #   clear() 无参数、无返回值，清掉全部历史。
        try:
            self.tf_buffer.clear()
        except Exception as exc:
            # 个别版本没有 clear()，退化成"记一条日志别静默"
            self.log('warn', f'tf 缓存清理失败（{type(exc).__name__}）：{exc}；'
                             '换栈后位姿可能短暂沿用旧值')
        self.log('info', f'{reason}：已清理上一套栈的地图/位姿/激光/路径缓存'
                         f'（地图代次 {epoch}{"" if had_map else "，原本就没有地图"}）')
        return epoch


    def send_command(self, command, args=None, route='', request_id=''):  # out 发业务指令
        msg = AgvCommand()
        msg.source = 'web'
        msg.command = str(command)
        msg.args = [str(a) for a in (args or [])]
        msg.route = str(route or '')
        msg.confidence = 1.0
        msg.request_id = str(request_id or f'web-{int(time.time() * 1000)}')
        self.cmd_web_pub.publish(msg)
        return msg.request_id

    def call_service(self, client, request, timeout=2.0):   # out 同步调服务（带超时）
        if not client.service_is_ready():
            return None, '服务尚未就绪（业务节点可能还没起来）'
        try:
            fut = client.call_async(request)
            deadline = time.time() + timeout
            while not fut.done() and time.time() < deadline:
                time.sleep(0.01)
            if not fut.done():
                return None, '服务调用超时'
            return fut.result(), None
        except Exception as exc:
            return None, f'服务调用异常：{exc}'

    def store_request(self, resource, action, name='', x=0.0, y=0.0,
                      yaw=0.0, kind='', point_names=None):  # out 调 /agv/store
        req = AgvStore.Request()
        req.resource = resource
        req.action = action
        req.name = name
        req.x, req.y, req.yaw = float(x), float(y), float(yaw)
        req.kind = kind
        req.point_names = list(point_names or [])
        return self.call_service(self.store_cli, req)

    def cargo_request(self, command):          # out 调 /agv/cargo（顶升/下降）
        req = CargoCmd.Request()
        req.command = command
        req.request_id = f'web-{int(time.time() * 1000)}'
        return self.call_service(self.cargo_cli, req)

    def set_initial_pose(self, x, y, yaw):      # out 重新定位（等价 RViz 2D Pose Estimate）
        """发布 /initialpose，让 AMCL 在该位姿附近重新撒粒子。

        **以前"重新定位用不了"的两层原因（本轮修）**

        ① 前端语义错了：旧代码把**当前位姿**原样发回来，等于"我在原地
           告诉 AMCL 我在原地" —— 除了把协方差重置成 0.25 以外没有任何
           定位修正作用。真正的 2D Pose Estimate 是由人指定"车实际在哪、
           朝哪"，所以前端必须让人改坐标（见 uiPose）。

        ② 后端**哑发**：只要一 publish 就回 {'ok': True}。可是在
           · 建图模式（栈里根本没有 AMCL）
           · 导航栈还没激活 / AMCL 已崩
           这两种情况下消息发出去没有任何订阅者，界面却显示"已重新定位"，
           用户点几次都一样，自然得出"这个按钮没用"的结论。
           现在先看订阅者数量，并在消息里说实话。

        参数：x, y, yaw - 目标位姿（地图坐标系，米 / 弧度）
        返回：(ok, message)。ok=False 表示这条消息没人接，等于没生效。
        """
        # ★ get_subscription_count() 是"此刻有多少个订阅者"，对 latching
        #   无要求 —— /initialpose 本来就是事件型 topic，没订阅者就是白发。
        try:
            n_sub = self.initial_pub.get_subscription_count()
        except Exception:
            n_sub = -1                     # 老版本 rclpy 没这接口，不阻塞
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        # 位置/朝向给一个中等不确定度，让 AMCL 在该点附近搜索而不是全图乱撒
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25
        cov[35] = 0.0685
        msg.pose.covariance = cov
        self.initial_pub.publish(msg)
        # ★ 发布后立刻把本地缓存位姿改成目标值：AMCL 收敛要 1~3 秒，
        #   这期间界面上小车还停在旧位置，用户会以为没生效又点一次。
        #   先"跳"到目标点、再由 AMCL 的真实结果覆盖，体感才对。
        with self._gw_lock:
            self._gw_pose = (float(x), float(y), float(yaw), time.time())
            # ★ 重定位是"人指定"的位姿，不是来自 TF，所以没有来源 stamp。
            #   stamp 置 0 表示"没有可比的来源时间戳"，让 _tf_is_fresh 走
            #   "第一次见"分支，不会拿 AMCL 的旧 stamp 来否掉它。
            self._gw_pose_stamp = 0.0
            self._gw_pose_stamp_at = 0.0
            self._gw_pose_clock = None
            self._gw_pose_fresh = True
            self._gw_pose_stale = False
        self._gw_last_pose_ok = time.monotonic()
        if n_sub == 0:
            return False, ('已发布 /initialpose，但当前没有任何节点订阅它'
                           '（建图模式没有 AMCL，或导航栈还没激活）——'
                           '这条重定位不会生效')
        return True, f'已发布重定位 (x {x:.2f}, y {y:.2f}, θ {math.degrees(float(yaw)):.0f}°)'

    # ── 任务缓冲：读写 tasks.yaml（等价终端控制台的 [9] 任务系统）──
    def get_task_store(self):                  # out 取任务缓冲读写器
        """返回 {'load': fn, 'save': fn}，文件是"当前地图目录下的 tasks.yaml"。

        ★ 与终端控制台共用同一个 tasks.yaml：网页加的任务，终端能看到，反之亦然。
          绝不能各存一份，否则两边看到的任务列表会不一样。
        """
        mm = self.get_map_manager()
        if mm is None:
            return None
        if not self.current_map:
            # 没选地图时退回默认地图目录（项目约定 maps/world666map）
            try:
                maps = mm.list_maps()
                if not maps:
                    return None
                self.current_map = maps[0]
            except Exception:
                return None
        path = mm.tasks_file(self.current_map)
        return {'load': lambda: _load_tasks(path),
                'save': lambda items: _save_tasks(path, items)}

    def start_task_buffer(self, items):        # out 后台逐条执行任务缓冲
        """把任务缓冲跑起来。每条任务按 loops 次数重复，失败即停并记录。

        ★ 为什么用后台线程而不是 asyncio 任务：里面要同步等 ROS 服务/状态，
          放进 event loop 会把整个网页卡住（连状态推送都停）。

        ★★ 为什么还要 join 旧线程（本轮修的真机 bug）：
          `stop_tasks()` 为了让界面立刻能点"开始执行"，会马上把
          `task_running` 置 False —— 但那一刻旧的后台线程可能还没从
          `_wait_task_outcome` 里退出（它每 0.3s 才看一次事件）。
          如果只凭 `task_running` 判断，用户在中止后立刻再点开始，
          就会**同时跑两个任务线程**：两个线程各自下发 start_task，
          task_manager 只受理第一单、拒绝第二单，于是"后面的任务
          得不到执行"，而且日志看起来像在跑、车却不动。
          这里先等旧线程收尾（stop 事件已置位，通常 <1s 退出），
          等不到就明确拒绝，绝不让两个线程叠在一起。
        """
        old = self._gw_task_thread
        if old is not None and old.is_alive():
            old.join(timeout=3.0)
            if old.is_alive():
                self.log('error', '上一个任务线程还没退出，请稍等一秒再点开始')
                return False
        if self.task_running:
            return False
        self._gw_task_stop.clear()
        self._gw_task_thread = threading.Thread(
            target=self._run_tasks, args=(list(items),), daemon=True)
        self.task_running = True
        self._gw_task_thread.start()
        return True

    def _run_tasks(self, items):               # in 内部：任务执行循环
        total = sum(max(1, int(t.get('loops', 1))) for t in items)
        done = idx = 0
        # ★ 记下总单数与当前单号：仪表要显示"搬运中 2/5"。
        #   为什么不从 task_manager 的 TaskState 推：它只报单条任务的
        #   内部进度（取货/顶升/运输），不知道缓冲里还有几单。这两个数
        #   只有网关自己知道，所以必须在这里维护。
        with self._gw_lock:
            self._gw_task_total = total
            self._gw_task_idx = 0
        self.log('info', f'开始执行任务缓冲：{len(items)} 条 / 共 {total} 次')
        try:
            for t in items:
                loops = max(1, int(t.get('loops', 1)))
                for _ in range(loops):
                    if self._gw_task_stop.is_set():
                        self.log('warn', f'任务缓冲已中止（完成 {done}/{total}）')
                        return
                    idx += 1
                    with self._gw_lock:
                        self._gw_task_idx = idx
                    pick, drop = t.get('pick', ''), t.get('drop', '')
                    route = t.get('route', '')
                    self.log('info', f'[{idx}/{total}] {pick} ⇒ {drop}'
                                     + (f'（经路线「{route}」）' if route else ''))
                    # ★ 关键：先把上一单残留的状态清掉，并记下"上一单的
                    #   task_id"。少了这两步，第二单刚开始就会被上一单残留的
                    #   DONE 立刻判成"已完成" —— 现象就是"只有第一单真跑了，
                    #   后面的唰唰全过，日志还说全部完成"。
                    outcome = self._start_one(idx, total, pick, drop, route)
                    if outcome == 'done':
                        done += 1
                        self.log('ok', f'[{idx}/{total}] 搬运完成')
                    elif outcome == 'cancel':
                        self.log('warn', '任务被取消，停止后续')
                        return
                    else:
                        self.log('error', f'[{idx}/{total}] 任务失败或超时，停止后续')
                        return
            self.log('ok', f'任务缓冲全部完成（{done}/{total} 次搬运）')
        finally:
            self.task_running = False
            # 结束后清掉进度：否则仪表会一直挂着"搬运中 5/5"，
            # 而实际上已经空闲（task_running=False 已能兜住显示，但
            # 留着旧单号下次开始时可能闪现上一轮的 5/5）。
            with self._gw_lock:
                self._gw_task_idx = 0
                self._gw_task_total = 0

    # 下发一单并等它跑完（带重试）
    # 参数：idx/total - 序号（日志用）；pick/drop - 点位名；route - 路线名
    # 返回：'done' / 'cancel' / 'fail'
    def _start_one(self, idx, total, pick, drop, route):   # in 内部：跑一单
        """下发 start_task 并等结果，遇到"没启动起来"就重发。

        ★ 为什么需要重试（真机 bug，第 2 单往后直接不动）：
          task_manager 跑完一单的收尾是「先发 DONE → 停 0.5 秒 → 状态机回
          IDLE」。网关一看到 DONE 就往下发第二单，此刻 task_manager 的
          `_can_start()` 里 `self._runner.is_alive()` 还是 True，于是
          第二单被**当场拒绝**（"已有任务在执行"）——但它照样 publish_state，
          网关读到的还是上一单的 DONE，看起来像"这单也成功了"，实际车没动。
          用户看到的就是"放多个任务，只执行第一个"。
          这里对"没启动起来"做有限重试（拒绝是瞬时的，0.5 秒后就正常），
          并把最终失败如实报出来，不再假装成功。
        """
        for attempt in range(3):
            prev = self._current_task_id()     # 上一单的 task_id（区分新旧帧）
            self.reset_task_state()
            self.send_command('start_task', [pick, drop], route=route)
            outcome = self._wait_task_outcome(prev, timeout=300.0,
                                              start_timeout=6.0)
            if outcome != 'nostart':
                return outcome
            if self._gw_task_stop.is_set():
                return 'cancel'
            self.log('warn', f'[{idx}/{total}] 任务没能启动（第 {attempt + 1} 次），'
                             f'1 秒后重发')
            time.sleep(1.0)
        # ★ 别猜原因。网关是通过话题下发指令的，**拿不到响应**，只能靠
        #   task_id 有没有变来推断。task_manager 拒绝时会打一句
        #   "[指令] start_task → 拒绝：<真实原因>"，on_rosout 已经缓存。
        #   以前这里硬写"检查是否处于手动模式 / 急停 / 路线不存在"，
        #   其中「路线「」不存在」在没选路线时必然是个空名字 —— 用户照着
        #   去查路线只会白费时间。现在有真原因就说真原因，没有就说没有。
        hint = ''
        rec = getattr(self, '_gw_last_reject', None)   # ★ getattr：测试用 AST 抽方法，不跑 __init__
        if rec and (time.monotonic() - rec[0]) <= 30.0:
            hint = f'；task_manager 回：{rec[1]}'
        self.log('error',
                 f'[{idx}/{total}] 任务没启动：6 秒内没收到新的 task_id'
                 + hint
                 + ('' if hint else '；task_manager 未响应或未上线，'
                                    '先确认导航栈已在运行、自检全绿'))
        return 'fail'

    def _current_task_id(self):                # in 内部：最近一帧的 task_id
        """取最近一帧 TaskState 的 task_id（没有则空串）。

        ★ 用途：任务缓冲要判断"这一单是不是新的一单"。
          task_manager 每 start 一次就把序号 +1（task-0001、task-0002…），
          所以 task_id 变化 = 新任务真的被受理了。靠它比靠"看到 IDLE"
          可靠得多 —— 上一单的 DONE / IDLE 帧都会带着**旧** task_id。
        """
        with self._gw_lock:
            ts = self._gw_task_state
        return str((ts or {}).get('task_id', '') or '')

    def _wait_task_outcome(self, prev_task_id='', timeout=300.0,
                           start_timeout=12.0, settle_timeout=8.0):
        """等**当前这一单**结束，返回 'done'/'cancel'/'fail'/'timeout'/'nostart'。

        参数：prev_task_id   - 下发前那一帧的 task_id（用来区分新旧单）
              timeout       - 整单最长等待（秒）
              start_timeout - 多久没看到"本单真在跑"就判定没启动
              settle_timeout- 看到 DONE 后再等多久让 task_manager 收尾回 IDLE

        ★ 为什么必须带 prev_task_id（这是一整类 bug 的根源）：
          `/agv/task_state` 是 1 Hz 周期发布，网关缓存的是**最后一帧**。
          上一单跑完留下 DONE，下一秒才回 IDLE。任务缓冲如果只判断
          "state == DONE"，那么在第二单下发后的头几百毫秒里读到的还是
          上一单的 DONE → 立刻返回 done → 第三单、第四单同样瞬间通过。
          结果就是"只执行第一个任务，后面的全跳过"。用 task_id 就能把
          "旧帧"和"本单的真实进展"分开。

        ★ 为什么看到 DONE 还要再等一会儿（settle）：
          task_manager 的 run_transport 结尾是「报 DONE → 睡 0.5s →
          状态机回 IDLE」。如果看到 DONE 立刻返回，调用方马上发下一单，
          那时后台线程还活着（`_runner.is_alive()`），会被 `_can_start()`
          拒掉。等它真正回到 IDLE 再返回，下一单才接得住。
        """
        ACTIVE = ('GO_LOAD', 'LIFTING', 'TRANSPORT', 'LOWERING')
        t0 = time.time()
        deadline = t0 + max(float(timeout), 1.0)
        done_at = 0.0          # 首次看到 DONE 的时刻（用于 settle）
        while time.time() < deadline:
            if self._gw_task_stop.is_set():
                return 'cancel'
            with self._gw_lock:
                ts = self._gw_task_state
            if ts is None:                     # 刚 reset 过，等第一帧
                if (time.time() - t0) > start_timeout:
                    return 'nostart'
                time.sleep(0.15)
                continue
            tid = str(ts.get('task_id', '') or '')
            st = str(ts.get('state', '')).upper()
            # 「本单被受理了」的判据：task_id 与下发前不同。
            #   task_manager 每次 start 成功都把序号 +1（task-0001 → 0002），
            #   **被拒绝时不会变** —— 所以这一条能干净地把"指令被拒/没响应"
            #   和"真的开始跑了"分开。
            #   兼容 goto 单点导航（task_id 恒为空串）：那种情况只认活跃态。
            if tid:
                accepted = (tid != prev_task_id)
            else:
                accepted = st in ACTIVE
            if not accepted:
                if (time.time() - t0) > start_timeout:
                    return 'nostart'
                time.sleep(0.15)
                continue
            # 受理了但状态还没走到活跃态：正常，继续等下一步
            if st == 'DONE':
                # ★ 看到 DONE 不等于"task_manager 已经能接下一单"。
                #   run_transport 的最后是「报 DONE → 睡 0.5s → 状态机回
                #   IDLE」，只等 DONE 就下发下一单，会被 _can_start() 里
                #   的 `_runner.is_alive()` 当场拒掉（"已有任务在执行"）。
                #   所以这里再等它真正回 IDLE，最多 settle_timeout 秒。
                if not done_at:
                    done_at = time.time()
                if (time.time() - done_at) > settle_timeout:
                    return 'done'              # 等不到 IDLE 也不卡死
                time.sleep(0.2)
                with self._gw_lock:
                    cur = str(((self._gw_task_state or {}).get('state', '')) or '')
                if cur in ('', 'IDLE'):
                    return 'done'              # 已收尾，可以安全接下一单
                continue
            if st in ('ERROR', 'FAILED'):
                return 'fail'
            if st in ('CANCELLED', 'CANCELED', 'PAUSED'):
                return 'cancel'
            time.sleep(0.25)
        return 'timeout'

    def stop_tasks(self, notify=True):         # out 中止任务缓冲
        """中止任务缓冲：置停止事件 + （可选）通知 task_manager 停车。

        ★ 为什么两件事都要做（以前只发了命令，是个 bug）：
          · 只发 `cancel_task`：task_manager 收到后确实取消导航、状态机回 IDLE，
            但网关自己的 `_run_tasks` 后台线程还卡在 `_wait_task_outcome`
            的循环里 —— 它退出的条件是 `_gw_task_stop` 被置位（或收到
            CANCELLED 状态）。而 task_manager 取消后几乎没有 CANCELLED
            这个瞬态（直接回 IDLE），于是线程会一直等到 300 秒超时才退出，
            期间 `task_running` 始终是 True → 界面永远"执行中"，
            而且因为 `task_running` 为真，连新任务都启动不了。
          · 只置 Event：任务缓冲停了，但车上的导航目标还在跑。
          所以必须成对做。

        参数：notify - 是否顺带发 cancel_task。急停场景传 False，
              由调用方自己发 `stop`（stop 本身就会取消任务，不必重复发）。
        """
        self._gw_task_stop.set()
        if notify:
            self.send_command('cancel_task')
        # 立刻把对外状态置 False：让界面马上能点"开始执行"，
        # 不用等后台线程的下一次循环判断。后台线程随后会自己收尾。
        self.task_running = False

    def reset_task_state(self):                # out 清掉上一次任务残留状态
        """把"上一次任务的状态"清干净，避免界面显示上一单的 DONE/ERROR。

        ★ 为什么需要：`/agv/task_state` 是 task_manager 的**周期发布**，
          最后一帧可能停在 DONE 或 ERROR 上（DONE 之后虽然内部回 IDLE，
          但下一帧要靠 1 Hz 定时器才发出来；如果那一刻网关刚好在忙，
          网页就会一直显示上一单的结果）。开新任务前清一下，界面才干净。
        """
        with self._gw_lock:
            self._gw_task_state = None

    # ── 保存地图（调 map_saver_cli，阻塞操作，放线程里跑）──
    def save_map_now(self, name, mm):          # out 保存当前 SLAM 地图
        """调 map_saver_cli 存当前 /map，再交给 MapManager 归位到地图库。

        参数：name - 地图名；mm - MapManager 实例
        返回：(ok, message)

        ★ 为什么要 `bash -lc 'source /opt/ros/humble/setup.bash && ...'`：
          本节点可能由 launch 起在一个只 source 了工作区、没有 source ROS 环境
          的 shell 里；不重新 source 会找不到 map_saver_cli 可执行文件。
        """
        import os
        import shlex
        import subprocess
        import tempfile
        tmp = tempfile.mkdtemp(prefix='agv_websave.')
        out = os.path.join(tmp, name)
        cmd = ('source /opt/ros/humble/setup.bash && '
               f'ros2 run nav2_map_server map_saver_cli -f {shlex.quote(out)}')
        try:
            r = subprocess.run(['bash', '-lc', cmd],
                               capture_output=True, text=True, timeout=45)
        except subprocess.TimeoutExpired:
            return False, '保存超时：建图栈还在跑吗？/map 有数据吗？'
        except Exception as exc:
            return False, f'调用 map_saver_cli 失败：{exc}'
        if not os.path.isfile(out + '.pgm'):
            tail = ((r.stdout or '')[-200:] + ' ' + (r.stderr or '')[-150:]).strip()
            return False, f'没生成 pgm，检查建图栈是否在跑、是否走过场地。{tail}'
        try:
            # ★ 建图时存的点位/路线落在 maps/_mapping 草稿目录（地图名要等
            #   保存这一刻才定），save_map 会在建空模板之前先把草稿并进来。
            return mm.save_map(name, out + '.pgm', out + '.yaml', overwrite=True)
        except Exception as exc:
            return False, f'写入地图库失败：{exc}'

    # ── 工作区定位（起停栈时需要知道 install/setup.bash 在哪）──
    def get_workspace_root(self):              # out 工作区根目录（找不到返回 '')
        """定位 colcon 工作区根目录（含 src/ 与 install/ 的那一层）。

        优先级：ws_root 参数 > AGV_WS 环境变量 > 从本文件位置往上找。
        ★ 不能只看 __file__：本文件装进 install/ 后的路径是
          install/agv_web/lib/python3.x/site-packages/agv_web/…，
          用 parents[N] 硬数层数在不同 colcon 布局下会错位；
          所以改成"逐级往上找，看到同时有 src 和 install 的那层就停"。
        """
        if self.ws_root_override and os.path.isdir(self.ws_root_override):
            return os.path.abspath(self.ws_root_override)
        env = os.environ.get('AGV_WS', '').strip()
        if env and os.path.isdir(env):
            return os.path.abspath(env)
        cur = pathlib.Path(__file__).resolve()
        for parent in [cur] + list(cur.parents):
            if (parent / 'src').is_dir() and (parent / 'install').is_dir():
                return str(parent)
        # 退路：源码树里调试运行时，src/ 的上层就是工作区
        for parent in [cur] + list(cur.parents):
            if (parent / 'src').is_dir():
                return str(parent)
        return ''

    # ── 地图库（懒建，纯逻辑模块，不依赖 ROS）──
    def get_map_manager(self):                 # out 取地图库管理器
        if self._gw_mm is not None:
            return self._gw_mm
        try:
            from agv_common.map_manager import MapManager, default_maps_root
            root = self.get_workspace_root()
            ws = pathlib.Path(root) if root else pathlib.Path(__file__).resolve().parents[3]
            self._gw_mm = MapManager(default_maps_root(ws=str(ws)))
        except Exception as exc:
            self.log('warn', f'地图库不可用：{exc}')
            return None
        return self._gw_mm

    # ── 快照：给 HTTP 一次取全 ──
    def snapshot(self):                        # out 当前全量状态
        with self._gw_lock:
            pose = self._gw_pose
            ts = self._gw_task_state
            scan = self._gw_scan
            plan = list(self._gw_plan or [])
            estop = self._gw_estop
            manual = self._gw_manual
            version = self._gw_map_version
            epoch = self._gw_map_epoch
            meta = self._gw_map_meta
            speed = self._gw_speed
            pose_fresh = self._gw_pose_fresh
            pose_ok_at = self._gw_last_pose_ok
            t_idx = self._gw_task_idx
            t_total = self._gw_task_total
            ws_n = len(self._gw_ws_clients)
        # 激光点要在锁外算（数量大），位姿用快照
        scan_pts = []
        front = None
        if scan is not None:
            # ★ 正前方距离不依赖位姿：它是纯激光角度的计算，
            #   定位丢了也该能显示"前方 0.5m"（那正是最需要看的时候）。
            front = front_distance(scan)
        if scan is not None and pose is not None:
            scan_pts = scan_points_to_map(
                scan['ranges'], scan['angle_min'], scan['inc'],
                pose[0], pose[1], pose[2],
                range_min=scan['range_min'], range_max=scan['range_max'])
            # 抽稀：网页画几千个点没必要，隔几个取一个；保留最近的点更密
            if len(scan_pts) > 720:
                step = max(1, len(scan_pts) // 720)
                scan_pts = scan_pts[::step]

        # ── 仪表输入 ──
        # 速度超过 1.5 秒没更新就当没有：发零速后仲裁仍以 publish_rate
        # 持续发 0，所以"有值"本身就说明链路通；停更即说明链路断了。
        # 挂着上一次的 "0.30" 比显示"——"危险得多。
        sp_now = None
        if speed is not None and (time.monotonic() - speed[2]) <= 1.5:
            sp_now = (speed[0], speed[1])
        pose_age = None
        if pose_ok_at > 0.0:
            pose_age = round(max(0.0, time.monotonic() - pose_ok_at), 1)
        gstate = {
            'pose_ok': bool(pose is not None and pose_fresh),
            'pose_age': pose_age,
            'task_state': ts,
            'task_running': bool(self.task_running),
            'task_idx': t_idx,
            'task_total': t_total,
            'cargo_busy': bool(self._gw_cargo_busy),
            'speed': sp_now,
            'front': front,
            'avoid_on': self._gw_logf.avoid_engaged(),
            'ws_n': ws_n,
            'map_meta': meta,
            'mode': self.mode,
            'manual': manual,
            'estop': estop,
        }
        return {'pose': ({'x': pose[0], 'y': pose[1], 'yaw': pose[2]} if pose else None),
                'task': ts, 'scan': scan_pts, 'plan': plan,
                'estop': estop, 'manual': manual,
                'mode': self.mode,
                'map_version': version,
                'map_epoch': epoch,
                'map_meta': meta,
                'lease': self.lease.state(),
                # ★ 顶部仪表：**固定 7 行**，每行 {key,label,text,tone}。
                #   值来自上面这些状态量，不是日志文本 —— 所以文本天生很短
                #   （10~20 字），不会出现"格子放不下、后面看不到"。
                #   每帧发全量的好处：前端可以永久复用 7 个 DOM 节点，
                #   只改 textContent，不增删、不闪、不跳滚动位置。
                'board': build_gauges(gstate),
                't': time.time()}


# ══════════════════════════ 二、HTTP / WebSocket ══════════════════════════

def _with_where(data):
    """顶升/下降日志里带上点位名（网页可能从点位卡片直接点按钮）。

    参数缺失时返回空串，不输出 "()" 这种半截括号。
    """
    nm = str((data or {}).get('where', '') or '').strip()
    return f'（{nm}）' if nm else ''


# ══════════════ 跨节点日志转发：只挑关键事件 ══════════════
#
# /rosout 是全系统日志汇总，全转发会把降噪一次性废掉。这里的判据只做
# 一件事：**这条日志是不是用户需要立刻在网页上看到的那几类**。
#
# ① _relay_wanted：要不要转发
#    命中"顶升/下降/任务边界/急停"，或任何 WARN 以上级别（错误必须可见）。
# ② _relay_is_success：要不要标绿
#    顶升/下降"结束"= 成功 → 绿字；"失败"由级别判红，不在这里。
_RELAY_WANT = re.compile(
    r'\[(?:顶升|下降|搬运|任务完成|任务失败)\]'
    r'|搬货服务|已顶升|已下降|/agv/cargo'
    # ★ 任务被拒必须能被看见：网关只能从 task_id 猜"受没受理"，
    #   真正的拒绝原因只有 task_manager 这句话里有。只放 "拒绝"，
    #   受理的那句（[指令] … → 受理）仍然不转，避免每单刷一条。
    r'|\[指令\].*?拒绝',
    re.I,
)
# 成功判据：只在"结束/成功/完成"且不含"失败/异常/拒绝/超时"时标绿。
# 顺序很重要：先排失败词，再看成功词 —— "下降失败：搬运超时" 里两者都有。
_RELAY_FAIL = re.compile(r'失败|异常|拒绝|超时|不可用|无法', re.I)
_RELAY_OK = re.compile(r'结束|成功|完成|已顶升|已下降', re.I)


def _relay_is_self(name, self_name):
    """这条 /rosout 日志是不是本节点自己发的？

    ★ 为什么单独抽成函数、还带测试：
      网关自己也会往 /rosout 写（self.get_logger().info(...)），而 self.log()
      已经记过一遍。不排除会**每条日志重复两遍**。

    ★ 为什么取路径最后一段，而不是只 lstrip('/')：
      `'/agv_web_gateway'.lstrip('/')` 恰好等于 'agv_web_gateway'，看起来够用；
      但一旦给节点加了命名空间，rosout 里的名字变成 '/agv/agv_web_gateway'，
      lstrip 后是 'agv/agv_web_gateway' ≠ 'agv_web_gateway' → 排除失效 →
      本节点每条日志都被当成"别的节点"转发，转发又写回 /rosout，
      形成自我放大（日志越转越多，CPU 打满）。取最后一段才对。
    """
    return str(name or '').strip('/').split('/')[-1] == str(self_name or '')


# 跨通道去重窗口（秒）。
# ★ 同一条业务事件有两条通路进 self.log()：
#     ① 子进程 stdout → _pump_output → translate → log()
#        （带 `[task_manager_node] ` 这样的前缀）
#     ② /rosout 转发 → on_rosout → log()（只有正文）
#   真车日志里每件事因此都出现两遍，比如顶升成功：
#     [task_manager_node] [顶升] 成功（1号货架）：已顶升，开始搬货
#     [顶升] 成功（1号货架）：已顶升，开始搬货
#   正常流程里同一条正文不可能在 2.5 秒内产生两次（两次顶升之间隔着
#   整趟运输），所以"归一化正文 + 时间窗"去重是安全的。
_DUP_WINDOW = 2.5

# 级别"醒目程度"排序：去重合并时取更高的那个，避免绿字/红字时有时无。
#   ok 高于 info（成功要绿），warn/error 高于 ok（失败/异常优先展示）。
_LEVEL_RANK = {'info': 0, 'ok': 1, 'warn': 2, 'error': 3}

# 归一化时要剥掉的前缀：只剥**身份前缀**，不剥业务标签。
#   `[task_manager_node] ` / `[信息·task_manager_node] ` → 剥
#   `[顶升] ` `[导航中] ` `[1/5] `                          → 保留
# ★ 为什么 `[1/5]` 必须保留：批量任务里 `[1/5] 搬运完成` 与 `[2/5] 搬运完成`
#   归一化后若一样，后面的单子会被当成重复丢掉（这是以前踩过的坑）。
_NORM_PREFIX = re.compile(
    r'^\[(?:'
    r'[\u4e00-\u9fa5]{1,4}·[^\]]*'        # 信息·task_manager_node
    r'|[A-Za-z_][A-Za-z0-9_.\-]*'          # task_manager_node / planner_server
    r')\]\s*')


def _norm_log(text):
    """把日志正文归一化，用于判断"两条日志是不是同一件事"。

    ★ 只剥开头最多两层的身份前缀（`[节点] `、`[信息·节点] `），
      正文里的业务信息一个字都不动 —— 去重不能把不同事件抹平。
    """
    s = str(text or '')
    for _ in range(2):
        t = _NORM_PREFIX.sub('', s, count=1)
        if t == s:
            break
        s = t
    return s.strip()


def _relay_wanted(text):
    """这条外部节点日志要不要转发到网页运行日志？"""
    return bool(_RELAY_WANT.search(str(text or '')))


def _relay_is_success(text):
    """这条日志该不该显示成绿字（成功）。

    ★ 必须先判失败词：`[下降] 失败：搬货服务无应答（超时 5s）` 同时含
      "下降"和"超时"，只看成功词会把它标成绿色 —— 用户看到绿的以为成功了。
    """
    t = str(text or '')
    if _RELAY_FAIL.search(t):
        return False
    return bool(_RELAY_OK.search(t))


def _lan_urls(port):
    """列出"别的设备能用"的访问地址，例如 ['http://10.122.131.203:8080']。

    ★ 为什么需要这个函数（真实故障）：
      手机连同一个 WiFi，在浏览器里敲 127.0.0.1:8080 → ERR_CONNECTION_REFUSED。
      127.0.0.1 是**本机回环**，在手机上它指的是手机自己，永远连不到虚拟机。
      网关本身监听 0.0.0.0 是对的，问题只在"人不知道该敲哪个 IP"。
      所以启动时把候选地址直接打印出来。

    ★ 为什么用 UDP connect 而不是解析 ifconfig：
      不需要 root、不依赖 iproute2、跨平台可用。开一个 UDP socket 连到
      公网地址（不会真发包，只查路由表选源地址）就能拿到"本机对外 IP"。
      失败就退化为解析 hostname，再失败返回空列表（调用方会给出桥接提示）。

    参数：port - HTTP 端口
    返回：list[str]，可空
    """
    ips = []
    # ① UDP connect 探源地址（最快、最准，不实际发包）
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(('8.8.8.8', 80))     # 只用于选路，不产生流量
            ips.append(s.getsockname()[0])
        finally:
            s.close()
    except OSError:
        pass
    # ② 用 hostname 兜底（有些环境禁了对外 UDP）
    if not ips:
        try:
            for info in socket.getaddrinfo(socket.gethostname(), None,
                                           socket.AF_INET):
                ips.append(info[4][0])
        except OSError:
            pass
    # 去重、剔除回环与链路本地（169.254.x 是没要到 DHCP 才会出现的地址）
    out = []
    for ip in ips:
        if not ip or ip.startswith('127.') or ip.startswith('169.254.'):
            continue
        if ip not in out:
            out.append(ip)
    return [f'http://{ip}:{int(port)}' for ip in out]


def make_app(node):
    """构造 aiohttp 应用。路由与 node 分离，便于单测时不必真起 HTTP。"""
    if not HAVE_AIOHTTP:
        raise RuntimeError('缺少 aiohttp，请先 pip install aiohttp')

    # ── 准入中间件：口令 + IP 白名单 ──
    @web.middleware
    async def guard_middleware(request, handler):
        ip = request.remote or ''
        # token 可以放在 query 或 Cookie 里（登录页写 Cookie，之后自动带上）
        token = request.query.get('token') or request.cookies.get('agv_token') or ''
        # ★ 静态资源、登录页、【以及登录接口本身】都要放行。
        #   把 /api/login 也放行是关键：用户第一次打开页面时浏览器还没有 Cookie，
        #   若连登录接口都要先有口令才放行，就形成死锁——
        #   永远 403「口令错误」，那个口令明明是对的，人也登不进来。
        #   （这是真机上踩到的死锁 bug：登录接口被自己的认证中间件拦住了。）
        if (request.path in ('/', '/login', '/api/login')
                or request.path.startswith('/static/')):
            ok, code = node.auth.check_ip(ip)
            if not ok:
                return web.json_response({'ok': False, 'code': code,
                                          'message': f'来源 IP {ip} 不在白名单'}, status=403)
            return await handler(request)
        ok, code = node.auth.check(ip, token)
        if not ok:
            status = 403
            return web.json_response({'ok': False, 'code': code,
                                      'message': '口令错误' if code == BAD_TOKEN
                                                 else f'来源 IP {ip} 不在白名单'},
                                     status=status)
        return await handler(request)

    app = web.Application(middlewares=[guard_middleware])
    app['node'] = node

    async def index(request):
        return web.FileResponse(str(STATIC_DIR / 'index.html'))

    async def login_page(request):
        return web.FileResponse(str(STATIC_DIR / 'index.html'))

    async def api_login(request):
        data = await request.json()
        ok, code = node.auth.check(request.remote or '', data.get('token', ''))
        if not ok:
            return web.json_response({'ok': False, 'code': code,
                                      'message': '口令错误' if code == BAD_TOKEN else 'IP 不允许'},
                                     status=403)
        resp = web.json_response({'ok': True})
        resp.set_cookie('agv_token', node.auth.token or '', httponly=True,
                        samesite='Lax', max_age=86400)
        return resp

    async def api_state(request):
        return web.json_response(node.snapshot())

    async def api_map_png(request):
        with node._gw_lock:
            png = node._gw_map_png
            version = node._gw_map_version
        if png is None:
            return web.Response(status=404, text='地图尚未收到')
        return web.Response(body=png, content_type='image/png',
                            headers={'X-Map-Version': str(version),
                                     'Cache-Control': 'no-store'})

    async def api_logs(request):
        with node._gw_lock:
            logs = list(node._gw_logs)
        return web.json_response({'logs': logs})

    # ── 控制权 ──
    async def api_lease(request):
        """控制权操作：acquire / renew / release。

        ★ 为什么单独立一个 renew 动作（而不是只靠 api_teleop 顺带续期）：
          旧实现只有发速度时才续期。可用户拿到控制权后往往先"想一下往哪走"、
          或者按一下停一下，中间不发速度，锁就按闲置超时被释放了。
          前端下一次按方向键才发现自己没锁，必须再点一次「取得控制权」，
          连起来就是"每走一步点一次锁"，根本没法用。
          加了 renew 之后，前端可以在"持有控制权期间"定时轻量续期，
          锁的语义变成"我还在用这台车"，而不是"我这几毫秒在发速度"。
        """
        data = await request.json()
        action = data.get('action', 'acquire')
        who = str(data.get('who', ''))
        if action == 'acquire':
            ok, code, owner = node.lease.acquire(who)
            if not ok:
                return web.json_response({'ok': False, 'code': code,
                                          'owner': owner,
                                          'message': f'控制权在另一台设备（{owner}）手上'},
                                         status=409)
            return web.json_response({'ok': True, 'owner': owner})
        if action == 'renew':
            # 续期失败不返回 409：这是后台定时调用，报错会刷满前端日志。
            # 用 200 + ok:false 让前端自己安静地把 hasLease 置回 false。
            ok = node.lease.renew(who)
            if ok:
                return web.json_response({'ok': True, 'owner': node.lease.owner()})
            return web.json_response({'ok': False, 'code': LOCKED_BY_OTHER,
                                      'owner': node.lease.owner(),
                                      'message': '控制权已不是你的'})
        if action == 'release':
            node.lease.release(who)
            node.publish_twist(0.0, 0.0)
            node.watchdog.idle()
            return web.json_response({'ok': True})
        return web.json_response({'ok': False, 'message': '未知 action'}, status=400)

    # ── 遥控 ──
    async def api_teleop(request):
        data = await request.json()
        who = str(data.get('who', ''))
        if node.lease.owner() != who:
            return web.json_response({'ok': False, 'code': LOCKED_BY_OTHER,
                                      'owner': node.lease.owner(),
                                      'message': '没有控制权，无法遥控'}, status=409)
        node.lease.renew(who)                  # # 手还按着：续期
        v = float(data.get('v', 0.0))
        w = float(data.get('w', 0.0))
        # ★ 松手与掉线要分开：前端松手时会主动补发一帧 (0,0) 让车停，
        #   那是**正常操作**，不是"前端失联"。把它也 feed() 进看门狗，
        #   0.35 秒后就会误报一次"遥控心跳超时"（日志噪音，且以前还会
        #   顺手把控制权收走）。零速直接 idle 掉看门狗，车已经停了，
        #   没有什么需要超时兜底的。
        if abs(v) < 1e-6 and abs(w) < 1e-6:
            node.watchdog.idle()               # # 主动松手：收手，不判掉线
        else:
            node.watchdog.feed()               # # 真在开：刷新掉线计时
        v, w = node.publish_twist(v, w)
        return web.json_response({'ok': True, 'v': v, 'w': w})

    async def api_stop(request):
        node.publish_twist(0.0, 0.0)
        node.watchdog.idle()
        node.lease.force_release()
        node._gw_estop = True
        node.send_command('stop')
        node.log('warn', '网页触发急停')
        return web.json_response({'ok': True})

    # ── 业务指令 ──
    async def api_command(request):
        data = await request.json()
        cmd = str(data.get('command', '')).strip()
        args = data.get('args') or []
        route = data.get('route', '')
        if not cmd:
            return web.json_response({'ok': False, 'message': '缺少 command'}, status=400)
        # ★ 取消类命令必须同时把网关自己的任务缓冲停掉。
        #   以前这里只发一条裸命令，于是出现"点了取消，网页还是执行中"：
        #   task_manager 那边停止了，网关的 _run_tasks 线程却还在等状态。
        #   这不是 task_manager 的锅，是网关漏了配对动作。
        if cmd in ('cancel_task', 'stop'):
            # 'stop' 是急停：只置网关事件，取消动作由 stop 命令自己完成
            node.stop_tasks(notify=(cmd == 'cancel_task'))
            if cmd == 'stop':
                node.send_command('stop')
            node.log('warn', f'网页下达取消：{cmd}，任务缓冲已停止')
            return web.json_response({'ok': True, 'running': False,
                                      'message': '已取消并停车'})
        rid = node.send_command(cmd, args, route)
        node.log('info', f'网页下发指令：{cmd} {args}（{rid}）')
        return web.json_response({'ok': True, 'request_id': rid})

    # ── 地图 / 点位 / 路线 ──
    async def api_store(request):
        data = await request.json()
        res, err = node.store_request(
            data.get('resource', 'waypoint'), data.get('action', 'list'),
            name=data.get('name', ''), x=data.get('x', 0.0), y=data.get('y', 0.0),
            yaw=data.get('yaw', 0.0), kind=data.get('kind', ''),
            point_names=data.get('point_names') or [])
        if res is None:
            return web.json_response({'ok': False, 'message': err}, status=503)
        items = [{'name': i.name, 'x': i.x, 'y': i.y, 'yaw': i.yaw, 'kind': i.kind}
                 for i in res.items]
        return web.json_response({'ok': bool(res.ok), 'message': res.message,
                                  'items': items, 'names': list(res.names),
                                  'counts': list(res.counts), 'count': int(res.count)})

    async def api_cargo(request):
        data = await request.json()
        cmd = str(data.get('command', 'lift'))
        # ★ 顶升/下降是**事件**（用户明确要求它们出现在运行日志里）：
        #   手动点按钮触发，天然稀疏，且每次都有业务意义 ——
        #   "顶升开始 / 顶升完成 / 顶升失败"必须留得住、翻得回。
        #   注意后端只记"下发"和"结果"两条：中间不会刷屏。
        zh = '顶升' if cmd == 'lift' else '下降'
        node.log('info', f'网页触发{zh}{_with_where(data)}')
        res, err = node.cargo_request(cmd)
        if res is None:
            node.log('error', f'{zh}失败：{err}')
            return web.json_response({'ok': False, 'message': err}, status=503)
        ok = bool(res.ok)
        if ok:
            node.log('ok', f'{zh}完成' + (f'：{res.message}' if res.message else ''))
        else:
            node.log('error', f'{zh}失败：{res.message or "搬货服务拒绝"}')
        return web.json_response({'ok': ok, 'busy': bool(res.busy),
                                  'message': res.message})

    async def api_relocalize(request):
        data = await request.json()
        x, y = float(data.get('x', 0.0)), float(data.get('y', 0.0))
        yaw = float(data.get('yaw', 0.0))
        ok, msg = node.set_initial_pose(x, y, yaw)
        node.log('warn' if ok else 'error', f'网页下发重新定位：{msg}')
        return web.json_response({'ok': bool(ok), 'message': msg})

    async def api_maps(request):
        # 地图库是纯逻辑模块，直接复用（不依赖 ROS）
        mm = node.get_map_manager()
        if mm is None:
            return web.json_response({'ok': False, 'message': '地图库不可用'}, status=503)
        try:
            maps = mm.list_maps()
        except Exception as exc:
            return web.json_response({'ok': False, 'message': f'读地图库失败：{exc}'},
                                     status=500)
        # info：每张图的保存时间/点位数，给前端"选地图"列表当副标题。
        # ★ maps 本身**必须保持纯字符串数组**：前端 renderMaps 和旧调用方
        #   都按字符串遍历，改成对象会连带把地图库列表页打坏。
        info = {}
        for nm in maps:
            try:
                info[nm] = mm.map_info(nm)
            except Exception:
                info[nm] = {'name': nm, 'points': 0, 'saved_at': ''}
        return web.json_response({'ok': True, 'maps': maps, 'current': node.current_map,
                                  'info': info})

    async def api_map_delete(request):
        mm = node.get_map_manager()
        if mm is None:
            return web.json_response({'ok': False, 'message': '地图库不可用'}, status=503)
        data = await request.json()
        name = str(data.get('name', '')).strip()
        if not name:
            return web.json_response({'ok': False, 'message': '缺少地图名'}, status=400)
        if name == node.current_map:
            return web.json_response({'ok': False,
                                      'message': '不能删除正在使用的地图'}, status=409)
        try:
            ok, msg = mm.delete_map(name)
        except Exception as exc:
            return web.json_response({'ok': False, 'message': f'删除失败：{exc}'},
                                     status=500)
        node.log('warn' if ok else 'error', f'网页删除地图 {name}：{msg}')
        return web.json_response({'ok': bool(ok), 'message': msg})

    # ── 任务缓冲（等价终端控制台的 [9] 任务系统）──
    async def api_tasks(request):
        mm = node.get_task_store()
        if mm is None:
            return web.json_response({'ok': False, 'message': '任务缓冲不可用'}, status=503)
        if request.method == 'GET':
            return web.json_response({'ok': True, 'tasks': mm['load'](),
                                      'running': node.task_running})
        data = await request.json()
        action = str(data.get('action', 'list'))
        try:
            # ★ POST 也必须支持 list。
            #   前端 api() 助手固定发 POST，refreshTasks 每 5 秒调一次
            #   `/api/tasks`，以前这里没有 list 分支，直接落到最后
            #   "未知 action" 400。后果是一条完整的恶性链：
            #     刷新失败 → catch 里只打一行 warn，S.taskRunning 不更新
            #     → 界面永远保持上一次的值（true）→ 按钮永远显示"执行中…"
            #     → 再点按钮走的是"中止"分支，永远开不了新任务。
            #   这就是"读取任务失败"刷屏 + "一直显示执行中"的同一个根。
            if action in ('list', ''):
                return web.json_response({'ok': True, 'tasks': mm['load'](),
                                          'running': node.task_running})
            if action == 'add':
                items = mm['load']()
                items.append({'pick': str(data.get('pick', '')),
                              'drop': str(data.get('drop', '')),
                              'loops': max(1, int(data.get('loops', 1))),
                              'route': str(data.get('route', ''))})
                mm['save'](items)
                node.log('info', f"网页新增搬运任务：{data.get('pick')} ⇒ {data.get('drop')}")
                return web.json_response({'ok': True, 'tasks': items})
            if action == 'delete':
                idx = int(data.get('index', -1))
                items = mm['load']()
                if 0 <= idx < len(items):
                    gone = items.pop(idx)
                    mm['save'](items)
                    return web.json_response({'ok': True, 'tasks': items,
                                              'message': f"已删除 {gone.get('pick')}"})
                return web.json_response({'ok': False, 'message': '序号超出范围'}, status=400)
            if action == 'replace':
                # ★ 整表覆盖：给某一单改路线用。
                #   以前想改路线只能"删了重新加"——顺序会被打到最后，
                #   用户精心排的执行序列就散了。整表回写既保顺序又只有一次 IO。
                raw = data.get('tasks')
                if not isinstance(raw, list):
                    return web.json_response({'ok': False, 'message': 'tasks 必须是数组'},
                                             status=400)
                if node.task_running:
                    return web.json_response({'ok': False,
                                              'message': '任务正在执行，先中止再改'}, status=409)
                items = [{'pick': str(t.get('pick', '')),
                          'drop': str(t.get('drop', '')),
                          'loops': max(1, int(t.get('loops', 1) or 1)),
                          'route': str(t.get('route', '') or '')}
                         for t in raw if isinstance(t, dict)]
                mm['save'](items)
                return web.json_response({'ok': True, 'tasks': items,
                                          'message': '任务序列已更新'})
            if action == 'clear':
                mm['save']([])
                return web.json_response({'ok': True, 'tasks': []})
            if action == 'start':
                if node.task_running:
                    return web.json_response({'ok': False,
                                              'message': '已有任务在执行'}, status=409)
                items = mm['load']()
                if not items:
                    return web.json_response({'ok': False, 'message': '任务缓冲为空'},
                                             status=400)
                node.start_task_buffer(items)      # use 后台线程逐条执行
                return web.json_response({'ok': True, 'message': '已开始执行任务'})
            if action == 'stop':
                # ★ 中止任务必须走这里，不能只发一条裸的 cancel_task 命令。
                #   只发命令的话：task_manager 那边确实取消了，但网关的
                #   _run_tasks 后台线程还在 _wait_task_outcome 里等，它靠的是
                #   _gw_task_stop 这个 Event 才会退出循环 →
                #   task_running 永远不置 False → 界面永远"执行中"。
                #   stop_tasks() 同时做两件事：置 Event + 发 cancel_task。
                was = node.task_running
                node.stop_tasks()
                return web.json_response(
                    {'ok': True, 'running': False,
                     'message': '已中止任务' if was else '当前没有正在执行的任务'})
            return web.json_response({'ok': False, 'message': '未知 action'}, status=400)
        except Exception as exc:
            return web.json_response({'ok': False, 'message': f'操作失败：{exc}'},
                                     status=500)

    # ── 路线管理（等价 [7]）──
    async def api_routes(request):
        # ★ 兼容 GET：GET 没有 body，直接 await request.json() 会抛
        #   "Expecting value"，然后变 500。GET 时从查询串取 action/name，
        #   默认 list —— 这样浏览器直接打开 /api/routes?action=list 就能看。
        try:
            data = await request.json()
        except Exception:
            data = {}
        if not isinstance(data, dict):
            data = {}
        if not data:
            data = {'action': request.query.get('action', 'list'),
                    'name': request.query.get('name', '')}
        action = str(data.get('action', 'list'))
        name = str(data.get('name', ''))
        if action == 'list':
            res, err = node.store_request('route', 'list')
        elif action == 'get':
            res, err = node.store_request('route', 'get', name=name)
        elif action == 'delete':
            res, err = node.store_request('route', 'delete', name=name)
        elif action == 'save':
            res, err = node.store_request('route', 'save', name=name,
                                          point_names=data.get('point_names') or [])
        else:
            return web.json_response({'ok': False, 'message': '未知 action'}, status=400)
        if res is None:
            return web.json_response({'ok': False, 'message': err}, status=503)
        items = [{'name': i.name, 'x': i.x, 'y': i.y, 'yaw': i.yaw, 'kind': i.kind}
                 for i in res.items]
        return web.json_response({'ok': bool(res.ok), 'message': res.message,
                                  'items': items, 'names': list(res.names),
                                  'counts': list(res.counts), 'count': int(res.count)})

    # ── 自检（等价 [c] 一键自检）──
    async def api_check(request):
        """逐项检查运行环境，返回结构化结果（全绿才好导航）。

        与终端控制台的 `check` 子命令对齐：节点是否在、服务是否就绪、
        地图/点位/路线文件是否齐全。
        """
        report = []
        # ① 关键节点
        # ★ rclpy 的 Node 没有 get_node_names；要用 get_node_names_and_namespaces
        try:
            pairs = node.get_node_names_and_namespaces()
            nodes = [n for n, _ns in pairs]
        except Exception:
            nodes = []
        want_nodes = ['task_manager_node', 'velocity_arbiter', 'amcl',
                      'controller_server', 'planner_server']
        # ★ 这里必须用 node.mode，不能写 self.mode：self 是 WebGateway 的方法里
        #   才有的，而 api_check 是 make_app() 内部的闭包函数，作用域里只有 node。
        #   写成 self.mode 会 NameError，整个自检接口 500，前端"一键自检"直接失败。
        if node.mode == 'mapping':
            # 建图栈里没有 arbiter / task_manager / amcl / nav2，硬查会全红；
            # 改成查建图真正依赖的东西，否则"自检全红"会误导用户。
            # 服务同理：建图栈只有 waypoint_node（点位/路线），
            # /agv/query 与 /agv/cargo 本来就不该存在 —— 以前也硬查，
            # 于是建图模式自检必然挂三个红项，看着像"节点都没起"。
            want_nodes = ['slam_toolbox', 'robot_state_publisher', 'waypoint_node']
            want_services = [('/agv/store', node.store_cli)]
        else:
            want_services = [('/agv/query', node.query_cli),
                             ('/agv/store', node.store_cli),
                             ('/agv/cargo', node.cargo_cli)]
        for n in want_nodes:
            hit = any(n in x for x in nodes)
            report.append({'item': f'节点 {n}', 'ok': hit,
                           'message': '在线' if hit else '未发现'})
        # ② 服务就绪
        for label, cli in want_services:
            ok = cli.service_is_ready()
            report.append({'item': f'服务 {label}', 'ok': ok,
                           'message': '就绪' if ok else '未就绪'})
        # ③ 数据
        with node._gw_lock:
            has_map = node._gw_map_png is not None
            has_pose = node._gw_pose is not None
            has_scan = node._gw_scan is not None
        # ★ 地图没收到时，如果有 map_server 的失败原文就如实转述。
        #   以前这里只会写"未收到 /map"，用户看到的是"全黑 + 未收到"，
        #   完全猜不到是地图文件/路径的问题（实际踩坑：中文地图名被转义）。
        map_msg = '已收到 /map'
        if not has_map:
            map_msg = '未收到 /map'
            rec = getattr(node, '_gw_map_error', None)   # getattr：测试用 AST 抽方法
            if rec and (time.monotonic() - rec[0]) <= 180.0:
                map_msg = f'地图加载失败：{rec[1][:80]}'
        report.append({'item': '地图已加载', 'ok': has_map, 'message': map_msg})
        report.append({'item': '定位可用', 'ok': has_pose,
                       'message': 'map→base 可查' if has_pose else '查不到 TF'})
        report.append({'item': '激光可用', 'ok': has_scan,
                       'message': '已收到 /scan' if has_scan else '未收到 /scan'})
        # ④ 地图库
        mm = node.get_map_manager()
        if mm is not None:
            try:
                n_maps = len(mm.list_maps())
                report.append({'item': '地图库', 'ok': n_maps > 0,
                               'message': f'{n_maps} 张地图'})
            except Exception as exc:
                report.append({'item': '地图库', 'ok': False, 'message': str(exc)})
        all_ok = all(r['ok'] for r in report)
        node.log('info' if all_ok else 'warn',
                 f"网页自检：{'全部通过' if all_ok else '有项目未通过'}")
        return web.json_response({'ok': all_ok, 'items': report})

    # ── 恢复（等价 [r] 恢复）──
    async def api_resume(request):
        node._gw_estop = False
        rid = node.send_command('resume')
        node.log('warn', '网页请求恢复（解除急停）')
        return web.json_response({'ok': True, 'request_id': rid})

    # ── 栈状态 / 起停 / 切模式（单命令体验的核心）──
    async def api_stack(request):
        """查栈状态。"""
        st = node.stack.state() if node.stack is not None else {
            'mode': node.mode, 'mode_text': MODE_TEXT.get(node.mode, node.mode),
            'running': None, 'pid': None, 'since': 0.0,
            'error': '' if node.stack is not None else '本进程不管理栈（manage_stack=false）'}
        st['manage'] = node.stack is not None
        st['teleop_topic'] = node.teleop_topic
        return web.json_response({'ok': True, 'stack': st})

    async def api_stack_start(request):
        """启动或切换栈。

        请求体：{mode: 'nav'|'mapping', map: 地图名(可选), name: 地图名}
        ★ 整个过程要 15~40 秒（Nav2 激活），必须放线程里，
          否则会把 aiohttp 事件循环堵死，连进度日志都推不出去。

        ★★ manage_stack=false 的降级路径（本轮补的真实缺口）
        以前这里第一句就是 `if node.stack is None: 503`。可 manage_stack=false
        正是"用户自己在别的终端起栈、网页只当遥控器"的场景 —— 那种情况下
        「启动导航 / 开始建图」两个按钮全都会 503，而**切换遥控通道**这件事
        恰恰必须由网关做（建图栈没有 arbiter，遥控要直发 /cmd_vel_raw，
        不然车一动都不动）。少了这条路径，手工起栈的用户在网页上永远
        切不过去，一按就被拒。

        所以：没有 StackRunner 时只做"网关自己能做的那半件事"——
        切遥控通道 + 作废旧栈缓存，并如实告诉用户"栈得你自己起"。
        """
        data = await request.json()
        mode = str(data.get('mode', '')).strip().lower()
        if mode not in MODES:
            return web.json_response(
                {'ok': False, 'message': f'未知模式 {mode!r}（只支持 {MODES}）'},
                status=400)
        if node.stack is None:
            changed = node.mode != mode
            # ★ 只清一次：set_mode 内部在"模式真的变了"时已经调了
            #   reset_runtime_cache（那里还顺手清 tf 缓存）。
            #   这里再清一遍会让 epoch 连跳两格，前端收到两个 map_epoch
            #   事件、连做两次复位（视图被二次 autoCenter，屏幕会跳一下）。
            #   幂等指的是"重复清理不出错"，不代表可以重复推进代次。
            node.set_mode(mode)
            if not changed:
                node.reset_runtime_cache(f'切到{MODE_TEXT.get(mode, mode)}')
            msg = (f'本进程不管理栈（manage_stack=false），已把网关切到'
                   f'{MODE_TEXT.get(mode, mode)}、遥控发往 '
                   f'{node.teleop_topic}。ROS 栈请你在终端自行启动/停止。')
            node.log('warn', msg)
            return web.json_response({'ok': True, 'message': msg, 'mode': mode,
                                      'manage': False,
                                      'teleop_topic': node.teleop_topic})
        # 导航模式需要地图：可以传地图名，也可以不传（让用户后面自己选）
        kwargs = {}
        # ★ 建图：点位/路线先落到固定草稿目录。
        #   建图时用户会顺手存点，但地图名要等"保存"那一刻才定，
        #   没法直接写进某张图。保存时 MapManager.save_map 会调
        #   promote_draft 把草稿并进新地图（只补空缺，不覆盖）。
        if mode == MODE_MAPPING:
            from agv_common.map_manager import MAPPING_DRAFT   # use 草稿目录名
            mm = node.get_map_manager()
            if mm is not None:
                kwargs['waypoint_file'] = mm.waypoints_file(MAPPING_DRAFT)
                kwargs['routes_file'] = mm.routes_file(MAPPING_DRAFT)
        map_name = str(data.get('map') or data.get('name') or '').strip()
        if map_name:
            mm = node.get_map_manager()
            if mm is None:
                return web.json_response({'ok': False, 'message': '地图库不可用'},
                                         status=503)
            ok, reason = validate_map_name(mode, map_name)
            if not ok:
                return web.json_response({'ok': False, 'message': reason}, status=400)
            if mode == MODE_NAV:
                if not mm.map_exists(map_name):
                    return web.json_response(
                        {'ok': False, 'message': f'地图 {map_name} 不存在'}, status=404)
                # ★ 这里曾经是 `paths[0] if isinstance(paths, (list, tuple))
                #   else str(paths)`，而 map_paths() 返回的是 **dict**，
                #   于是 map_yaml 变成了一整个 dict 的 str()：
                #     "{'dir': '/...', 'pgm': '/...', 'yaml': '/...', ...}"
                #   map_server 拿到这个"路径"当然找不到文件，导航栈起来了
                #   但地图加载失败 —— 表现就是"有图却进不了导航"。
                #   字典就用键取，别猜类型。
                #
                # ★ 用 launch_map_yaml() 而不是 map_yaml_file()：
                #   地图名是中文时（控制台按时间自动生成「地图10月19日16时」），
                #   这个值走完 agv_stack → navigation → nav2_bringup 之后，
                #   map_server 拿到的是 "u5730u56FE…" 这种被转义过的串（实测），
                #   直接 Failed processing YAML … bad file → 导航栈起不来、
                #   网页地图全黑。launch_map_yaml() 对非 ASCII 名返回一份
                #   纯 ASCII 别名；ASCII 名（world666map）原样返回，行为不变。
                kwargs['map_yaml'] = mm.launch_map_yaml(map_name)
                kwargs['waypoint_file'] = mm.waypoints_file(map_name)
                kwargs['routes_file'] = mm.routes_file(map_name)
                node.current_map = map_name
        elif mode == MODE_NAV:
            # 没指定地图：如果之前选过就用之前的，否则明确报错（导航没图起不来）
            mm = node.get_map_manager()
            if node.current_map and mm is not None and mm.map_exists(node.current_map):
                kwargs['map_yaml'] = mm.launch_map_yaml(node.current_map)
                kwargs['waypoint_file'] = mm.waypoints_file(node.current_map)
                kwargs['routes_file'] = mm.routes_file(node.current_map)
            else:
                return web.json_response(
                    {'ok': False,
                     'message': '导航模式需要先选一张地图（在建图模式里建，或从地图库选）'},
                    status=400)

        loop = asyncio.get_event_loop()
        try:
            ok, msg = await loop.run_in_executor(
                None, lambda: _do_switch(node, mode, kwargs))
        except Exception as exc:
            node.log('error', f'切换栈失败：{exc}')
            return web.json_response({'ok': False, 'message': f'切换失败：{exc}'},
                                     status=500)
        node.log('info' if ok else 'error', f'切到{MODE_TEXT[mode]}：{msg}')
        return web.json_response({'ok': bool(ok), 'message': msg,
                                  'mode': mode, 'stack': node.stack.state()})

    def _do_switch(node, mode, kwargs):
        """线程里执行：先停旧的，再起新的，最后把网关遥控通道切过去。"""
        ok, msg = node.stack.switch(mode, **kwargs)
        if ok:
            # ★ 必须先确认开关成功再切遥控通道：建图栈没起来就切到
            #   /cmd_vel_raw 会让遥控在导航栈下绕过仲裁，很危险。
            # ★ 换栈必须清缓存（重影 / 小车停在旧位置），但只能清一次：
            #   set_mode 在"模式变了"时内部已经清过，这里只补"模式没变、
            #   但栈被原地重启"那种情况。重复推进 epoch 会让前端连做
            #   两次复位，视图被二次 autoCenter，屏幕会跳一下。
            prev_mode = node.mode
            node.set_mode(mode)
            if prev_mode == mode:
                node.reset_runtime_cache(f'重启{MODE_TEXT[mode]}栈')
        return ok, msg

    async def api_stack_stop(request):
        if node.stack is None:
            return web.json_response(
                {'ok': False, 'message': '本进程不管理栈'}, status=503)
        loop = asyncio.get_event_loop()
        node.publish_twist(0.0, 0.0)
        ok, msg = await loop.run_in_executor(None, node.stack.stop)
        node.log('info' if ok else 'error', f'停止栈：{msg}')
        return web.json_response({'ok': bool(ok), 'message': msg})

    async def api_shutdown(request):
        """网页上点「退出」：把整套栈停掉，然后把网关自己也退掉。

        **这个接口为什么存在**
        用户的实际抱怨是："关了网页，车还在跑/进程还在后台，下次启动一堆残留"。
        关浏览器窗口时浏览器不会等我们做完收尾，所以必须由页面主动发这个请求。

        **区分"刷新"和"退出"（关键）**
        不能一断 WS 就关机 —— 用户按 F5 刷新页面、手机切到别的 App 再回来，
        都会短暂断开连接。如果那样就停机，用起来会非常烦。
        所以：
          · 页面刷新 / 切后台 → 只断 WS，不调用本接口 → 栈继续跑；
          · 用户点「退出」按钮 / 关标签页（beforeunload 时用 sendBeacon 发）
            → 调用本接口 → 停栈 + 退出网关。
        另外 handle_signals=False 是因为我们自己在请求里调 web.run_app 的退出，
        让 aiohttp 不用再装自己的信号处理器（它会和 rclpy 抢）。

        请求体：{who: 设备标识, stop_stack: true/false}
        ★ 只允许"控制权持有者"或"没有别人持有"时关机，避免一台设备把
          另一台正在操作的车关掉。
        """
        data = {}
        try:
            data = await request.json()
        except Exception:
            pass
        who = str(data.get('who', ''))
        owner = node.lease.owner()
        # 别人正握着方向盘时，不许关机
        if owner and who and owner != who:
            return web.json_response(
                {'ok': False, 'message': f'控制权在「{owner}」手上，不能从这里关机'},
                status=409)

        stop_stack = data.get('stop_stack', True)
        node.log('warn', '收到网页退出请求：正在停止 ROS 栈并关闭网关…')
        node.publish_twist(0.0, 0.0)
        node.watchdog.idle()

        # 起个后台任务去做真正的收尾，先把 HTTP 响应发回去。
        # ★ 不能在请求处理里直接 await stop()（要 12 秒），浏览器会以为超时。
        async def _do_shutdown():
            await asyncio.sleep(0.4)        # 让响应先发出去
            if stop_stack and node.stack is not None and node.stack.is_running():
                loop = asyncio.get_running_loop()
                try:
                    await loop.run_in_executor(None, lambda: node.stack.stop(10.0))
                except Exception as exc:
                    node.log('error', f'退出时停栈失败：{exc}')
            node.publish_twist(0.0, 0.0)
            node.log('warn', '网关即将退出')

            # ★ 退出方式踩了两次坑，记下来别重犯：
            #   ① `raise SystemExit(0)` 在 Task 里只会让这个任务以异常结束，
            #      run_app 的 run_forever 照旧转圈 —— 完全没用。
            #   ② `loop.stop()` 也不行：aiohttp 的 run_app 内部有 try/finally
            #      要 await runner.cleanup()，一旦循环被硬停，那些 await 永远
            #      等不到，进程就卡在退出路径上（实测 30 秒都退不掉，日志里
            #      "网关即将退出"打了但进程还在）。
            #   最终用"向自己发 SIGINT"：Python 的默认处理器把它变成
            #   KeyboardInterrupt，能从协程里一路穿出去，run_app 的 finally
            #   正常执行 runner.cleanup()，再落到 main() 的 finally 停栈 +
            #   rclpy.shutdown()。这条链是 aiohttp 官方支持的退出路径。
            #   用 SIGTERM 不行 —— 它的默认动作是直接终止，不跑 finally。
            await asyncio.sleep(0.1)         # 让上面的日志先刷出去
            os.kill(os.getpid(), signal.SIGINT)

        asyncio.ensure_future(_do_shutdown())
        return web.json_response({'ok': True, 'message': '正在停止并退出…'})

    # ── 保存建图结果（等价控制台建图模式的 [m]）──
    async def api_map_save(request):
        """把当前 SLAM 地图存进地图库。

        流程与控制台一致：调 `map_saver_cli` 生成 .pgm/.yaml → MapManager 归位。
        ★ 必须放到线程里执行：map_saver_cli 要 1~3 秒，直接在事件循环里
          subprocess.run 会把整个网页卡住（连状态推送都停）。
        """
        data = await request.json()
        name = str(data.get('name', '')).strip()
        if not name:
            return web.json_response({'ok': False, 'message': '请给地图起个名字'},
                                     status=400)
        try:
            from agv_common.map_manager import valid_name
        except Exception:
            valid_name = None
        if valid_name is not None:
            good, reason = valid_name(name)
            if not good:
                return web.json_response({'ok': False, 'message': reason}, status=400)
        mm = node.get_map_manager()
        if mm is None:
            return web.json_response({'ok': False, 'message': '地图库不可用'}, status=503)
        loop = asyncio.get_event_loop()
        try:
            ok, msg = await loop.run_in_executor(None, node.save_map_now, name, mm)
        except Exception as exc:
            node.log('error', f'保存地图失败：{exc}')
            return web.json_response({'ok': False, 'message': f'保存失败：{exc}'},
                                     status=500)
        node.log('info' if ok else 'error', f'网页保存地图 {name}：{msg}')
        return web.json_response({'ok': bool(ok), 'message': msg, 'name': name})

    # ── 换地图（等价 [0]/[1]）──
    async def api_map_switch(request):
        """切换地图：写回选择并提示需要重启导航栈（ROS 2 里换图必须重启 map_server）。

        ★ 这里不尝试热切换：Nav2 的 map_server 生命周期虽可重载地图，
          但本项目的点位/路线文件是按地图名注入 launch 参数的，热切会不一致。
          如实告诉用户"要重启栈"比悄悄半生效安全。
        """
        data = await request.json()
        name = str(data.get('name', '')).strip()
        mm = node.get_map_manager()
        if mm is None:
            return web.json_response({'ok': False, 'message': '地图库不可用'}, status=503)
        if not name:
            return web.json_response({'ok': False, 'message': '缺少地图名'}, status=400)
        if not mm.map_exists(name):
            return web.json_response({'ok': False, 'message': f'地图 {name} 不存在'},
                                     status=404)
        node.current_map = name
        node.log('warn', f'网页选择地图 {name}；换图需重启导航栈才能生效')
        return web.json_response({'ok': True, 'current': name,
                                  'message': f'已选择 {name}；需重启导航栈生效'})

    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=15.0)
        await ws.prepare(request)
        node._gw_ws_clients.add(ws)
        try:
            last_ver = -1
            last_epoch = -1
            while not ws.closed:
                snap = node.snapshot()
                await ws.send_json({'type': 'state', 'data': snap})
                # ★ 代次（换栈）优先于版本（同一套栈里的地图变化）。
                #   代次变了要让前端"作废一切"，所以单独发一个事件，
                #   不能只靠 map_changed —— 那个的语义是"图更新了，去重载"。
                if snap['map_epoch'] != last_epoch:
                    last_epoch = snap['map_epoch']
                    last_ver = -1               # 代次变了，版本比较要重来
                    await ws.send_json({'type': 'map_epoch', 'epoch': last_epoch})
                if snap['map_version'] != last_ver:
                    last_ver = snap['map_version']
                    await ws.send_json({'type': 'map_changed',
                                        'version': last_ver})
                with node._gw_lock:
                    logs = list(node._gw_logs)[-40:]
                await ws.send_json({'type': 'logs', 'data': logs})
                await asyncio.sleep(0.25)      # # 4 Hz 推送，手机流量也够省
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            node.log('warn', f'WebSocket 断开：{exc}')
        finally:
            node._gw_ws_clients.discard(ws)
        return ws

    app.router.add_get('/', index)
    app.router.add_get('/login', login_page)
    app.router.add_post('/api/login', api_login)
    app.router.add_get('/api/state', api_state)
    app.router.add_get('/api/map.png', api_map_png)
    app.router.add_get('/api/logs', api_logs)
    app.router.add_post('/api/lease', api_lease)
    app.router.add_post('/api/teleop', api_teleop)
    app.router.add_post('/api/stop', api_stop)
    app.router.add_post('/api/command', api_command)
    app.router.add_post('/api/store', api_store)
    app.router.add_post('/api/cargo', api_cargo)
    app.router.add_post('/api/relocalize', api_relocalize)
    # ★ 下面三个原来只注册了 add_get，但前端 api() 助手默认发 POST，
    #   于是 POST /api/maps 直接吃 405 Method Not Allowed。
    #   前端 catch 到异常只打一行 warn 就把 maps 当成空数组，用户看到的现象
    #   就是"地图库是空的 → 必须先建图 → 建完图导航还是不行"，非常误导。
    #   这里改成 GET/POST 都收：GET 保持对外／调试友好，POST 兼容前端与
    #   其它只发 POST 的客户端。不要二选一 —— 少一个支持就是一种"看起来
    #   莫名其妙失败"的隐患。
    app.router.add_route('*', '/api/maps', api_maps)
    app.router.add_post('/api/map_switch', api_map_switch)
    app.router.add_post('/api/map_delete', api_map_delete)
    app.router.add_route('*', '/api/tasks', api_tasks)
    # ★ /api/routes 原来只注册了 add_post，和上面三个是同一类 bug：
    #   用 GET（浏览器直接访问、curl、第三方脚本）一律 405。
    #   前端 api() 恰好发 POST 所以没暴露，但这种"只支持一半方法"的接口
    #   迟早会在调试或换客户端时咬人，统一收 GET/POST。
    app.router.add_route('*', '/api/routes', api_routes)
    app.router.add_route('*', '/api/check', api_check)
    app.router.add_post('/api/resume', api_resume)
    app.router.add_post('/api/map_save', api_map_save)
    app.router.add_route('*', '/api/stack', api_stack)
    app.router.add_post('/api/stack_start', api_stack_start)
    app.router.add_post('/api/stack_stop', api_stack_stop)
    app.router.add_post('/api/shutdown', api_shutdown)
    app.router.add_get('/ws', ws_handler)
    app.router.add_static('/static/', STATIC_DIR)
    return app


import pathlib                             # use 定位静态资源目录


def _find_static_dir():                    # out 前端静态文件所在目录
    """找前端静态文件目录，兼容"源码树"和"已安装"两种布局。

    ★ 不能只写 `__file__/static`：setup.py 里 static/ 是作为 data_files 装到
      `share/agv_web/static` 的，**不会**留在 Python 模块目录里。
      用 --symlink-install 时因为符号链接回源码树，`__file__/static` 恰好能找到，
      于是这个问题在开发时被藏住了；一旦用普通安装（或换机器部署），
      前端就会 404（页面能开？不，是 index.html 都找不到）。
    所以这里按优先级依次探测：
      ① 源码树布局：<包目录>/static            （开发/符号链接安装）
      ② 安装布局  ：<prefix>/share/agv_web/static（普通安装）
      ③ ament 资源索引：用 get_package_share_directory 问 ROS
    """
    here = pathlib.Path(__file__).parent
    # ① 源码树 / symlink 安装
    cand = here / 'static'
    if (cand / 'index.html').is_file():
        return cand
    # ② 安装布局：__file__ 在 install/agv_web/lib/python3.x/site-packages/agv_web/
    for parent in [here] + list(here.parents):
        c = parent / 'share' / 'agv_web' / 'static'
        if (c / 'index.html').is_file():
            return c
    # ③ 交给 ament 索引去问（最可靠，但依赖 ROS 环境已 source）
    try:
        from ament_index_python.packages import get_package_share_directory
        c = pathlib.Path(get_package_share_directory('agv_web')) / 'static'
        if (c / 'index.html').is_file():
            return c
    except Exception:
        pass
    # 都找不到就返回源码树那个（报错信息里路径更有用）
    return here / 'static'


STATIC_DIR = _find_static_dir()


# ══════════════════════════ tasks.yaml 读写 ══════════════════════════
# 与终端控制台共用同一份文件格式：{'tasks': [{pick, drop, loops, route}, ...]}
def _load_tasks(path):                     # out 读任务缓冲
    try:
        import yaml
        p = pathlib.Path(path)
        if not p.is_file():
            return []
        doc = yaml.safe_load(p.read_text(encoding='utf-8')) or {}
        items = doc.get('tasks') or []
        out = []
        for it in items:
            if not isinstance(it, dict):
                continue
            out.append({'pick': str(it.get('pick', '')),
                        'drop': str(it.get('drop', '')),
                        'loops': max(1, int(it.get('loops', 1) or 1)),
                        'route': str(it.get('route', '') or '')})
        return out
    except Exception:
        return []


def _save_tasks(path, items):              # out 写任务缓冲（保持 LF 与中文不转义）
    import yaml
    p = pathlib.Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    text = yaml.safe_dump({'tasks': list(items)}, allow_unicode=True,
                          sort_keys=False, default_flow_style=False)
    with open(p, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)


# ══════════════════ 二之一、栈起停（本进程管理 ROS 栈）══════════════════

class StackRunner:
    """在网关进程里起停整套 ROS 栈，让用户只敲一条命令。

    **为什么要它**
    原来建图/导航要分别敲两条 launch 命令，跟终端控制台"起一次、界面里选模式"
    的体验差很远。现在网关自己 Popen 一个 launch 子进程，网页上点按钮就切换。

    **两个必须做对的细节**
    ① 用 `start_new_session=True` 让 launch 自成一个进程组，停的时候对**整个组**
       发信号。launch 会派生子进程（gzserver/nav2/amcl…），只杀父进程会留下一堆
       孤儿节点抢话题，表现就是"明明停了车还在飘"。
    ② 先 SIGINT 给 Nav2 体面收尾的时间，超时才 SIGKILL。直接 SIGKILL 会让
       Gazebo 留下共享内存段，下次启动报 "Address already in use"。
    """

    def __init__(self, node):
        self.node = node
        self.proc = None             # def launch 子进程
        self.mode = ''               # def 当前模式（''=没起）
        self.since = 0.0             # def 启动时刻
        self.error = ''              # def 最近错误
        self._gw_lock = threading.Lock()  # def 保护起停（网页可能连点）
        self._gw_log_thread = None      # def 读子进程输出的线程
        # ★ 另一个锁：把"停旧栈 + 起新栈"整段串起来。
        #   为什么不能只用 _gw_lock：停栈要等 10 秒以上（等整组死+等端口），
        #   而 /api/state 每秒都在抢 _gw_lock 读状态，锁被占住会让网页卡住。
        #   所以 _gw_lock 只管"改几个字段"，_op_lock 管"这一整套动作别并发"。
        #   用 RLock：switch() 里会嵌套调 stop()/start()，普通 Lock 会自锁。
        self._op_lock = threading.RLock()

    def _say(self, msg):             # in 往网页日志写一行（给下面的纯函数当回调）
        self.node.log('info', msg)

    # ── 状态 ──
    def state(self):                 # out 当前状态（给网页）
        with self._gw_lock:
            running = self.proc is not None and self.proc.poll() is None
            if not running:
                self.mode = '' if self.proc is None else self.mode
            return {'mode': self.mode,
                    'mode_text': MODE_TEXT.get(self.mode, '未启动'),
                    'running': running,
                    'pid': (self.proc.pid if (self.proc is not None and running) else None),
                    'since': self.since,
                    'error': self.error}

    def is_running(self):            # out 栈是否在跑
        with self._gw_lock:
            return self.proc is not None and self.proc.poll() is None

    # ── 起 ──
    def start(self, mode, map_yaml='', waypoint_file='', routes_file='',
              world='', gui=False, rviz=False):   # out (ok, message)
        """起一套栈。已在跑会先拒绝（由调用方走 plan_switch）。"""
        m = str(mode or '').strip().lower()
        ok, reason = validate_map_name(m, map_yaml)
        if not ok:
            return False, reason
        try:
            argv = build_launch_command(
                m, map_yaml=map_yaml, waypoint_file=waypoint_file,
                routes_file=routes_file, world=world, gui=gui, rviz=rviz,
                ws_setup=self._ws_setup())
        except StackError as exc:
            return False, str(exc)

        with self._op_lock:
            # ★ 先判"是不是已经有栈在跑"，再谈清理。
            #   顺序反了的话，连点两下启动按钮时，第二次会白等 6 秒端口、
            #   还往日志里刷一堆"等等等"的噪音，最后才回一句"已有一套在运行"。
            with self._gw_lock:
                if self.proc is not None and self.proc.poll() is None:
                    return False, '已有一套栈在运行，请先停止'
            # ★ 起之前再兜一层：扫掉上一套可能没清理干净的东西。
            #   正常换栈已经走 stop()（那边做得更全），这里是给
            #   "上一次崩在半路 / 用户在外面手工 kill 过节点" 兜底的 ——
            #   这两种情况下没有进程组可等，只能靠 SHM 和端口来判活动静。
            purge_stale_shm(log=self._say)
            wait_port_free(log=self._say)      # 两种模式都带 Gazebo，都要等
            with self._gw_lock:
                if self.proc is not None and self.proc.poll() is None:
                    return False, '已有一套栈在运行，请先停止'
                try:
                    proc = subprocess.Popen(
                        argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                        start_new_session=True,       # ★ 自成进程组，停时整组收
                        text=True, bufsize=1,
                        env=self._child_env())
                except Exception as exc:
                    self.error = str(exc)
                    return False, f'启动失败：{exc}'
                self.proc = proc
                self.mode = m
                self.since = time.time()
                self.error = ''
                # ★ 把真正执行的命令打进日志。
                #   为什么必须打：map_yaml 曾经被传成一整个 dict 的 str()，
                #   栈照样起得来、日志只有一句"已启动"，从界面上完全看不出
                #   map_server 拿到的是个假路径。只有把 argv 摊开才能一眼看见。
                #   顺带核对：导航模式下地图文件必须真实存在。
                self.node.log('info', '启动命令：' + ' '.join(shlex.quote(a) for a in argv))
                if m == MODE_NAV:
                    if map_yaml and os.path.isfile(map_yaml):
                        self.node.log('info', f'地图文件就绪：{map_yaml}')
                        # ★ 别名说明：中文地图名的路径走完 launch 链会被转义成
                        #   uXXXX 再交给 map_server（→ bad file → 地图全黑，实测）。
                        #   所以传给 launch 的是纯 ASCII 别名，这里把原始地图也打出来，
                        #   免得以后日志里出现一条用户没见过的路径又被当成"路径错了"。
                        try:
                            from agv_common.map_manager import LAUNCH_ALIAS_DIR
                            is_alias = (f'{os.sep}{LAUNCH_ALIAS_DIR}{os.sep}') in map_yaml
                        except Exception:
                            is_alias = False
                        if is_alias:
                            mm = self.node.get_map_manager()
                            real = (mm.map_yaml_file(self.node.current_map)
                                    if mm is not None and self.node.current_map else '')
                            self.node.log('info',
                                          '地图名含非 ASCII 字符，map_server 改用纯 ASCII 别名'
                                          f'（原始地图：{real or "（未知）"}）')
                    else:
                        self.node.log('error',
                                      f'地图文件不存在，map_server 会加载失败：{map_yaml!r}')
        self._gw_log_thread = threading.Thread(
            target=self._pump_output, args=(proc,), daemon=True)
        self._gw_log_thread.start()
        self.node.log('info', f'已启动{MODE_TEXT[m]}（PID {proc.pid}）')
        return True, f'{MODE_TEXT[m]}启动中（Nav2 激活约需 15~40 秒）'

    def _ws_setup(self):             # out 工作区的 setup.bash 路径
        """找本工作区的 install/setup.bash，让子进程能用自建的包。"""
        env = os.environ.get('AGV_WS_SETUP', '').strip()
        if env and os.path.isfile(env):
            return env
        root = self.node.get_workspace_root()
        if root:
            cand = os.path.join(root, 'install', 'setup.bash')
            if os.path.isfile(cand):
                return cand
        return ''

    def _child_env(self):            # out 子进程环境（关掉输出缓冲 + 关 SHM）
        env = dict(os.environ)
        env.setdefault('PYTHONUNBUFFERED', '1')
        # ★ 显式覆盖，不依赖父进程继承：网关可能由 systemd / 别的 shell
        #   拉起，万一父环境里没带这个变量，栈又会走回共享内存。
        env.update(_FASTDDS_ENV)
        return env

    def _pump_output(self, proc):    # in 把子进程输出转到网页日志
        """把 launch 的输出逐行读进网页日志（过滤噪音 + 翻成中文）。

        ★ 必须一直读：PIPE 写满（默认 64KB）后子进程会阻塞在 write 上，
          整个栈就"卡住不动"了——这是这类代码最经典的坑。

        ★ 为什么要过滤和翻译（用户原话："一堆英文日志，根本看不懂"）：
          ROS 栈的原始输出混着大量没信息量的行（launch 样板话、参数回显、
          bond timer…），一条条原样塞进手机上的日志面板，用户只能看到
          一屏英文。这里交给 log_translate：噪音直接丢，已知句式翻中文，
          没识别出的保留原文（宁可留着英文也不能丢信息）。
          过滤/翻译都在**这个读线程**里做，不影响 ROS 回调。
        """
        from agv_web import log_translate as LT
        try:
            for line in proc.stdout:
                s = (line or '').rstrip()
                if not s:
                    continue
                if LT.is_noise(s):
                    continue
                out = LT.translate(s)
                if not out:
                    continue
                lvl = LT.guess_level(s)
                if len(out) > 300:
                    out = out[:300] + '…'
                # 网页日志里不再加 [栈] 前缀：中文前缀里已经带了模块名
                # （[定位] / [路径规划] / [地图服务]…），再加反而更乱
                self.node.log(lvl, out)
        except Exception:
            pass
        finally:
            rc = proc.poll()
            if rc is not None:
                if rc:
                    self.node.log('error',
                                  f'栈异常退出（返回码 {rc}），请看上面几行日志定位原因')
                else:
                    self.node.log('info', '栈已正常退出')

    # ── 停 ──
    def stop(self, timeout=12.0):    # out (ok, message)
        """停掉整套栈，并把它留下的东西清干净（换栈能不能成功全看这一步）。

        顺序不能换，每一步都对应一类真实故障：
          ① SIGINT 整组 —— 给 Nav2 / Gazebo 体面收尾的机会（直接 -9 会留 SHM）；
          ② **等整组死透** —— `proc.wait()` 只保证 launch 父进程退了，
             gzserver 是它的兄弟进程，还要几秒才释放 11345 端口；
          ③ 还没退就 SIGKILL 整组，再等一次；
          ④ 清 /dev/shm 里**无主**的 FastDDS 段 —— 被 Ctrl-C 打断的节点
             几乎都死在 destroy_node() 里，段来不及 unlink，下一个栈
             撞上就是 `Failed init_port fastrtps_port74xx`，然后新旧参与者
             发现不到彼此，网页上表现为"节点未发现 / 收不到 /scan / 地图黑屏"；
          ⑤ 等 Gazebo master 端口空出来 —— gzserver bind 失败是静默的。
        """
        say = self._say
        with self._op_lock:
            with self._gw_lock:
                proc = self.proc
                # 先翻状态：从这一刻起网页就该显示"没在跑"，
                # 别让用户在看板上看到一个正在慢慢死的栈。
                self.proc = None
                self.mode = ''
                self.since = 0.0
            if proc is None:
                # 本来就没在跑：仍然清一遍（上一次可能崩在半路，没人收尾）
                purge_stale_shm(log=say)
                return True, '本来就没在跑'
            # start_new_session=True → 子进程自成会话，组号就等于 launch 的 PID
            pgid = proc.pid
            self.node.log('warn', '正在停止栈（最多等 12 秒）…')
            if proc.poll() is None:
                self._signal_group(proc, signal.SIGINT)
                try:
                    proc.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    self.node.log('warn', '栈未在 12 秒内退出，强制结束')
                    self._signal_group(proc, signal.SIGKILL)
                    try:
                        proc.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        self.error = '强制结束也超时'
            # ② launch 退了 ≠ 一切退干净
            if not wait_group_dead(pgid, timeout=6.0, log=say):
                self.node.log('warn', '还有子进程赖着，强制结束整组')
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError, OSError):
                    pass
                wait_group_dead(pgid, timeout=4.0, log=say)
            # ④⑤ 残留 SHM + 端口
            purge_stale_shm(log=say)
            wait_port_free(log=say)
        self.node.log('info', '栈已停止')
        return True, '已停止'

    def _signal_group(self, proc, sig):    # in 给整个进程组发信号
        """对进程组发信号。

        ★ 不能只 proc.send_signal：launch 派生的 gzserver/nav2 不在它的
          PID 上，漏杀会留下孤儿节点抢话题（"车还在飘"就是这么来的）。
        """
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except (ProcessLookupError, PermissionError, OSError):
            try:
                if sig == signal.SIGKILL:
                    proc.kill()
                else:
                    proc.send_signal(sig)
            except Exception:
                pass

    # ── 一次性切换 ──
    def switch(self, target, **kwargs):    # out (ok, message)
        """按 plan_switch 的决策把栈切到 target（'' 表示停）。

        ★ 整段包在 _op_lock 里：换栈是"停旧的 + 起新的"两步，
          中间绝不能被另一个请求插进来（连点两个模式按钮就会）。
        """
        with self._op_lock:
            with self._gw_lock:
                cur = self.mode
                running = self.proc is not None and self.proc.poll() is None
            try:
                plan = plan_switch(cur, running, target)
            except StackError as exc:
                return False, str(exc)
            msg = []
            if 'stop' in plan.actions:
                ok, m = self.stop()
                msg.append(m)
                if not ok:
                    return False, '；'.join(msg)
            if 'start' in plan.actions:
                ok, m = self.start(plan.mode, **kwargs)
                msg.append(m)
                if not ok:
                    return False, '；'.join(msg)
            if not plan.actions:
                msg.append(plan.reason)
            return True, '；'.join(msg) or plan.reason


# ══════════════════════════ 三、入口 ══════════════════════════

# 启动自清理要匹配的模式。
# ★ 每条都必须"长得足够特别"，否则会误杀。这里踩过一次真实事故，记下来：
#   最初写了 'ros2 launch agv_'，想一次匹配所有本项目的 launch。结果
#   `ros2 launch agv_web web_console.launch.py`（就是我自己的父进程！）
#   也含这个字符串，于是网关启动时把自己的 launch 杀了 —— 日志里出现
#   "34 已杀死 ros2 launch agv_web ..."，网页还没起来就没了。
#   所以 launch 那一条必须**排除自己**，见 _CLEANUP_LAUNCH_EXCLUDE。
#
# 其它注意事项：
#   · 绝对不能用 'python3' / 'python' / 'node' 这种宽泛词——会连 WorkBuddy
#     本体、用户别的脚本一起干掉；
#   · 'ros2 run agv_' 匹配不到真正干活的节点（只匹配一次性壳），意义不大，
#     所以直接匹配具体的节点可执行名。
_CLEANUP_PATTERNS = (
    'ros2 launch agv_',            # launch 父进程（子进程随进程组一起收）
    'gzserver', 'gzclient', 'spawn_entity',
    'nav2_', 'slam_toolbox',
    'robot_state_publisher', 'robot_localization',
    'task_manager_node', 'cargo_node', 'waypoint_node',
    'velocity_arbiter', 'agv_lift',
    'teleop_twist_keyboard',
    'rviz2',
)

# 这些 launch **不能杀**：它们是"当前这个网页网关自己"。
# 杀了自己就没网页了——比不清理危险得多。
_CLEANUP_LAUNCH_EXCLUDE = (
    'agv_web',                     # 网页网关本身（本进程就是它拉起来的）
)


def _pid_alive(pid):                    # out 进程是否还活着
    try:
        os.kill(pid, 0)
        return True
    except (ProcessLookupError, ValueError):
        return False
    except PermissionError:
        return True                     # 别人的进程，存在但无权操作


def _match_pids(pattern):               # out 匹配 pattern 的 PID 列表（含自身保护）
    """pgrep -f 的 Python 版，且主动排除"查询命令自己"。

    ★ 为什么要排除自己：`pgrep -f web_gateway` 会把执行这条命令的 shell
      也匹配上（它的命令行里含这个字符串），于是清理逻辑可能把正在
      启动的自己杀掉。所以这里比对 /proc/<pid>/cmdline 时，凡是
      "这条 cmdline 里包含 pgrep/grep 字样"的一律跳过。
    """
    pids = []
    me = os.getpid()
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == me:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                raw = f.read().replace(b'\x00', b' ').decode('utf-8', 'replace')
        except (OSError, PermissionError):
            continue
        if not raw.strip():
            continue
        if pattern not in raw:
            continue
        # 排除"查询动作本身"，避免自杀
        if ('pgrep' in raw or 'grep ' in raw) and pattern not in ('grep ',):
            continue
        pids.append(pid)
    return pids


def _ancestor_pids():                    # out 自己的祖先 PID 集合（含自己）
    """收集"我自己这一条血脉"上的所有 PID。

    ★ 为什么必须排除祖先：网关通常是 `ros2 launch agv_web web_console.launch.py`
      拉起来的，那条命令行里含有 'ros2 launch agv_'。清理逻辑如果不排除它，
      就会在启动时把自己的 launch 杀掉 —— 实测日志：
          SIGINT 34 (ros2 launch agv_)
          34 已杀死  ros2 launch agv_web web_console.launch.py ...
      结果网页根本没起来。这比"不清理"危险得多。
    """
    pids = set()
    pid = os.getpid()
    for _ in range(20):                  # # 最多上溯 20 层，防环
        if pid <= 1 or pid in pids:
            break
        pids.add(pid)
        try:
            with open(f'/proc/{pid}/stat', 'rb') as f:
                fields = f.read().rsplit(b')', 1)[1].split()
            pid = int(fields[1])         # # stat 第 4 个字段是 ppid
        except (OSError, IndexError, ValueError):
            break
    return pids


def cleanup_residual(timeout=6.0, log=None):   # out 清掉上次残留，返回清理条数
    """启动时清掉上次没退干净的 ROS 进程。

    **为什么需要它（用户实际遇到的问题）**
    网页控制台是"关浏览器窗口"就走的，进程往往还挂在后台。下次再启动，
    就出现两套栈的同名节点互抢话题/服务/action，表现是：
      · 「节点没起 / 无响应 / 查询超时」
      · 导航原地转圈、车还在飘
      · 地图加载了但 TF 查不到
    终端版 `agv_console.py` 早就在启动时调 `_pkill_ros()` 了，网页版一直漏了
    这一步 —— 这就是"明明重启了还是有怪问题"的根源。

    **做法**
    先 SIGINT 给 6 秒体面退出（launch/Gazebo 靠它释放 /dev/shm 和 DDS 端口），
    超时才 SIGKILL。直接 -9 会留下共享内存段，下次报 Address already in use。

    **安全边界（三重保险，缺一不可）**
    ① 只匹配本项目独有的可执行名，绝不用 python3/node 这类宽泛词；
    ② 排除自己的整条祖先链（否则会杀掉拉起自己的 launch，见 _ancestor_pids）；
    ③ 排除名字含 agv_web 的 launch（那就是网页网关自己）。
    """
    say = log or (lambda msg: print(msg, flush=True))
    protect = _ancestor_pids()

    found = {}
    for pat in _CLEANUP_PATTERNS:
        for pid in _match_pids(pat):
            if pid in protect:           # # 保险②：别杀自己的血脉
                continue
            found.setdefault(pid, pat)
    if not found:
        return 0

    # 保险③：再按命令行内容过一道，把"网页网关自己"摘出去
    real = {}
    for pid, pat in found.items():
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\x00', b' ').decode('utf-8', 'replace')
        except (OSError, PermissionError):
            continue
        if any(x in cmd for x in _CLEANUP_LAUNCH_EXCLUDE):
            say(f'  跳过 {pid}（是网页网关自己）')
            continue
        real[pid] = pat
    found = real
    if not found:
        return 0

    say(f'清理上次残留进程：{len(found)} 个')
    for pid, pat in found.items():
        say(f'  SIGINT {pid} ({pat})')
        try:
            os.kill(pid, signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            pass

    deadline = time.time() + timeout
    while time.time() < deadline:
        alive = [p for p in found if _pid_alive(p)]
        if not alive:
            say('  已全部退出')
            return len(found)
        time.sleep(0.25)

    alive = [p for p in found if _pid_alive(p)]
    for pid in alive:
        say(f'  SIGKILL {pid}（未在 {timeout:.0f}s 内退出）')
        try:
            os.kill(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
    return len(found)


# ══════════════ 换栈前的"彻底清理"（导航 ⇄ 建图 必须走这套）══════════════
#
# ★ 这一组函数解决的是用户反复报的那类 bug：
#     "切到建图模式地图黑屏、自检说节点都没起来"
#     "切换后老是有遗留，车还在飘 / 看不到激光"
#   根因不在建图代码，而在**换栈时旧的那套没收干净**：
#
#   ① `proc.wait()` 只保证 `ros2 launch` 这个父进程退了。它派生的
#      gzserver / nav2 / slam_toolbox 是**进程组**里的兄弟进程，
#      父进程退出时它们可能还在跑（gzserver 收尾要好几秒）。
#      于是新栈刚起来时，11345 端口还被老 gzserver 占着 ——
#      而 Gazebo 的 bind 失败是**静默**的：新 gzserver 日志照打、
#      世界里却什么都没有，看着像"仿真起了但地图一片黑"。
#
#   ② 被 Ctrl-C 打断的 rclpy 节点几乎都死在 `destroy_node()` 里
#      （日志里那一串 KeyboardInterrupt traceback 就是），
#      于是 FastDDS 的共享内存段来不及 unlink，留在 /dev/shm。
#      下一个栈的所有参与者在 `open_and_lock_file` 上撞到这些残留：
#         [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port74xx
#      本机走的是"同机优先共享内存"这条路，SHM 一废，新旧参与者
#      就发现不到彼此 —— 网页上的表现就是「节点未发现 / 收不到 /scan /
#      地图黑屏」，而终端里节点其实活得好好的。
#
#   所以顺序必须是：发信号 → 等**整组**死透 → 清残留 SHM → 等端口空 → 才起新栈。

# 只碰这两类名字。FastDDS 的段名形如 fastrtps_port7411_xxxx、sem.fastrtps_...
_SHM_STALE_PREFIXES = ('fastrtps_', 'sem.fastrtps_')

# Gazebo Classic 默认 master 端口（gzserver bind 这个）。
GAZEBO_MASTER_PORT = 11345


def _group_members(pgid):               # out 进程组里还活着的 PID 列表
    """列出某进程组里**还没死**的成员（僵尸不算）。

    ★ 为什么不用 `os.killpg(pgid, 0)` 判活：僵尸进程也占着 PID，
      killpg(0) 会一直成功，`while 组还活着` 就永远退不出来。
      容器里 PID 1 不一定帮忙回收孤儿，这个坑很容易踩。
      所以直接读 /proc/<pid>/stat 拿 pgid 和状态，跳过 Z。
    """
    out = []
    try:
        names = os.listdir('/proc')
    except OSError:
        return out
    for name in names:
        if not name.isdigit():
            continue
        try:
            with open(f'/proc/{name}/stat', 'rb') as f:
                raw = f.read().decode('utf-8', 'replace')
        except OSError:
            continue                            # # 进程刚好退了
        # stat 第二列是 comm，可能含空格和括号 → 先切掉再按空格分
        try:
            rest = raw.rsplit(')', 1)[1].split()
            state, pgrp = rest[0], int(rest[2])  # # 0=state 1=ppid 2=pgrp
        except (IndexError, ValueError):
            continue
        if pgrp != pgid or state == 'Z':
            continue
        out.append(int(name))
    return out


def wait_group_dead(pgid, timeout=6.0, log=None):   # out 是否整组已消失
    """等整个进程组真正死光（不只是 launch 父进程）。

    参数：pgid - 进程组号（本项目里 == launch 的 PID，因为 start_new_session=True）
          timeout - 最多等几秒
    返回：True=整组已退出；False=还有活口（调用方该 SIGKILL 了）
    """
    say = log or (lambda msg: None)
    if not pgid:
        return True
    deadline = time.time() + max(0.0, timeout)
    while True:
        left = _group_members(pgid)
        if not left:
            return True
        if time.time() >= deadline:
            say(f'还有 {len(left)} 个进程没退（PID {left[:4]}）')
            return False
        time.sleep(0.2)


def _mapped_shm_names():                # out 正被活进程映射的 /dev/shm 文件名集合
    """扫 /proc/*/maps，收集所有"有主"的共享内存段名。

    ★ 这是删除残留 SHM 的**安全边界**：只有没被任何进程映射的段才允许删。
      网关自己的 FastDDS 段、用户另开的 ROS 程序的段都在这里面，
      因此不会误删（删掉正在用的段会让那个进程直接崩）。
    """
    ref = set()
    try:
        names = os.listdir('/proc')
    except OSError:
        return ref
    for name in names:
        if not name.isdigit():
            continue
        try:
            with open(f'/proc/{name}/maps', 'r', errors='replace') as f:
                for line in f:
                    if '/dev/shm/' not in line:
                        continue
                    seg = line.split('/dev/shm/', 1)[1]
                    seg = seg.split(' (deleted)')[0].strip()
                    if seg:
                        ref.add(seg)
        except OSError:
            continue
    return ref


def purge_stale_shm(shm_dir='/dev/shm', log=None, grace=2.0):   # out 删掉的段数
    """清掉没有任何进程在用的 FastDDS 残留段。

    参数：shm_dir - 共享内存目录（单测传临时目录）
          grace   - 只删"生成超过 grace 秒"的文件，避开正在启动的参与者
    返回：删除条数

    ★ 只删 _SHM_STALE_PREFIXES 里的名字 + 只删无主的，两条同时成立才动。
    """
    say = log or (lambda msg: None)
    try:
        entries = os.listdir(shm_dir)
    except OSError:
        return 0
    ref = _mapped_shm_names()
    now = time.time()
    freed = 0
    for name in entries:
        if not name.startswith(_SHM_STALE_PREFIXES):
            continue
        if name in ref:                     # # 有活进程在用，绝对不能删
            continue
        path = os.path.join(shm_dir, name)
        try:
            if now - os.stat(path).st_mtime < grace:
                continue                    # # 刚生成的，可能是刚要起来的参与者
            os.unlink(path)
            freed += 1
        except OSError:
            continue
    if freed:
        say(f'清理残留共享内存：{freed} 段（上一次被强杀的节点没来得及释放）')
    return freed


def wait_port_free(port=GAZEBO_MASTER_PORT, timeout=6.0, log=None):   # out 端口是否空出
    """等 Gazebo master 端口空出来。

    ★ 为什么必须等：gzserver bind 失败**不打日志、不退出**，
      新栈会"看起来正常启动"，实际世界里没数据 —— 这就是
      "切到建图模式，地图一片黑、自检说节点都没起"的直接成因之一。
    返回：True=端口已空；False=超时仍被占（调用方只告警，不阻塞启动）
    """
    say = log or (lambda msg: None)
    deadline = time.time() + max(0.0, timeout)
    warned = False
    while True:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.bind(('', port))
            return True
        except OSError:
            if time.time() >= deadline:
                say(f'端口 {port} 仍被占用，新栈的 Gazebo 可能起不来'
                    f'（它 bind 失败是静默的，看不出报错）')
                return False
            if not warned:
                say(f'端口 {port} 还被旧栈占着，等它释放…')
                warned = True
            time.sleep(0.2)
        finally:
            sock.close()


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
        #     网页"节点未发现 / 收不到 /scan / 地图黑屏"（本轮查的就是这个）。
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
    #   run_app 内部会自己创建/运行循环，所以这里取"即将运行的那个"——
    #   用 asyncio.get_event_loop() 在 run_app 之前拿到的是同一个。
    try:
        node._gw_loop = asyncio.get_event_loop()
    except RuntimeError:
        node._gw_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(node._gw_loop)

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
        #   started_ok 只是让日志别在"根本没起来"的情况下说"退出"。
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
