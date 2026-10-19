"""
node.py —— `WebGateway` 的组装：参数、共享状态、ROS 输入输出、启动横幅。

**为什么 `__init__` 要拆**
原来它是一个 252 行的巨型构造函数，25 个 declare_parameter、34 个
`_gw_*` 字段、8 个订阅、4 个发布、3 个 client、2 个 timer 全挤在一起，
读的时候根本分不清"哪一行是必须的、哪一行只是顺手"。拆成 6 段之后，
看方法名就知道这一段的职责，加新订阅也知道该往哪儿放。

**继承链**
    WebGateway → StateMixin → CommandMixin → Node

两个 mixin 都不定义 `__init__`，所以 `super().__init__('agv_web_gateway')`
直达 `Node.__init__`。**调用顺序不能换**：
`_init_shared_state` 建出 `_gw_lock` 和一堆字段，而 `_init_ros_io` 建的
订阅可能在返回前就收到第一帧 —— 那时回调要读这些字段。
"""

import threading                           # use 共享状态锁

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy,
)

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

try:                                       # # rosout 缺失时仍要能起来（只是不转发）
    from rcl_interfaces.msg import Log as RosoutLog
except Exception:                          # pragma: no cover
    RosoutLog = None

from agv_interfaces.msg import AgvCommand, TaskState
from agv_interfaces.srv import AgvQuery, AgvStore, CargoCmd
from agv_common.web_guard import AuthGate, ControlLease, TeleopWatchdog

from ..status_board import LogFilter
from ..stack_manager import (
    MODE_MAPPING, MODE_NAV, MODE_TEXT, teleop_topic_for,
)
from .util import _lan_urls
from .commands import CommandMixin
from .stack import StackRunner
from .state import StateMixin

class WebGateway(StateMixin, CommandMixin, Node):
    """网关节点：持有全部 ROS 侧句柄，对外只暴露线程安全的方法。

    ★ `_gw_lock` 是非重入 `threading.Lock()`，`log()` 内部要取它 ——
      持锁时绝不能调 `log()`（会当场死锁），锁内只做字段赋值。
    """

    def __init__(self):
        super().__init__('agv_web_gateway')

        # ★ 六段的顺序就是依赖顺序，别为了"看起来整齐"去调整：
        #   参数 → 字段（含锁）→ ROS 句柄 → 栈起停器 → 打印。
        self._declare_parameters()
        self._read_parameters()
        self._init_shared_state()
        self._init_ros_io()
        self._init_stack_runner()
        self._log_startup_banner()


    # ── 一、参数声明：全部可由 launch / yaml 覆盖 ──
    def _declare_parameters(self):
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

    # ── 二、参数读取：一次性抓成本地属性，回调里不再查参数树 ──
    def _read_parameters(self):
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

    # ── 三、准入/控制权 + 共享状态（被 asyncio 线程读，故加锁）──
    def _init_shared_state(self):
        gp = self.get_parameter

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

    # ── 四、ROS 输入输出：订阅 / 发布 / 服务客户端 / 计时器 ──
    def _init_ros_io(self):
        gp = self.get_parameter

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

    # ── 五、栈起停器：放在 ROS 部分之后，因为它要用 get_workspace_root ──
    def _init_stack_runner(self):
        gp = self.get_parameter

        # ── 栈起停器（放在最后：它要用到 get_workspace_root）──
        # 单命令模式下，网页点按钮就能拉/停整套 ROS 栈，像终端控制台一样。
        if bool(gp('manage_stack').value):
            self.stack = StackRunner(self)
        if self.mode == MODE_MAPPING:
            self.teleop_topic = teleop_topic_for(MODE_MAPPING)

    # ── 六、启动横幅：告诉用户该敲哪个地址 ──
    def _log_startup_banner(self):
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
