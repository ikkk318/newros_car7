"""
stack_manager —— 网页端起停 ROS 栈（纯逻辑部分，不直接碰 subprocess）。

**为什么需要它**
原来的设计要用户自己敲两条命令（一条起网关、一条起建图栈），
这跟终端控制台"启动一次、在界面里选模式"的体验差得远。现在改成：
只起一个网关进程，建图/导航都由网页里的一个按钮拉起对应的栈。

**这里只放"纯逻辑"**
把"该敲什么命令""现在是什么状态""这个切换允不允许"算清楚，
真正的 Popen / kill 交给 web_gateway 去做。这样命令拼装和状态机可以单测，
不必为了测一个"能不能切模式"而去真的拉起 Gazebo。

**两条硬约束（都来自项目实况）**
① 底盘永远只听 `/cmd_vel_raw`（URDF 里 `cmd_vel:=cmd_vel_raw` 是写死的 remap）。
   导航栈有 `velocity_arbiter` 把 `/cmd_vel` 转过去；建图栈**没有** arbiter，
   所以两种模式的遥控话题必须不同——这条在 `teleop_topic_for()` 里固定下来。
② 多个栈同时跑会互相抢话题/服务/action，表现为"节点没起/无响应/原地转"。
   所以"切模式"必须是"先停干净的旧的，再起新的"，不允许并存。
"""

import shlex                              # use 拼命令时做安全转义


# 支持的两种模式
MODE_NAV = 'nav'                         # def 导航模式：仿真+定位+Nav2+业务节点+仲裁
MODE_MAPPING = 'mapping'                 # def 建图模式：仿真+SLAM，无 Nav2/无仲裁
MODES = (MODE_NAV, MODE_MAPPING)

MODE_TEXT = {MODE_NAV: '导航模式', MODE_MAPPING: '建图模式'}   # 显示用中文

# 遥控话题：导航走 task_manager 校验后转发，建图直接驱动底盘
TELEOP_TOPIC = {MODE_NAV: '/agv/web_teleop',       # def 导航：经互锁校验
                MODE_MAPPING: '/cmd_vel_raw'}      # def 建图：直连底盘（无仲裁）


class StackError(Exception):             # def 栈操作错误
    pass


def teleop_topic_for(mode):              # out 该模式下的遥控话题
    """返回该模式应该把速度发到哪个话题。

    ★ 这是建图能不能动起来的唯一关键点：建图栈里没有 velocity_arbiter，
      发到 /agv/web_teleop 没有任何东西会转发给底盘，车一动不动。
    """
    m = str(mode or '').strip().lower()
    if m not in TELEOP_TOPIC:
        return TELEOP_TOPIC[MODE_NAV]    # # 未知模式按导航处理（安全默认）
    return TELEOP_TOPIC[m]


def build_launch_command(mode, map_yaml='', waypoint_file='', routes_file='',
                         world='', gui=False, rviz=False, use_sim_time=True,
                         extra_args=None, ros_setup='/opt/ros/humble/setup.bash',
                         ws_setup=''):
    """拼出要执行的 `ros2 launch ...` 命令（返回 argv 列表，不经过 shell）。

    参数：mode          - 'nav' 或 'mapping'
          map_yaml      - 导航模式要加载的地图
          world         - 建图模式用的 Gazebo 世界
          gui / rviz    - 是否开 Gazebo / RViz 窗口
          extra_args    - {名字: 值} 追加的 launch 参数
    返回：list[str]，可直接交给 Popen(argv, ...)

    ★ 返回列表而不是字符串：避免地图名里带空格/引号时被 shell 拆开。
      但因为是 `bash -lc` 起来（需要 source 环境），内部再用 shlex.quote 兜一层。
    """
    m = str(mode or '').strip().lower()
    if m not in MODES:
        raise StackError(f'未知模式：{mode!r}（只支持 {MODES}）')

    if m == MODE_MAPPING:
        # 建图：复用 agv_navigation 的建图栈（仿真 + slam_toolbox + 点位服务）
        pkg, launch = 'agv_navigation', 'mapping.launch.py'
        args = {'gui': 'true' if gui else 'false',
                'rviz': 'true' if rviz else 'false',
                'use_sim_time': 'true' if use_sim_time else 'false'}
        if world:
            args['world'] = world
        # ★ 建图栈现在也起 waypoint_node，点位/路线落在"草稿目录"里
        #   （地图名要等保存时才定）。不传就用 launch 的 /tmp 默认值 ——
        #   那样保存地图后点位就丢了，所以控制台一定会传。
        if waypoint_file:
            args['waypoint_file'] = waypoint_file
        if routes_file:
            args['routes_file'] = routes_file
    else:
        # 导航：走 agv_bringup 的 agv_stack（含业务节点 + 速度仲裁 + Nav2），
        # 不要直接用 navigation.launch.py —— 那样会漏掉 task_manager/waypoint/
        # cargo/velocity_arbiter，表现为"能导航但没任务层、且不避障"。
        pkg, launch = 'agv_bringup', 'agv_stack.launch.py'
        args = {'with_sim': 'true',
                'gui': 'true' if gui else 'false',
                'rviz': 'true' if rviz else 'false'}
        # ★ 这里【不能】传 use_sim_time：agv_stack.launch.py 没有声明这个参数，
        #   它是由 with_sim 自己推导的（use_sim_time = {'use_sim_time': with_sim=='true'}）。
        #   传了会因为"未声明的 launch 参数"而报错/被忽略，属于坑。
        if map_yaml:
            args['map_yaml'] = map_yaml
        if waypoint_file:
            args['waypoint_file'] = waypoint_file
        if routes_file:
            args['routes_file'] = routes_file

    for k, v in (extra_args or {}).items():     # # 调用方追加/覆盖
        args[k] = v

    inner = ['ros2', 'launch', pkg, launch]
    for k, v in args.items():
        inner.append(f'{k}:={v}')

    # 需要先 source 才能找到 ros2 / 包：用 bash -lc 包一层
    src = f'source {shlex.quote(ros_setup)}'
    if ws_setup:
        src += f' && source {shlex.quote(ws_setup)}'
    cmd_str = src + ' && exec ' + ' '.join(shlex.quote(a) for a in inner)
    return ['bash', '-c', cmd_str]


def mode_of_launch_text(text):           # out 从命令文本反推模式（判断是否已在跑）
    """从拼接后的命令文本判断它属于哪个模式；认不出返回 ''。"""
    t = str(text or '')
    if 'mapping.launch.py' in t:
        return MODE_MAPPING
    if 'agv_stack.launch.py' in t:
        return MODE_NAV
    return ''


class StackState:                        # out 栈的运行状态（给网页显示用）
    def __init__(self, mode='', pid=None, running=False, since=0.0,
                 error=''):
        self.mode = mode                 # def 当前模式
        self.pid = pid                   # def 进程组 leader 的 PID
        self.running = running           # def 是否在跑
        self.since = since               # def 启动时刻（monotonic）
        self.error = error               # def 最近一次错误

    def as_dict(self):                   # out 转成可 JSON 化的 dict
        return {'mode': self.mode,
                'mode_text': MODE_TEXT.get(self.mode, '未启动'),
                'pid': self.pid,
                'running': self.running,
                'since': self.since,
                'error': self.error}


class StackPlan:                         # out 切换决策（纯逻辑，先算清楚再动手）
    """决定"要点哪个按钮"该做哪些动作，避免在 web 层写一堆 if。

    返回值 actions 是动作序列，web 层照着执行：
        'stop'  先停掉现有栈
        'start' 起新栈
    典型：
        没跑 + 要 nav        → ['start']
        建图在跑 + 要 nav    → ['stop', 'start']
        已在 nav + 要 nav    → []           （幂等，不重启）
        在跑 + 要停          → ['stop']
    """
    def __init__(self, actions, mode='', reason=''):
        self.actions = list(actions)
        self.mode = mode
        self.reason = reason

    def __repr__(self):
        return f'StackPlan({self.actions}, {self.mode!r}, {self.reason!r})'


def plan_switch(current, running, target):   # out 计算切换动作
    """计算把栈切到 target 需要做什么。

    参数：current - 当前模式（'' 表示没起）
          running - 当前是否有栈在跑
          target  - 目标模式；传 '' 表示"停止"
    返回：StackPlan
    """
    tgt = str(target or '').strip().lower()
    if tgt and tgt not in MODES:
        raise StackError(f'未知目标模式：{target!r}')

    # 想停
    if not tgt:
        if running:
            return StackPlan(['stop'], '', '按请求停止当前栈')
        return StackPlan([], '', '本来就没在跑')

    # 想跑同一个：幂等，不重启（重启会白等 15~40 秒 Nav2 激活）
    if running and current == tgt:
        return StackPlan([], tgt, f'{MODE_TEXT[tgt]}已在运行，无需重启')

    # 在跑但模式不同：必须先停（否则两套栈抢话题/服务，表现为原地转/没响应）
    if running and current != tgt:
        return StackPlan(['stop', 'start'], tgt,
                         f'从{MODE_TEXT.get(current, "未知")}切到{MODE_TEXT[tgt]}')

    # 没在跑：直接起
    return StackPlan(['start'], tgt, f'启动{MODE_TEXT[tgt]}')


def validate_map_name(mode, map_yaml):   # out 校验"导航模式必须有地图"
    """导航模式没地图是起不来的（map_server 会空转，AMCL 对不上）。

    返回：(ok, reason)
    """
    if str(mode or '').strip().lower() != MODE_NAV:
        return True, ''                  # # 建图模式不需要地图
    if str(map_yaml or '').strip():
        return True, ''
    return False, '导航模式必须指定地图（map_yaml），否则 map_server 起不来'
