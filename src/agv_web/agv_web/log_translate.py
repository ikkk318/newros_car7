"""
log_translate —— 把 ROS / launch / Nav2 的英文日志翻成中文，并过滤噪音。

**为什么需要它（用户的原话）**
"还有一堆英文日志，根本看不懂，给我日志中文化"

ROS 栈的输出是这样的：

    [INFO] [launch]: Default logging verbosity is set to INFO
    [WARNING] [map_server-3]: Timed out waiting for transform from base_link to map
    [ERROR] [planner_server-9]: Failed to create plan: Goal is in collision
    [nav2_controller]: Passing new controller ...

这些信息对调试很有用（谁发的、什么级别、哪条警告），但直接甩到手机屏幕上，
非专业用户只能看出"一堆英文"。所以这里做两件事：

① **翻译**：按"模式 → 中文模板"的顺序匹配，命中就换成中文。
   没命中的**保留原文**——宁可留着英文，也不能把有用信息翻丢或翻错。
   翻译只做"已知高频句式"，不做通用机翻。
② **降噪**：ROS 有些日志天然是垃圾（参数声明回显、"Creating bond timer"、
   `[INFO] [launch]: All log files can be found below ...`），
   一条都不该出现在用户日志面板里，直接丢掉。

**设计原则**
- 纯函数、无依赖、可单测（本模块不 import rclpy）。
- 规则表用 (正则, 中文模板) 元组，模板里可用 `\\1` 反向引用捕获组。
- 必须先过滤、再翻译：某些噪音行里带 "error" 字样（如
  `use_sim_time parameter` 的告警），先翻译会浪费匹配。
"""

import re

# ══════════════════════════ 一、要整条丢弃的噪音 ══════════════════════════
# 全部是"启动时必打、但没有任何信息量"的行。
# 判据：只要正则匹配到整行就丢，不保留任何痕迹。
_NOISE = [
    # launch 自己的样板话：日志路径、默认 verbosity
    re.compile(r'All log files can be found below'),
    re.compile(r'Default logging verbosity is set to'),
    # 参数声明 / 覆盖回显：每个节点启动都会打，一屏几十行
    re.compile(r'^\s*Parameter\s+\S+\s+(?:doesn\'t exist|is not declared)', re.I),
    re.compile(r'declare_parameter.*already declared', re.I),
    # Nav2 生命周期节点的例行通告
    re.compile(r'Creating bond timer', re.I),
    re.compile(r'Activating bond', re.I),
    re.compile(r'Destroying bond', re.I),
    re.compile(r'^\[?INFO\]?\s*\S*\s*\[?lifecycle_manager', re.I),
    # launch 的子进程启动/退出（我们自己的日志已经把这件事说清楚了）
    re.compile(r'^\[?INFO\]?\s*\[[a-z_]+-\d+\]:\s*process started with pid', re.I),
    re.compile(r'^\[?INFO\]?\s*\[[a-z_]+-\d+\]:\s*process has finished cleanly', re.I),
    # ★ 实测漏掉的：spawn_entity.py 在 rclpy 日志里长这样，
    #   前面有个 [节点名] 而不是 [INFO]，所以上面那条没接住。
    re.compile(r'process started with pid', re.I),
    # spawn_entity 的等待回显：每个字都是一样的，纯噪音
    re.compile(r'Spawn Entity started', re.I),
    re.compile(r'Loading entity published on topic', re.I),
    re.compile(r'Waiting for entity xml', re.I),
    re.compile(r'Waiting for service /spawn_entity', re.I),
    re.compile(r'Successfully spawned entity', re.I),
    # robot_state_publisher 的连杆列表：机械臂一多就刷十几行，
    # 而且这些信息在 urdf 里本来就有
    re.compile(r'got segment \w+', re.I),
    # slam_toolbox 的求解器插件选择（固定不变）
    re.compile(r'Using solver plugin solver_plugins', re.I),
    re.compile(r'CeresSolver: Using \w+ preconditioner', re.I),
    re.compile(r'Node using stack size', re.I),
    # tf2 的启动噪声
    re.compile(r'^\[?INFO\]?\s*\S*\s*tf2_ros.*Static? ?Transform? ?Broadcaster', re.I),
    # EKF / robot_localization 的周期回显
    re.compile(r'^\[?INFO\]?\s*\S*\s*(?:ekf_node|ukf_node).*(?:initialized|started)', re.I),
    # ── basic_navigator（nav2_simple_commander）的例行通告 ──
    # ★ 这两句是"我们自己调库造成的"，不是 Nav2 服务端的输出：
    #   · `Nav2 is ready for use!` —— 每调一次 waitUntilNav2Active() 打一条。
    #     网关是**每单**都调（goto_pose 第一句），所以跑 5 单就是 5 条。
    #     它的信息量是"Nav2 就绪"，而这件事我们已经在栈启动事件里报过了。
    #   · `Navigating to goal: x y...` —— 每发一个目标一条，而我们在 goto_pose
    #     里已经打了"发送导航目标：x=… y=… yaw=…（坐标系 map）"，那条更全。
    #   实测 221 行日志里这两句共 33 条（15%），全是重复信息。
    #   ★ 除了这里，nav_runner 里还把 basic_navigator 的 logger 级别压到了
    #     WARN —— 双保险：级别压住了这里根本收不到；万一某人改了级别，
    #     这里还能兜住。
    # ★ 必须排除带级别的行：`[ERROR] ... Navigating to goal aborted` 是
    #   失败事件，绝不能当噪音丢掉。判据放在正则里而不是 is_noise 里，
    #   因为其它噪音规则（process started with pid 之类）确实不分级别。
    re.compile(r'^(?!.*\[(?:ERROR|WARN|WARNING|FATAL)\])'
               r'\s*(?:\[[^\]]*\]\s*[:：]?\s*)*Nav2 is ready for use', re.I),
    re.compile(r'^(?!.*\[(?:ERROR|WARN|WARNING|FATAL)\])'
               r'\s*(?:\[[^\]]*\]\s*[:：]?\s*)*Navigating to goal', re.I),
]

# ══════════════════════════ 二、翻译规则（顺序敏感） ══════════════════════════
# 每条 (正则, 中文模板)。正则用 re.search（不必整行匹配），模板可用 \1 \2。
# ★ 顺序很重要：具体的放前面，宽泛的放后面。比如
#   "Failed to create plan: Goal is in collision" 要先被"路径规划失败"接住，
#   否则会被后面宽泛的 "Failed" 规则吃掉。
_TRANSLATE = [
    # ── 位置 / TF / 定位 ──
    (r'Timed out waiting for transform from (\w+) to (\w+)',
     r'等待坐标变换超时（\1 → \2），定位可能尚未收敛'),
    (r'Lookup would require extrapolation into the future',
     r'坐标变换时间超前（TF 时间戳不同步）'),
    (r'Lookup would require extrapolation into the past',
     r'坐标变换时间滞后（TF 时间戳不同步）'),
    (r'Message Filter dropping message.*queue is full',
     r'传感器消息被丢弃（处理不过来，队列已满）'),
    (r'(?:AMCL|amcl).*(?:initial pose|initialpose).*(?:set|received)',
     r'定位已收到初始位姿'),
    (r'No laser scan received.*timeout',
     r'超时未收到激光数据，检查雷达是否工作'),

    # ── 导航 / 路径规划 ──
    # ★ 顺序是"整句 → 局部"：先把 "Failed to create plan: X" 整句吃掉，
    #   再单独处理 X 里的 "Goal is in collision"。反过来的话，
    #   "Goal is in collision" 会先命中、把句子切成两半拼回来，
    #   结果前半截英文还留着（实测过：'Failed to create plan: 目标点落在障碍物里'）。
    (r'Failed to create(?: a)? plan[:\s]*(.*)',
     r'路径规划失败：\1'),
    (r'Goal is in collision',
     r'目标点落在障碍物里（请换一个点）'),
    (r'Planning (?:succeeded|completed)',
     r'路径规划成功'),
    (r'Received (?:a )?(?:new )?goal|New goal received',
     r'收到新的导航目标'),
    (r'Received initial pose',
     r'收到初始位姿（AMCL 已重新定位）'),
    (r'(?:Goal|NavigateToPose).*(?:succeeded|reached)',
     r'已到达导航目标'),
    (r'(?:Goal|NavigateToPose).*(?:aborted|cancell?ed)',
     r'导航目标被中止或取消'),
    (r'Failed to compute (?:a )?(?:velocity|control)(?: command)?',
     r'速度控制失败（可能被障碍物卡住或路径被堵）'),
    (r'Goal was rejected by (?:the )?server',
     r'导航目标被服务器拒绝（目标不可达）'),
    (r'Global (?:plan|costmap).*(?:updated|received)',
     r'全局路径已更新'),

    # ── 代价地图 ──
    (r'Timed out waiting for (?:the )?costmap',
     r'等待代价地图超时（可能刚启动还没就绪）'),
    (r'Sensor .* (?:clearing|marking).*costmap|costmap.*(?:resize|resizing)',
     r'代价地图尺寸调整中'),
    (r'Costmap.*(?:received|updated)',
     r'代价地图已更新'),

    # ── 底盘 / 速度 ──
    # ★ 这条要能同时吃住 "cmd_vel timeout" 和
    #   "cmd_vel timeout, publishing zero velocity" 两种写法，
    #   前者是原因、后者是处置，合并成一句中文更清楚。
    (r'(?:cmd_vel|twist)\s*timeout[^,;]*[,;]?\s*(?:publishing zero (?:velocity|twist))?',
     r'速度指令超时（上层不再发指令），已自动停车'),
    (r'Publishing zero (?:velocity|twist)',
     r'已发零速（车已停住）'),
    (r'Received (?:cmd_vel|Twist)',
     r'收到速度指令'),

    # ── 建图 / 地图 ──
    (r'(?:map|occupancy).*(?:received|updated|published)',
     r'地图已更新'),
    (r'slam_toolbox.*(?:initialized|started|activated)',
     r'建图模块已就绪'),
    (r'(?:Failed|Unable) to (?:load|open) map[:\s]*(.*)',
     r'地图加载失败：\1'),
    (r'map_saver.*(?:saved|complete)',
     r'地图已保存'),

    # ── 业务 / 顶升 ──
    (r'(?:cargo|lift).*(?:up|raise).*(?:complete|done|finish)',
     r'顶升完成'),
    (r'(?:cargo|lift).*(?:down|lower).*(?:complete|done|finish)',
     r'下降完成'),

    # ── 进程崩溃（launch 报的，最容易被用户看到又最看不懂）──
    # ★ 括号要一起吃掉：原文是 `[pid 60, exit code 255, cmd 'gzserver ...']`，
    #   只匹配到 exit code 会留下 `, cmd gzserver ...]` 这种半截。
    (r'process has died \[pid (\d+), exit code (-?\d+)(?:,\s*cmd\s*[^\]]*)?\]\.?',
     r'进程崩溃（PID \1，退出码 \2）'),
    (r'exit code (-?\d+)',
     r'退出码 \1'),

    # ── 通用失败的兜底（放最后，避免吃掉上面更具体的） ──
    (r'^(.*?)\b(?:Traceback \(most recent call last\))\b(.*)$',
     r'程序内部异常（已记录详细堆栈，可上报给维护者）\2'),
    (r'^(.*?)\bFailed to\b(.*)$', r'\1执行失败：\2'),
    (r'^(.*?)\bUnable to\b(.*)$', r'\1无法完成：\2'),
]

# 已编译版本（模块加载时算一次，日志是高频路径，不能每次 re.compile）
_NOISE_RE = [(p, ) for p in _NOISE]
_TRANS_RE = [(re.compile(p, re.I), tpl) for p, tpl in _TRANSLATE]

# 级别前缀：[INFO]/[WARN]/[ERROR] → 中文标识（保留级别，一眼看出严重性）
_LEVEL_ZH = {
    'info': '信息',
    'warn': '警告',
    'error': '错误',
    'fatal': '致命',
    'debug': '调试',
}


def is_noise(line):
    """判断这一行是否该整条丢弃。"""
    if not line:
        return True
    s = line.strip()
    if not s:
        return True
    for (pat, ) in _NOISE_RE:
        if pat.search(s):
            return True
    return False


def guess_level(line):
    """从原始行猜日志级别：'error' / 'warn' / 'info'。

    ROS 的级别标记有好几种写法：
        [ERROR] [1234.5] [node]: ...
        [ERROR] [map_server-3]: ...
        [map_server-3] [ERROR] ...
    """
    up = (line or '').upper()
    if '[FATAL]' in up or '[ERROR]' in up or 'EXCEPTION' in up:
        return 'error'
    # ★ Python 堆栈的第一行没有 [ERROR] 前缀，但它是错误的起点。
    #   漏掉它会让整段 traceback 的首行被标成 info，用户以为是普通信息。
    if 'TRACEBACK (MOST RECENT CALL LAST)' in up:
        return 'error'
    if '[WARN]' in up or '[WARNING]' in up:
        return 'warn'
    return 'info'


def translate(line):
    """把一行英文日志翻成中文。

    参数：line - 原始行（可能带 [INFO] [节点名] 之类前缀）
    返回：str —— 翻译后的整行；没命中任何规则就返回原行（绝不丢信息）

    ★ 为什么没命中要返回原文：翻译表只覆盖"已知高频句式"，
      剩下的多半是某个节点特有的重要信息（比如具体报错参数）。
      返回原文至少还能搜、还能贴给维护者；丢了就真没了。
    """
    s = (line or '').strip()
    if not s:
        return ''

    # ① 拆出前缀，剩下的部分再翻译 —— 这样前缀（级别/节点名）保持统一
    prefix, body = _split_prefix(s)

    # ② 按规则表逐条试，命中第一条就替换并跳出
    #    （规则按"具体 → 宽泛"排序，只应用最具体的那条，避免叠加翻译）
    for pat, tpl in _TRANS_RE:
        m = pat.search(body)
        if not m:
            continue
        try:
            translated = m.expand(tpl)
        except (re.error, IndexError):
            continue        # 模板写错不能把整条日志吃掉
        # ★ 捕获组里可能还是英文（"路径规划失败：Goal is in collision"）。
        #   再对整段翻译结果做一次"短语级"替换，把残余英文也中文化。
        #   只做一层、不递归，避免自我循环。
        for p2, t2 in _TRANS_RE:
            m2 = p2.search(translated)
            if not m2:
                continue
            try:
                translated = (translated[:m2.start()] + m2.expand(t2)
                              + translated[m2.end():])
            except (re.error, IndexError):
                pass
            break
        # 只替换命中的那一段，保留句子其余部分
        head, tail = body[:m.start()], body[m.end():]
        body = (head + translated + tail).strip()
        break

    # ③ 收拾残留的英文级别词与常见短语尾巴
    # ★ 先剔掉 ROS 的纳秒时间戳 [1792257198.790424458]。
    #   每一行都带，占了小半屏宽度，对用户毫无意义（网页日志前面
    #   本来就有 HH:MM:SS）。注意只剔纯数字的，别误伤包名。
    body = re.sub(r'\[\d{9,}\.\d+\]\s*', '', body)
    body = re.sub(r'\[(?:INFO|WARNING|WARN|ERROR|FATAL|DEBUG)\]', '', body,
                  flags=re.I).strip()

    # ★ 剔掉正文里残留的 `[节点名]:`。
    #   真车日志原来长这样（用户截图）：
    #     [task_manager_node] [task_manager_node]: [顶升] 开始（1号货架）
    #     [路径规划] [planner_server]: Planner loop missed its desired rate...
    #   原因：ros2 launch 在每行前面加了一层 `[节点名-N] `，_split_prefix 把它
    #   当作节点前缀拆走并翻成中文了，但 ROS 自己那份 `[节点名]:` 还留在正文里，
    #   于是同一个名字出现两遍，白白占掉半行宽度。
    #   只剔"英文/数字/下划线/点 + 冒号"，这样 `[顶升]`、`[导航中]`、`[1/5]`
    #   这些业务标签不会被误伤（它们没有紧跟冒号）。
    body = re.sub(r'^\[[A-Za-z_][A-Za-z0-9_.\-]*\]:\s*', '', body).strip()
    body = _tidy_tail(body)
    body = re.sub(r'\s{2,}', ' ', body)

    out = f'{prefix}{body}' if prefix else body
    return out.strip()


# 翻译后常见的英文尾巴 → 中文（只处理高频且语义确定的，不做机翻）
_TAIL = [
    (r'\bin (\d+(?:\.\d+)?)\s*ms\b', r'（耗时 \1 ms）'),
    (r'\bin (\d+(?:\.\d+)?)\s*s(?:ec(?:onds)?)?\b', r'（耗时 \1 秒）'),
    (r',?\s*begin computing control effort\.?', '，开始计算速度指令'),
    (r',?\s*navigating\.?$', '，正在前往'),
    (r',?\s*starting\.?$', '，正在启动'),
    (r',?\s*done\.?$', '，完成'),
]
_TAIL_RE = [(re.compile(p, re.I), r) for p, r in _TAIL]


def _tidy_tail(body):
    """把翻译后残留的常见英文尾巴换成中文。

    为什么单独一层：规则表是"整句模板"，但很多日志是
    `Planning succeeded in 32ms` 这种"已翻译的主体 + 英文尾巴"。
    在模板里为每种组合都写一条会爆炸，所以统一在末尾收一次。
    """
    for pat, rep in _TAIL_RE:
        body = pat.sub(rep, body)
    return body


def _split_prefix(s):
    """把 `[ERROR] [node-3]:` 这类前缀拆出来。

    返回：(中文前缀, 正文)。识别不出来就返回 ('', 原文)。
    """
    m = re.match(
        r'^\[(?P<lvl>INFO|WARN|WARNING|ERROR|FATAL|DEBUG)\]'
        r'\s*\[(?P<node>[^\]]*)\]:?\s*(?P<rest>.*)$', s, re.I)
    if not m:
        # 没有级别前缀，但可能有 [node-3]: 这种
        m2 = re.match(r'^\[(?P<node>[a-z0-9_]+-\d+)\]:?\s*(?P<rest>.*)$', s, re.I)
        if not m2:
            return '', s
        node = _node_zh(m2.group('node'))
        return f'[{node}] ', m2.group('rest').strip()
    lvl = _LEVEL_ZH.get(m.group('lvl').lower().replace('warning', 'warn'), '信息')
    node = _node_zh(m.group('node'))
    return f'[{lvl}·{node}] ', m.group('rest').strip()


def _node_zh(node):
    """把节点名（可能带 `-3` 序号）整理成人能看懂的中文模块名。

    launch 给节点加的序号（map_server-3）对我们没意义，去掉更快读。
    ★ 但保留节点身份很重要：同一个问题来自 amcl 还是 controller_server，
      排查方向完全不同。所以"去序号"不等于"去名字"。
    """
    if not node:
        return '系统'
    # ★ 节点名可能是 `spawn_entity.py-3` 这种"带扩展名 + 序号"的形式，
    #   所以先去 -N 序号，再查中文名。只去掉末尾的序号，
    #   中间的数字（如 nav2_）属于名字本身，不能动。
    name = re.sub(r'-\d+$', '', node).strip()
    # 去掉 .py 后缀便于查表（但查不到时要把原样还回去）
    bare = re.sub(r'\.py$', '', name)
    # 先按带后缀的名字查（spawn_entity.py），再按去后缀的查（spawn_entity）
    role = _ROLE.get(name) or _ROLE.get(bare)
    return role or name


# 常见节点 → 中文角色名（模块级常量：日志是高频路径，不能在函数里重建字典）
# 查不到就保留原名，绝不硬翻 —— 认不出的名字至少还能搜。
_ROLE = {
    'web_gateway': '网页网关',
    'map_server': '地图服务',
    'amcl': '定位',
    'planner_server': '路径规划',
    'controller_server': '运动控制',
    'bt_navigator': '导航调度',
    'behavior_server': '恢复行为',
    'slam_toolbox': '建图',
    'async_slam_toolbox_node': '建图',
    'robot_state_publisher': '车体模型',
    'velocity_arbiter': '速度仲裁',
    'task_manager': '任务管理',
    'waypoint': '点位',
    'cargo': '搬货',
    'rviz2': '可视化',
    'gzserver': '仿真',
    'gzclient': '仿真界面',
    'ekf_node': '位置融合',
    'spawn_entity.py': '生成模型',
    'launch': '启动器',
    'launch_service': '启动器',
    'controller_manager': '控制器',
    'lifecycle_manager': '生命周期',
}


def translate_lines(lines, max_len=300):
    """批量处理：过滤噪音 + 翻译 + 截断。

    参数：lines - 原始行列表；max_len - 单行最长字符数
    返回：list[(level, text)]，level 是 'info'/'warn'/'error'
    """
    out = []
    for raw in lines:
        s = (raw or '').rstrip()
        if is_noise(s):
            continue
        lvl = guess_level(s)
        txt = translate(s)
        if not txt:
            continue
        if len(txt) > max_len:
            txt = txt[:max_len] + '…'
        out.append((lvl, txt))
    return out


# 给"启动/退出"这类我们自己产生的事件用的现成文案
MSG_STACK_STARTING = '正在启动{label}…（首次启动约需 15~40 秒）'
MSG_STACK_READY = '{label}已就绪'
MSG_STACK_EXITED = '{label}已退出（返回码 {code}）'
MSG_STACK_EXITED_ERR = '{label}异常退出（返回码 {code}），请看下面几行日志定位原因'
