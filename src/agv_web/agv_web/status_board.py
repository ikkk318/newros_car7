"""
status_board —— 顶部"实时数据"面板：固定仪表槽位 + 日志降噪。

**为什么要重做（用户的两次原话）**

  第一次："运行日志打印速度太快根本看不过来，能不能在顶部增加一个数据面板，
          对于总是容易重复触发的打印添加到面板里面，每次触发打印只替换变化的
          文字或数值，并且增加打印的时间间隔。"

  第二次（重做的原因）："数据面板显示的日志是固定死的，日志长的话看不到后面的
          信息。……数据面板，就显示一些重点状态标志。"

第一版把面板做成了"日志镜像"：把日志原文按 key 塞进格子，格子内容随原文变长。
真实环境里避障一行 60~80 字，360px 宽的侧栏一格只能放约 32 字 —— 最该看的
"减到多少"恰好被截掉，而且**没有任何查看全文的入口**。用户说的"固定死"就是
这个：格子被第一版写入什么，就一直显示什么，值不再更新，也展不开。

**这一版的分工（彻底切开，不再互相依赖）**

  ① **仪表槽位（本模块 build_gauges）**：写死 8 个槽位 ——
     任务 / 速度 / 避障 / 定位 / 连接 / 地图 / 模式。
     每个槽位的值**直接取自网关已有的状态量**（任务状态、/cmd_vel_raw、
     激光最近距离、TF 新鲜度、WebSocket 客户端数、地图几何、工作模式），
     **不再从日志文本里猜**。所以：
       · 文本天生短（10~20 字），长度问题不存在；
       · 值每秒刷新，不会"固定死"；
       · 数据源断开时能如实显示"——"（而不是把几百年前的原文挂在那里）。

  ② **日志过滤器（本模块 LogFilter）**：只干一件事 —— 判断这条日志
     "该不该留在日志列表里"。三类处置：
       · 事件（顶升开始/结束、任务完成、换栈、急停…）→ 永远留；
       · 已知高频状态（避障、代价地图、地图扩张…）→ **限流留存**
         （默认 5~30 秒一条），既保留可回看的痕迹，又不冲掉历史；
       · 没预料到的高频重复 → 连续超过 N 次后折叠，只提示一次。
     注意它**不再向面板写文本** —— 面板的值来自状态量，不来自日志。

为什么把这两件事放在一个文件而不是两个模块：它们的判据表是同一份
（"哪些日志属于高频状态"），分开会各写一遍，迟早不一致。但两者的
数据流是单向的（日志 → 过滤器 → 日志列表；状态 → 仪表 → 前端），
互不影响，所以放在一个文件里也不会重新耦合。

**设计约束**：纯逻辑、不 import rclpy，可单测（和 log_translate 一样）。
"""

import re
import time

# ══════════════════════════ 一、仪表槽位定义 ══════════════════════════
# (key, 标题)。**顺序即显示顺序**，前端不再排序 —— 固定槽位的好处就是
# 位置稳定：用户扫一眼就知道在哪看速度、在哪看定位，不用每次找。
#
# 为什么不加更多槽位：手机上一屏侧栏放得下 8 行（每行约 34px）。加第 9 行
# 就要滚动，而"要滚动的仪表"等于没仪表。真需要新指标时，替换掉最不常用的
# 那个，别往下堆。
#
# ★ 'act'（动作）是第 8 行，2026-10-18 用户明确要求"最重要的一项：载货状态"。
#   为守住"一屏放得下"的约束，同时把 'link'（连接）挪到最后一格 ——
#   连接状态只在断线时才需要看，放末尾不影响日常扫读；而动作状态是
#   判断"车上有没有货"的唯一依据，必须靠前（紧跟任务之后）。
#   实测仍是一屏：移动端 ≤860px 排两列，8 行正好 4 行 × 2 列。
GAUGES = [
    ('task',  '任务'),
    ('act',   '动作'),      # 取货中 / 载货中 / 无动作
    ('speed', '速度'),
    ('avoid', '避障'),
    ('pose',  '定位'),
    ('map',   '地图'),
    ('mode',  '模式'),
    ('link',  '连接'),
]
# 动作状态取值（业务语义，前端/测试都按这三个字面量判断）
ACT_LOADING = '取货中'      # 搬运任务在跑，但还没顶升成功（含前往取货点）
ACT_CARRY = '载货中'        # 顶升成功之后、下降成功之前
ACT_IDLE = '无动作'         # 没有任何搬运任务在跑（纯导航 / 空闲）

# 值的色调：前端按这个上色（ok 绿 / warn 黄 / error 红 / off 灰）。
# 不用具体颜色值，避免和后端主题耦合。
TONE_OK = 'ok'
TONE_WARN = 'warn'
TONE_ERR = 'error'
TONE_OFF = 'off'

# 空值占位。用"——"而不是空串：空串会让前端不知道该删节点还是留空，
# 而且"格子消失了"和"这一项暂时无数据"是两件事，视觉上必须可区分。
DASH = '——'

# 避障判定：激光前方扇区半角（度）。±25° 约等于车体宽度在 1m 处的张角，
# 再宽会把侧墙当成"正前方障碍"，车停在原地不敢走。
_AVOID_HALF_DEG = 25.0
# 减速起点（m）。和 velocity_arbiter 的 slow_distance 默认值一致；
# 这里**只用于显示措辞**（"畅通/减速"），不参与任何控制判断。
_AVOID_SLOW_M = 0.55


def front_distance(scan, half_deg=_AVOID_HALF_DEG):
    """从一帧 LaserScan 算"正前方最近障碍距离"。

    参数：scan - dict，含 ranges / angle_min / inc / range_min / range_max
                  （网关 on_scan 缓存的就是这个形状）
         half_deg - 扇区半角（度）
    返回：float 距离(m)；没有有效点就返回 None。

    ★ 为什么自己算而不去听 velocity_arbiter 的日志：
      ① 仲裁节点只在"接近障碍"时才打印，远离障碍时没有任何输出，
         于是面板永远停在最后一帧"前方 0.52m"，用户以为障碍还在
         （这正是"面板固定死"的一个来源）；
      ② 建图模式根本没有仲裁节点，但避障照样要显示。
      直接读激光是唯一的"始终有值"的来源。

    ★ 为什么要过滤 inf / nan：很多驱动的 LaserScan 里，超出量程的点是
      inf（不是 range_max），直接 min() 会得到 inf；个别驱动给 nan。
      这两种都必须当"无回波"丢掉，否则 min 会变成 inf 或 nan，
      面板显示 "infm" 或 "nan"，看起来像程序坏了。
    """
    if not scan:
        return None
    ranges = scan.get('ranges') or []
    if not ranges:
        return None
    try:
        a0 = float(scan.get('angle_min', 0.0))
        inc = float(scan.get('inc', 0.0))
        rmin = float(scan.get('range_min', 0.0) or 0.0)
        rmax = float(scan.get('range_max', 0.0) or 0.0)
    except (TypeError, ValueError):
        return None
    if inc == 0.0:
        return None
    half = abs(half_deg) * 3.141592653589793 / 180.0
    best = None
    for i, r in enumerate(ranges):
        ang = a0 + i * inc
        # 角度先归一化到 [-pi, pi]，否则 angle_min=-3.14 时正前方会算到别处
        while ang > 3.141592653589793:
            ang -= 2 * 3.141592653589793
        while ang < -3.141592653589793:
            ang += 2 * 3.141592653589793
        if ang > half or ang < -half:
            continue
        try:
            v = float(r)
        except (TypeError, ValueError):
            continue
        if v != v or v in (float('inf'), float('-inf')):   # nan / ±inf
            continue
        if v <= 0.0:
            continue
        if rmin > 0.0 and v < rmin:
            continue
        if rmax > 0.0 and v > rmax:
            continue
        if best is None or v < best:
            best = v
    return best


# ══════════════════════════ 二、任务状态文案 ══════════════════════════
# 状态枚举 → 中文。前端也有一份（TASK_STATE_ZH），这里这份只用于**仪表文案**：
# 仪表要的是"搬运中 2/5"这种短标签，和前端的"取货中"措辞可以不同步，
# 但枚举覆盖必须完整，否则漏掉的状态会显示成英文。
_TASK_ZH = {
    'IDLE': '空闲',
    'GO_LOAD': '去取货',
    'LIFTING': '顶升中',
    'TRANSPORT': '运输中',
    'LOWERING': '下降中',
    'DONE': '本单完成',
    'ERROR': '任务出错',
}


def _gauge(key, text, tone=TONE_OK):
    """造一个槽位。title 从 GAUGES 取，保证标题只在 GAUGES 里维护一份。"""
    for k, label in GAUGES:
        if k == key:
            return {'key': key, 'label': label, 'text': text, 'tone': tone}
    return {'key': key, 'label': key, 'text': text, 'tone': tone}


def _fmt_sec(v):
    """秒数显示：小于 10 秒给一位小数，大了给整数。仪表位置寸土寸金。"""
    try:
        v = float(v)
    except (TypeError, ValueError):
        return DASH
    if v < 0:
        return DASH
    return f'{v:.1f}s' if v < 10 else f'{v:.0f}s'


def build_gauges(state):
    """把网关的状态字典编成 8 个仪表槽位。

    参数：state - dict，可含以下键（缺一个不影响其它，全缺也能出 8 行）：
        pose_age     float  位姿数据多久没更新(s)；None = 从没拿到过
        pose_ok      bool   位姿是否新鲜
        task_state   dict   最近一条 TaskState（含 state/target/progress/message）
        task_running bool   任务缓冲是否在执行
        task_idx     int    当前第几单（1 起）
        task_total   int    总共几单
        cargo_busy   bool   搬货服务报的"正在载货"（/agv/cargo/busy）
        speed        (v, w) 或 None  底盘实际速度（/cmd_vel_raw）
        front        float  正前方最近障碍(m)；None = 无数据
        avoid_on     bool   仲裁是否在减速/接管（来自日志限流器）
        ws_n         int    已连接的网页客户端数
        ws_lost      bool   是否刚断线（前端断线时后端并不知道，由心跳判）
        map_meta     dict   地图几何（含 width/height/resolution）
        mode         str    'nav' / 'mapping'
        manual       bool   手动模式
        estop        bool   急停

    返回：list[dict]，顺序与 GAUGES 一致，每项 {key,label,text,tone}。

    ★ 为什么用"一个大 dict 进、固定长度 list 出"：
      前端要的是**每帧完整的一屏**（固定行程），而不是增量。
      固定长度让前端可以永久复用 8 个 DOM 节点、只改文字，
      不需要任何增删逻辑 —— 这是"不闪、不跳"的前提。
    """
    st = state or {}
    out = []

    # ── 任务 ──
    # 优先级：正在跑缓冲 > task_manager 报的状态 > 空闲。
    # 缓冲在跑时 task_state 可能还是上一单的 DONE（收尾没回 IDLE），
    # 只看 task_state 会显示"本单完成"却其实还有 3 单在排队。
    ts = st.get('task_state') or {}
    tstate = str(ts.get('state') or '')
    running = bool(st.get('task_running'))
    idx = st.get('task_idx') or 0
    total = st.get('task_total') or 0
    prog = f' {idx}/{total}' if (running and total) else ''
    if st.get('estop'):
        out.append(_gauge('task', '急停中', TONE_ERR))
    elif running:
        out.append(_gauge('task', '搬运中' + prog, TONE_OK))
    elif tstate in ('GO_LOAD', 'LIFTING', 'TRANSPORT', 'LOWERING'):
        out.append(_gauge('task', _TASK_ZH.get(tstate, tstate), TONE_OK))
    elif tstate == 'ERROR':
        out.append(_gauge('task', '任务出错', TONE_ERR))
    else:
        out.append(_gauge('task', '空闲', TONE_OFF))

    # ── 动作（取货中 / 载货中 / 无动作）──
    #
    # ★ 用户定义（原话）：
    #     取货中 = 搬货任务里"顶升之前"或"下降之后"
    #     载货中 = "顶升之后"或"下降之前"
    #     无动作 = 前往导航点、无任务……
    #   翻译成状态机：
    #     GO_LOAD（前往取货点）、LIFTING（顶升还没成功）→ 取货中
    #     TRANSPORT（顶升成功、运输中）、LOWERING（正在下降，货还没落地）→ 载货中
    #     其余（IDLE/DONE/ERROR/纯导航）→ 无动作
    #
    # ★ 为什么必须靠 task_id 区分：**导航任务和搬运任务共用 GO_LOAD**
    #       task_manager.GO_LOAD   = 前往取货点（搬运）
    #       goto_controller.GO_LOAD = 导航中（单点导航）
    #   两个状态机报上来的字符串一模一样，只有 task_id 能分开
    #   （单点导航上报时 task_id 恒为空串，见 state_reporter.compose）。
    #   不判 task_id 的话，纯导航会一直显示"取货中"，是错的。
    #
    # ★ cargo_busy 的作用：状态串在 ERROR 时无法判断货上没上。搬货服务自己
    #   维护的 busy 标志是唯一权威的"载货"事实，急停/出错时用它兜底，
    #   免得用户以为货已经放下、结果开动时车上还压着东西。
    cargo_busy = bool(st.get('cargo_busy'))
    is_transport = bool(ts.get('task_id'))    # 有 task_id = 这单是搬运，不是纯导航
    if tstate == 'ERROR' and is_transport:
        # 出错：状态机说不清货在哪，只能听搬货服务的
        out.append(_gauge('act', ACT_CARRY if cargo_busy else ACT_LOADING,
                          TONE_ERR))
    elif tstate in ('TRANSPORT', 'LOWERING') and is_transport:
        out.append(_gauge('act', ACT_CARRY, TONE_WARN))
    elif tstate in ('GO_LOAD', 'LIFTING') and is_transport:
        out.append(_gauge('act', ACT_LOADING, TONE_OK))
    elif running:
        # 缓冲在跑但本单 TaskState 还没到（第一帧的极短窗口）→ 按取货中
        out.append(_gauge('act', ACT_LOADING, TONE_OK))
    elif cargo_busy:
        # 任务已结束但服务还说载货：真实存在的情形（上一单没下降成功）。
        # 不能显示"无动作"，那是安全事故的起点。
        out.append(_gauge('act', ACT_CARRY, TONE_ERR))
    else:
        # 纯导航 / 空闲 → 车没在搬东西
        out.append(_gauge('act', ACT_IDLE, TONE_OFF))

    # ── 速度 ──
    # 订阅的是 /cmd_vel_raw（底盘真正收到的指令），不是遥控下发的值：
    # 这样"网页发了 0.3 但仲裁减到 0.12"能在仪表上直接看出来。
    sp = st.get('speed')
    if not sp:
        out.append(_gauge('speed', DASH, TONE_OFF))
    else:
        try:
            v, w = float(sp[0]), float(sp[1])
        except (TypeError, ValueError, IndexError):
            out.append(_gauge('speed', DASH, TONE_OFF))
        else:
            # 0.00 一律显示"停止"：静止时满屏 "0.00 / 0.00" 没信息量
            if abs(v) < 0.005 and abs(w) < 0.005:
                out.append(_gauge('speed', '停止', TONE_OFF))
            else:
                out.append(_gauge('speed', f'{v:.2f} · {w:.2f}', TONE_OK))

    # ── 避障 ──
    # 距离来自激光（永远有值），"减速"来自仲裁日志（说明它真的在干预）。
    front = st.get('front')
    if front is None:
        out.append(_gauge('avoid', '无激光', TONE_OFF))
    else:
        try:
            fv = float(front)
        except (TypeError, ValueError):
            fv = None
        if fv is None:
            out.append(_gauge('avoid', '无激光', TONE_OFF))
        elif st.get('avoid_on'):
            # 仲裁介入 → 这是"车正在被限制"，必须显眼
            out.append(_gauge('avoid', f'{fv:.2f}m 减速', TONE_WARN))
        elif fv < _AVOID_SLOW_M:
            # 距离已进减速区但仲裁没报（建图模式没仲裁）→ 照样提示
            out.append(_gauge('avoid', f'{fv:.2f}m 接近', TONE_WARN))
        else:
            out.append(_gauge('avoid', f'{fv:.2f}m 畅通', TONE_OK))

    # ── 定位 ──
    # ★ 判"丢失"用数据年龄，不能用"查没查到"：tf2 Buffer 缓存 10 秒历史，
    #   发布者死了 lookup 仍返回最后一个变换（stamp 很旧），看着一切正常。
    #   这是"小车停在切换前位置"那个老 bug 的同源问题。
    if st.get('pose_ok'):
        out.append(_gauge('pose', '正常', TONE_OK))
    else:
        age = st.get('pose_age')
        if age is None:
            out.append(_gauge('pose', '未就绪', TONE_WARN))
        else:
            out.append(_gauge('pose', '丢失 ' + _fmt_sec(age), TONE_WARN))

    # ── 地图 ──
    meta = st.get('map_meta') or {}
    if meta.get('width') and meta.get('height'):
        # 格数比米数信息量大得多（用户在网页上看到的就是多少格），
        # 米数放不下时砍掉 —— 但保留一位小数便于换算
        out.append(_gauge('map', f"{meta['width']}×{meta['height']}", TONE_OK))
    else:
        out.append(_gauge('map', DASH, TONE_OFF))

    # ── 模式 ──
    # 模式缺失不该显示成 "—— · 自动"（看着像 bug），整格退化成占位
    mode = str(st.get('mode') or '')
    if not mode:
        out.append(_gauge('mode', DASH, TONE_OFF))
    else:
        mz = {'nav': '导航', 'mapping': '建图'}.get(mode, mode)
        who = '手动' if st.get('manual') else '自动'
        tone = TONE_WARN if st.get('manual') else TONE_OK
        out.append(_gauge('mode', f'{mz} · {who}', tone))

    # ── 连接（挪到最后一格）──
    # ★ 为什么连接放最后：它只在断线时才需要看，日常扫读不关心；
    #   而"动作"（有没有载货）是安全信息，必须靠前。
    #   两个槽位的排序调整是同一次改动（7→8 槽），别只改一半。
    if st.get('ws_lost'):
        out.append(_gauge('link', '重连中', TONE_ERR))
    else:
        n = int(st.get('ws_n') or 0)
        if n <= 0:
            # 后端没人在线：页面还在（说明是别的设备/刚断），照样标 warn
            out.append(_gauge('link', '无连接', TONE_WARN))
        else:
            out.append(_gauge('link', f'已连接{(" ×" + str(n)) if n > 1 else ""}',
                              TONE_OK))

    return out


# ══════════════════════════ 三、日志过滤器 ══════════════════════════
# 每项：(正则, 面板 key（仅供计数/调试，不写面板）, 最小留存间隔(s), 说明)
#
# ★ 判据：**这条日志会以秒级频率重复，且用户只关心"现在是多少"**。
#   反例：'导航目标被中止' 可能重复，但每次都是有意义的事件，留在日志。
# ★ 间隔为什么是"留存间隔"而不是"丢弃"：用户原话是"增加打印的时间间隔"，
#   不是"别打了"。隔几秒留一条，出事时还能翻到"当时前方 0.4m"的痕迹；
#   全丢的话面板一旦没显示就彻底没证据了。
_THROTTLE = [
    # 避障：1 Hz，最长。5 秒足够看清一次绕障过程，又不冲历史。
    (r'\[避障\]', 'avoid', 5.0),
    # 代价地图：等待/重算会连续几十条，30 秒留一条（Nav2 启动阶段的事）
    (r'(?:Timed out waiting for|等待代价地图超时)', 'costmap', 30.0),
    (r'(?:costmap.*(?:resize|update)|代价地图已更新|代价地图尺寸调整)', 'costmap', 30.0),
    # Nav2 的周期性碎碎念：信息量为零，但每秒都在刷。
    # ★ 这几条在真车日志里占了近一半行数（用户贴的 2.5 分钟日志里
    #   "Passing new path" 10 条、"Planner loop missed" 8 条），
    #   而且全是"性能跟不上"的同一句话，重复看没有丝毫新信息。
    #   不做全丢弃：留 30~60 秒一条，真出现持续掉帧仍然看得出趋势。
    (r'Passing new path to controller', 'nav_path', 30.0),
    (r'Planner loop missed its desired rate', 'nav_rate', 60.0),
    (r'Behavior Tree tick rate', 'nav_bt', 60.0),
    # 建图中地图扩张：一场建图几十条，15 秒留一条就能看出"还在长"
    (r'建图中：地图扩张到', 'map_grow', 15.0),
    # 速度链路：超时补零速是本项目的常态告警，10 秒一条
    (r'(?:cmd_vel|twist)\s*timeout|速度指令超时', 'cmd_timeout', 10.0),
    (r'(?:Publishing zero|已发零速)', 'zero_vel', 10.0),
    # 连接：断线重连一次会连打几十条
    (r'WebSocket 断开', 'link', 10.0),
    (r'(?:连接断开|正在自动重连|重新连上)', 'link', 10.0),
    (r'遥控心跳超时', 'heartbeat', 10.0),
    # 定位：丢失期间每次刷新都判一次，15 秒一条
    (r'(?:定位丢失|定位未就绪)', 'pose', 15.0),
    (r'位姿解析异常', 'pose_err', 15.0),
]


class _Rules:
    """规则编译一次（日志是高频路径，不能每次 re.compile）。"""

    def __init__(self):
        self.throttle = [(re.compile(p, re.I), k, sec) for p, k, sec in _THROTTLE]


_R = _Rules()

# 数字归一化：把 0.35 / -1.2 / 17 都换成 #，用于识别"同一条消息只是数值不同"。
# ★ 小数点、负号、科学计数都要吃掉，否则 0.3 / 0.30 会被当成两条不同消息。
_NUM_RE = re.compile(r'[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?')

# 事件词：带这些词的消息**永远**留在日志。
# ★ 这是端到端测试抓出来的真 bug：兜底逻辑按"数字归一化后同一条消息反复出现"
#   判刷屏，而 `[1/5] 搬运完成` / `[2/5] 搬运完成` 归一化后完全一样，
#   批量任务连跑几单会在几毫秒内出现 5 次 —— 后 3 条被折叠掉，
#   用户翻历史只看得到 2 条。恰恰是批量任务时最想看的记录。
_EVENT_WORDS = re.compile(
    r'完成|成功|失败|出错|错误|异常|开始|结束|已保存|已启动|已停止|已取消|'
    r'已中止|急停|取消|中止|超时|就绪|到达|跳过|拒绝|恢复|'
    r'\b(?:done|fail(?:ed|ure)?|error|start(?:ed)?|stop(?:ped)?|'
    r'saved|abort(?:ed)?|cancel(?:l?ed)?|timeout|reached|ready)\b',
    re.I)


def normalize(text):
    """把一行日志里的数字抹平，得到"这条消息的身份"。

    这样 `[避障] 前方 0.35m` 和 `[避障] 前方 0.42m` 归一化后是同一个 key，
    才能统计出"它在反复出现"，而不是被当成几百条不同消息。
    """
    return _NUM_RE.sub('#', str(text or '').strip())


def is_event(text):
    """这条消息是否是"一次性业务事件"（必须留在日志里）。"""
    return bool(_EVENT_WORDS.search(str(text or '')))


class LogFilter:
    """判断一条日志该不该留在日志列表里。

    三种处置：
      · 'log'  正常留存
      · 'drop' 丢弃（已知高频状态的限流窗口内 / 未知重复超过阈值）

    不做的事：**不向面板写文本**。面板的值来自 build_gauges 的状态量，
    日志只是日志 —— 这是第一版"面板固定死"的根因，必须切开。

    参数：
      dup_window  未知重复的计数窗口(s)
      dup_limit   窗口内同一条（数字归一化后）超过这么多次就折叠
      fold_note   折叠时是否补一条提示（默认开：不然用户会以为日志丢了）
    """

    def __init__(self, dup_window=15.0, dup_limit=8, fold_note=True):
        self.dup_window = float(dup_window)
        self.dup_limit = int(dup_limit)
        self.fold_note = bool(fold_note)
        self._last_pass = {}     # key -> 上次放行的 monotonic 时刻（限流用）
        self._recent = {}        # 归一化文本 -> [次数, 末次时刻]
        self._folded = {}        # 归一化文本 -> 已提示过折叠？（只提示一次）
        self._avoid_at = 0.0     # 最近一次"仲裁在干预"的时刻（供仪表判断）

    # ── 对外：喂一行日志，决定去留 ──
    def feed(self, level, text, now=None):
        """处理一行日志。

        返回 {'claim': 'log'|'drop'}（折叠时额外带 'note'）。

        ★★ 判定顺序是"限流规则 → 事件词 → 未知重复兜底"，**不能反过来**。
          这是被测出来的真 bug：`等待代价地图超时（可能刚启动还没就绪）`
          里含"超时"，而"超时"是事件词 —— 事件守卫放在前面就永远放行，
          于是这条 Nav2 每 100ms 刷一次的唠叨根本限不住。
          正确理解是：
            · `_THROTTLE` 是**人工确认过的高频源**，属于专家知识，
              它说"这条会刷屏"就是会刷屏，优先采信；
            · 事件词是**启发式兜底**，只用来保护"没进限流表、但可能
              被兜底去重误伤"的消息（比如 `[1/5] 搬运完成`）。
          所以限流表里既有"超时"又有真事件的词时，以限流表为准。
        """
        now = time.monotonic() if now is None else now
        s = str(text or '').strip()
        if not s:
            return {'claim': 'drop'}

        # ① 已知高频状态：按最小留存间隔放行（优先于事件词，理由见上）
        for pat, key, min_sec in _R.throttle:
            if not pat.search(s):
                continue
            # 避障这条额外记时刻：仪表据此判断"仲裁此刻是否在干预"
            if key == 'avoid':
                self._avoid_at = now
            last = self._last_pass.get(key)
            if last is not None and (now - last) < min_sec:
                return {'claim': 'drop'}
            self._last_pass[key] = now
            return {'claim': 'log'}

        # ② 事件：无条件放行（不参与后面的兜底折叠）
        if is_event(s):
            return {'claim': 'log'}

        # ③ 未知重复：兜底折叠（给"没预料到的高频源"留一条后路）
        norm = normalize(s)
        rec = self._recent.get(norm)
        if rec is None:
            self._recent[norm] = [1, now]
            self._prune(now)
            return {'claim': 'log'}
        if (now - rec[1]) > self.dup_window:      # 窗口外，重新计数
            rec[0], rec[1] = 1, now
            return {'claim': 'log'}
        rec[0] += 1
        rec[1] = now
        if rec[0] <= self.dup_limit:
            return {'claim': 'log'}
        # 超过阈值：丢弃，但**第一次**丢时补一条提示，否则用户会以为日志坏了
        if self.fold_note and not self._folded.get(norm):
            self._folded[norm] = True
            return {'claim': 'log',
                    'note': f'（上面这类消息重复过多，已自动折叠，'
                            f'{self.dup_window:.0f} 秒内只留前 {self.dup_limit} 条）'}
        return {'claim': 'drop'}

    def avoid_engaged(self, now=None, hold=3.0):
        """仲裁此刻是否在减速/接管（供仪表显示"减速"）。"""
        now = time.monotonic() if now is None else now
        return (now - self._avoid_at) <= hold

    def _prune(self, now):
        """`_recent` / `_folded` 只是防刷用的临时表，别让它无限长。"""
        if len(self._recent) > 400:
            dead = [k for k, v in self._recent.items()
                    if (now - v[1]) > self.dup_window]
            for k in dead:
                self._recent.pop(k, None)
        # `_folded` 的 key 来自 `_recent`，跟着一起清，否则长跑会慢慢涨
        if len(self._folded) > 400:
            for k in [k for k in self._folded if k not in self._recent]:
                self._folded.pop(k, None)

    # ── 对外：清空（换栈时必须清）──
    def reset(self):
        """换栈时清空限流状态。

        ★ 必须清：不清的话新栈第一次避障会被旧栈 5 秒窗口吞掉，
          而那一刻正是最需要看到"新栈避障生效了"的时候。
        """
        self._last_pass.clear()
        self._recent.clear()
        self._folded.clear()
        self._avoid_at = 0.0


