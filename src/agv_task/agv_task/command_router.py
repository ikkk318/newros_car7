"""
CommandRouter —— 指令路由/门卫（纯 Python，可单独测试）。

职责（接口清单 §2 模块 B3）：所有指令都从 /agv/command 进来，它是门卫：
    ① 校验指令是否在白名单；
    ② 校验参数（goto 坐标交给 GoalValidator 判是否可去 / 必填参数位）；
    ③ 按 request_id 去重（同一请求只受理一次）；
    ④ 把合法指令分派给注册过的处理器。

★ 坐标合法性只有 GoalValidator 一个判据（见 agv_common.goal_check）：
  有实测地图就以地图为准（图外 / 压障碍 / 净空不足直接拒），
  没地图才退回矩形范围。这样"能不能去"不会出现两种结论。

route() 返回 (accepted: bool, reason: str)：accepted=是否受理，reason=中文原因。
`stop` 是最高优先级安全通道：绕过一切排队/去重直接执行（规格书 §4.3）。

本模块不依赖 ROS：msg 只要是带 source/command/args/confidence/request_id 属性的对象即可
（真跑时是 AgvCommand，单测时可用简单替身对象）。
"""

# 合法指令白名单（统一指令语义）
# 注：搬货（顶升/下降）不再走指令总线——它由任务层直接调 /agv/cargo 服务，
#     所以白名单里没有 lift/lower/dock。
VALID_COMMANDS = (                       # def 白名单元组
    'goto', 'start_task', 'cancel_task', 'stop', 'resume',
)

# 需要至少 1 个参数（点名或坐标）的指令
_NEEDS_ARGS = ('goto',)                  # def 必带参数
# 需要至少 2 个参数（起始点, 目标点）的指令
_NEEDS_TWO_ARGS = ('start_task',)        # def 需两个参数


# 把 token 解析成 float（不接受 nan/inf）
# 参数：token - 字符串或数字
# 返回：float；不可解析返回 None
def _parse_float(token):                 # in 内部辅助
    try:
        v = float(str(token).strip())
    except (TypeError, ValueError):
        return None
    if v != v or v in (float('inf'), float('-inf')):   # # 排除 nan/inf
        return None
    return v


class CommandRouter:                     # out 指令路由（门卫）
    # 初始化
    # 参数：validator - 目标点合法性判据（GoalValidator）。★ 这是坐标校验的
    #                   唯一来源；不传则不限坐标（纯逻辑测试用）。
    #       max_seen  - 去重表最多记多少个 request_id（防内存无限增长）
    # 返回：无
    def __init__(self, validator=None, max_seen=1000):   # out 对外构造函数
        self.validator = validator       # def 目标点判据（可为 None）
        self.max_seen = int(max_seen)    # def 去重表上限
        self._handlers = {}              # def 指令名 → 处理器
        self._seen = {}                  # def request_id → 受理与否（插入序保序）
        self._seen_order = []            # def 按插入顺序记 request_id，便于淘汰最旧

    # 内部：坐标是否可去（委托唯一判据 GoalValidator）
    # 参数：x, y - 坐标(m)
    # 返回：(ok, reason)
    def _in_field(self, x, y):                 # in 内部方法
        if self.validator is None:             # # 未配置判据：放行（纯逻辑测试）
            return True, '未配置判据'
        return self.validator.check(x, y)

    # 注册一个指令处理器
    # 参数：name - 指令名（须在白名单）；handler - callable(args, msg) -> (bool, str)
    # 返回：无
    def register(self, name, handler):       # out 对外接口
        self._handlers[name] = handler       # def 登记

    # 主入口：校验 + 去重 + 分派
    # 参数：msg - 带 command/source/args/confidence/request_id 的消息对象
    # 返回：(accepted: bool, reason: str)
    def route(self, msg):                    # out 对外接口
        command = (getattr(msg, 'command', '') or '').strip().lower()   # use 规整指令名
        args = list(getattr(msg, 'args', []) or [])                     # use 参数列表
        request_id = str(getattr(msg, 'request_id', '') or '').strip()  # use 请求号
        source = str(getattr(msg, 'source', '') or '').strip()          # use 来源
        # ★ 去重键必须带上 source，否则多个入口（网页 / 语音 / 控制台 / 调度）
        #   各自从 1 开始编号时会【互相顶掉】：网页发 request_id="1" 被受理后，
        #   语音再发 "1" 就会被判"重复"直接拒绝——现象是"某个入口偶尔没反应"，
        #   极难排查。带上 source 后各入口的去重空间彼此独立。
        dedup_key = f'{source}:{request_id}' if request_id else ''

        # ① stop 最高优先级：绕过白名单外的排队与去重，直接走安全通道
        if command == 'stop':            # # 安全通道：永远受理
            handler = self._handlers.get('stop')    # use 看有没有注册 stop 处理器
            if handler is not None:      # # 有则执行（例如安全层发零速）
                return handler(args, msg)
            return True, '收到 stop，已执行安全停车'

        # ② 白名单校验
        if command not in VALID_COMMANDS:       # # 未知指令
            return False, f'未知指令「{command}」，已拒绝'
        if command not in self._handlers:       # # 白名单内但无人处理
            return False, f'指令「{command}」暂未实现，已拒绝'

        # ③ 参数校验
        if command in _NEEDS_ARGS and len(args) < 1:      # # 缺参数
            return False, f'指令「{command}」缺少参数（需目标点名或坐标）'
        if command in _NEEDS_TWO_ARGS and len(args) < 2:  # # 缺两个参数
            return False, f'指令「{command}」缺少参数（需起始点与目标点）'

        if command == 'goto':            # # goto 若是坐标，需校验是否越界
            ok, reason = self._check_goto_args(args)
            if not ok:                   # # 越界/格式坏：拒绝
                return False, reason

        # ④ 去重（同一个 source 内，同一个 request_id 只受理一次）
        if dedup_key:                    # # 请求号非空才去重（空号=匿名请求，不去重）
            if dedup_key in self._seen:  # # 见过的请求号
                return False, f'请求「{request_id}」重复，已拒绝'

        accepted, reason = self._dispatch(command, args, msg)   # use 分派给处理器

        if dedup_key:                    # # 记下去重（无论受理与否都记，避免重试风暴）
            self._remember(dedup_key)
        return accepted, reason

    # 内部：校验 goto 参数。args 可能是 [点名]，也可能是 [x, y, yaw] 坐标串
    # 参数：args - 参数列表
    # 返回：(ok, reason)
    def _check_goto_args(self, args):        # in 内部方法
        if len(args) == 1:               # # 单参数：一律当点名（点名可以是纯数字如「666」）
            name = str(args[0]).strip()
            if not name:                 # # 空点名
                return False, 'goto 目标点名不能为空'
            return True, '点名合法'
        if len(args) in (2, 3):          # # 两个/三个：当坐标（yaw 可省）
            x = _parse_float(args[0])
            y = _parse_float(args[1])
            if x is None or y is None:   # # 坐标解析失败/含 nan/inf
                return False, 'goto 坐标格式非法，应为 [x, y] 或 [x, y, yaw]'
            if len(args) == 3 and _parse_float(args[2]) is None:
                return False, 'goto 坐标格式非法，应为 [x, y] 或 [x, y, yaw]'
            if self.validator is not None:    # # ★ 交给唯一判据（地图优先）
                ok, reason = self.validator.check(x, y)
                if not ok:
                    # 判据给的串通常已含"已拒绝"（如 MapChecker），
                    # 这里只在缺了的时候补一次，免得打印成"…，已拒绝，已拒绝"
                    if '已拒绝' not in reason:
                        reason += '，已拒绝'
                    return False, reason
                return True, '坐标合法'
            return True, '坐标合法（未配置判据，放行）'
        return False, 'goto 参数不合法'

    # 内部：分派给已注册的处理器
    # 参数：command - 指令名；args - 参数；msg - 原始指令消息（处理器可读 route/source）
    # 返回：(accepted, reason)
    def _dispatch(self, command, args, msg):     # in 内部方法
        handler = self._handlers.get(command)   # use 取处理器
        if handler is None:              # # 理论上前面已挡，双保险
            return False, f'指令「{command}」无处理器'
        try:
            result = handler(args, msg)  # use 调用处理器
            if isinstance(result, tuple) and len(result) == 2:
                return bool(result[0]), str(result[1])
            return bool(result), ''      # # 处理器只返回 bool 也兼容
        except Exception as exc:         # # 处理器抛异常：不让它拖垮路由
            return False, f'指令「{command}」执行异常：{exc}'

    # 内部：记录 request_id（超上限淘汰最旧的）
    # 参数：request_id - 请求号
    # 返回：无
    def _remember(self, request_id):         # in 内部方法
        self._seen[request_id] = True        # def 标记见过
        self._seen_order.append(request_id)  # def 记插入序
        while len(self._seen_order) > self.max_seen:   # # 超上限淘汰最旧
            old = self._seen_order.pop(0)
            self._seen.pop(old, None)
