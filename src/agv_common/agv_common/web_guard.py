"""
web_guard —— 网页控制台的准入与并发控制（纯逻辑，不依赖 ROS / 不依赖 HTTP 框架）。

**为什么需要它**
网页控制台允许多台设备（手机 + 电脑）同时打开。直接放开会有两个真实事故：
  ① 两台设备同时按方向键 → 速度指令互相覆盖，车在原地抽搐；
  ② 一个人点"导航"，另一个人同时在遥控 → 指令打架。
所以必须有一个"方向盘"：同一时刻只有一台设备能操作，其余只读。

**它管两件事**
  ① 认证：口令 + 可选 IP 白名单 —— 局域网不等于安全，挡住误入的人；
  ② 控制权锁：谁先操作谁拿锁，闲置超时自动释放，别人可接管。

**设计取舍**
- **闲置超时用"最后一次有效操作"计时，而不是连接时长**：手机锁屏 / 切后台会断
  WebSocket，但用户回来还想接着控。按操作计时更符合直觉，也能自动回收"人走了
  却还挂着"的锁。
- **遥控有独立的心跳续期**：按住方向键期间必须持续续期（否则 3 s 就到点释放，
  车在手按着的时候被别人抢走，很危险）。所以 `touch()` 与 `renew()` 分开：
  `touch()` 用于"开始操作/点了按钮"，`renew()` 用于"我还按着"。
- **认证失败与锁失败要返回不同错误**：前端据此决定"重新登录"还是"显示被谁占用"。
"""

import time                              # use 单调时钟（不受系统改时间影响）


# 认证 / 控制权的结果码：前端按它决定怎么提示
OK = 'ok'                                # def 通过
BAD_TOKEN = 'bad_token'                  # def 口令错
BAD_IP = 'bad_ip'                        # def IP 不在白名单
LOCKED_BY_OTHER = 'locked_by_other'      # def 方向盘被别人占着


class AuthGate:                          # out 准入闸门（口令 + IP 白名单）
    # 初始化：设定口令与白名单
    # 参数：token     - 访问口令；空串表示不校验口令（不建议，仅供本地调试）
    #       allow_ips - 允许的来源 IP 前缀列表（空列表=不限制）；匹配用 startswith，
    #                   这样写 '192.168.' 就能放行整个网段，'127.0.0.1' 精确匹配本机
    # 返回：无
    def __init__(self, token='', allow_ips=None):     # out 对外构造函数
        self.token = str(token or '')                 # def 口令
        self.allow_ips = [str(x) for x in (allow_ips or []) if str(x)]   # def IP 白名单

    # 校验来源 IP 是否在白名单内
    # 参数：ip - 客户端 IP 字符串
    # 返回：(ok, code)
    def check_ip(self, ip):                            # out 对外接口
        if not self.allow_ips:                         # # 没配白名单：不限制
            return True, OK
        ip = str(ip or '')                             # use 规整入参
        for prefix in self.allow_ips:                  # use 逐个前缀比对
            if ip.startswith(prefix):
                return True, OK
        return False, BAD_IP

    # 校验口令
    # 参数：token - 客户端提交的口令
    # 返回：(ok, code)
    def check_token(self, token):                       # out 对外接口
        if not self.token:                              # # 未设口令：放行（调试模式）
            return True, OK
        if str(token or '') == self.token:              # # 口令一致
            return True, OK
        return False, BAD_TOKEN

    # 一次性校验（IP + 口令）
    # 参数：ip - 客户端 IP；token - 客户端口令
    # 返回：(ok, code)
    def check(self, ip, token):                         # out 对外接口
        ok, code = self.check_ip(ip)                    # use 先看来源
        if not ok:
            return False, code
        return self.check_token(token)                  # use 再看口令


class ControlLease:                      # out 控制权锁（同一时刻只一台设备能操作）
    # 初始化
    # 参数：idle_timeout - 闲置多少秒后自动释放(s)；<=0 表示不自动释放
    #       clock        - 取当前时间的函数（测试可注入假时钟）
    # 返回：无
    def __init__(self, idle_timeout=3.0, clock=None):  # out 对外构造函数
        self.idle_timeout = float(idle_timeout)         # def 闲置释放阈值(s)
        self._clock = clock or time.monotonic           # def 时钟（可注入）
        self._owner = None                              # def 当前持有者标识
        self._last = 0.0                                # def 最后一次操作时刻

    # 当前持有者（顺带处理超时释放）
    # 参数：无
    # 返回：持有者标识或 None
    def owner(self):                                   # out 对外接口
        self._expire()                                 # use 先看是否已该释放
        return self._owner

    # 内部：闲置超时就清掉持有者
    # 参数：无
    # 返回：无
    def _expire(self):                                 # in 内部方法
        if self._owner is None:                        # # 没人持有
            return
        if self.idle_timeout <= 0.0:                   # # 不自动释放
            return
        if (self._clock() - self._last) >= self.idle_timeout:   # # 闲置超时
            self._owner = None                         # use 释放
            self._last = 0.0

    # 请求控制权（无主则拿到，自己已持有则续期，别人持有则被拒）
    # 参数：who - 设备标识（例如前端生成的 session id）
    # 返回：(ok, code, owner)；code 为 OK 或 LOCKED_BY_OTHER
    def acquire(self, who):                            # out 对外接口
        who = str(who or '').strip()
        if not who:                                    # # 没有标识：不给锁
            return False, LOCKED_BY_OTHER, self.owner()
        self._expire()                                 # use 先处理超时
        if self._owner is None:                        # # 无主：拿到
            self._owner = who
            self._last = self._clock()
            return True, OK, self._owner
        if self._owner == who:                         # # 自己已持有：续期
            self._last = self._clock()
            return True, OK, self._owner
        return False, LOCKED_BY_OTHER, self._owner      # # 被别人占着

    # 续期（按住方向键期间持续调用，防止手还按着却被判闲置释放）
    # 参数：who - 设备标识
    # 返回：True=续期成功；False=已不是持有者（前端应提示"控制权已被接管"）
    def renew(self, who):                              # out 对外接口
        who = str(who or '').strip()
        self._expire()                                 # use 先处理超时
        if self._owner is None:                        # # 超时已释放：顺手重新拿到
            if not who:
                return False
            self._owner = who
            self._last = self._clock()
            return True
        if self._owner != who:                         # # 已被别人拿走
            return False
        self._last = self._clock()                     # use 续期
        return True

    # 释放控制权（前端点"交还"、关页面时调用）
    # 参数：who - 设备标识
    # 返回：True=确实释放了；False=本来就不是持有者
    def release(self, who):                            # out 对外接口
        who = str(who or '').strip()
        self._expire()                                 # use 先处理超时
        if self._owner is not None and self._owner == who:
            self._owner = None                         # use 释放
            self._last = 0.0
            return True
        return False                                   # # 非持有者：啥也不做

    # 强制收回（急停 / 管理员操作）
    # 参数：无
    # 返回：无
    def force_release(self):                           # out 对外接口
        self._owner = None
        self._last = 0.0

    # 查当前状态（给前端显示"谁在控制 / 还能空多久"）
    # 参数：无
    # 返回：dict {owner, idle_for, idle_timeout, free}
    def state(self):                                   # out 对外接口
        self._expire()                                 # use 先处理超时
        if self._owner is None:                        # # 无主
            return {'owner': None, 'idle_for': 0.0,
                    'idle_timeout': self.idle_timeout, 'free': True}
        idle = self._clock() - self._last             # def 已闲置时长
        return {'owner': self._owner,                 # def 当前持有者
                'idle_for': round(idle, 3),           # def 已闲置秒数
                'idle_timeout': self.idle_timeout,    # def 阈值
                'free': False}


class TeleopWatchdog:                    # out 遥控看门狗（网络抖动时保证车停得住）
    # 初始化
    # 参数：stale_timeout - 多久没收到遥控就判"前端掉线"(s)，应小于 ROS 侧的
    #                       cmd_timeout（本项目 0.5 s），这样由网页这边先收手
    # 返回：无
    def __init__(self, stale_timeout=0.4, clock=None):  # out 对外构造函数
        self.stale_timeout = float(stale_timeout)        # def 判超时阈值(s)
        self._clock = clock or time.monotonic            # def 时钟（可注入）
        self._last = 0.0                                 # def 最近一次收到遥控的时刻
        self._active = False                             # def 是否处于遥控中

    # 收到一帧遥控：记为活跃
    # 参数：无
    # 返回：无
    def feed(self):                                    # out 对外接口
        self._last = self._clock()
        self._active = True

    # 是否应输出零速（遥控中但已超时 = 前端掉线）
    # 参数：无
    # 返回：True=该发零速
    def should_stop(self):                             # out 对外接口
        if not self._active:                           # # 本来就没在遥控
            return False
        return (self._clock() - self._last) >= self.stale_timeout

    # 主动结束遥控（前端松手 / 点停车）
    # 参数：无
    # 返回：无
    def idle(self):                                    # out 对外接口
        self._active = False
        self._last = 0.0
