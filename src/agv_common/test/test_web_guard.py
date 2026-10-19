"""
web_guard 单元测试：准入闸门（口令 / IP 白名单）+ 控制权锁 + 遥控看门狗。

重点验证三件真会出事故的事：
  ① 局域网只挡误入的人，但口令错必须真的进不来；
  ② 手机和电脑不能同时操作——但"手还按着"绝不能被判闲置而抢走；
  ③ 前端掉线（锁屏 / 切后台 / WiFi 断）后必须判出"该停车"。

★ 用注入的假时钟，不 sleep——超时逻辑必须确定可复现。
"""

from agv_common.web_guard import (
    OK, BAD_TOKEN, BAD_IP, LOCKED_BY_OTHER,
    AuthGate, ControlLease, TeleopWatchdog)


class FakeClock:                          # def 假时钟
    def __init__(self, t=0.0):
        self.t = t
    def __call__(self):
        return self.t
    def advance(self, dt):
        self.t += dt


# ─────────────────────────── ① 准入闸门 ───────────────────────────

def test_auth_token_is_enforced():
    g = AuthGate(token='agv2024')
    assert g.check_token('agv2024')[0] is True
    ok, code = g.check_token('wrong')
    assert ok is False and code == BAD_TOKEN
    # 空口令提交也不行（不能因为客户端没给就放行）
    assert g.check_token('')[0] is False
    assert g.check_token(None)[0] is False


def test_auth_without_token_allows_all():
    g = AuthGate(token='')                 # # 调试模式：不设口令
    assert g.check_token('whatever')[0] is True
    assert g.check_token('')[0] is True


def test_auth_ip_whitelist_prefix():
    g = AuthGate(token='', allow_ips=['192.168.', '127.0.0.1'])
    assert g.check_ip('192.168.1.23')[0] is True      # # 网段前缀匹配
    assert g.check_ip('192.168.31.7')[0] is True      # # 同网段另一台设备（手机）
    assert g.check_ip('127.0.0.1')[0] is True         # # 写入的精确 IP 本身
    assert g.check_ip('127.0.0.10')[0] is True        # # ★ startswith 语义：'127.0.0.1' 也匹配它
    assert g.check_ip('10.0.0.5')[1] == BAD_IP        # # 不在白名单：拒绝
    assert g.check_ip('8.8.8.8')[1] == BAD_IP         # # 公网 IP：拒绝


def test_auth_empty_whitelist_means_no_ip_limit():
    g = AuthGate(token='', allow_ips=[])
    assert g.check_ip('8.8.8.8')[0] is True


def test_auth_check_combines_ip_and_token():
    g = AuthGate(token='t', allow_ips=['192.168.'])
    assert g.check('192.168.1.9', 't')[0] is True
    assert g.check('10.1.1.1', 't')[1] == BAD_IP        # # IP 先被拦
    assert g.check('192.168.1.9', 'bad')[1] == BAD_TOKEN
    # IP 错时不应泄露"口令对不对"（先返回 BAD_IP）
    assert g.check('10.1.1.1', 'bad')[1] == BAD_IP


# ─────────────────────────── ② 控制权锁 ───────────────────────────

def test_lease_first_caller_wins():
    clk = FakeClock()
    lease = ControlLease(idle_timeout=3.0, clock=clk)
    ok, code, owner = lease.acquire('phone')
    assert ok is True and code == OK and owner == 'phone'
    assert lease.owner() == 'phone'


def test_lease_second_device_is_rejected_with_owner():
    clk = FakeClock()
    lease = ControlLease(idle_timeout=3.0, clock=clk)
    lease.acquire('phone')
    ok, code, owner = lease.acquire('pc')
    assert ok is False and code == LOCKED_BY_OTHER
    assert owner == 'phone', '必须告诉前端"现在是谁在控制"'


def test_lease_same_device_reaquires_ok():
    clk = FakeClock()
    lease = ControlLease(idle_timeout=3.0, clock=clk)
    lease.acquire('phone')
    ok, code, _ = lease.acquire('phone')
    assert ok is True and code == OK          # # 自己再点一次不算抢


def test_lease_expires_after_idle_and_others_can_take():
    clk = FakeClock()
    lease = ControlLease(idle_timeout=3.0, clock=clk)
    lease.acquire('phone')
    clk.advance(2.9)
    assert lease.owner() == 'phone', '没到 3 s 不该释放'
    clk.advance(0.2)                          # # 累计 3.1 s
    assert lease.owner() is None, '闲置超时必须自动释放'
    ok, _, owner = lease.acquire('pc')
    assert ok is True and owner == 'pc'       # # 别人可以接管


def test_lease_renew_keeps_it_alive():
    clk = FakeClock()
    lease = ControlLease(idle_timeout=3.0, clock=clk)
    lease.acquire('phone')
    # 模拟"手一直按着"：每 1 s 续一次，跨过 3 s 阈值也不该释放
    for _ in range(10):
        clk.advance(1.0)
        assert lease.renew('phone') is True
    assert lease.owner() == 'phone', '★ 手还按着时绝不能被判闲置而抢走'


def test_lease_renew_fails_after_takeover():
    clk = FakeClock()
    lease = ControlLease(idle_timeout=1.0, clock=clk)
    lease.acquire('phone')
    clk.advance(1.5)                          # # 闲置被释放
    lease.acquire('pc')                       # # 别人拿走
    assert lease.renew('phone') is False, '被接管后续期必须失败，前端据此提示'


def test_lease_release_and_force_release():
    clk = FakeClock()
    lease = ControlLease(idle_timeout=3.0, clock=clk)
    lease.acquire('phone')
    assert lease.release('pc') is False       # # 非持有者释放无效
    assert lease.release('phone') is True
    assert lease.owner() is None
    lease.acquire('pc')
    lease.force_release()                     # # 急停/管理员强收
    assert lease.owner() is None


def test_lease_no_auto_expire_when_timeout_disabled():
    clk = FakeClock()
    lease = ControlLease(idle_timeout=0.0, clock=clk)
    lease.acquire('phone')
    clk.advance(9999.0)
    assert lease.owner() == 'phone', 'timeout<=0 表示不自动释放'


def test_lease_rejects_empty_identity():
    clk = FakeClock()
    lease = ControlLease(idle_timeout=3.0, clock=clk)
    ok, code, _ = lease.acquire('')
    assert ok is False, '没有设备标识不能给控制权（否则无法区分是谁）'


def test_lease_state_reports_idle_for():
    clk = FakeClock()
    lease = ControlLease(idle_timeout=3.0, clock=clk)
    assert lease.state()['free'] is True
    lease.acquire('phone')
    clk.advance(1.25)
    st = lease.state()
    assert st['free'] is False and st['owner'] == 'phone'
    assert abs(st['idle_for'] - 1.25) < 1e-6
    assert st['idle_timeout'] == 3.0


# ─────────────────────────── ③ 遥控看门狗 ───────────────────────────

def test_watchdog_idle_before_any_teleop():
    clk = FakeClock()
    wd = TeleopWatchdog(stale_timeout=0.4, clock=clk)
    assert wd.should_stop() is False, '没在遥控时不该凭空发零速'


def test_watchdog_stops_when_frames_stop():
    clk = FakeClock()
    wd = TeleopWatchdog(stale_timeout=0.4, clock=clk)
    wd.feed()
    clk.advance(0.2)
    assert wd.should_stop() is False, '还在阈值内不该停'
    clk.advance(0.3)                          # # 累计 0.5 s > 0.4
    assert wd.should_stop() is True, '★ 前端掉线必须判出该停车'


def test_watchdog_feed_keeps_it_alive():
    clk = FakeClock()
    wd = TeleopWatchdog(stale_timeout=0.4, clock=clk)
    for _ in range(20):                       # # 持续 20 帧（每 0.1 s）
        wd.feed()
        clk.advance(0.1)
        assert wd.should_stop() is False, '持续收到帧时绝不判超时'


def test_watchdog_idle_clears_state():
    clk = FakeClock()
    wd = TeleopWatchdog(stale_timeout=0.4, clock=clk)
    wd.feed()
    clk.advance(1.0)
    assert wd.should_stop() is True
    wd.idle()                                 # # 主动结束遥控
    assert wd.should_stop() is False, '结束后不再要求发零速'
