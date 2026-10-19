"""
CommandRouter 单元测试：白名单、目标点判据、缺参数、request_id 去重、stop 快通道。

★ 本文件同时是"判据只有一份"的回归防线：
  旧实现里 CommandRouter 自己按 field_* 矩形判、MapChecker 又按地图判，
  两套范围不一致 → 同一坐标"门卫放行、预检拒绝"。
  现在路由器只把坐标交给注入的 GoalValidator，测试用两种判据各验一遍。
"""

from types import SimpleNamespace        # use 构造轻量消息替身

from agv_common.goal_check import rect_validator, GoalValidator
from agv_task.command_router import CommandRouter


def make_msg(command, args=None, request_id='', source='web', route=''):   # # 造一条指令替身
    return SimpleNamespace(
        command=command, args=args or [], request_id=request_id,
        source=source, confidence=1.0, route=route)


def make_router():                       # # 造一个各指令都已注册的路由器（判据 = ±10 矩形）
    r = CommandRouter(validator=rect_validator(-10.0, 10.0, -10.0, 10.0))
    for name in ('goto', 'start_task', 'cancel_task'):
        r.register(name, lambda args, msg: (True, f'{name} 已受理'))
    r.register('stop', lambda args, msg: (True, '安全停车'))
    r.register('resume', lambda args, msg: (True, '已恢复'))
    return r


def make_router_axis():                  # # x/y 边界不同的矩形（旧 field_* 的真实数值）
    return CommandRouter(validator=rect_validator(-10.6, 4.3, -9.83, 9.22))


def test_peraxis_bounds_reject_out_of_map():   # # y 轴独立边界：矩形外的 y 被拒
    r = make_router_axis()
    r.register('goto', lambda args, msg: (True, 'goto 已受理'))
    ok, reason = r.route(make_msg('goto', ['3.0', '9.9', '0']))   # y=9.9 > 9.22
    assert ok is False and '超出场地范围' in reason


def test_peraxis_bounds_accept_inside():       # # 矩形内的点受理
    r = make_router_axis()
    r.register('goto', lambda args, msg: (True, 'goto 已受理'))
    ok, _ = r.route(make_msg('goto', ['3.0', '9.0', '0']))
    assert ok is True


def test_peraxis_bounds_reject_positive_x():   # # x 上界收紧后，(5,5) 被拒
    r = make_router_axis()
    r.register('goto', lambda args, msg: (True, 'goto 已受理'))
    ok, reason = r.route(make_msg('goto', ['5.0', '5.0', '0']))
    assert ok is False and '超出场地范围' in reason


def test_unknown_command_rejected():     # # 未知指令被拒
    r = make_router()
    ok, reason = r.route(make_msg('fly'))
    assert ok is False
    assert '未知指令' in reason


def test_removed_commands_rejected():    # # 搬货已改走服务：lift/lower/dock 不再是总线指令
    r = make_router()
    for cmd in ('lift', 'lower', 'dock'):
        ok, reason = r.route(make_msg(cmd))
        assert ok is False and '未知指令' in reason


def test_goto_by_name_ok():              # # 点名导航受理
    r = make_router()
    ok, reason = r.route(make_msg('goto', ['货架A'], request_id='r1'))
    assert ok is True


def test_goto_numeric_name_ok():         # # 纯数字当点名也要受理（如点名「666」）
    r = make_router()
    ok, reason = r.route(make_msg('goto', ['666'], request_id='rn'))
    assert ok is True


def test_goto_two_args_as_coords_ok():   # # 两参数按坐标处理（yaw 缺省 0）
    r = make_router()
    ok, reason = r.route(make_msg('goto', ['3.0', '1.2'], request_id='r2d'))
    assert ok is True


def test_goto_two_arg_oob_rejected():    # # 两参数坐标越界也要拒
    r = make_router()
    ok, reason = r.route(make_msg('goto', ['99', '99'], request_id='r2o'))
    assert ok is False
    assert '超出场地范围' in reason


def test_goto_coords_ok():               # # 场内坐标受理
    r = make_router()
    ok, _ = r.route(make_msg('goto', ['3.0', '1.2', '0.0'], request_id='r2'))
    assert ok is True


def test_goto_coords_out_of_field_rejected():    # # 越界坐标被拒
    r = make_router()
    ok, reason = r.route(make_msg('goto', ['99', '99', '0'], request_id='r3'))
    assert ok is False
    assert '超出场地范围' in reason


def test_goto_missing_args_rejected():   # # 缺参数被拒
    r = make_router()
    ok, reason = r.route(make_msg('goto', []))
    assert ok is False
    assert '缺少参数' in reason


def test_goto_bad_coords_rejected():     # # 坐标格式坏被拒
    r = make_router()
    ok, reason = r.route(make_msg('goto', ['x', 'y', 'z']))
    assert ok is False


def test_goto_nan_inf_rejected():        # # nan / inf 不是合法坐标（float() 会放过它们）
    r = make_router()
    for bad in ('nan', 'inf', '-inf'):
        ok, reason = r.route(make_msg('goto', [bad, '0', '0']))
        assert ok is False and '格式非法' in reason


def test_start_task_needs_two_args():    # # start_task 需两个点
    r = make_router()
    ok, reason = r.route(make_msg('start_task', ['A']))
    assert ok is False
    assert '缺少参数' in reason
    ok, _ = r.route(make_msg('start_task', ['A', 'B']))
    assert ok is True


def test_request_id_dedup():             # # 同 request_id 只受理一次
    r = make_router()
    ok1, _ = r.route(make_msg('goto', ['A'], request_id='same'))
    ok2, reason2 = r.route(make_msg('goto', ['A'], request_id='same'))
    assert ok1 is True
    assert ok2 is False
    assert '重复' in reason2


def test_stop_bypasses_dedup_and_args(): # # stop 永远受理，绕过去重
    r = make_router()
    ok1, _ = r.route(make_msg('stop', request_id='s1'))
    ok2, _ = r.route(make_msg('stop', request_id='s1'))
    assert ok1 is True and ok2 is True


def test_dedup_is_per_source():          # # ★ 去重按 source 隔离，多入口不互相顶掉
    """回归：网页 / 语音 / 控制台各自从 1 开始编号时，不能互相把你挤掉。

    曾经的缺陷：去重表只按 request_id 记，不分来源。于是网页发了 "1" 之后，
    语音再发 "1" 会被判"重复"直接拒绝——现象是"某个入口偶尔没反应"，
    而且因为 stop 绕过去重、goto 不绕，表现会更莫名其妙（停车好使、导航不好使）。
    加上网页入口后这是必然会踩的坑，所以钉死它。
    """
    r = make_router()
    # 同一个字面 request_id，来自不同 source：都应受理
    ok_web, _ = r.route(make_msg('goto', ['A'], request_id='1', source='web'))
    ok_voice, _ = r.route(make_msg('goto', ['A'], request_id='1', source='voice'))
    ok_console, _ = r.route(make_msg('goto', ['A'], request_id='1', source='console'))
    assert ok_web is True, '网页入口的 request_id=1 应受理'
    assert ok_voice is True, '语音入口的 request_id=1 不应被网页顶掉'
    assert ok_console is True, '控制台入口的 request_id=1 不应被前两者顶掉'
    # 但同一个 source 内重复，仍必须拒绝（去重不能被削弱）
    again, reason = r.route(make_msg('goto', ['A'], request_id='1', source='web'))
    assert again is False, '同一来源内重复仍须拒绝'
    assert '重复' in reason
    # 空 source 也要能正常工作（老调用方不填 source 时不能崩、仍按同源去重）
    ok_blank1, _ = r.route(make_msg('goto', ['A'], request_id='z'))
    ok_blank2, reason2 = r.route(make_msg('goto', ['A'], request_id='z'))
    assert ok_blank1 is True and ok_blank2 is False
    assert '重复' in reason2


def test_dedup_requires_probe_that_passes_earlier_checks():   # # 回归：去重的探针必须能过前置校验
    # 越界坐标在去重之前就被拦下，两次都只报"越界"，永远测不到去重。
    r = make_router()
    rid = 'oob-probe'
    ok1, reason1 = r.route(make_msg('goto', ['9999', '9999', '0'], request_id=rid))
    ok2, reason2 = r.route(make_msg('goto', ['9999', '9999', '0'], request_id=rid))
    assert ok1 is False and ok2 is False
    assert '超出场地范围' in reason1 and '超出场地范围' in reason2
    assert '重复' not in reason2          # 走不到去重那一层

    rid2 = 'good-probe'
    ok3, _ = r.route(make_msg('cancel_task', request_id=rid2))
    ok4, reason4 = r.route(make_msg('cancel_task', request_id=rid2))
    assert ok3 is True
    assert ok4 is False and '重复' in reason4


def test_handler_receives_route():       # # 处理器能拿到 route 字段（多点导航靠它）
    r = CommandRouter()
    seen = {}
    r.register('goto', lambda args, msg: (seen.update(route=msg.route) or True, 'ok'))
    ok, _ = r.route(make_msg('goto', ['A'], route='巡检A'))
    assert ok is True
    assert seen['route'] == '巡检A'


def test_unimplemented_command_rejected():   # # 白名单内但未注册 → 拒绝
    r = CommandRouter()                  # # 什么都不注册
    ok, reason = r.route(make_msg('goto', ['A']))
    assert ok is False
    assert '暂未实现' in reason


def test_handler_exception_isolated():   # # 处理器抛异常不拖垮路由
    r = CommandRouter()

    def boom(args, msg):
        raise RuntimeError('炸了')
    r.register('goto', boom)
    ok, reason = r.route(make_msg('goto', ['A']))
    assert ok is False
    assert '异常' in reason


# ══════════════════ 判据注入：地图优先，且是唯一判据 ══════════════════

def test_no_validator_passes_any_coord():     # # 未注入判据：不拦坐标（纯逻辑测试场景）
    r = CommandRouter()
    r.register('goto', lambda args, msg: (True, 'ok'))
    ok, _ = r.route(make_msg('goto', ['99999', '99999', '0']))
    assert ok is True


def test_custom_validator_governs_goto():     # # 换成自定义判据后，路由按它判（不再看 field_*）
    calls = []

    class _V:                            # # 只允许 x>0 的极简判据
        def check(self, x, y):
            calls.append((x, y))
            if x <= 0:
                return False, f'自定义判据拒绝 ({x:.1f}, {y:.1f})'
            return True, '自定义判据放行'

    r = CommandRouter(validator=_V())
    r.register('goto', lambda args, msg: (True, 'ok'))
    ok1, reason1 = r.route(make_msg('goto', ['-1.0', '0.0', '0']))
    ok2, _ = r.route(make_msg('goto', ['1.0', '0.0', '0']))
    assert ok1 is False and '自定义判据拒绝' in reason1
    assert ok2 is True
    assert calls == [(-1.0, 0.0), (1.0, 0.0)]     # # 判据确实被调用（唯一来源）


def test_validator_is_used_for_two_arg_coords():   # # 两参数坐标同样过判据
    r = CommandRouter(validator=rect_validator(-1.0, 1.0, -1.0, 1.0))
    r.register('goto', lambda args, msg: (True, 'ok'))
    ok, reason = r.route(make_msg('goto', ['5.0', '5.0']))
    assert ok is False and '超出场地范围' in reason


def test_goal_validator_describe_mentions_fallback():   # # describe() 能说明当前判据
    v = GoalValidator(map_yaml='')
    assert '矩形兜底' in v.describe()
    assert v.map_based() is False


def test_goal_validator_bad_map_falls_back():   # # 地图读不到 → 退化为矩形，不抛异常
    v = GoalValidator(map_yaml='D:/不存在的地图.yaml',
                      field={'min_x': -1.0, 'max_x': 1.0,
                             'min_y': -1.0, 'max_y': 1.0})
    assert v.map_based() is False
    assert v.error                       # # 记录失败原因
    ok, reason = v.check(2.0, 0.0)
    assert ok is False and '超出场地范围' in reason


# ══════════════════ 拒绝原因不得重复后缀 ══════════════════

def test_reject_reason_not_double_suffixed():   # # ★ 不得打印"…，已拒绝，已拒绝"
    # MapChecker 的中文串本身就以"，已拒绝"结尾；路由器若再无脑拼一次，
    # 日志会变成"……，已拒绝，已拒绝"。拦截行为对，但尾巴重复。
    class _V:                            # # 模拟 MapChecker：自己已带"已拒绝"
        def check(self, x, y):
            return False, f'目标点 ({x:.2f}, {y:.2f}) 在地图范围外，已拒绝'

    r = CommandRouter(validator=_V())
    r.register('goto', lambda args, msg: (True, 'ok'))
    ok, reason = r.route(make_msg('goto', ['99', '0', '0']))
    assert ok is False
    assert reason.count('已拒绝') == 1, f'拒绝原因重复了后缀：{reason}'


def test_reject_reason_gets_suffix_when_missing():   # # 判据没带后缀时补一次
    class _V:                            # # 只在断言里用的极简判据
        def check(self, x, y):
            return False, '这个点不行'

    r = CommandRouter(validator=_V())
    r.register('goto', lambda args, msg: (True, 'ok'))
    ok, reason = r.route(make_msg('goto', ['99', '0', '0']))
    assert ok is False and reason == '这个点不行，已拒绝'
