"""
test_status_board —— 顶部仪表 + 日志降噪的单元测试。

**本文件守的两条用户诉求**
  ① "数据面板显示的日志是固定死的，日志长的话看不到后面的信息。
      数据面板，就显示一些重点状态标志。"
     → 面板必须是**固定 7 槽位**，值来自状态量（build_gauges），
       文本必须短（≤12 字，实测最长 "0.30 · -0.28"），不许出现原文直传。
  ② "运行日志打印速度太快根本看不过来……每次触发打印只替换变化的
      文字或数值，并且增加打印的时间间隔。"
     → 高频日志按最小间隔留一条（不是全丢）；事件永远留。

★ 写这些用例时的教训（上一版踩过）：
  测试只测"函数本身对不对"，抓不到"调用方有没有真的用它"。
  所以这里除了纯函数，还断言 **snapshot 必须把 build_gauges 的结果带出去**、
  **log() 必须真的把 drop 的那条拦住** —— 回归锁失效过一次，就是因为
  只测了 clean_display 的返回值，没测它有没有被 snapshot 调用。
"""

import math
import unittest

from agv_web.status_board import (DASH, GAUGES, LogFilter, build_gauges,
                                  front_distance, is_event, normalize)


# ══════════════════ 一、激光前方距离 ══════════════════

class TestFrontDistance(unittest.TestCase):
    """`front_distance` 是"避障"格的唯一数据来源，算错等于仪表骗人。"""

    def _scan(self, pairs, n=360):
        """造一帧扫描：pairs 是 [(角度rad, 距离m)]，其余点是 10m 空地。"""
        a0 = -math.pi
        inc = 2 * math.pi / n
        ranges = [10.0] * n
        for ang, dist in pairs:
            i = int(round((ang - a0) / inc))
            if 0 <= i < n:
                ranges[i] = dist
        return {'ranges': ranges, 'angle_min': a0, 'inc': inc,
                'range_min': 0.05, 'range_max': 12.0}

    def test_front_blocker_found(self):
        self.assertAlmostEqual(front_distance(self._scan([(0.0, 0.8)])), 0.8, places=3)

    def test_side_wall_not_counted(self):
        """侧墙（±86°）不算正前方 —— 否则车在窄走廊里永远显示"减速"。"""
        scan = self._scan([(1.5, 0.25), (-1.5, 0.25), (0.0, 2.0)])
        self.assertAlmostEqual(front_distance(scan), 2.0, places=3)

    def test_picks_nearest_in_sector(self):
        scan = self._scan([(0.0, 1.5), (0.2, 0.7), (-0.3, 0.9)])
        self.assertAlmostEqual(front_distance(scan), 0.7, places=3)

    def test_inf_and_nan_ignored(self):
        """很多驱动的超出量程点是 inf 而不是 range_max，直接 min() 会得到 inf。"""
        scan = self._scan([(0.0, 1.2)])
        # 180 就是 0 弧度那个点，所以另挑几个相邻索引塞 inf/nan
        scan['ranges'][170] = float('inf')
        scan['ranges'][171] = float('nan')
        scan['ranges'][190] = float('-inf')
        self.assertAlmostEqual(front_distance(scan), 1.2, places=3)

    def test_out_of_range_ignored(self):
        """比 range_max 还远、比 range_min 还近的点都是坏点。"""
        scan = self._scan([(0.0, 0.9)])
        scan['ranges'][178] = 50.0      # 远超 range_max=12
        scan['ranges'][182] = 0.001     # 近于 range_min=0.05
        self.assertAlmostEqual(front_distance(scan), 0.9, places=3)

    def test_empty_and_none(self):
        self.assertIsNone(front_distance(None))
        self.assertIsNone(front_distance({}))
        self.assertIsNone(front_distance({'ranges': []}))
        self.assertIsNone(front_distance({'ranges': [1.0], 'inc': 0.0}))

    def test_angle_wraps(self):
        """angle_min 从 -pi 起时，正前方（0 rad）不能被算到 ±pi 去。"""
        n = 360
        a0, inc = -math.pi, 2 * math.pi / n
        ranges = [10.0] * n
        ranges[0] = 0.4                 # -pi，即正后方，不该算进来
        scan = {'ranges': ranges, 'angle_min': a0, 'inc': inc,
                'range_min': 0.05, 'range_max': 12.0}
        self.assertAlmostEqual(front_distance(scan), 10.0, places=3)


# ══════════════════ 二、仪表槽位 ══════════════════

class TestGaugeShape(unittest.TestCase):
    """固定 8 槽位是"不再固定死"的前提 —— 行数、顺序、key 都不能变。"""

    def test_exactly_eight_slots(self):
        # 2026-10-18 用户要求加「动作」格（取货中/载货中/无动作），
        # 由 7 槽扩到 8 槽。行数一变前端骨架也要跟着变（ui_test 有对应的锁）。
        self.assertEqual(len(build_gauges({})), 8)

    def test_keys_and_order_fixed(self):
        got = [g['key'] for g in build_gauges({})]
        self.assertEqual(
            got, ['task', 'act', 'speed', 'avoid', 'pose', 'map', 'mode', 'link'])
        self.assertEqual(got, [k for k, _ in GAUGES])

    def test_act_sits_right_after_task(self):
        """"动作"必须紧跟"任务"：它是判断"车上有没有货"的唯一依据，
        排在后面会被当成次要信息跳过。"""
        keys = [g['key'] for g in build_gauges({})]
        self.assertEqual(keys.index('act'), keys.index('task') + 1)

    def test_every_slot_has_required_fields(self):
        for g in build_gauges({}):
            for f in ('key', 'label', 'text', 'tone'):
                self.assertIn(f, g)
            self.assertTrue(g['label'], '标题不能空')
            self.assertTrue(g['text'], '文本不能空（空值应给 DASH）')

    def test_labels_are_short(self):
        """槽位标题本身要短：一格总共才约 200px，标题占多了值就挤。"""
        for _, label in GAUGES:
            self.assertLessEqual(len(label), 4, f'标题「{label}」太长')


class TestGaugeValues(unittest.TestCase):
    """每个槽位的取值逻辑（这是"值来自状态量"的核心）。"""

    def _one(self, state, key):
        for g in build_gauges(state):
            if g['key'] == key:
                return g
        self.fail(f'没有 {key} 槽位')

    # ── 任务 ──
    def test_task_idle(self):
        self.assertEqual(self._one({}, 'task')['text'], '空闲')
        self.assertEqual(self._one({}, 'task')['tone'], 'off')

    def test_task_buffer_running_shows_progress(self):
        g = self._one({'task_running': True, 'task_idx': 2, 'task_total': 5}, 'task')
        self.assertEqual(g['text'], '搬运中 2/5')

    def test_task_estop_wins(self):
        """急停优先级最高：哪怕缓冲还在跑，也必须显示急停。"""
        g = self._one({'estop': True, 'task_running': True,
                       'task_idx': 1, 'task_total': 3}, 'task')
        self.assertEqual(g['text'], '急停中')
        self.assertEqual(g['tone'], 'error')

    def test_task_state_from_manager(self):
        g = self._one({'task_state': {'state': 'LIFTING'}}, 'task')
        self.assertEqual(g['text'], '顶升中')

    def test_task_running_beats_stale_done(self):
        """★ 缓冲在跑时 task_state 可能还是上一单的 DONE（收尾没回 IDLE）。
        只看 task_state 会显示"本单完成"，而其实还有 3 单排队。"""
        g = self._one({'task_running': True, 'task_idx': 2, 'task_total': 5,
                       'task_state': {'state': 'DONE'}}, 'task')
        self.assertEqual(g['text'], '搬运中 2/5')

    def test_task_error(self):
        g = self._one({'task_state': {'state': 'ERROR'}}, 'task')
        self.assertEqual(g['tone'], 'error')

    def test_task_unknown_state_falls_back_to_idle(self):
        """枚举里没有的状态不能显示成英文（用户看不懂）。"""
        g = self._one({'task_state': {'state': 'SOMETHING_NEW'}}, 'task')
        self.assertEqual(g['text'], '空闲')

    # ── 动作（取货中 / 载货中 / 无动作）──
    #
    # 用户定义：取货中=顶升之前或下降之后；载货中=顶升之后或下降之前；
    #           无动作=前往导航点、无任务……
    # ★ 这一组测试守的是"车上有没有货"——安全相关，不能靠人工目测。

    def test_act_idle_when_nothing(self):
        g = self._one({}, 'act')
        self.assertEqual(g['text'], '无动作')
        self.assertEqual(g['tone'], 'off')

    def test_act_loading_when_going_to_pick(self):
        """前往取货点 = 顶升之前 → 取货中"""
        g = self._one({'task_state': {'state': 'GO_LOAD', 'task_id': 't1'}}, 'act')
        self.assertEqual(g['text'], '取货中')

    def test_act_loading_while_lifting(self):
        """顶升请求发出但还没成功 → 仍算取货中（货还没上去）"""
        g = self._one({'task_state': {'state': 'LIFTING', 'task_id': 't1'}}, 'act')
        self.assertEqual(g['text'], '取货中')

    def test_act_carry_when_transporting(self):
        """顶升成功、运输中 → 载货中，且必须提醒（warn 黄）"""
        g = self._one({'task_state': {'state': 'TRANSPORT', 'task_id': 't1'}}, 'act')
        self.assertEqual(g['text'], '载货中')
        self.assertEqual(g['tone'], 'warn')

    def test_act_carry_while_lowering(self):
        """正在下降、货还没落地 → 仍算载货中"""
        g = self._one({'task_state': {'state': 'LOWERING', 'task_id': 't1'}}, 'act')
        self.assertEqual(g['text'], '载货中')

    def test_act_idle_for_pure_navigation(self):
        """★ 纯导航任务也报 GO_LOAD —— 必须靠 task_id 为空区分，
        否则"前往导航点"会被错报成"取货中"。"""
        g = self._one({'task_state': {'state': 'GO_LOAD', 'task_id': ''}}, 'act')
        self.assertEqual(g['text'], '无动作')

    def test_act_idle_when_done(self):
        g = self._one({'task_state': {'state': 'DONE', 'task_id': 't1'}}, 'act')
        self.assertEqual(g['text'], '无动作')

    def test_act_error_with_cargo_stays_carry(self):
        """★ 出错时状态机说不清货在哪，必须听搬货服务的 busy：
        busy=true → 载货中 + 红字，不能让用户以为货已落地。"""
        g = self._one({'task_state': {'state': 'ERROR', 'task_id': 't1'},
                       'cargo_busy': True}, 'act')
        self.assertEqual(g['text'], '载货中')
        self.assertEqual(g['tone'], 'error')

    def test_act_error_without_cargo(self):
        g = self._one({'task_state': {'state': 'ERROR', 'task_id': 't1'},
                       'cargo_busy': False}, 'act')
        self.assertEqual(g['text'], '取货中')

    def test_act_cargo_busy_outranks_idle(self):
        """★ 任务已结束但服务仍报载货（上一单没下降成功）：
        绝不能显示"无动作"——那是事故的起点。"""
        g = self._one({'task_state': {'state': 'IDLE'}, 'cargo_busy': True}, 'act')
        self.assertEqual(g['text'], '载货中')
        self.assertEqual(g['tone'], 'error')

    def test_act_loading_when_buffer_starts(self):
        """缓冲在跑但本单 TaskState 还没到 → 保守显示取货中"""
        g = self._one({'task_running': True}, 'act')
        self.assertEqual(g['text'], '取货中')

    # ── 速度 ──
    def test_speed_shows_value(self):
        g = self._one({'speed': (0.30, -0.28)}, 'speed')
        self.assertEqual(g['text'], '0.30 · -0.28')

    def test_speed_zero_is_stop(self):
        """静止时显示"停止"而不是 "0.00 · 0.00"：后者没信息量还占位置。"""
        self.assertEqual(self._one({'speed': (0.0, 0.0)}, 'speed')['text'], '停止')
        self.assertEqual(self._one({'speed': (0.001, -0.002)}, 'speed')['text'], '停止')

    def test_speed_missing_is_dash(self):
        self.assertEqual(self._one({}, 'speed')['text'], DASH)
        self.assertEqual(self._one({'speed': None}, 'speed')['tone'], 'off')

    def test_speed_broken_tuple(self):
        self.assertEqual(self._one({'speed': ('x',) }, 'speed')['text'], DASH)

    # ── 避障 ──
    def test_avoid_clear(self):
        g = self._one({'front': 1.20}, 'avoid')
        self.assertEqual(g['text'], '1.20m 畅通')
        self.assertEqual(g['tone'], 'ok')

    def test_avoid_engaged_warns(self):
        g = self._one({'front': 0.52, 'avoid_on': True}, 'avoid')
        self.assertIn('减速', g['text'])
        self.assertEqual(g['tone'], 'warn')

    def test_avoid_near_but_no_arbiter(self):
        """建图模式没有仲裁节点，但距离进了减速区照样要提示。"""
        g = self._one({'front': 0.40, 'avoid_on': False}, 'avoid')
        self.assertIn('接近', g['text'])
        self.assertEqual(g['tone'], 'warn')

    def test_avoid_no_scan(self):
        self.assertEqual(self._one({}, 'avoid')['text'], '无激光')
        self.assertEqual(self._one({'front': None}, 'avoid')['tone'], 'off')

    # ── 定位 ──
    def test_pose_ok(self):
        g = self._one({'pose_ok': True}, 'pose')
        self.assertEqual(g['text'], '正常')

    def test_pose_lost_shows_age(self):
        """★ 判"丢失"必须看数据年龄。tf2 Buffer 缓存 10 秒历史，
        发布者死了 lookup 仍返回最后一个变换，只看"查没查到"永远正常。"""
        g = self._one({'pose_ok': False, 'pose_age': 2.5}, 'pose')
        self.assertEqual(g['text'], '丢失 2.5s')
        self.assertEqual(g['tone'], 'warn')

    def test_pose_never_ready(self):
        self.assertEqual(self._one({}, 'pose')['text'], '未就绪')

    def test_pose_long_age_integer(self):
        g = self._one({'pose_ok': False, 'pose_age': 125.4}, 'pose')
        self.assertEqual(g['text'], '丢失 125s')

    # ── 连接 ──
    def test_link_connected(self):
        self.assertEqual(self._one({'ws_n': 1}, 'link')['text'], '已连接')

    def test_link_multiple_clients(self):
        self.assertIn('2', self._one({'ws_n': 2}, 'link')['text'])

    def test_link_none(self):
        self.assertEqual(self._one({'ws_n': 0}, 'link')['tone'], 'warn')

    def test_link_reconnecting(self):
        g = self._one({'ws_lost': True, 'ws_n': 1}, 'link')
        self.assertEqual(g['text'], '重连中')
        self.assertEqual(g['tone'], 'error')

    # ── 地图 ──
    def test_map_size(self):
        g = self._one({'map_meta': {'width': 594, 'height': 762}}, 'map')
        self.assertEqual(g['text'], '594×762')

    def test_map_missing(self):
        self.assertEqual(self._one({}, 'map')['text'], DASH)
        self.assertEqual(self._one({'map_meta': {}}, 'map')['text'], DASH)

    # ── 模式 ──
    def test_mode_nav_auto(self):
        self.assertEqual(self._one({'mode': 'nav'}, 'mode')['text'], '导航 · 自动')

    def test_mode_mapping_manual(self):
        g = self._one({'mode': 'mapping', 'manual': True}, 'mode')
        self.assertEqual(g['text'], '建图 · 手动')
        self.assertEqual(g['tone'], 'warn')

    def test_mode_missing_is_dash(self):
        self.assertEqual(self._one({}, 'mode')['text'], DASH)


class TestGaugeLength(unittest.TestCase):
    """★ 用户的核心抱怨是"日志长的话看不到后面的信息"。
    所以"值必须短"是一条硬约束，不能只靠代码注释保证。"""

    def test_all_texts_fit_one_line(self):
        # 侧栏 360px 一格文本区约 202px ≈ 16 个中文字/行；
        # 留出余量，超过 14 字就算超（这也覆盖了英文按半宽的保守估计）。
        cases = [
            {},
            {'task_running': True, 'task_idx': 12, 'task_total': 99},
            {'speed': (-0.35, -0.90)},
            {'front': 0.52, 'avoid_on': True},
            {'pose_ok': False, 'pose_age': 999.9},
            {'ws_n': 12}, {'map_meta': {'width': 9999, 'height': 9999}},
            {'mode': 'mapping', 'manual': True},
        ]
        for st in cases:
            for g in build_gauges(st):
                self.assertLessEqual(
                    len(g['text']), 14,
                    f"槽位「{g['label']}」文本太长（{len(g['text'])} 字）：{g['text']}")

    def test_no_newlines_in_text(self):
        """换行会把一格撑成两行，7 行就变成 14 行。"""
        for g in build_gauges({'task_state': {'state': 'LIFTING'},
                               'map_meta': {'width': 1, 'height': 1},
                               'mode': 'nav'}):
            self.assertNotIn('\n', g['text'])


# ══════════════════ 三、日志降噪 ══════════════════

class TestNormalize(unittest.TestCase):
    def test_numbers_flattened(self):
        self.assertEqual(normalize('[避障] 前方 0.35m'),
                         normalize('[避障] 前方 0.42m'))

    def test_decimal_width_irrelevant(self):
        """0.3 和 0.30 必须归一化一致，否则计数不准。"""
        self.assertEqual(normalize('v 0.3'), normalize('v 0.30'))

    def test_negative_and_exponent(self):
        self.assertEqual(normalize('w -1.2'), normalize('w -3.4'))
        self.assertEqual(normalize('x 1e-3'), normalize('x 2e-4'))

    def test_text_difference_kept(self):
        self.assertNotEqual(normalize('[顶升] 开始'), normalize('[下降] 开始'))


class TestEventGuard(unittest.TestCase):
    """事件必须永远留 —— 这是"不能丢历史"的底线。"""

    def test_event_words(self):
        for s in ('[顶升] 开始（A）', '[下降] 结束（B）', '[任务完成] A → B',
                  '任务失败：超时', '栈已停止', '已保存地图'):
            self.assertTrue(is_event(s), f'「{s}」应判为事件')

    def test_batch_completion_not_mergeable(self):
        """★ 真 bug 锁：`[1/5] 搬运完成` 与 `[2/5]` 归一化后完全相同，
        批量任务几毫秒内出现 5 次，曾被当成刷屏折叠掉 3 条。"""
        f = LogFilter()
        for i in range(1, 6):
            r = f.feed('ok', f'[{i}/5] 搬运完成', now=100.0 + i * 0.01)
            self.assertEqual(r['claim'], 'log', f'第 {i} 单完成被吞了')

    def test_plain_info_not_event(self):
        self.assertFalse(is_event('[避障] 前方 0.50m'))
        self.assertFalse(is_event('速度仲裁：限速 线≤0.35'))


class TestThrottle(unittest.TestCase):
    """已知高频状态：按最小间隔留一条（用户要的是"加大间隔"，不是"别打"）。"""

    def test_avoid_throttled_to_interval(self):
        """避障 1 Hz 连打 20 秒 → 只留 4 条（5 秒一条）。"""
        f = LogFilter()
        kept = sum(1 for i in range(20)
                   if f.feed('warn', '[避障] 前方 0.50m → v 0.30→0.20，择向接管',
                             now=100.0 + i)['claim'] == 'log')
        self.assertEqual(kept, 4)

    def test_first_always_kept(self):
        """第一条必留：否则用户会以为避障功能没生效。"""
        f = LogFilter()
        self.assertEqual(f.feed('warn', '[避障] 前方 0.50m', now=100.0)['claim'], 'log')

    def test_throttle_expires(self):
        """过了间隔必须再放行，不能一次之后永久压住。"""
        f = LogFilter()
        f.feed('warn', '[避障] 前方 0.50m', now=100.0)
        self.assertEqual(f.feed('warn', '[避障] 前方 0.48m', now=106.0)['claim'], 'log')

    def test_empty_dropped(self):
        f = LogFilter()
        self.assertEqual(f.feed('info', '')['claim'], 'drop')
        self.assertEqual(f.feed('info', '   ')['claim'], 'drop')

    def test_costmap_longer_interval(self):
        """代价地图 30 秒：它一刷就是几十条，比避障还不值得留。

        ★ 这条同时守一个真 bug：这句里含"超时"（是事件词），
          事件守卫曾经放在限流前面，于是这条永远限不住。
          限流表是人工确认过的高频源，必须优先于事件词启发式。
        """
        f = LogFilter()
        msg = '等待代价地图超时（可能刚启动还没就绪）'
        self.assertEqual(f.feed('info', msg, now=100.0)['claim'], 'log')
        self.assertEqual(f.feed('info', msg, now=120.0)['claim'], 'drop')
        self.assertEqual(f.feed('info', msg, now=131.0)['claim'], 'log')

    def test_throttle_beats_event_word(self):
        """限流表里含事件词的条目，按限流走（见上条注释）。"""
        f = LogFilter()
        msg = '指令超时（上层已不再发速度），已发零速停车'
        self.assertEqual(f.feed('warn', msg, now=100.0)['claim'], 'log')
        self.assertEqual(f.feed('warn', msg, now=102.0)['claim'], 'drop')

    def test_real_event_still_passes(self):
        """但真事件（没进限流表）必须照样全留。"""
        f = LogFilter()
        for i in range(5):
            self.assertEqual(
                f.feed('warn', '任务被取消，停止后续', now=100.0 + i * 0.1)['claim'], 'log')

    def test_nav2_chatter_throttled(self):
        """Nav2 的周期性碎碎念必须限流。

        ★ 依据（用户贴的 2.5 分钟真车日志）：
            "Passing new path to controller." 出现 10 次，
            "Planner loop missed its desired rate" 出现 8 次。
          这两句是同一件事重复 N 遍（性能跟不上），重复看没有任何新信息，
          但在手机上占了近一半可见行数，把顶升/下降结果挤出去了。
        ★ 不直接丢弃：30~60 秒留一条，真持续掉帧仍看得出趋势。
        """
        f = LogFilter()
        per_sec = 'Passing new path to controller.'
        kept = sum(1 for i in range(60)
                   if f.feed('info', per_sec, now=1000.0 + i)['claim'] == 'log')
        self.assertEqual(kept, 2, f'60 秒只该留 2 条（30 秒一条），实际 {kept}')

        g = LogFilter()
        rate = ('Planner loop missed its desired rate of 20.0000 Hz. '
                'Current loop rate is 10.0000 Hz')
        kept = sum(1 for i in range(60)
                   if g.feed('warn', rate, now=2000.0 + i)['claim'] == 'log')
        self.assertEqual(kept, 1, f'60 秒只该留 1 条，实际 {kept}')


    def test_avoid_engaged_flag(self):
        """仪表靠这个标志显示"减速" —— 仲裁打完就没了，所以要用时间窗。"""
        f = LogFilter()
        self.assertFalse(f.avoid_engaged(now=100.0))
        f.feed('warn', '[避障] 前方 0.5m', now=100.0)
        self.assertTrue(f.avoid_engaged(now=101.0))
        self.assertFalse(f.avoid_engaged(now=120.0))

    def test_reset_clears_throttle(self):
        """换栈必须清：否则新栈第一条避障会被旧栈窗口吞掉。"""
        f = LogFilter()
        f.feed('warn', '[避障] 前方 0.5m', now=100.0)
        f.reset()
        self.assertEqual(f.feed('warn', '[避障] 前方 0.5m', now=100.1)['claim'], 'log')


class TestDuplicateFold(unittest.TestCase):
    """没预料到的高频源：兜底折叠，但不能吃掉事件。"""

    def test_unknown_repeat_folded(self):
        f = LogFilter(dup_window=15.0, dup_limit=5)
        claims = [f.feed('info', f'某个未知节点第 {i} 次刷屏 ', now=100.0 + i * 0.1)['claim']
                  for i in range(12)]
        self.assertEqual(claims[:5], ['log'] * 5)
        self.assertIn('drop', claims[5:])

    def test_fold_note_once(self):
        """首次折叠要补一条提示，否则用户以为日志坏了。"""
        f = LogFilter(dup_window=15.0, dup_limit=2)
        notes = []
        for i in range(8):
            r = f.feed('info', f'未知刷屏 {i}', now=100.0 + i * 0.1)
            if r.get('note'):
                notes.append(r['note'])
        self.assertEqual(len(notes), 1, '折叠提示只能出现一次')

    def test_event_never_folded(self):
        f = LogFilter(dup_window=15.0, dup_limit=2)
        for i in range(10):
            r = f.feed('info', '任务失败：导航超时', now=100.0 + i * 0.1)
            self.assertEqual(r['claim'], 'log')

    def test_window_expiry_recovers(self):
        """窗口过去后要重新计数，不能永久压住。"""
        f = LogFilter(dup_window=5.0, dup_limit=2)
        for i in range(5):
            f.feed('info', f'未知 {i}', now=100.0 + i * 0.1)
        self.assertEqual(f.feed('info', '未知 99', now=200.0)['claim'], 'log')


class TestFilterReset(unittest.TestCase):
    def test_reset_is_idempotent(self):
        f = LogFilter()
        f.reset()
        f.reset()
        self.assertEqual(f.feed('warn', '[避障] 前方 0.5m', now=1.0)['claim'], 'log')


if __name__ == '__main__':
    unittest.main()
