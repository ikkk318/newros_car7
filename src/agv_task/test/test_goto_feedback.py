#!/usr/bin/env python3
"""导航反馈（[导航中]）的文案与节流判据测试。

★ 为什么单独建这个文件（真车日志暴露的两个问题，都是"看着没问题、
  跑一趟就现形"的类型）：

  ① 文案自相矛盾：
        [导航中] 剩余 13.54 米 · 约 0 秒
     刚发出导航目标时 Nav2 的 estimated_time_remaining 还是 0。用户第一反应
     是"导航算错了"，而不是"还没算出来"。

  ② 降噪在真车上完全失效：
     判据是"距上次满 1 秒 且 剩余距离变化 ≥0.3 米"。车正常速度就是
     0.3 m/s，1 秒正好走 0.30~0.32 米 —— 每一帧都判定为"动了"，
     于是退回 1 Hz 刷屏。实车日志：
        13.54 → 13.20 → 12.88 → 12.58 → 12.28 → 11.98 → 11.66 → 11.35
     11 秒里 8 条，一条不少。
     这类问题只能靠判据单测守住，靠肉眼看日志永远看不出来"阈值恰好等于
     每秒行程"。

★ 判据被抽成 _should_report 纯函数就是为了能在这里测。
"""

import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))          # src/agv_task

from builtin_interfaces.msg import Duration as _DurationMsg  # noqa: E402

from agv_task.goto_pose import (               # noqa: E402
    _FB_ALIVE_S, _FB_GAP_MIN, _FB_MOVE_M, _format_feedback, _should_report)


class _DurMsg(object):
    """保留给"故意构造坏时间戳"的用例（比如 sec 是非法值）。"""

    def __init__(self, sec):
        self.sec = int(sec)
        self.nanosec = int(round((float(sec) - int(sec)) * 1e9))


class _Feedback(object):
    """最小化模拟 NavigateToPose 反馈（只要这三个字段）。"""

    def __init__(self, left, remain=0.0, rec=0):
        self.distance_remaining = float(left)
        # ★ 必须用真的 builtin_interfaces/Duration：
        #   Duration.from_msg 会做类型检查，传个同名属性的假对象会抛
        #   "Must pass a builtin_interfaces.msg.Duration object"，
        #   于是走到 except 分支拿到降级文案，测试就测了个寂寞。
        msg = _DurationMsg()
        msg.sec = int(remain)
        msg.nanosec = int(round((float(remain) - int(remain)) * 1e9))
        self.estimated_time_remaining = msg
        self.number_of_recoveries = int(rec)


class TestFeedbackText(unittest.TestCase):
    """文案：算不出时间就不能显示 0 秒。"""

    def test_zero_time_is_omitted(self):
        out = _format_feedback(_Feedback(13.54, remain=0.0))
        self.assertIn('剩余 13.54 米', out)
        self.assertNotIn('约 0 秒', out)
        self.assertNotIn('约', out)

    def test_normal_time_is_shown(self):
        out = _format_feedback(_Feedback(3.20, remain=12.0))
        self.assertIn('剩余 3.20 米', out)
        self.assertIn('约 12 秒', out)

    def test_round_up_to_seconds(self):
        out = _format_feedback(_Feedback(1.00, remain=1.4))
        self.assertIn('约 1 秒', out)

    def test_recoveries_only_when_positive(self):
        self.assertNotIn('已恢复', _format_feedback(_Feedback(2.0, 5.0, 0)))
        self.assertIn('已恢复 2 次', _format_feedback(_Feedback(2.0, 5.0, 2)))

    def test_broken_feedback_does_not_raise(self):
        """字段缺失也要给出可读的降级文案，不能抛到导航循环里。"""
        out = _format_feedback(object())
        self.assertIn('[导航中]', out)


class TestReportCadence(unittest.TestCase):
    """节流判据：正常行驶不能退化成 1 Hz。"""

    def test_constants_are_not_per_second_travel(self):
        """★ 锁死"阈值不能等于每秒行程"这个坑。"""
        self.assertGreaterEqual(_FB_GAP_MIN, 5.0)      # 最小间隔至少 5 秒
        self.assertGreater(_FB_MOVE_M, 0.3)            # 必须大于 0.3 m/s 的秒行程
        self.assertGreater(_FB_ALIVE_S, _FB_GAP_MIN)   # 保活间隔比常规间隔更长

    def test_first_frame_reports(self):
        self.assertTrue(_should_report(0.0, 0.0, 13.54, None, 0, None, 1.0))

    def test_real_drive_does_not_report_every_second(self):
        """复现实车那段：每秒走 0.3 米，11 秒内从 8 条压到 2 条。

        ★ 关键数字：判据是"满 8 秒且真的挪了"，所以 9 帧只会打
          t=0（首帧基线）和 t=8 两条 —— 原来 8~9 条。
          这条测试锁的就是"0.3 米阈值 ≠ 每秒行程"，改回 0.3 立刻失败。
        """
        # 实车日志里的真实序列（米）
        seq = [13.54, 13.20, 12.88, 12.58, 12.28, 11.98, 11.66, 11.35, 11.04]
        last_print, last_left, last_rec = 0.0, None, None
        reported = []
        for i, left in enumerate(seq):
            now = float(i)                     # 每秒一帧
            if _should_report(now, last_print, left, last_left, 0, last_rec, 1.0):
                reported.append((now, left))
                last_print, last_left, last_rec = now, left, 0
        self.assertEqual(len(reported), 2, f'实际打了 {len(reported)} 条：{reported}')
        # 两次之间必须真的隔够最小间隔，不能是"连着两帧"
        self.assertGreaterEqual(reported[1][0] - reported[0][0], _FB_GAP_MIN)

    def test_reports_again_after_gap_with_real_progress(self):
        """满 8 秒且真的挪了 ≥0.8 米 → 该打。"""
        self.assertTrue(_should_report(8.0, 0.0, 11.0, 13.54, 0, 0, 1.0))

    def test_silent_when_stuck(self):
        """卡住不动（等障碍/等电梯）：间隔到了也不打。"""
        self.assertFalse(_should_report(9.0, 0.0, 5.00, 5.00, 0, 0, 1.0))
        self.assertFalse(_should_report(19.0, 0.0, 5.00, 5.00, 0, 0, 1.0))

    def test_alive_heartbeat_when_stuck_long(self):
        """卡住超过保活间隔 → 必须留一条，否则用户以为程序死了。"""
        self.assertTrue(_should_report(_FB_ALIVE_S, 0.0, 5.00, 5.00, 0, 0, 1.0))

    def test_recovery_change_is_reported(self):
        """恢复次数变了（卡住→自恢复）是必须看见的事件。"""
        self.assertTrue(_should_report(8.0, 0.0, 5.00, 5.00, 1, 0, 1.0))

    def test_configured_period_smaller_than_floor_is_clamped(self):
        """配置 0.1 秒也不能真的 0.1 Hz —— 下限优先。"""
        self.assertFalse(_should_report(1.0, 0.0, 11.0, 13.54, 0, 0, 0.1))


if __name__ == '__main__':
    unittest.main()
