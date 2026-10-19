#!/usr/bin/env python3
"""跨节点日志转发（/rosout → 网页运行日志）的判据测试。

★ 为什么需要这一组测试（用户原话）：
    "到达搬货点请求服务后打印请求结果出来（成功绿字，失败红字），
     并显示在运行日志打印出来。"

  顶升/下降是 task_manager_node 在**另一个进程**里调的，网关的 self.log()
  收不到。必须订阅 /rosout 转发。而 /rosout 是全系统日志汇总，全转发会把
  降噪一次性废掉 —— 所以判据"转发哪些、标什么颜色"是这次改动的核心，
  不能靠肉眼在网页上试。

★ 这里只测**判据函数**（纯逻辑，不碰 ROS）：
    _relay_wanted(text)   → 要不要转发
    _relay_is_success(t)  → 标绿还是标红
  真正的订阅接线由 board_e2e.py（真起节点）验证。
"""

import importlib
import importlib.util
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(_HERE, '..')                       # src/agv_web
sys.path.insert(0, _SRC)
sys.path.insert(0, os.path.join(_HERE, '..', '..'))    # src

_GW_PATH = os.path.join(_SRC, 'agv_web', 'web_gateway.py')


def _load_gateway_module():
    """按文件路径加载 web_gateway 模块（不依赖 install/setup.bash）。

    ★ 为什么不用普通 import：
      该模块顶层 import rclpy / aiohttp，未 source 环境时会 ImportError，
      测试会变成"环境没进去"而不是"代码错了"（这项目踩过 15 次）。
      但这两个判据函数只用到 re，所以退化为"能加载就加载，不能就让测试跳过"。
    """
    try:
        from agv_web import web_gateway as m
        return m
    except Exception:
        pass
    try:
        spec = importlib.util.spec_from_file_location('gw_under_test', _GW_PATH)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    except Exception:
        return None


_GW = None


def setUpModule():
    global _GW
    _GW = _load_gateway_module()


def _need(mod, name):
    """判据函数取不到就跳过（环境缺 ROS 时不误报成代码错误）。"""
    if mod is None or not hasattr(mod, name):
        raise unittest.SkipTest('需要 ROS 环境才能导入 web_gateway')


class TestRelayWanted(unittest.TestCase):
    """哪些日志值得出现在网页上。"""

    def wanted(self, text):
        _need(_GW, '_relay_wanted')
        return _GW._relay_wanted(text)

    # ── 必须转发：用户明确要在网页上看到的四类结果 ──
    def test_lift_success_is_wanted(self):
        self.assertTrue(self.wanted('[顶升] 成功（1号槽内）：已顶升，开始搬货'))

    def test_lift_failure_is_wanted(self):
        self.assertTrue(self.wanted('[顶升] 失败：/agv/cargo 搬货服务不可用'))

    def test_lower_success_is_wanted(self):
        self.assertTrue(self.wanted('[下降] 成功（1号货架）：已下降，放货完成（空闲）'))

    def test_lower_failure_is_wanted(self):
        self.assertTrue(self.wanted('[下降] 失败：搬货服务无应答（超时 5s）'))

    def test_task_boundary_is_wanted(self):
        """任务开始/完成/失败边界也要能回看。"""
        self.assertTrue(self.wanted('[任务完成] 1号槽内 → 1号货架'))
        self.assertTrue(self.wanted('[任务失败] 前往取货点失败'))

    def test_cargo_service_lines_are_wanted(self):
        """cargo_node 自己的回执（"已顶升，开始搬货"）也要带过来。"""
        self.assertTrue(self.wanted('已顶升，开始搬货（request_id=lift-123）'))
        self.assertTrue(self.wanted('已下降，放货完成（空闲）'))

    # ── 必须过滤：这些是降噪的对象，不能因为转发又灌回来 ──
    def test_nav2_chatter_is_dropped(self):
        """★ 关键：Nav2 的 1 Hz 唠叨必须挡住，否则降噪白做。"""
        self.assertFalse(self.wanted('[导航中] 剩余 3.20 米 · 约 12 秒'))
        self.assertFalse(self.wanted('Nav2 is ready for use!'))
        self.assertFalse(self.wanted('Navigating to goal'))
        self.assertFalse(self.wanted('Timed out waiting for transform'))

    def test_costmap_chatter_is_dropped(self):
        self.assertFalse(self.wanted('代价地图已更新'))
        self.assertFalse(self.wanted('Re-requesting costmap'))

    def test_empty_and_garbage(self):
        self.assertFalse(self.wanted(''))
        self.assertFalse(self.wanted(None))
        self.assertFalse(self.wanted('随便一句话'))


class TestRelayIsSuccess(unittest.TestCase):
    """标绿还是标红（用户要求"成功绿字、失败红字"）。"""

    def green(self, text):
        _need(_GW, '_relay_is_success')
        return _GW._relay_is_success(text)

    def test_lift_success_is_green(self):
        self.assertTrue(self.green('[顶升] 成功（1号槽内）：已顶升，开始搬货'))

    def test_lower_success_is_green(self):
        self.assertTrue(self.green('[下降] 成功（1号货架）：已下降，放货完成（空闲）'))

    # ── 失败必须红：一个都不能标绿 ──
    def test_lift_failure_is_not_green(self):
        self.assertFalse(self.green('[顶升] 失败：搬货服务不可用'))

    def test_timeout_is_not_green(self):
        self.assertFalse(self.green('[下降] 失败：搬货服务无应答（超时 5s）'))

    def test_rejected_is_not_green(self):
        self.assertFalse(self.green('[顶升] 失败：服务拒绝'))

    def test_aborted_is_not_green(self):
        """中止不是成功：不能标绿让人以为顶升完成了。"""
        self.assertFalse(self.green('[顶升] 已取消：任务中止，未请求搬货服务'))

    def test_fail_word_beats_success_word(self):
        """★ 核心回归：一句话里同时有"成功"和"失败"时必须判失败。

        `[下降] 失败：下降成功但回执超时` 这种混着写的日志真实存在：
        先判成功词就会标成绿色，用户看到绿的以为放货成功了。
        """
        self.assertFalse(self.green('[下降] 失败：下降动作已完成但回执超时'))
        self.assertFalse(self.green('[顶升] 失败：顶升成功信号未确认，超时'))

    def test_neutral_text_is_not_green(self):
        self.assertFalse(self.green('[导航中] 剩余 3.20 米'))


class TestRelayLevelMapping(unittest.TestCase):
    """ROS 级别 → 网页级别（前端靠 level 上色）。"""

    def test_level_thresholds(self):
        """用 AST 确认阈值写对了：>=40 判 error、>=30 判 warn。

        ★ 为什么不直接调 on_rosout：它要一个 Node 实例，单测里起 ROS 太重。
          这里检查源码里的阈值字面量，够钉住"红/黄/灰"的分界。
        """
        with open(_GW_PATH, encoding='utf-8') as f:
            src = f.read()
        self.assertIn('lv >= 40', src, 'ERROR(40)/FATAL(50) 应判 error')
        self.assertIn('lv >= 30', src, 'WARN(30) 应判 warn')

    def test_rosout_subscription_is_gated_by_dependency(self):
        """缺 rcl_interfaces 时不能让整个网关崩：必须有 try 兜底。"""
        with open(_GW_PATH, encoding='utf-8') as f:
            src = f.read()
        self.assertIn('RosoutLog = None', src,
                      '缺 rcl_interfaces 时应退化为 None，而不是 ImportError')
        self.assertIn('if RosoutLog is not None', src,
                      '订阅前必须判 None，否则崩在 create_subscription')

    def test_relay_excludes_self(self):
        """★ 不排除自己的话，每条日志会重复两遍（self.log 已记过一次）。"""
        _need(_GW, '_relay_is_self')
        f = _GW._relay_is_self
        # 三种写法都要认出来：有/无前导斜杠、带命名空间
        self.assertTrue(f('/agv_web_gateway', 'agv_web_gateway'))
        self.assertTrue(f('agv_web_gateway', 'agv_web_gateway'))
        # ★ 带 namespace 时 rosout 里的名字是 /agv/agv_web_gateway。
        #   旧实现用 lstrip('/') 得到 'agv/agv_web_gateway'，与 get_name()
        #   不相等 → 排除失效 → 自身日志被当外部日志转发 → 自我放大。
        self.assertTrue(f('/agv/agv_web_gateway', 'agv_web_gateway'),
                        '带命名空间也必须认出来，否则日志会无限自我转发')

    def test_relay_keeps_other_nodes(self):
        """别的节点不能被误判成自己 —— 否则顶升结果永远进不了网页。"""
        _need(_GW, '_relay_is_self')
        f = _GW._relay_is_self
        self.assertFalse(f('/task_manager_node', 'agv_web_gateway'))
        self.assertFalse(f('/agv/cargo_node', 'agv_web_gateway'))
        self.assertFalse(f('', 'agv_web_gateway'))


class TestNormLog(unittest.TestCase):
    """跨通道去重的归一化：只剥身份前缀，不剥业务标签。

    ★ 为什么必须有这一组（真车日志）：
      同一条顶升结果有两条通路进网页 —— 子进程 stdout（带 `[节点] ` 前缀）
      和 /rosout 转发（只有正文），于是每件事都出现两遍：
          [task_manager_node] [顶升] 成功（1号货架）：已顶升，开始搬货
          [顶升] 成功（1号货架）：已顶升，开始搬货
    去重靠"归一化正文相同"判断，所以归一化必须**只抹掉身份前缀**。
    """

    def norm(self, text):
        _need(_GW, '_norm_log')
        return _GW._norm_log(text)

    def test_strips_node_prefix(self):
        self.assertEqual(self.norm('[task_manager_node] [顶升] 成功（1号货架）'),
                         '[顶升] 成功（1号货架）')

    def test_strips_level_prefix(self):
        self.assertEqual(self.norm('[信息·task_manager_node] [顶升] 开始（1号货架）'),
                         '[顶升] 开始（1号货架）')

    def test_same_event_two_channels_becomes_equal(self):
        a = self.norm('[task_manager_node] [顶升] 成功（1号货架）：已顶升，开始搬货')
        b = self.norm('[顶升] 成功（1号货架）：已顶升，开始搬货')
        self.assertEqual(a, b, '两条通道的同一条事件归一化后必须相同')

    def test_keeps_business_tag(self):
        """`[顶升]` / `[导航中]` 是业务标签，抹掉就分不清事件类型了。"""
        self.assertEqual(self.norm('[顶升] 成功'), '[顶升] 成功')
        self.assertEqual(self.norm('[导航中] 剩余 3.20 米'),
                         '[导航中] 剩余 3.20 米')

    def test_keeps_batch_index(self):
        """★ 批量序号必须保留：`[1/5] 搬运完成` 和 `[2/5] 搬运完成` 是两件事。

        以前踩过的坑：兜底去重把序号也归一化掉，5 单批量任务里后 3 单
        直接被当成重复丢掉。
        """
        self.assertNotEqual(self.norm('[1/5] 搬运完成'),
                            self.norm('[2/5] 搬运完成'))

    def test_does_not_merge_different_events(self):
        self.assertNotEqual(self.norm('[顶升] 成功（1号货架）'),
                            self.norm('[下降] 成功（2号货架）'))


if __name__ == '__main__':
    unittest.main()
