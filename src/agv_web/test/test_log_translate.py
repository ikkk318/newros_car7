"""日志中文化模块的单测（纯逻辑，不需要 ROS）。

这些测试守的是用户的原始抱怨：
"还有一堆英文日志，根本看不懂，给我日志中文化"

★ 但"中文化"不能变成"翻译错误"或"把信息翻丢"，所以除了检查
  "该翻的翻了"，还要检查"没把握的保留原文"。
"""

import os
import re
import sys
import unittest

# 直接按源码路径导入，避开 colcon 安装目录（改完不 build 也能测）
_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from agv_web import log_translate as LT          # noqa: E402

# 中文字符（任一即认为"已中文化"）
_HAS_ZH = re.compile(r'[\u4e00-\u9fff]')


class TestNoise(unittest.TestCase):
    """必须整条丢掉的噪音。"""

    def test_launch_boilerplate_is_noise(self):
        for s in (
            '[INFO] [launch]: All log files can be found below /home/x/.ros/log/2026',
            '[INFO] [launch]: Default logging verbosity is set to INFO',
        ):
            self.assertTrue(LT.is_noise(s), f'应丢弃：{s}')

    def test_process_started_is_noise(self):
        # launch 的子进程启动回显：我们自己已经记了"已启动XX模式"
        self.assertTrue(LT.is_noise('[INFO] [map_server-3]: process started with pid [123]'))

    def test_bond_timer_is_noise(self):
        self.assertTrue(LT.is_noise('[INFO] [bt_navigator-7]: Creating bond timer'))

    def test_real_error_is_not_noise(self):
        # ★ 反向检查：报错绝不能被当噪音丢掉
        for s in (
            '[ERROR] [map_server-3]: Failed to load map: file not found',
            '[WARNING] [amcl-2]: Timed out waiting for transform from base_link to map',
        ):
            self.assertFalse(LT.is_noise(s), f'不能丢：{s}')


class TestTranslateCommon(unittest.TestCase):
    """高频日志要真的变成中文。"""

    def _zh(self, raw):
        out = LT.translate(raw)
        self.assertTrue(_HAS_ZH.search(out), f'没翻成中文：{raw!r} → {out!r}')
        return out

    def test_tf_timeout(self):
        out = self._zh('[WARNING] [map_server-3]: Timed out waiting for transform'
                       ' from base_link to map')
        self.assertIn('坐标变换', out)
        self.assertIn('base_link', out, '帧名要保留，排查时靠它')

    def test_plan_collision_fully_chinese(self):
        # ★ 这条曾经只翻了一半，留下 'Failed to create plan:'
        out = self._zh('[ERROR] [planner_server-9]: Failed to create plan:'
                       ' Goal is in collision')
        self.assertNotIn('Failed to create plan', out,
                         '整句都要翻掉，不能留英文前半截')
        self.assertIn('障碍物', out)

    def test_controller_failure(self):
        out = self._zh('[ERROR] [controller_server-5]: Failed to compute velocity command')
        self.assertIn('速度控制', out)

    def test_map_load_failure_keeps_path(self):
        out = self._zh('[ERROR] [map_server-3]: Failed to load map: file not found')
        self.assertIn('地图加载失败', out)
        self.assertIn('file not found', out, '具体原因要保留，不能只说"失败"')

    def test_amcl_initial_pose(self):
        out = self._zh('[INFO] [amcl-2]: Received initial pose')
        self.assertIn('位姿', out)

    def test_cmd_vel_timeout(self):
        out = self._zh('[WARNING] [velocity_arbiter-4]: cmd_vel timeout,'
                       ' publishing zero velocity')
        self.assertIn('速度指令超时', out)
        self.assertIn('停车', out)

    def test_planning_succeeded_with_duration(self):
        out = self._zh('[INFO] [planner_server-9]: Planning succeeded in 32ms')
        self.assertIn('路径规划成功', out)
        self.assertIn('32', out, '耗时数字要保留')


class TestPreserveUnknown(unittest.TestCase):
    """翻译不了的内容必须原样保留，不能丢信息。"""

    def test_unknown_body_kept(self):
        raw = '[INFO] [some_node-2]: Totally unknown message xyz'
        out = LT.translate(raw)
        self.assertIn('Totally unknown message xyz', out,
                      '没规则的日志要保留原文')

    def test_unknown_node_name_kept(self):
        out = LT.translate('[INFO] [weird_thing-9]: hello')
        self.assertIn('weird_thing', out, '不认识的节点名保留原名，别硬翻')

    def test_node_roles_translated(self):
        cases = {
            '[INFO] [map_server-1]: x': '地图服务',
            '[INFO] [amcl-1]: x': '定位',
            '[INFO] [planner_server-1]: x': '路径规划',
            '[INFO] [controller_server-1]: x': '运动控制',
            '[INFO] [velocity_arbiter-1]: x': '速度仲裁',
            '[INFO] [task_manager-1]: x': '任务管理',
        }
        for raw, want in cases.items():
            self.assertIn(want, LT.translate(raw), f'{raw} 应含 {want}')

    def test_sequence_number_stripped(self):
        # launch 给节点加的 -3 序号对用户没意义，去掉（但名字要留）
        out = LT.translate('[INFO] [map_server-37]: x')
        self.assertNotIn('-37', out)
        self.assertIn('地图服务', out)


class TestLaunchPrefix(unittest.TestCase):
    """ros2 launch 的前缀 + ROS 自己的 `[节点]:` 不能出现两遍。

    ★ 真车日志原来长这样（用户贴的）：
        [task_manager_node] [task_manager_node]: [顶升] 开始（1号货架）
        [路径规划] [planner_server]: Planner loop missed its desired rate...
      launch 在每行前面加了一层 `[节点名-N] `，被 _split_prefix 拆走并翻成
      中文；但 ROS 自己那份 `[节点名]:` 还留在正文里，于是同一个名字占两遍，
      白白吃掉半行宽度（手机上本来就窄）。
    """

    def test_node_name_appears_once(self):
        raw = ('[task_manager_node-1] [INFO] [1792328059.586507734] '
               '[task_manager_node]: [顶升] 开始（1号货架）')
        out = LT.translate(raw)
        self.assertEqual(out.count('task_manager_node'), 1, out)
        self.assertIn('[顶升] 开始（1号货架）', out)

    def test_chinese_role_does_not_keep_english_duplicate(self):
        raw = ('[planner_server-1] [WARN] [1792328059.586507734] '
               '[planner_server]: Planner loop missed its desired rate')
        out = LT.translate(raw)
        self.assertIn('[路径规划]', out)
        self.assertNotIn('[planner_server]', out)

    def test_business_tag_is_not_eaten(self):
        """`[搬运]` 这类业务标签没有紧跟冒号，不能被当成节点前缀剔掉。"""
        out = LT.translate(
            '[INFO] [task_manager-1]: [搬运] 运输到放货点「2号货架」')
        self.assertIn('[搬运] 运输到放货点', out)


class TestLevel(unittest.TestCase):
    """级别判定要准，否则界面上严重性标错。"""

    def test_error(self):
        self.assertEqual(LT.guess_level('[ERROR] [x-1]: boom'), 'error')
        self.assertEqual(LT.guess_level('[FATAL] [x-1]: boom'), 'error')
        self.assertEqual(LT.guess_level('Traceback (most recent call last):'), 'error')

    def test_warn(self):
        self.assertEqual(LT.guess_level('[WARN] [x-1]: hmm'), 'warn')
        self.assertEqual(LT.guess_level('[WARNING] [x-1]: hmm'), 'warn')

    def test_info(self):
        self.assertEqual(LT.guess_level('[INFO] [x-1]: ok'), 'info')


class TestTranslateLines(unittest.TestCase):
    """批量处理：过滤 + 翻译 + 截断一次做完。"""

    def test_mixed_batch(self):
        raw = [
            '[INFO] [launch]: Default logging verbosity is set to INFO',   # 丢
            '[WARNING] [amcl-2]: Timed out waiting for transform from base_link to map',
            '',
            '[ERROR] [map_server-3]: Failed to load map: x',
        ]
        out = LT.translate_lines(raw)
        self.assertEqual(len(out), 2, f'噪音行应被丢掉，实际 {out}')
        lv = [l for l, _ in out]
        self.assertEqual(lv, ['warn', 'error'])

    def test_truncate(self):
        long = '[INFO] [x-1]: ' + 'y' * 900
        out = LT.translate_lines([long], max_len=50)
        self.assertEqual(len(out), 1)
        self.assertLessEqual(len(out[0][1]), 60, '超长行要截断')

    def test_empty_input(self):
        self.assertEqual(LT.translate_lines([]), [])
        self.assertEqual(LT.translate_lines(['', '   ', None]), [])


class TestNoCrash(unittest.TestCase):
    """任何诡异输入都不能把日志线程搞崩（它是长跑线程）。"""

    def test_weird_inputs(self):
        for s in ('[', '[]', '[:]', '[ERROR]', '[ERROR] [x]:', '\x00', 'a' * 5000,
                  '[INFO] [x-1]: \\1 \\2', '[ERROR] [中文节点-1]: 测试'):
            try:
                LT.translate(s)
                LT.is_noise(s)
                LT.guess_level(s)
            except Exception as exc:      # pragma: no cover
                self.fail(f'输入 {s!r} 抛异常：{exc}')


if __name__ == '__main__':
    unittest.main()
