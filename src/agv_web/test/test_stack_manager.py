#!/usr/bin/env python3
"""
stack_manager 的单元测试（纯逻辑，不需要 ROS）。

覆盖三条最容易出错的约定：
① 建图遥控必须发 /cmd_vel_raw（建图栈没有 arbiter，发错车不动）
② 导航模式不能给 agv_stack 传 use_sim_time（它没声明这个参数）
③ 切模式必须先停后起（两套栈并存会抢话题，表现为原地转）
"""

import os
import sys
import unittest

# ★ 要把 package 的【父目录】加进 sys.path：agv_web/agv_web/stack_manager.py
#   里有一层同名目录，加到 agv_web 这一层会 import 不到。
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))          # src/agv_web
sys.path.insert(0, os.path.join(_HERE, '..', '..'))    # src

from agv_web.stack_manager import (      # noqa: E402
    MODE_MAPPING, MODE_NAV, MODE_TEXT, MODES, TELEOP_TOPIC,
    StackError, StackPlan, build_launch_command, mode_of_launch_text,
    plan_switch, teleop_topic_for, validate_map_name,
)


class TestTeleopTopic(unittest.TestCase):
    """遥控话题分流——建图能不能动起来全靠这条。"""

    def test_nav_uses_interlock_topic(self):
        # 导航模式必须走 /agv/web_teleop，由 task_manager 校验手动模式后转发
        self.assertEqual(teleop_topic_for(MODE_NAV), '/agv/web_teleop')

    def test_mapping_uses_raw_topic(self):
        # ★ 建图栈没有 velocity_arbiter，只有 /cmd_vel_raw 能驱动底盘
        self.assertEqual(teleop_topic_for(MODE_MAPPING), '/cmd_vel_raw')

    def test_mapping_and_nav_never_share_topic(self):
        # 两者若相同，其中一个模式必然不动
        self.assertNotEqual(teleop_topic_for(MODE_NAV),
                            teleop_topic_for(MODE_MAPPING))

    def test_unknown_mode_falls_back_to_nav(self):
        # 未知模式按导航处理：宁可车不动，也不要绕过互锁直驱底盘
        self.assertEqual(teleop_topic_for('whatever'), TELEOP_TOPIC[MODE_NAV])
        self.assertEqual(teleop_topic_for(''), TELEOP_TOPIC[MODE_NAV])
        self.assertEqual(teleop_topic_for(None), TELEOP_TOPIC[MODE_NAV])

    def test_case_and_space_insensitive(self):
        self.assertEqual(teleop_topic_for('  MAPPING '), '/cmd_vel_raw')


class TestBuildLaunchCommand(unittest.TestCase):
    """命令拼装。"""

    def _joined(self, cmd):
        return ' '.join(cmd)

    def test_returns_argv_not_string(self):
        # 必须是列表：地图名带空格时不该被 shell 拆开
        cmd = build_launch_command(MODE_NAV, map_yaml='/a/b.yaml')
        self.assertIsInstance(cmd, list)
        self.assertTrue(all(isinstance(x, str) for x in cmd))

    def test_nav_uses_agv_stack_not_navigation(self):
        # ★ 只用 navigation.launch.py 会漏掉 task_manager/arbiter，
        #   表现为"能导航但没任务层、且不避障"
        joined = self._joined(build_launch_command(MODE_NAV, map_yaml='/m.yaml'))
        self.assertIn('agv_stack.launch.py', joined)
        self.assertNotIn('navigation.launch.py', joined)   # 不能被后者顶掉

    def test_nav_requires_map_passed_through(self):
        joined = self._joined(build_launch_command(MODE_NAV, map_yaml='/maps/a.yaml'))
        self.assertIn('map_yaml:=/maps/a.yaml', joined)

    def test_nav_does_not_pass_use_sim_time(self):
        # ★ agv_stack.launch.py 没有声明 use_sim_time，
        #   它由 with_sim 推导；硬传是坑
        joined = self._joined(build_launch_command(MODE_NAV, map_yaml='/m.yaml',
                                                   use_sim_time=True))
        self.assertNotIn('use_sim_time:=', joined)
        self.assertIn('with_sim:=true', joined)

    def test_nav_passes_waypoint_and_routes(self):
        joined = self._joined(build_launch_command(
            MODE_NAV, map_yaml='/m.yaml',
            waypoint_file='/w.yaml', routes_file='/r.yaml'))
        self.assertIn('waypoint_file:=/w.yaml', joined)
        self.assertIn('routes_file:=/r.yaml', joined)

    def test_mapping_uses_mapping_launch(self):
        joined = self._joined(build_launch_command(MODE_MAPPING))
        self.assertIn('mapping.launch.py', joined)
        self.assertIn('agv_navigation', joined)

    def test_mapping_passes_use_sim_time(self):
        # 建图栈【有】这个参数，可以传
        joined = self._joined(build_launch_command(MODE_MAPPING, use_sim_time=True))
        self.assertIn('use_sim_time:=true', joined)

    def test_mapping_passes_world(self):
        joined = self._joined(build_launch_command(MODE_MAPPING, world='/w.world'))
        self.assertIn('world:=/w.world', joined)

    def test_mapping_passes_waypoint_files(self):
        """建图栈也要带点位/路线文件（建图时能顺手标点，保存时并入新图）。"""
        joined = self._joined(build_launch_command(
            MODE_MAPPING, waypoint_file='/m/waypoints.yaml',
            routes_file='/m/routes.yaml'))
        self.assertIn('waypoint_file:=/m/waypoints.yaml', joined)
        self.assertIn('routes_file:=/m/routes.yaml', joined)

    def test_mapping_omits_empty_waypoint_file(self):
        """没传就别拼空参数：launch 收到空串会当成当前目录，反而更乱。"""
        joined = self._joined(build_launch_command(MODE_MAPPING))
        self.assertNotIn('waypoint_file:=', joined)
        self.assertNotIn('routes_file:=', joined)

    def test_rviz_and_gui_default_off(self):
        # 跑在车机上，默认不该开窗口
        joined = self._joined(build_launch_command(MODE_MAPPING))
        self.assertIn('gui:=false', joined)
        self.assertIn('rviz:=false', joined)

    def test_rviz_gui_can_be_enabled(self):
        joined = self._joined(build_launch_command(MODE_MAPPING, gui=True, rviz=True))
        self.assertIn('gui:=true', joined)
        self.assertIn('rviz:=true', joined)

    def test_map_name_with_spaces_is_quoted(self):
        # 名字带空格时必须被引起来，否则 launch 只收到前半个词
        cmd = build_launch_command(MODE_NAV, map_yaml='/maps/my map.yaml')
        joined = ' '.join(cmd)
        self.assertTrue("'map_yaml:=/maps/my map.yaml'" in joined
                        or '"map_yaml:=/maps/my map.yaml"' in joined,
                        f'地图名没被引用起来：{joined[-120:]}')

    def test_sources_ros_setup(self):
        joined = self._joined(build_launch_command(MODE_NAV, map_yaml='/m.yaml'))
        self.assertIn('source /opt/ros/humble/setup.bash', joined)

    def test_sources_workspace_setup_when_given(self):
        joined = self._joined(build_launch_command(
            MODE_NAV, map_yaml='/m.yaml', ws_setup='/ws/install/setup.bash'))
        self.assertIn('/ws/install/setup.bash', joined)
        # ROS 环境要先于工作区
        self.assertLess(joined.index('/opt/ros/humble/setup.bash'),
                        joined.index('/ws/install/setup.bash'))

    def test_unknown_mode_raises(self):
        with self.assertRaises(StackError):
            build_launch_command('teleport')

    def test_extra_args_override(self):
        joined = self._joined(build_launch_command(
            MODE_MAPPING, extra_args={'gui': 'true'}))
        self.assertIn('gui:=true', joined)
        self.assertNotIn('gui:=false', joined)


class TestModeOfLaunchText(unittest.TestCase):
    """从命令文本反推模式（判断"已经在跑的是哪套栈"）。"""

    def test_recognizes_mapping(self):
        self.assertEqual(mode_of_launch_text('ros2 launch agv_navigation mapping.launch.py'),
                         MODE_MAPPING)

    def test_recognizes_nav(self):
        self.assertEqual(mode_of_launch_text('ros2 launch agv_bringup agv_stack.launch.py'),
                         MODE_NAV)

    def test_unknown_returns_empty(self):
        self.assertEqual(mode_of_launch_text('ros2 run foo bar'), '')
        self.assertEqual(mode_of_launch_text(''), '')
        self.assertEqual(mode_of_launch_text(None), '')

    def test_roundtrip_with_build(self):
        # 拼出来的命令应能被反向识别（保证两处字符串一致）
        for mode in MODES:
            cmd = build_launch_command(mode, map_yaml='/m.yaml')
            self.assertEqual(mode_of_launch_text(' '.join(cmd)), mode,
                             f'{mode} 无法被反向识别')


class TestPlanSwitch(unittest.TestCase):
    """切换决策：先停后起，绝不允许两套栈并存。"""

    def test_start_when_nothing_running(self):
        p = plan_switch('', False, MODE_NAV)
        self.assertEqual(p.actions, ['start'])
        self.assertEqual(p.mode, MODE_NAV)

    def test_same_mode_is_idempotent(self):
        # 已在跑就别重启：Nav2 激活要 15~40 秒
        p = plan_switch(MODE_NAV, True, MODE_NAV)
        self.assertEqual(p.actions, [])
        self.assertIn('无需重启', p.reason)

    def test_switch_mode_must_stop_first(self):
        # ★ 两套栈并存会抢话题/服务，表现为原地转、无响应
        p = plan_switch(MODE_MAPPING, True, MODE_NAV)
        self.assertEqual(p.actions, ['stop', 'start'])

    def test_switch_back_also_stops_first(self):
        p = plan_switch(MODE_NAV, True, MODE_MAPPING)
        self.assertEqual(p.actions, ['stop', 'start'])

    def test_stop_running(self):
        p = plan_switch(MODE_NAV, True, '')
        self.assertEqual(p.actions, ['stop'])
        self.assertEqual(p.mode, '')

    def test_stop_when_not_running(self):
        p = plan_switch('', False, '')
        self.assertEqual(p.actions, [])

    def test_running_but_unknown_mode_forces_restart(self):
        # 状态不明（比如被外部命令起的）也得先停，否则可能两套并存
        p = plan_switch('', True, MODE_NAV)
        self.assertEqual(p.actions, ['stop', 'start'])

    def test_unknown_target_raises(self):
        with self.assertRaises(StackError):
            plan_switch('', False, 'fly')

    def test_plan_repr_is_readable(self):
        self.assertIn('start', repr(plan_switch('', False, MODE_NAV)))


class TestValidateMapName(unittest.TestCase):
    """导航模式没地图起不来。"""

    def test_nav_without_map_rejected(self):
        ok, reason = validate_map_name(MODE_NAV, '')
        self.assertFalse(ok)
        self.assertIn('地图', reason)

    def test_nav_with_map_ok(self):
        ok, _ = validate_map_name(MODE_NAV, '/maps/a.yaml')
        self.assertTrue(ok)

    def test_mapping_never_needs_map(self):
        # 建图本来就是在造地图，当然不需要预置地图
        for v in ('', None, '/whatever.yaml'):
            ok, _ = validate_map_name(MODE_MAPPING, v)
            self.assertTrue(ok, f'{v!r} 不该被拒绝')

    def test_whitespace_map_treated_as_empty(self):
        ok, _ = validate_map_name(MODE_NAV, '   ')
        self.assertFalse(ok)


class TestModeText(unittest.TestCase):
    """中文显示。"""

    def test_all_modes_have_chinese_text(self):
        for m in MODES:
            self.assertTrue(MODE_TEXT[m].strip(), f'{m} 缺中文名')
            self.assertNotEqual(MODE_TEXT[m], m, f'{m} 没翻译')

    def test_texts_are_distinct(self):
        self.assertNotEqual(MODE_TEXT[MODE_NAV], MODE_TEXT[MODE_MAPPING])


class TestStackPlan(unittest.TestCase):
    """StackPlan 容器。"""

    def test_actions_copied_not_shared(self):
        src = ['start']
        p = StackPlan(src, MODE_NAV, 'x')
        src.append('stop')          # 改外部列表不该影响已建好的 plan
        self.assertEqual(p.actions, ['start'])

    def test_defaults(self):
        p = StackPlan([])
        self.assertEqual(p.mode, '')
        self.assertEqual(p.reason, '')


if __name__ == '__main__':
    unittest.main(verbosity=2)
