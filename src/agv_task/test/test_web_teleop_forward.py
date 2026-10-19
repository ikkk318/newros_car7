#!/usr/bin/env python3
"""导航模式网页遥控的回归测试（/agv/web_teleop → /cmd_vel）。

★ 对应真机上报的现象（用户原话）：

    "导航模式时任务执行不了，建图模式时手动控制小车没反应，
     可能其他功能也有问题"

  查下来其中一条是：**导航模式下网页遥控根本没有接收方**。
  网关把速度发到 `/agv/web_teleop`（`stack_manager.TELEOP_TOPIC[MODE_NAV]`），
  而 `task_manager_node` 只订阅了 `/agv/command`、`/cmd_vel`、`/agv/manual_mode`
  —— 全项目没有任何节点接 `/agv/web_teleop`。于是导航模式按方向键，
  消息发出去就没了，车一动不动，也没有任何报错。

  本文件守三件事：
    ① 订阅契约：task_manager 必须订阅 `/agv/web_teleop`；
    ② 正常转发：非急停/非手动时把收到的 Twist 原样发到 `/cmd_vel`；
    ③ 互锁优先：急停或手动模式下一律发零速，不能把速度透传下去。

  用 AST 把 `on_web_teleop` 抽出来真跑（不 import rclpy），
  这样既不需要 ROS 环境，也不会因为节点构造牵扯一堆依赖。
"""

import ast
import os
import sys
import unittest
from types import SimpleNamespace

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.join(_HERE, '..')            # src/agv_task
_SRC = os.path.join(_HERE, '..', '..')      # src（跨包找网关）
sys.path.insert(0, _PKG)

_NODE_PATH = os.path.join(_PKG, 'agv_task', 'task_manager_node.py')


def _node_src():
    with open(_NODE_PATH, encoding='utf-8') as f:
        return f.read()


def _method_src(name):
    """从 TaskManagerNode 里抽一个方法的源码（保持缩进无关，unparse 后可直接 exec）。"""
    tree = ast.parse(_node_src())
    for cls in [n for n in tree.body if isinstance(n, ast.ClassDef)]:
        if cls.name != 'TaskManagerNode':
            continue
        for fn in cls.body:
            if isinstance(fn, ast.FunctionDef) and fn.name == name:
                return ast.unparse(fn)
    raise AssertionError(f'TaskManagerNode 里找不到方法 {name}')


class _Pub:
    """替身发布者：只记录收到了什么。"""

    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class _Safety:
    """替身安全层：只回答 estopped() / manual()。"""

    def __init__(self, estop=False, manual=False):
        self._estop = estop
        self._manual = manual

    def estopped(self):
        return self._estop

    def manual(self):
        return self._manual


class _Node:
    """替身节点：`on_web_teleop` 只用到这几个成员。"""

    def __init__(self, estop=False, manual=False):
        self.safety = _Safety(estop, manual)
        self.cmd_vel_pub = _Pub()
        self.zero_count = 0

    def _publish_zero_twist(self):
        self.zero_count += 1


def _call(msg, **kw):
    """把 `on_web_teleop` 从源码里抽出来，用替身 self 跑一次。"""
    ns = {}
    exec(_method_src('on_web_teleop'), ns)      # noqa: S102 - 只跑本仓库自己的源码
    node = _Node(**kw)
    ns['on_web_teleop'](node, msg)
    return node


class TestSubscriptionContract(unittest.TestCase):
    """① 订阅契约：没有这条订阅，导航遥控永远是空放。"""

    def test_subscribes_web_teleop(self):
        src = _node_src()
        self.assertIn('/agv/web_teleop', src,
                      'task_manager 必须订阅 /agv/web_teleop，否则导航模式遥控无人接收')
        self.assertIn('self.on_web_teleop', src,
                      '订阅回调必须指向 on_web_teleop')

    def test_teleop_topic_name_matches_gateway(self):
        """网关建图/导航两个话题名必须与节点侧对得上。

        ★ 这条守的是"改名不同步"：网关那边 TELEOP_TOPIC 改一个字，
          节点这边不会自动跟着变，症状还是"车不动"。
        """
        gw = os.path.join(_SRC, 'agv_web', 'agv_web', 'stack_manager.py')
        with open(gw, encoding='utf-8') as f:
            gw_src = f.read()
        self.assertIn("'/agv/web_teleop'", gw_src,
                      '导航模式的遥控话题应该是 /agv/web_teleop')
        self.assertIn('/agv/web_teleop', _node_src())


class TestForward(unittest.TestCase):
    """② 正常路径：原样转发。"""

    def test_forwards_twist_unchanged(self):
        msg = SimpleNamespace(linear=SimpleNamespace(x=0.4, y=0.0, z=0.0),
                              angular=SimpleNamespace(x=0.0, y=0.0, z=-0.8))
        node = _call(msg)
        self.assertEqual(node.zero_count, 0, '正常遥控不该发零速')
        self.assertEqual(len(node.cmd_vel_pub.sent), 1, '正常遥控必须转发一帧')
        self.assertIs(node.cmd_vel_pub.sent[0], msg,
                      '要原样转发（连角度一起），不能在中间改值')

    def test_zero_speed_still_forwarded(self):
        """松手时前端会补发一帧零速 —— 那也要转下去，车才停得住。"""
        msg = SimpleNamespace(linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                              angular=SimpleNamespace(x=0.0, y=0.0, z=0.0))
        node = _call(msg)
        self.assertEqual(len(node.cmd_vel_pub.sent), 1)


class TestInterlock(unittest.TestCase):
    """③ 互锁：急停 / 手动模式下宁可停车，也不能透传速度。"""

    def _msg(self):
        return SimpleNamespace(linear=SimpleNamespace(x=0.5, y=0.0, z=0.0),
                               angular=SimpleNamespace(x=0.0, y=0.0, z=0.0))

    def test_estop_blocks_and_stops(self):
        node = _call(self._msg(), estop=True)
        self.assertEqual(node.cmd_vel_pub.sent, [],
                         '急停时绝不能把速度转发给底盘')
        self.assertEqual(node.zero_count, 1,
                         '急停时要主动发零速：车可能正带着上一次的速度在跑')

    def test_manual_mode_blocks_and_stops(self):
        node = _call(self._msg(), manual=True)
        self.assertEqual(node.cmd_vel_pub.sent, [],
                         '手动模式（互锁）时不能透传导航遥控')
        self.assertEqual(node.zero_count, 1)


if __name__ == '__main__':
    unittest.main()
