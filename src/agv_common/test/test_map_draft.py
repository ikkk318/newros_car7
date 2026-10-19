#!/usr/bin/env python3
"""建图草稿并入正式地图的回归测试（MapManager.promote_draft / save_map）。

★ 为什么需要这套东西：

  建图模式现在也起 waypoint_node（建图时能顺手标点位），但**地图名要等
  "保存"那一刻才定**，没法直接把点位写进某张图。所以建图期间点位落在
  固定草稿目录 `maps/_mapping/`，保存地图时再并入新图。

  这里守四条容易踩的边界：

    ① 草稿有、目标没有 → 并进来（这是正常路径）；
    ② 目标已经有 waypoints.yaml → **绝不覆盖**（可能是用户手工维护的点位，
       覆盖等于静默删数据）；
    ③ 没有草稿 → 什么也不做，仍按老逻辑建空模板（不能把保存流程搞挂）；
    ④ 顺序：`save_map` 必须先并草稿再建空模板。反了的话，空模板里的
       "起始点"会把位置占住，草稿永远并不进来 —— 用户会觉得"建图存的点丢了"。

  用临时目录真跑文件操作，不碰真实地图库。
"""

import os
import sys
import tempfile
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(_HERE, '..')
sys.path.insert(0, _SRC)

from agv_common.map_manager import MAPPING_DRAFT, MapManager   # noqa: E402


_PGM = b'P5\n1 1\n255\n\x00'          # 1x1 灰度，足够让 save_map 的 isfile 通过
_MAP_YAML = 'image: src.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n'


def _touch_map(dirpath, stem):
    """在 dirpath 里造一对最小可用的 pgm/yaml（save_map 只检查存在 + 复制）。"""
    os.makedirs(dirpath, exist_ok=True)
    pgm = os.path.join(dirpath, stem + '.pgm')
    yml = os.path.join(dirpath, stem + '.yaml')
    with open(pgm, 'wb') as f:
        f.write(_PGM)
    with open(yml, 'w', encoding='utf-8') as f:
        f.write(_MAP_YAML)
    return pgm, yml


class TestPromoteDraft(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = self._tmp.name
        self.mm = MapManager(self.root)

    def tearDown(self):
        self._tmp.cleanup()

    def _write(self, path, text):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text)

    def _read(self, path):
        with open(path, encoding='utf-8') as f:
            return f.read()

    def test_copies_both_files_when_target_missing(self):
        src = self.mm.map_paths(MAPPING_DRAFT)
        self._write(src['waypoints'], 'frame_id: map\nwaypoints:\n  1号槽内: {x: 3.0}\n')
        self._write(src['routes'], 'frame_id: map\nroutes:\n  巡检线: []\n')

        n = self.mm.promote_draft('新图')

        self.assertEqual(n, 2, '点位与路线两个文件都该并进来')
        self.assertIn('1号槽内', self._read(self.mm.waypoints_file('新图')))
        self.assertIn('巡检线', self._read(self.mm.routes_file('新图')))

    def test_never_overwrites_existing_target(self):
        src = self.mm.map_paths(MAPPING_DRAFT)
        self._write(src['waypoints'], 'frame_id: map\nwaypoints:\n  草稿点: {x: 1.0}\n')
        dst_wp = self.mm.waypoints_file('老图')
        self._write(dst_wp, 'frame_id: map\nwaypoints:\n  用户手工点: {x: 9.0}\n')

        n = self.mm.promote_draft('老图')

        self.assertEqual(n, 0, '目标已有 waypoints 就不该复制')
        text = self._read(dst_wp)
        self.assertIn('用户手工点', text, '用户自己的点位必须还在')
        self.assertNotIn('草稿点', text, '不能拿草稿覆盖用户数据')

    def test_no_draft_is_noop(self):
        self.assertEqual(self.mm.promote_draft('空图'), 0)

    def test_draft_name_is_rejected_as_target(self):
        """把草稿目录本身当目标会自己覆盖自己，直接拒绝。"""
        self.assertEqual(self.mm.promote_draft(MAPPING_DRAFT), 0)

    def test_draft_never_listed_as_map(self):
        """草稿目录没有 <name>.pgm，不能被地图库列出来。"""
        src = self.mm.map_paths(MAPPING_DRAFT)
        self._write(src['waypoints'], 'frame_id: map\nwaypoints: {}\n')
        self.assertNotIn(MAPPING_DRAFT, self.mm.list_maps())


class TestSaveMapOrder(unittest.TestCase):
    """④ 顺序契约：并草稿必须发生在建空模板之前。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = self._tmp.name
        self.mm = MapManager(self.root)

    def tearDown(self):
        self._tmp.cleanup()

    def test_saved_map_keeps_building_points(self):
        src = self.mm.map_paths(MAPPING_DRAFT)
        os.makedirs(os.path.dirname(src['waypoints']), exist_ok=True)
        with open(src['waypoints'], 'w', encoding='utf-8') as f:
            f.write('frame_id: map\nwaypoints:\n  建图时标的点: {x: 2.5, y: -1.0, yaw: 0.0}\n')

        pgm, yml = _touch_map(self._tmp.name, 'src')     # 造 map_saver 的产出
        ok, msg = self.mm.save_map('world666map', pgm, yml)

        self.assertTrue(ok, msg)
        text = self.mm.waypoints_file('world666map')
        with open(text, encoding='utf-8') as f:
            body = f.read()
        self.assertIn('建图时标的点', body,
                      '建图时存的点必须跟着地图走（顺序反了就会被"起始点"占位）')
        self.assertNotIn('起始点', body,
                         '有草稿时不该再写默认的"起始点"模板')
        self.assertIn('并入建图草稿', msg, '保存结果里要告诉用户点位已并入')

    def test_no_draft_still_creates_templates(self):
        pgm, yml = _touch_map(self._tmp.name, 'src2')
        ok, _msg = self.mm.save_map('干净图', pgm, yml)
        self.assertTrue(ok)
        self.assertTrue(os.path.isfile(self.mm.waypoints_file('干净图')),
                        '没有草稿时要按老逻辑建空模板，否则 waypoint_node 找不到文件')
        self.assertTrue(os.path.isfile(self.mm.routes_file('干净图')))


if __name__ == '__main__':
    unittest.main()
