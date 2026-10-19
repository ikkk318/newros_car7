#!/usr/bin/env python3
"""中文地图名的 ASCII 发射路径回归测试（MapManager.launch_map_yaml）。

★ 为什么需要这套东西（真机踩坑，2026-10-19）：

  控制台自动生成的地图名是中文（「地图10月19日16时」）。保存、列出、预览全都
  正常，但**导航一加载就全黑**：自检说「未收到 /map」，页面上只剩一个起始点，
  小车也不显示。

  根因不在文件本身（pgm 是合法 P5、yaml 数值都对），而在"参数从控制台走到
  map_server"这条链上：中文在 agv_stack → navigation → nav2_bringup 的参数
  重写/传递过程中变成了 uXXXX 形态（PyYAML 对非 ASCII 的转义写法，反斜杠还丢了）。

  实测（同一条 navigation.launch.py，只换 map 的值）：

      map:=<中文路径>   →  Loading yaml file: …/maps/u5730u56FE10u6708…
                           Failed processing YAML … reason: bad file
                           Failed to bring up all requested nodes. Aborting bringup.
      map:=<ASCII 别名> →  Read map … 596 X 761 map @ 0.025 m/cell
                           Managed nodes are active

  没有 /map → AMCL 不发 map→odom → 网页没底图也没小车。
  ASCII 地图（world666map）一点问题都没有，所以只在"新建的中文名地图"上出现。

  修法不是改地图库命名，而是给 map_server 一份**纯 ASCII 的别名**：
      maps/.launch/<sha1前12位>/map.yaml   ← image 改成同目录的 map.pgm
      maps/.launch/<sha1前12位>/map.pgm    ← 软链到真实 pgm

  这里守住的边界：
    ① 中文名 → 返回的绝对路径必须**全 ASCII**，且文件真的存在；
    ② 别名里的 resolution/origin/阈值必须与真实 yaml 一致（否则导航坐标全错）；
    ③ 别名 pgm 读出来的字节要和真实 pgm 一样（软链/复制都得成立）；
    ④ ASCII 名 → 原样返回真实路径（world666map 的契约一分不改）；
    ⑤ 用户手改真实 yaml（比如 origin）后再调用 → 别名要跟着更新；
    ⑥ 删地图 → 别名目录一起清掉；
    ⑦ list_maps() 不能把 .launch 目录当成一张地图。
"""

import os
import shutil
import sys
import tempfile
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.join(_HERE, '..')
sys.path.insert(0, _SRC)

from agv_common.map_manager import LAUNCH_ALIAS_DIR, MapManager   # noqa: E402

_PGM = b'P5\n2 2\n255\n\x00\xcd\xcd\xfe'      # 2x2：占用/未知/未知/自由


def _make_map(root, name, resolution=0.025, origin=(-10.7, -9.81, 0.0)):
    """在 root/name 下造一份最小但合法（P5）的地图，返回 MapManager。"""
    d = os.path.join(root, name)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, f'{name}.pgm'), 'wb') as fh:
        fh.write(_PGM)
    with open(os.path.join(d, f'{name}.yaml'), 'w', encoding='utf-8') as fh:
        fh.write('image: %s.pgm\n'
                 'mode: trinary\n'
                 'resolution: %s\n'
                 'origin: [%s, %s, %s]\n'
                 'negate: 0\n'
                 'occupied_thresh: 0.65\n'
                 'free_thresh: 0.25\n'
                 % (name, resolution, origin[0], origin[1], origin[2]))
    return MapManager(root)


def _read_yaml(path):
    import yaml
    with open(path, encoding='utf-8') as fh:
        return yaml.safe_load(fh) or {}


class TestLaunchAlias(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp(prefix='agvalias_')
        self.addCleanup(shutil.rmtree, self.root, True)

    def test_chinese_name_returns_ascii_path(self):
        """① 中文地图名 → 返回的路径必须全 ASCII 且真实存在。"""
        mm = _make_map(self.root, '地图10月19日16时')
        got = mm.launch_map_yaml('地图10月19日16时')
        self.assertTrue(os.path.isabs(got), f'必须是绝对路径：{got}')
        self.assertTrue(os.path.abspath(got).isascii(),
                        f'map_server 只能吃 ASCII 路径，实际给了：{got}')
        self.assertTrue(os.path.isfile(got), f'别名文件不存在：{got}')

    def test_alias_keeps_geometry(self):
        """② 分辨率/原点/阈值必须原样带过去（否则坐标全错）。"""
        mm = _make_map(self.root, '中文图', resolution=0.05, origin=(1.5, -2.5, 0.7))
        got = mm.launch_map_yaml('中文图')
        data = _read_yaml(got)
        self.assertAlmostEqual(data['resolution'], 0.05)
        self.assertEqual([float(v) for v in data['origin']], [1.5, -2.5, 0.7])
        self.assertEqual(data['mode'], 'trinary')
        self.assertEqual(data['negate'], 0)
        self.assertAlmostEqual(data['occupied_thresh'], 0.65)
        self.assertAlmostEqual(data['free_thresh'], 0.25)

    def test_alias_image_is_ascii_and_readable(self):
        """③ image 必须是 ASCII 相对名，且指向与真实 pgm 相同的字节。"""
        mm = _make_map(self.root, '槽位图')
        got = mm.launch_map_yaml('槽位图')
        data = _read_yaml(got)
        self.assertEqual(data['image'], 'map.pgm',
                         'image 要用同目录的 ASCII 名，别留中文文件名')
        pgm = os.path.join(os.path.dirname(got), 'map.pgm')
        self.assertTrue(os.path.isfile(pgm), '别名目录里缺 map.pgm')
        with open(pgm, 'rb') as fh:
            self.assertEqual(fh.read(), _PGM, '别名 pgm 的内容与真实地图不一致')

    def test_ascii_name_untouched(self):
        """④ ASCII 名原样返回（world666map 行为不变）。"""
        mm = _make_map(self.root, 'world666map')
        got = mm.launch_map_yaml('world666map')
        self.assertEqual(got, os.path.join(self.root, 'world666map', 'world666map.yaml'))
        self.assertFalse(os.path.isdir(os.path.join(self.root, LAUNCH_ALIAS_DIR))
                         and os.listdir(os.path.join(self.root, LAUNCH_ALIAS_DIR)),
                         'ASCII 地图不该生成任何别名')

    def test_alias_follows_source_edits(self):
        """⑤ 手改真实 yaml 后再调用，别名要跟上（每次调用都重写）。"""
        mm = _make_map(self.root, '改过的图', origin=(0.0, 0.0, 0.0))
        first = mm.launch_map_yaml('改过的图')
        self.assertEqual([float(v) for v in _read_yaml(first)['origin']], [0.0, 0.0, 0.0])
        # 模拟用户/别的工具改了原点
        real = mm.map_yaml_file('改过的图')
        with open(real, 'w', encoding='utf-8') as fh:
            fh.write('image: 改过的图.pgm\nresolution: 0.05\n'
                     'origin: [3.5, -4.5, 0.0]\nnegate: 0\n'
                     'occupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n')
        second = mm.launch_map_yaml('改过的图')
        self.assertEqual([float(v) for v in _read_yaml(second)['origin']], [3.5, -4.5, 0.0],
                         '别名没有跟着更新，导航会按旧原点算坐标')

    def test_delete_map_removes_alias(self):
        """⑥ 删地图要把派生出来的别名一起清掉。"""
        mm = _make_map(self.root, '待删图')
        alias = mm.launch_map_yaml('待删图')
        self.assertTrue(os.path.isfile(alias))
        ok, msg = mm.delete_map('待删图')
        self.assertTrue(ok, msg)
        self.assertFalse(os.path.exists(os.path.dirname(alias)),
                         '别名目录残留了（它指向的 pgm 已经没了）')

    def test_list_maps_ignores_alias_dir(self):
        """⑦ .launch 不能被 list_maps() 当成一张地图。"""
        mm = _make_map(self.root, '正经图')
        mm.launch_map_yaml('正经图')
        self.assertEqual(mm.list_maps(), ['正经图'])

    def test_missing_map_returns_real_path(self):
        """地图不存在时原样返回真实路径，让调用方去报"图不存在"，别造空别名。"""
        mm = MapManager(self.root)
        got = mm.launch_map_yaml('根本没有')
        self.assertEqual(got, os.path.join(self.root, '根本没有', '根本没有.yaml'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
