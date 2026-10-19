"""
test_map_manager —— 地图库管理纯逻辑测试（不依赖 ROS）。

覆盖：合法名校验、建目录、存/列/删、点位文件绑定、yaml image 字段修正。
"""
import os
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from agv_common.map_manager import MapManager, valid_name   # noqa: E402


def _fake_map(tmp_path, tag='x'):
    """造一对假的 pgm/yaml（内容不重要，只要文件存在）。"""
    pgm = tmp_path / f'{tag}.pgm'
    yml = tmp_path / f'{tag}.yaml'
    pgm.write_bytes(b'P5\n1 1\n255\n\xff')
    yml.write_text('image: something_else.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    return str(pgm), str(yml)


def test_valid_name():
    assert valid_name('仓库A')[0] is True
    assert valid_name('')[0] is False
    assert valid_name('a/b')[0] is False
    assert valid_name('..')[0] is False


def test_save_list_delete(tmp_path):
    mm = MapManager(str(tmp_path / 'lib'))
    assert mm.list_maps() == []
    pgm, yml = _fake_map(tmp_path, 'm1')
    ok, msg = mm.save_map('m1', pgm, yml)
    assert ok, msg
    assert mm.list_maps() == ['m1']
    assert mm.map_exists('m1')
    # 点位文件被自动补上
    assert os.path.isfile(mm.waypoints_file('m1'))
    # yaml 的 image 字段被修正为同名 pgm
    data = yaml.safe_load(open(mm.map_paths('m1')['yaml'], encoding='utf-8'))
    assert data['image'] == 'm1.pgm'
    # 删除：整目录（含点位）一起走
    ok, _ = mm.delete_map('m1')
    assert ok and mm.list_maps() == []
    assert not os.path.isdir(mm.map_paths('m1')['dir'])


def test_save_missing_source(tmp_path):
    mm = MapManager(str(tmp_path / 'lib'))
    ok, msg = mm.save_map('m', '/nope.pgm', '/nope.yaml')
    assert ok is False and '不存在' in msg


def test_delete_missing(tmp_path):
    mm = MapManager(str(tmp_path / 'lib'))
    ok, msg = mm.delete_map('nope')
    assert ok is False and '不存在' in msg


def test_valid_name_blocks_traversal(tmp_path):
    mm = MapManager(str(tmp_path / 'lib'))
    ok, _ = mm.save_map('../evil', '/a', '/b')
    assert ok is False


def test_validate_incomplete(tmp_path):
    mm = MapManager(str(tmp_path / 'lib'))
    # 只有 pgm 没 yaml：不算有效地图
    mm.create_dir('half')
    open(os.path.join(mm.map_paths('half')['pgm']), 'wb').write(b'x')
    assert mm.list_maps() == []
    ok, msg = mm.validate('half')
    assert ok is False
