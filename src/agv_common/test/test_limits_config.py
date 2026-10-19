"""
limits / config_loader 模块单元测试：速度限幅、YAML 合并与覆盖加载。
"""

import os                                # use 临时目录与路径拼接
import textwrap                          # use 便于书写多行 YAML 文本

from agv_common.limits import VelocityLimits        # use 被测：速度限幅
from agv_common.config_loader import (              # use 被测：配置加载
    deep_merge, load_config, load_yaml)


def test_velocity_limits():              # # 线/角速度对称限幅
    lim = VelocityLimits(max_linear=1.0, max_angular=1.0)
    assert lim.clamp_linear(0.4) == 0.4
    assert lim.clamp_linear(5.0) == 1.0
    assert lim.clamp_linear(-5.0) == -1.0
    v, w = lim.clamp_twist(3.0, -3.0)
    assert (v, w) == (1.0, -1.0)


def test_deep_merge_scalar_and_nested(): # # 标量覆盖 + 嵌套字典递归
    base = {'a': 1, 'b': {'x': 1, 'y': 2}}
    over = {'a': 9, 'b': {'y': 20, 'z': 30}}
    out = deep_merge(base, over)
    assert out == {'a': 9, 'b': {'x': 1, 'y': 20, 'z': 30}}
    assert base == {'a': 1, 'b': {'x': 1, 'y': 2}}     # # 原字典不被污染


def test_load_yaml_empty(tmp_path):      # # 空文件返回 {}
    p = tmp_path / 'empty.yaml'
    p.write_text('', encoding='utf-8')
    assert load_yaml(str(p)) == {}


def test_load_config_override(tmp_path): # # default → sim 覆盖加载
    # 造出 default/ 与 sim/ 两个层，sim 覆盖 default
    (tmp_path / 'default').mkdir()
    (tmp_path / 'sim').mkdir()
    (tmp_path / 'default' / 'task.yaml').write_text(
        textwrap.dedent('''
            speed: 0.5
            retries: 1
            nested:
              a: 1
              b: 2
        '''), encoding='utf-8')
    (tmp_path / 'sim' / 'task.yaml').write_text(
        textwrap.dedent('''
            speed: 0.8
            nested:
              b: 20
        '''), encoding='utf-8')

    merged = load_config(str(tmp_path), overrides=['sim'], names=['task'])
    assert merged['task']['speed'] == 0.8          # # 被 sim 覆盖
    assert merged['task']['retries'] == 1          # # 保留 default
    assert merged['task']['nested'] == {'a': 1, 'b': 20}   # # 嵌套逐键覆盖


def test_load_config_missing_file(tmp_path):   # # 文件全层都不存在则跳过
    (tmp_path / 'default').mkdir()
    merged = load_config(str(tmp_path), overrides=[], names=['nope'])
    assert merged == {}
