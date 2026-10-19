"""
config_loader —— YAML 参数加载与 default→覆盖层合并。

对应规格书 §5.1 参数机制：任何数值不写死在代码里，全部进配置；
加载顺序 default → 覆盖层（后者盖前者），切换仿真/实车只改一层。

本模块是纯 Python，只用 PyYAML，不依赖 ROS。各节点在构造时调用 load_config()
把最终参数拿到手，再 declare/declare_parameters 给 ROS 参数系统。

配置文件布局（agv_bringup/config/）：
    default/   与硬件无关的公共默认值（唯一必有的层）
    real/      实车覆盖（仿真与 default 无差异，故不设 sim/ 层）

★ 覆盖层是可选的：某层目录不存在，load_config 会直接跳过（只看文件在不在），
  所以"去掉一个空层"不需要改这里的代码。仅当**确实有值不同**时才建新层，
  否则会出现一个永远不生效的空目录，让人误以为切档位有用。
"""

import copy                              # use 深拷贝，避免合并时污染源字典
import os                                # use 路径拼接与存在性检查

import yaml                              # use YAML 解析


# 读一个 YAML 文件为字典
# 参数：path - 文件绝对路径
# 返回：dict；文件为空时返回 {}
def load_yaml(path):                     # out 对外接口
    with open(path, 'r', encoding='utf-8') as f:   # io 以 UTF-8 打开（中文注释）
        data = yaml.safe_load(f)          # use 安全解析
    return data or {}                     # use 空文件 → {}


# 递归合并：把 override 覆盖到 base 上（就地修改 base 的深拷贝并返回）
# 参数：base - 基础字典（default）；override - 覆盖字典（sim/real）
# 返回：合并后的新字典（不修改入参）
def deep_merge(base, override):          # out 对外接口
    result = copy.deepcopy(base)         # use 深拷贝，保护原字典
    for key, value in (override or {}).items():     # # 逐键覆盖
        if (key in result and isinstance(result[key], dict)
                and isinstance(value, dict)):       # # 双方都是字典：继续递归
            result[key] = deep_merge(result[key], value)
        else:                            # # 否则直接覆盖（含 list/标量）
            result[key] = copy.deepcopy(value)
    return result


# 按 default → 覆盖层 的顺序加载并合并一组配置目录
# 参数：config_root - 配置根目录（其下应有 default/ 及可选的 override 子目录）
#       overrides   - 覆盖层子目录名列表，按顺序叠加（如 ['sim'] 或 ['real']）
#       names       - 要加载的配置文件名（不含 .yaml），如 ['task','safety']
# 返回：dict {文件名: 合并后的参数字典}；某文件在全部层都不存在则跳过
def load_config(config_root, overrides=None, names=None):   # out 对外接口
    overrides = overrides or []          # use 无覆盖层则只读 default
    names = names or []                  # use 未指定则返回空

    merged = {}                          # def 累积结果
    for name in names:                   # # 逐个配置文件处理
        # 先攒出该文件的完整层路径列表：default 层 + 各覆盖层
        layers = [os.path.join(config_root, 'default', name + '.yaml')]
        for layer in overrides:          # # 追加覆盖层路径
            layers.append(os.path.join(config_root, layer, name + '.yaml'))

        data = {}                        # def 该文件的合并结果（逐层叠加）
        found = False                    # def 是否至少有一层存在
        for path in layers:              # # 按顺序叠加
            if os.path.isfile(path):     # # 该层文件存在才读
                found = True
                data = deep_merge(data, load_yaml(path))   # use 后者盖前者
        if found:                        # # 有内容才收录
            merged[name] = data
    return merged                        # use 返回 {文件名: 参数}
