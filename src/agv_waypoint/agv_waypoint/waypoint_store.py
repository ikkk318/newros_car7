"""
WaypointStore —— 点位存储（纯 Python，不碰 ROS，可单独测试）。

职责：把 (x, y, yaw) 存成一个「名字」并标好类型，以后说"去1号货架"就能取到坐标。
持久化到 YAML 文件，重启后点位还在。

点位分两类（kind）：
    cargo  搬货点：完整搬货任务的取货点 / 放货点
    nav    导航点：只用来组成路线（不会被当成取货点/放货点）

YAML 落盘格式（便于人读与手改）：
    frame_id: map
    waypoints:
      起始点: {x: 0.0, y: 0.0, yaw: 0.0, kind: cargo}
      1号货架: {x: 1.63, y: 2.66, yaw: 0.0, kind: cargo}
      走廊A: {x: 0.5, y: 0.5, yaw: 0.0, kind: nav}

    注：老文件若某条没写 kind，按 cargo 处理（见 clean_kind），因此"起始点"这种
    通用点也会落成 cargo；要让它只用于组路线，请显式写 kind: nav。
"""

import os                                # use 目录创建与路径处理

import yaml                              # use YAML 读写

KIND_CARGO = 'cargo'                     # def 搬货点
KIND_NAV = 'nav'                         # def 导航点
KINDS = (KIND_CARGO, KIND_NAV)           # def 合法类型


# 把任意输入规整成合法类型（不合法一律按搬货点处理）
# 参数：kind - 类型字符串
# 返回：str（cargo / nav）
def clean_kind(kind):                    # out 对外接口
    k = str(kind or '').strip().lower()
    return k if k in KINDS else KIND_CARGO


class WaypointStore:                     # out 点位存储（对象，构造时读文件）
    # 初始化：记住文件路径并立即读入磁盘内容
    # 参数：path - YAML 文件路径；frame_id - 这些点所属坐标系（默认 map）
    # 返回：无
    def __init__(self, path, frame_id='map'):   # out 对外构造函数
        self.path = path                 # def 持久化文件路径
        self.frame_id = frame_id         # def 坐标系名
        self._points = {}                # def 内存中的点名 → {x,y,yaw,kind}
        self._mtime = None               # def 最近一次读到的文件修改时间
        self._load()                     # io 启动时读文件（不存在则为空表）

    # 文件被外部改过就重新读一遍（终端控制台 / 手工编辑 / 另一台设备的网页）
    # 参数：无
    # 返回：True=确实重载了；False=文件没变或读不到
    def _reload_if_changed(self):        # in 内部方法
        """检测 waypoints.yaml 的 mtime，变了就重读。

        ★ 与 RouteStore 同一个原因（真机 bug + 数据丢失风险）：
          点位文件也是网页/终端/手工共用的。构造时读入内存后如果不看盘，
          终端新加的点位网页永远看不到；而网页再存一个点时会拿**旧内存**
          整表覆盖，直接把终端新加的点抹掉。
        """
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            return False
        if self._mtime is not None and mtime == self._mtime:
            return False
        self._load()
        return True

    # 把一个原始 dict 清洗成内部点位结构
    # 参数：pose - YAML 里某点的字典
    # 返回：dict（含 x/y/yaw/kind）或 None（缺 x/y/yaw 判非法）
    def _clean_point(self, pose):        # in 内部方法
        if not (isinstance(pose, dict) and all(k in pose for k in ('x', 'y', 'yaw'))):
            return None                  # use 缺基本三点：非法点，丢弃
        try:
            return {'x': float(pose['x']), 'y': float(pose['y']),
                    'yaw': float(pose['yaw']), 'kind': clean_kind(pose.get('kind'))}
        except (TypeError, ValueError):
            return None                  # use 坐标不是数字：非法点，丢弃

    # 从磁盘读取点位表
    # 参数：无
    # 返回：无
    def _load(self):                     # in 内部方法
        if not os.path.isfile(self.path):      # # 文件不存在：从空表开始
            self._mtime = None
            return
        try:
            with open(self.path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            self._mtime = os.path.getmtime(self.path)   # use 记下版本戳
        except (OSError, yaml.YAMLError):      # # 读失败/格式坏：降级为空表，不让节点崩
            self._points = {}
            self._mtime = None
            return
        raw = data.get('waypoints', {}) if isinstance(data, dict) else {}
        points = {}                      # def 清洗后的点位
        for name, pose in (raw or {}).items():
            pt = self._clean_point(pose)
            if pt is not None:
                points[str(name)] = pt
        self._points = points            # use 载入内存
        if isinstance(data, dict) and data.get('frame_id'):   # # 文件里带坐标系就用它
            self.frame_id = str(data['frame_id'])

    # 写回磁盘
    # 参数：无
    # 返回：True=成功；False=写失败
    def _save(self):                     # in 内部方法
        parent = os.path.dirname(os.path.abspath(self.path))   # use 父目录
        if parent and not os.path.isdir(parent):   # # 目录不存在先建
            try:
                os.makedirs(parent, exist_ok=True)
            except OSError:
                return False
        payload = {'frame_id': self.frame_id, 'waypoints': self._points}   # def 落盘结构
        try:
            with open(self.path, 'w', encoding='utf-8') as f:
                yaml.safe_dump(payload, f, allow_unicode=True, sort_keys=True)   # use 中文不转义
            try:
                self._mtime = os.path.getmtime(self.path)   # use 记住自己写的版本
            except OSError:
                self._mtime = None
            return True
        except OSError:
            return False

    # 存一个点（重名覆盖）
    # 参数：name - 点名；x, y - 位置(m)；yaw - 朝向(rad)；kind - 类型
    # 返回：True=保存成功；False=写盘失败
    def save(self, name, x, y, yaw, kind=KIND_CARGO):   # out 对外接口
        self._reload_if_changed()        # use 先跟盘对齐，别覆盖别人的改动
        try:
            pt = {'x': float(x), 'y': float(y), 'yaw': float(yaw), 'kind': clean_kind(kind)}
        except (TypeError, ValueError):
            return False                 # use 坐标非法：不写盘
        self._points[str(name)] = pt     # def 覆盖式写入
        return self._save()              # io 立即持久化

    # 查一个点（含类型）
    # 参数：name - 点名
    # 返回：dict {'x','y','yaw','kind'}；不存在返回 None
    def get(self, name):                 # out 对外接口
        self._reload_if_changed()        # use 每次查都看一眼盘上有没有新东西
        pose = self._points.get(str(name))   # use 查内存
        return dict(pose) if pose else None

    # 查一个点的类型（点不存在按搬货点处理）
    # 参数：name - 点名
    # 返回：str
    def kind(self, name):                # out 对外接口
        self._reload_if_changed()
        pose = self._points.get(str(name))
        return pose['kind'] if pose else KIND_CARGO

    # 删一个点（不存在返回 False，不报错）
    # 参数：name - 点名
    # 返回：True=确实删掉了；False=本来就没有或写盘失败
    def delete(self, name):              # out 对外接口
        self._reload_if_changed()        # use 先对齐再删，别删掉看不见的旧版本
        if str(name) not in self._points:      # # 不存在：直接 False
            return False
        del self._points[str(name)]      # use 删除
        return bool(self._save())        # io 持久化；写盘失败返回 False（不谎报成功）

    # 列出所有点名（排序后返回，稳定好读）
    # 参数：无
    # 返回：list[str]
    def list_names(self):                # out 对外接口
        self._reload_if_changed()        # use 网页"刷新"时能看到终端新加的点位
        return sorted(self._points.keys())   # use 排序返回

    # 按类型列点名（控制台"选搬货点/选导航点"用）
    # 参数：kind - 类型（cargo / nav）
    # 返回：list[str]
    def names_of_kind(self, kind):       # out 对外接口
        self._reload_if_changed()
        k = clean_kind(kind)
        return [n for n in self.list_names() if self._points[n]['kind'] == k]

    # 返回全部点名 → 位姿（副本）
    # 参数：无
    # 返回：dict[str, dict]（空表返回 {}）
    def all(self):                       # out 对外接口
        self._reload_if_changed()
        return {name: dict(pose) for name, pose in self._points.items()}   # use 深一层拷贝

    # 点位数量
    # 参数：无
    # 返回：int
    def count(self):                     # out 对外接口
        self._reload_if_changed()
        return len(self._points)
