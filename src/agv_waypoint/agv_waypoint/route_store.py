"""
RouteStore —— 路线存储（纯 Python，不碰 ROS，可单独测试）。

路线 = 一串"按行进顺序排列的点位"，用来做多点导航：
    点名导航 + 路线 → 路线各点 → 目标点
    完整搬货 + 路线 → 取货点 → 路线各点 → 放货点

存路线时按点名从点位表里**复制位姿**（快照），所以之后改名/删点都不影响已存的路线。
路线与地图绑定，单独一份 routes.yaml。

YAML 落盘格式（便于人读与手改）：
    frame_id: map
    routes:
      巡检A:
        points:
          - {name: 走廊1, x: 1.0, y: 2.0, yaw: 0.0}
          - {name: 走廊2, x: 3.0, y: 2.0, yaw: 0.0}
"""

import os                                # use 目录创建与路径处理

import yaml                              # use YAML 读写


class RouteStore:                        # out 路线存储（对象，构造时读文件）
    # 初始化：记住文件路径并立即读入磁盘内容
    # 参数：path - routes.yaml 路径；frame_id - 坐标系（默认 map）
    # 返回：无
    def __init__(self, path, frame_id='map'):   # out 对外构造函数
        self.path = path                 # def 持久化文件路径
        self.frame_id = frame_id         # def 坐标系名
        self._routes = {}                # def 内存中的 路线名 → [点位 dict, ...]
        self._mtime = None               # def 最近一次读到的文件修改时间
        self._load()                     # io 启动时读文件（不存在则为空表）

    # 文件被外部改过就重新读一遍（终端控制台 / 手工编辑 / 另一台设备的网页）
    # 参数：无
    # 返回：True=确实重载了；False=文件没变或读不到
    def _reload_if_changed(self):        # in 内部方法
        """检测 routes.yaml 的 mtime，变了就重读。

        ★ 为什么必须有它（真机 bug + 数据丢失风险）：
          本类在构造时把文件读进内存，之后一直用内存副本。而 routes.yaml
          是**多个入口共用**的：网页、终端控制台、手工编辑都会改它。
          一旦别人在终端里加了一条路线，本进程完全不知道 ——
          网页上自然"看不到之前保存的路线"。
          更严重的是：此时网页再保存一条路线，`_save()` 会把**内存里的旧表**
          （不含终端新加的路线）覆盖回文件，把别人的数据直接抹掉。
          所以每次对外读写前先看 mtime，变了就重读，把两边的视图对齐。
        """
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            return False                 # 文件不存在：保持现状（可能本来就没存过）
        if self._mtime is not None and mtime == self._mtime:
            return False
        self._load()                     # use 重新读盘
        return True

    # 把一个原始 dict 清洗成"途经点"
    # 参数：raw - YAML 里某个途经点的字典
    # 返回：dict（含 name/x/y/yaw）或 None（非法）
    def _clean_point(self, raw):         # in 内部方法
        if not (isinstance(raw, dict) and all(k in raw for k in ('x', 'y', 'yaw'))):
            return None
        try:
            return {'name': str(raw.get('name', '') or ''),
                    'x': float(raw['x']), 'y': float(raw['y']), 'yaw': float(raw['yaw'])}
        except (TypeError, ValueError):
            return None

    # 从磁盘读取路线表
    # 参数：无
    # 返回：无
    def _load(self):                     # in 内部方法
        if not os.path.isfile(self.path):      # # 文件不存在：空表
            self._mtime = None
            return
        try:
            with open(self.path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            self._mtime = os.path.getmtime(self.path)   # use 记下版本戳
        except (OSError, yaml.YAMLError):      # # 读失败：降级为空表，不让节点崩
            self._routes = {}
            self._mtime = None
            return
        raw = data.get('routes', {}) if isinstance(data, dict) else {}
        routes = {}                      # def 清洗后的路线表
        for name, body in (raw or {}).items():
            pts = body.get('points', []) if isinstance(body, dict) else []
            cleaned = [p for p in (self._clean_point(x) for x in (pts or [])) if p]
            if cleaned:                  # # 空路线不入表（避免留下没用的空壳）
                routes[str(name)] = cleaned
        self._routes = routes            # use 载入内存
        if isinstance(data, dict) and data.get('frame_id'):
            self.frame_id = str(data['frame_id'])

    # 写回磁盘
    # 参数：无
    # 返回：True=成功；False=写失败
    def _save(self):                     # in 内部方法
        parent = os.path.dirname(os.path.abspath(self.path))   # use 父目录
        if parent and not os.path.isdir(parent):
            try:
                os.makedirs(parent, exist_ok=True)
            except OSError:
                return False
        payload = {'frame_id': self.frame_id,
                   'routes': {n: {'points': p} for n, p in self._routes.items()}}   # def 落盘结构
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

    # 存一条路线（重名覆盖）
    # 参数：name - 路线名；points - 途经点列表，每项 dict{name,x,y,yaw}
    # 返回：True=成功；False=参数非法或写盘失败
    def save(self, name, points):        # out 对外接口
        self._reload_if_changed()        # use 先跟盘对齐，别覆盖别人的改动
        cleaned = [p for p in (self._clean_point(x) for x in (points or [])) if p]
        if not str(name).strip() or not cleaned:   # # 空名/空路线：拒绝
            return False
        self._routes[str(name)] = cleaned
        return bool(self._save())        # io 立即持久化

    # 查一条路线
    # 参数：name - 路线名
    # 返回：list[dict]（副本）；不存在返回 None
    def get(self, name):                 # out 对外接口
        self._reload_if_changed()        # use 每次查都看一眼盘上有没有新东西
        pts = self._routes.get(str(name))
        return [dict(p) for p in pts] if pts else None

    # 删一条路线
    # 参数：name - 路线名
    # 返回：True=确实删掉了；False=本来就没有或写盘失败
    def delete(self, name):              # out 对外接口
        self._reload_if_changed()        # use 先对齐再删，别删掉看不见的旧版本
        if str(name) not in self._routes:
            return False
        del self._routes[str(name)]
        return bool(self._save())

    # 列出所有路线名（排序）
    # 参数：无
    # 返回：list[str]
    def list_names(self):                # out 对外接口
        self._reload_if_changed()        # use 网页"刷新"时能看到终端新加的路线
        return sorted(self._routes.keys())

    # 各条路线的途经点数
    # 参数：无
    # 返回：dict[str, int]
    def counts(self):                    # out 对外接口
        self._reload_if_changed()
        return {n: len(p) for n, p in self._routes.items()}

    # 路线数量
    # 参数：无
    # 返回：int
    def count(self):                     # out 对外接口
        self._reload_if_changed()
        return len(self._routes)

    # 全部路线（副本）
    # 参数：无
    # 返回：dict[str, list[dict]]
    def all(self):                       # out 对外接口
        self._reload_if_changed()
        return {n: [dict(p) for p in pts] for n, pts in self._routes.items()}
