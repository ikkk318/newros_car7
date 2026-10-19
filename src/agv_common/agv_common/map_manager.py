"""
map_manager —— 地图库管理（纯 Python，不依赖 ROS，可单独测试）。

对应需求：控制台要能"建图 → 命名保存 → 列出 → 删除 → 加载"，并且
**每份地图绑定一份自己的点位文件**——加载哪份地图，就用哪份点位，互不串。

目录约定（地图与点位/路线同放一处，便于一起备份/搬迁）：
    地图库根（默认 <工作区>/maps/，可用环境变量 AGV_MAPS_DIR 覆盖）
      ├── world666map/               ← 一份地图 = 一个同名子目录
      │     ├── world666map.pgm      ← 栅格图（Nav2 用）
      │     ├── world666map.yaml     ← 地图元数据（分辨率/原点）
      │     ├── waypoints.yaml       ← 这份地图专属的点位文件
      │     ├── routes.yaml          ← 这份地图专属的路线文件
      │     └── tasks.yaml           ← 任务缓冲（控制台"任务系统"用，按需生成）
      └── 我的地图/
            ├── 我的地图.pgm
            ├── 我的地图.yaml
            ├── waypoints.yaml
            └── routes.yaml

为什么"一份地图一个目录"：地图和它的点位/路线天然配套（换了地图，坐标就没意义了），
放同一个目录里，删地图=删目录，点位与路线跟着走，不会留下孤儿数据。
"""

import os
import shutil


# 地图库根目录：优先环境变量 AGV_MAPS_DIR，否则用传入的 ws 拼 <ws>/maps
# 参数：ws - 工作区根目录（可选）
# 返回：地图库根目录绝对路径
def default_maps_root(ws=None):
    env = os.environ.get('AGV_MAPS_DIR', '').strip()
    if env:
        return os.path.abspath(env)
    if ws:
        return os.path.join(os.path.abspath(ws), 'maps')
    return os.path.abspath('maps')


# 地图名是否合法：非空、无路径分隔符、不是 . / ..（防目录穿越）
# 参数：name - 地图名
# 返回：(ok, reason)
def valid_name(name):
    name = (name or '').strip()
    if not name:
        return False, '地图名不能为空'
    if name in ('.', '..'):
        return False, '地图名非法'
    if '/' in name or '\\' in name or '\0' in name:
        return False, '地图名不能含路径分隔符'
    return True, '合法'


# 建图期间的"草稿"目录名。
# ★ 为什么需要：建图时用户会顺手存点位，但**地图名要等保存那一刻才定**，
#   不能直接把点位写进某张图。所以先写到固定草稿目录，保存地图后再并进去。
# ★ 为什么不会污染地图库：MapManager.list_maps() 只认目录里同时有
#   <name>.pgm 和 <name>.yaml 的子目录；草稿目录只有 waypoints/routes，
#   永远不会被当成一张地图列出来。
MAPPING_DRAFT = '_mapping'

# 给 map_server 用的"纯 ASCII 别名"目录（在每份地图目录之外，
# 因为真正要避开的正是"中文目录名"本身）。名字以 . 开头，
# 与 MAPPING_DRAFT 同理：list_maps() 要求同目录存在 <目录名>.pgm
# 与 <目录名>.yaml，这里两者都没有，永远不会被当成一张地图。
LAUNCH_ALIAS_DIR = '.launch'


# 数一份 yaml 里某个顶层键下面有多少条（点位/路线各用一次）
# 参数：path - yaml 文件路径；key - 顶层键名（如 'waypoints'）
# 返回：int，文件不存在或解析失败时返回 0（绝不抛异常，调用方是展示逻辑）
# 为什么要有它：网页"选地图"要给每张图显示"几个点位"，
#   这样地图一多，用户不用逐个点开猜。终端版 agv_console.py 的
#   _count_in_yaml 是同一件事，但那边直接 import yaml；agv_common 定位是
#   纯标准库可单测，所以这里做成"有 pyyaml 就用，没有就文本扫"。
def _count_yaml_entries(path, key):
    if not os.path.isfile(path):
        return 0
    try:
        import yaml
        with open(path, encoding='utf-8') as fh:
            data = yaml.safe_load(fh) or {}
        sub = data.get(key)
        return len(sub) if isinstance(sub, (dict, list)) else 0
    except Exception:
        pass
    # 退回文本扫描：只数 key 那一层往下的"直接子键"，
    # 即行首恰好 2 个空格的键行（更深的 x/y/yaw 不算）。
    n, indented = 0, False
    try:
        with open(path, encoding='utf-8') as fh:
            for line in fh:
                s = line.rstrip('\n')
                stripped = s.strip()
                if not stripped or stripped.startswith('#'):
                    continue
                if not s.startswith(' '):
                    # 顶格行：是 key 本身或别的顶层键
                    indented = stripped.rstrip(':').strip() == key
                    continue
                if not indented:
                    continue
                if s.startswith('   '):        # 三级及更深，跳过
                    continue
                if ':' in s:
                    n += 1
    except OSError:
        return 0
    return n


class MapManager:                          # out 地图库管理器
    # 初始化
    # 参数：root - 地图库根目录；不存在时后续 save 会自动创建
    # 返回：无
    def __init__(self, root):              # out 对外构造函数
        self.root = os.path.abspath(root)  # def 地图库根

    # 确保根目录存在
    # 参数：无
    # 返回：无
    def ensure_root(self):                 # out 对外接口
        os.makedirs(self.root, exist_ok=True)

    # 列出全部地图名（含 xxx.pgm+xxx.yaml 的子目录才算一份有效地图）
    # 参数：无
    # 返回：list[str]（按名排序）
    def list_maps(self):                   # out 对外接口
        if not os.path.isdir(self.root):
            return []
        names = []
        for entry in sorted(os.listdir(self.root)):
            d = os.path.join(self.root, entry)
            if not os.path.isdir(d):
                continue
            if (os.path.isfile(os.path.join(d, f'{entry}.pgm'))
                    and os.path.isfile(os.path.join(d, f'{entry}.yaml'))):
                names.append(entry)
        return names

    # 取一份地图的展示用信息（给网页"选地图"列表做副标题）
    # 参数：name - 地图名
    # 返回：dict {name, saved_at, saved_ts, points, has_routes}
    #   saved_at  - 人类可读的保存时间，如 '2026-10-16 21:03'；取不到文件时为空串
    #   saved_ts  - 同一个时间的 unix 时间戳（排序/比较用，取不到为 0）
    #   points    - 该地图下的点位数（waypoints.yaml 里的条目数，取不到为 0）
    #   has_routes- 是否存了路线
    # 为什么需要它：地图一多，光看名字(如 map1/map2)根本分不清哪张是哪张，
    #   用户得知道"这张图几点建的、里面标了几个点"才敢选。
    def map_info(self, name):              # out 对外接口
        import time as _time
        p = self.map_paths(name)
        info = {'name': name, 'saved_at': '', 'saved_ts': 0,
                'points': 0, 'has_routes': False}
        # 保存时间：优先用 yaml（地图本体），退回 pgm
        for key in ('yaml', 'pgm'):
            try:
                ts = os.path.getmtime(p[key])
            except OSError:
                continue
            info['saved_ts'] = ts
            info['saved_at'] = _time.strftime('%Y-%m-%d %H:%M', _time.localtime(ts))
            break
        # 点位数：优先用 pyyaml 正经解析；没有 pyyaml（本模块是纯标准库定位）
        # 就退回文本扫描。
        # ★ 这里踩过两次：
        #   ① 先写成数 `- name:` 的列表式 —— 本项目点位文件是**映射式**
        #      （`点位名: {x: .., y: ..}` 或块式），数出来永远是 0；
        #   ② 纯文本扫描的判据要精确到"二级缩进的键行"，否则会把
        #      块式点位下面的 x/y/yaw 也算进去，数字虚高。
        info['points'] = _count_yaml_entries(p['waypoints'], 'waypoints')
        info['has_routes'] = os.path.isfile(p['routes'])
        return info

    # 地图是否存在
    # 参数：name - 地图名
    # 返回：bool
    def map_exists(self, name):            # out 对外接口
        d = os.path.join(self.root, name)
        return (os.path.isfile(os.path.join(d, f'{name}.pgm'))
                and os.path.isfile(os.path.join(d, f'{name}.yaml')))

    # 取一份地图的相关路径
    # 参数：name - 地图名
    # 返回：dict {dir, pgm, yaml, waypoints, routes, tasks}
    def map_paths(self, name):             # out 对外接口
        d = os.path.join(self.root, name)
        return {
            'dir': d,
            'pgm': os.path.join(d, f'{name}.pgm'),
            'yaml': os.path.join(d, f'{name}.yaml'),
            'waypoints': os.path.join(d, 'waypoints.yaml'),
            'routes': os.path.join(d, 'routes.yaml'),
            'tasks': os.path.join(d, 'tasks.yaml'),
        }

    # 地图 yaml 路径（map_server 要的就是这个）
    # 参数：name - 地图名
    # 返回：str
    def map_yaml_file(self, name):         # out 对外接口
        return self.map_paths(name)['yaml']

    # 给 map_server 的 yaml 路径：路径含中文时返回一份纯 ASCII 的别名
    # 参数：name - 地图名
    # 返回：str（别名不可用时退回真实路径，行为与旧版一致）
    #
    # ★★★ 为什么必须有它（真机踩坑，2026-10-19）★★★
    #   症状：新建的中文名地图（控制台自动生成「地图10月19日16时」）保存、列出、
    #   预览全正常，一进导航就"全黑 + 只剩起始点 + 小车不见"，
    #   自检只说「未收到 /map」「查不到 TF」。
    #
    #   实测链路（不是猜的）：
    #     ① 同一条 navigation.launch.py，map:=<中文路径> 启动 →
    #          [map_io] Loading yaml file: …/maps/u5730u56FE10u670819u65E516u65F6/…
    #          [map_io] Failed processing YAML … reason: bad file
    #          [lifecycle] Failed to bring up all requested nodes. Aborting bringup.
    #     ② 同一个 launch，map:=<ASCII 别名> →
    #          [map_io] Read map … 596 X 761 map @ 0.025 m/cell
    #          [lifecycle] Managed nodes are active
    #   即：中文在"agv_stack → navigation → nav2_bringup"这条参数重写/传递链里
    #   变成了 uXXXX（PyYAML 对非 ASCII 的转义形态，反斜杠都丢了），
    #   map_server 拿着这串当然打不开文件 → configure 失败 → 没有 /map
    #   → AMCL 不发布 map→odom → 网页没底图也没小车。
    #   ASCII 地图（world666map）全程没事，所以只在"新建的中文名地图"上出现，
    #   特别隐蔽。别名目录全 ASCII，从根上绕开，且**不改地图库的中文命名**。
    #
    #   注意：只改传给 launch/map_server 的这一个值；点位/路线/地图预览仍用真实路径
    #   （那几处走的是普通参数，实测中文正常）。
    #   为什么不直接改 nav2 的 RewrittenYaml / 打 monkey-patch：那是 ROS 安装目录里的
    #   东西，升级 Humble 就会被覆盖；别名是纯数据层做法，升级不受影响。
    def launch_map_yaml(self, name):       # out 对外接口
        real = self.map_yaml_file(name)
        if not os.path.isfile(real):
            return real                    # 文件都没有：原样返回，让调用方去报错
        # 路径本来就全是 ASCII（world666map 等）：一步都不多做，行为完全不变
        if os.path.abspath(real).isascii():
            return real
        root = self._alias_root()
        if not root:
            return real                    # 两种根目录都建不出来：退化为旧行为
        try:
            return self._write_alias(name, real, root)
        except OSError:
            return real                    # 落盘失败不该让整单导航起不来

    # 选一个"绝对路径全 ASCII"的别名根目录
    # 返回：str 或 ''（都不可用）
    # 为什么两个候选：maps/ 一般就在工作区里（ASCII），但也可能整个工作区
    #   被放在中文目录下（比如 /home/张三/…）——那时只能用 /tmp 兜底。
    def _alias_root(self):                 # in 内部方法
        import tempfile
        for cand in (os.path.join(self.root, LAUNCH_ALIAS_DIR),
                     os.path.join(tempfile.gettempdir(), 'agv_map_alias')):
            if not os.path.abspath(cand).isascii():
                continue                   # 这个根自己就带中文，用了也白用
            try:
                os.makedirs(cand, exist_ok=True)
            except OSError:
                continue
            return cand
        return ''

    # 写（或刷新）一份别名：map.yaml + map.pgm
    # 参数：name - 地图名；real - 真实 yaml 路径；root - 别名根目录
    # 返回：别名 yaml 的绝对路径
    # 设计要点：
    #   1) 目录名用地图名的 sha1 前缀 —— 稳定、无中文、天然避免同名冲突；
    #   2) 每次调用都重写，用户手改 origin/resolution 后下次导航就能跟上；
    #   3) yaml 用"临时文件 + os.replace"原子替换，避免 map_server 读到半个文件；
    #   4) pgm 优先软链接（不占空间、不复制 450KB），失败才复制。
    def _write_alias(self, name, real, root):   # in 内部方法
        import hashlib
        import yaml
        d = os.path.join(root, hashlib.sha1(name.encode('utf-8')).hexdigest()[:12])
        os.makedirs(d, exist_ok=True)

        # ── 1. 栅格图：软链到真实 pgm（别名里统一叫 map.pgm，纯 ASCII）──
        link = os.path.join(d, 'map.pgm')
        src_pgm = self.map_paths(name)['pgm']
        if os.path.islink(link) or os.path.exists(link):
            try:
                os.remove(link)
            except OSError:
                pass
        try:
            os.symlink(os.path.abspath(src_pgm), link)
        except OSError:
            shutil.copyfile(src_pgm, link)

        # ── 2. 元数据：抄真实 yaml，只把 image 改成同目录的 map.pgm ──
        #   （image 若留中文名，虽然 yaml-cpp 读 UTF-8 一般没事，但没必要赌；
        #     改成 ASCII 相对名后，map_server 打开的就是别名字符串，零风险。）
        with open(real, encoding='utf-8') as fh:
            data = yaml.safe_load(fh) or {}
        data['image'] = 'map.pgm'
        out = os.path.join(d, 'map.yaml')
        tmp = out + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as fh:
            yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=True)
        os.replace(tmp, out)
        return out

    # 点位文件路径（加载地图时，waypoint_node 用这个文件）
    # 参数：name - 地图名
    # 返回：str
    def waypoints_file(self, name):        # out 对外接口
        return self.map_paths(name)['waypoints']

    # 路线文件路径（与点位同一份地图绑定，同样交给 waypoint_node 管）
    # 参数：name - 地图名
    # 返回：str
    def routes_file(self, name):           # out 对外接口
        return self.map_paths(name)['routes']

    # 任务缓冲文件路径（控制台"任务系统"读写，节点不关心）
    # 参数：name - 地图名
    # 返回：str
    def tasks_file(self, name):            # out 对外接口
        return self.map_paths(name)['tasks']

    # 建一份地图目录（建图开始时先建空目录；保存时再落 pgm/yaml）
    # 参数：name - 地图名
    # 返回：(ok, message)
    def create_dir(self, name):            # out 对外接口
        ok, reason = valid_name(name)
        if not ok:
            return False, reason
        self.ensure_root()
        d = os.path.join(self.root, name)
        try:
            os.makedirs(d, exist_ok=True)
        except OSError as exc:
            return False, f'建目录失败：{exc}'
        return True, f'已创建地图目录「{name}」'

    # 保存一份地图（把 pgm/yaml 拷进地图库目录，并补一份专属点位文件）
    # 参数：name - 地图名；pgm_src / yaml_src - 源文件路径（map_saver_cli 产出）
    #       overwrite - 已存在时是否覆盖
    # 返回：(ok, message)
    def save_map(self, name, pgm_src, yaml_src, overwrite=True):   # out 对外接口
        ok, reason = valid_name(name)
        if not ok:
            return False, reason
        if not (os.path.isfile(pgm_src) and os.path.isfile(yaml_src)):
            return False, '源地图文件不存在（map_saver 没成功？）'
        d = os.path.join(self.root, name)
        if os.path.isdir(d) and self.map_exists(name) and not overwrite:
            return False, f'地图「{name}」已存在'
        try:
            os.makedirs(d, exist_ok=True)
            shutil.copyfile(pgm_src, os.path.join(d, f'{name}.pgm'))
            shutil.copyfile(yaml_src, os.path.join(d, f'{name}.yaml'))
        except OSError as exc:
            return False, f'保存地图失败：{exc}'
        # 修正 yaml 里的 image 字段，指向本目录同名 pgm（map_saver 可能写成别的名）
        try:
            self._fix_yaml_image(os.path.join(d, f'{name}.yaml'), f'{name}.pgm')
        except OSError:
            pass
        # 补一份专属点位文件与路线文件（不存在才建，避免覆盖已存内容）
        # ★ 顺序不能反：建图草稿要**先**并进来，再建空模板。
        #   反了的话空模板（里面有一条"起始点"）会先把位置占住，
        #   草稿就被判成"目标已存在"永远并不进来。
        adopted = self.promote_draft(name)
        wp = os.path.join(d, 'waypoints.yaml')
        if not os.path.isfile(wp):
            self.write_waypoints(name, initial=True)
        rt = os.path.join(d, 'routes.yaml')
        if not os.path.isfile(rt):
            self.write_routes(name)
        msg = f'已保存地图「{name}」（点位：{wp}）'
        if adopted:
            msg += f'；已并入建图草稿里的 {adopted} 个文件（点位/路线）'
        return True, msg

    # 把建图草稿里的点位/路线并入正式地图
    # 参数：name  - 目标地图名
    #       draft - 草稿目录名（默认 MAPPING_DRAFT）
    # 返回：真正并入的文件数（0 = 没草稿，或目标已各有自己的文件）
    def promote_draft(self, name, draft=MAPPING_DRAFT):   # out 对外接口
        """建图期间存下的点位/路线，在保存地图时并入新地图。

        ★ 只补空缺、绝不覆盖：目标目录里已经有 waypoints.yaml / routes.yaml
          就说明这份图有自己的数据（可能是用户手工维护过的），
          覆盖等于静默删掉别人的点位。

        ★ 为什么建图草稿不直接清空：用户可能连建几张图，同一轮标的点
          想复用到下一张；要清也是用户自己决定，不在这里偷偷删。
        """
        ok, _ = valid_name(name)
        if not ok or str(name) == str(draft):
            return 0
        src = self.map_paths(draft)
        dst = self.map_paths(name)
        try:
            os.makedirs(dst['dir'], exist_ok=True)
        except OSError:
            return 0
        n = 0
        for key in ('waypoints', 'routes'):
            if not os.path.isfile(src[key]):
                continue
            if os.path.isfile(dst[key]):      # # 目标已有：不动
                continue
            try:
                shutil.copyfile(src[key], dst[key])
                n += 1
            except OSError:
                pass
        return n

    # 内部：把地图 yaml 的 image 字段改成期望的文件名
    # 参数：yaml_path / image_name
    # 返回：无
    def _fix_yaml_image(self, yaml_path, image_name):   # in 内部方法
        import yaml
        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        data['image'] = image_name
        with open(yaml_path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, allow_unicode=True)

    # 写一份点位文件（initial=True 时带一个起始点，否则建空表）
    # 参数：name - 地图名；initial - 是否预置"起始点"
    # 返回：(ok, message)
    def write_waypoints(self, name, initial=True):      # out 对外接口
        d = os.path.join(self.root, name)
        try:
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, 'waypoints.yaml'), 'w', encoding='utf-8') as f:
                f.write('# 本文件的点位与地图「%s」绑定（换地图请换点位文件）\n' % name)
                f.write('frame_id: map\nwaypoints:\n')
                if initial:
                    # 显式写 kind，避免依赖 WaypointStore 的"缺省即 cargo"兜底
                    f.write('  起始点: {x: 0.0, y: 0.0, yaw: 0.0, kind: cargo}\n')
        except OSError as exc:
            return False, f'写点位文件失败：{exc}'
        return True, '已初始化点位文件'

    # 写一份路线文件（建空表；路线由控制台/节点按需写入）
    # 参数：name - 地图名
    # 返回：(ok, message)
    def write_routes(self, name):          # out 对外接口
        d = os.path.join(self.root, name)
        try:
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, 'routes.yaml'), 'w', encoding='utf-8') as f:
                f.write('# 本文件的路线与地图「%s」绑定（路线由若干点位的位姿快照组成）\n' % name)
                f.write('frame_id: map\nroutes: {}\n')
        except OSError as exc:
            return False, f'写路线文件失败：{exc}'
        return True, '已初始化路线文件'

    # 删除一份地图（整目录删除，含其点位与路线文件）
    # 参数：name - 地图名
    # 返回：(ok, message)
    def delete_map(self, name):            # out 对外接口
        ok, reason = valid_name(name)
        if not ok:
            return False, reason
        d = os.path.join(self.root, name)
        if not os.path.isdir(d):
            return False, f'地图「{name}」不存在'
        try:
            shutil.rmtree(d)
        except OSError as exc:
            return False, f'删除失败：{exc}'
        # 顺手清掉给 map_server 的 ASCII 别名（派生文件，删不掉也不影响功能：
        # 下次同名地图保存时会重新生成，且它的 map.pgm 指向已删文件时
        # 会因为"真实 yaml 不存在"在 launch_map_yaml() 里直接回退）。
        try:
            import hashlib
            for root in (os.path.join(self.root, LAUNCH_ALIAS_DIR),
                         os.path.join(__import__('tempfile').gettempdir(),
                                      'agv_map_alias')):
                sub = os.path.join(root, hashlib.sha1(
                    name.encode('utf-8')).hexdigest()[:12])
                if os.path.isdir(sub):
                    shutil.rmtree(sub, ignore_errors=True)
        except OSError:
            pass
        return True, f'已删除地图「{name}」（含其点位与路线文件）'

    # 校验一份地图可用于导航（pgm/yaml 都在）
    # 参数：name - 地图名
    # 返回：(ok, message)
    def validate(self, name):              # out 对外接口
        if not self.map_exists(name):
            return False, f'地图「{name}」不存在或不完整（缺 pgm/yaml）'
        return True, '地图完整'
