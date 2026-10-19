"""网关的叶子工具：日志转发判据、局域网地址、静态目录、tasks.yaml 读写。

**这一层不依赖任何其它 gateway 模块**，也不 import rclpy ——
所以它可以被单独 import、单独思考，不会牵扯出节点或 aiohttp。
"""

import pathlib
import re
import socket


# ══════════════════════════ 〇、通信传输：只用 UDP ══════════════════════════
# ★ 为什么关掉 FastDDS 的共享内存（这条修的是最贵的一类故障）：
#   同机 Node 默认优先走 SHM，而 SHM 靠 /dev/shm 里的 fastrtps_portXXXX 文件
#   做 rendezvous。被 Ctrl-C 打断的节点经常死在 destroy_node() 里，
#   段来不及 unlink；下一个栈的 participant 撞上残留文件就会刷
#   `[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port74xx`，
#   然后**半死不活**：节点全在、话题全在，彼此就是收不到数据。
#   现象五花八门 —— 自检说节点未发现、地图黑屏、任务发出去 task_manager
#   没反应，但独立 ros2 CLI 看又一切正常。
#
#   本项目单机、数据量小（激光 10Hz、TF 50Hz、地图偶发），UDP 完全够用，
#   而且没有共享内存文件可残留 —— 从根上消掉这一类故障。
#   注意：必须在 rclpy.init() **之前**写进环境变量，participant 一创建
#   就固定了 transport，之后再改没用。
_FASTDDS_ENV = {'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4'}


def _with_where(data):
    """顶升/下降日志里带上点位名（网页可能从点位卡片直接点按钮）。

    参数缺失时返回空串，不输出 "()" 这种半截括号。
    """
    nm = str((data or {}).get('where', '') or '').strip()
    return f'（{nm}）' if nm else ''


# ══════════════ 跨节点日志转发：只挑关键事件 ══════════════
#
# /rosout 是全系统日志汇总，全转发会把降噪一次性废掉。这里的判据只做
# 一件事：**这条日志是不是用户需要立刻在网页上看到的那几类**。
#
# ① _relay_wanted：要不要转发
#    命中"顶升/下降/任务边界/急停"，或任何 WARN 以上级别（错误必须可见）。
# ② _relay_is_success：要不要标绿
#    顶升/下降"结束"= 成功 → 绿字；"失败"由级别判红，不在这里。
_RELAY_WANT = re.compile(
    r'\[(?:顶升|下降|搬运|任务完成|任务失败)\]'
    r'|搬货服务|已顶升|已下降|/agv/cargo'
    # ★ 任务被拒必须能被看见：网关只能从 task_id 猜"受没受理"，
    #   真正的拒绝原因只有 task_manager 这句话里有。只放 "拒绝"，
    #   受理的那句（[指令] … → 受理）仍然不转，避免每单刷一条。
    r'|\[指令\].*?拒绝',
    re.I,
)
# 成功判据：只在"结束/成功/完成"且不含"失败/异常/拒绝/超时"时标绿。
# 顺序很重要：先排失败词，再看成功词 —— "下降失败：搬运超时" 里两者都有。
_RELAY_FAIL = re.compile(r'失败|异常|拒绝|超时|不可用|无法', re.I)
_RELAY_OK = re.compile(r'结束|成功|完成|已顶升|已下降', re.I)


def _relay_is_self(name, self_name):
    """这条 /rosout 日志是不是本节点自己发的？

    ★ 为什么单独抽成函数、还带测试：
      网关自己也会往 /rosout 写（self.get_logger().info(...)），而 self.log()
      已经记过一遍。不排除会**每条日志重复两遍**。

    ★ 为什么取路径最后一段，而不是只 lstrip('/')：
      `'/agv_web_gateway'.lstrip('/')` 恰好等于 'agv_web_gateway'，看起来够用；
      但一旦给节点加了命名空间，rosout 里的名字变成 '/agv/agv_web_gateway'，
      lstrip 后是 'agv/agv_web_gateway' ≠ 'agv_web_gateway' → 排除失效 →
      本节点每条日志都被当成"别的节点"转发，转发又写回 /rosout，
      形成自我放大（日志越转越多，CPU 打满）。取最后一段才对。
    """
    return str(name or '').strip('/').split('/')[-1] == str(self_name or '')


# 跨通道去重窗口（秒）。
# ★ 同一条业务事件有两条通路进 self.log()：
#     ① 子进程 stdout → _pump_output → translate → log()
#        （带 `[task_manager_node] ` 这样的前缀）
#     ② /rosout 转发 → on_rosout → log()（只有正文）
#   真车日志里每件事因此都出现两遍，比如顶升成功：
#     [task_manager_node] [顶升] 成功（1号货架）：已顶升，开始搬货
#     [顶升] 成功（1号货架）：已顶升，开始搬货
#   正常流程里同一条正文不可能在 2.5 秒内产生两次（两次顶升之间隔着
#   整趟运输），所以"归一化正文 + 时间窗"去重是安全的。
_DUP_WINDOW = 2.5

# 级别"醒目程度"排序：去重合并时取更高的那个，避免绿字/红字时有时无。
#   ok 高于 info（成功要绿），warn/error 高于 ok（失败/异常优先展示）。
_LEVEL_RANK = {'info': 0, 'ok': 1, 'warn': 2, 'error': 3}

# 归一化时要剥掉的前缀：只剥**身份前缀**，不剥业务标签。
#   `[task_manager_node] ` / `[信息·task_manager_node] ` → 剥
#   `[顶升] ` `[导航中] ` `[1/5] `                          → 保留
# ★ 为什么 `[1/5]` 必须保留：批量任务里 `[1/5] 搬运完成` 与 `[2/5] 搬运完成`
#   归一化后若一样，后面的单子会被当成重复丢掉（这是以前踩过的坑）。
_NORM_PREFIX = re.compile(
    r'^\[(?:'
    r'[\u4e00-\u9fa5]{1,4}·[^\]]*'        # 信息·task_manager_node
    r'|[A-Za-z_][A-Za-z0-9_.\-]*'          # task_manager_node / planner_server
    r')\]\s*')


def _norm_log(text):
    """把日志正文归一化，用于判断"两条日志是不是同一件事"。

    ★ 只剥开头最多两层的身份前缀（`[节点] `、`[信息·节点] `），
      正文里的业务信息一个字都不动 —— 去重不能把不同事件抹平。
    """
    s = str(text or '')
    for _ in range(2):
        t = _NORM_PREFIX.sub('', s, count=1)
        if t == s:
            break
        s = t
    return s.strip()


def _relay_wanted(text):
    """这条外部节点日志要不要转发到网页运行日志？"""
    return bool(_RELAY_WANT.search(str(text or '')))


def _relay_is_success(text):
    """这条日志该不该显示成绿字（成功）。

    ★ 必须先判失败词：`[下降] 失败：搬货服务无应答（超时 5s）` 同时含
      "下降"和"超时"，只看成功词会把它标成绿色 —— 用户看到绿的以为成功了。
    """
    t = str(text or '')
    if _RELAY_FAIL.search(t):
        return False
    return bool(_RELAY_OK.search(t))


def _lan_urls(port):
    """列出"别的设备能用"的访问地址，例如 ['http://10.122.131.203:8080']。

    ★ 为什么需要这个函数（真实故障）：
      手机连同一个 WiFi，在浏览器里敲 127.0.0.1:8080 → ERR_CONNECTION_REFUSED。
      127.0.0.1 是**本机回环**，在手机上它指的是手机自己，永远连不到虚拟机。
      网关本身监听 0.0.0.0 是对的，问题只在"人不知道该敲哪个 IP"。
      所以启动时把候选地址直接打印出来。

    ★ 为什么用 UDP connect 而不是解析 ifconfig：
      不需要 root、不依赖 iproute2、跨平台可用。开一个 UDP socket 连到
      公网地址（不会真发包，只查路由表选源地址）就能拿到"本机对外 IP"。
      失败就退化为解析 hostname，再失败返回空列表（调用方会给出桥接提示）。

    参数：port - HTTP 端口
    返回：list[str]，可空
    """
    ips = []
    # ① UDP connect 探源地址（最快、最准，不实际发包）
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(('8.8.8.8', 80))     # 只用于选路，不产生流量
            ips.append(s.getsockname()[0])
        finally:
            s.close()
    except OSError:
        pass
    # ② 用 hostname 兜底（有些环境禁了对外 UDP）
    if not ips:
        try:
            for info in socket.getaddrinfo(socket.gethostname(), None,
                                           socket.AF_INET):
                ips.append(info[4][0])
        except OSError:
            pass
    # 去重、剔除回环与链路本地（169.254.x 是没要到 DHCP 才会出现的地址）
    out = []
    for ip in ips:
        if not ip or ip.startswith('127.') or ip.startswith('169.254.'):
            continue
        if ip not in out:
            out.append(ip)
    return [f'http://{ip}:{int(port)}' for ip in out]


def _find_static_dir():                    # out 前端静态文件所在目录
    """找前端静态文件目录，兼容"源码树"和"已安装"两种布局。

    ★ 不能只写 `__file__/static`：setup.py 里 static/ 是作为 data_files 装到
      `share/agv_web/static` 的，**不会**留在 Python 模块目录里。
      用 --symlink-install 时因为符号链接回源码树，`__file__/static` 恰好能找到，
      于是这个问题在开发时被藏住了；一旦用普通安装（或换机器部署），
      前端就会 404（页面能开？不，是 index.html 都找不到）。
    所以这里按优先级依次探测：
      ① 源码树布局：<包目录>/static            （开发/符号链接安装）
      ② 安装布局  ：<prefix>/share/agv_web/static（普通安装）
      ③ ament 资源索引：用 get_package_share_directory 问 ROS
    """
    here = pathlib.Path(__file__).resolve().parent
    # ① 源码树 / symlink 安装
    #   ★ 本文件在 gateway/ 子目录里，包根是它的父目录 —— 两个都试，
    #     否则下沉一级之后 index.html 会 404（页面直接打不开）。
    for base in (here, here.parent):
        cand = base / 'static'
        if (cand / 'index.html').is_file():
            return cand
    # ② 安装布局：__file__ 在 install/agv_web/lib/python3.x/site-packages/agv_web/
    for parent in [here] + list(here.parents):
        c = parent / 'share' / 'agv_web' / 'static'
        if (c / 'index.html').is_file():
            return c
    # ③ 交给 ament 索引去问（最可靠，但依赖 ROS 环境已 source）
    try:
        from ament_index_python.packages import get_package_share_directory
        c = pathlib.Path(get_package_share_directory('agv_web')) / 'static'
        if (c / 'index.html').is_file():
            return c
    except Exception:
        pass
    # 都找不到就返回源码树那个（报错信息里路径更有用）
    return here.parent / 'static'


STATIC_DIR = _find_static_dir()


# ══════════════════════════ tasks.yaml 读写 ══════════════════════════
# 与终端控制台共用同一份文件格式：{'tasks': [{pick, drop, loops, route}, ...]}
def _load_tasks(path):                     # out 读任务缓冲
    try:
        import yaml
        p = pathlib.Path(path)
        if not p.is_file():
            return []
        doc = yaml.safe_load(p.read_text(encoding='utf-8')) or {}
        items = doc.get('tasks') or []
        out = []
        for it in items:
            if not isinstance(it, dict):
                continue
            out.append({'pick': str(it.get('pick', '')),
                        'drop': str(it.get('drop', '')),
                        'loops': max(1, int(it.get('loops', 1) or 1)),
                        'route': str(it.get('route', '') or '')})
        return out
    except Exception:
        return []


def _save_tasks(path, items):              # out 写任务缓冲（保持 LF 与中文不转义）
    import yaml
    p = pathlib.Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    text = yaml.safe_dump({'tasks': list(items)}, allow_unicode=True,
                          sort_keys=False, default_flow_style=False)
    with open(p, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
