"""
stack.py —— 在网关进程里起停整套 ROS 栈，以及"换栈前把旧的那套收干净"。

**为什么这两件事放在一个文件里**
它们共同构成本项目最贵的一类故障面。用户反复报的
    「切到建图模式地图黑屏、自检说节点都没起来」
    「切换后老是留东西，车还在飘 / 看不到激光」
根因都不在建图代码，而在**旧的那套没收干净**：
  · `proc.wait()` 只保证 `ros2 launch` 父进程退了，它派生的
    gzserver/nav2/slam_toolbox 还在进程组里跑，11345 端口被占，
    而 Gazebo 的 bind 失败是**静默**的 —— 新栈"看起来正常"、世界里没数据；
  · 被 Ctrl-C 打断的 rclpy 节点死在 `destroy_node()` 里，FastDDS 的
    /dev/shm 段来不及 unlink，下一套参与者在 `open_and_lock_file` 上撞残留，
    表现是「节点未发现 / 收不到 /scan / 地图黑屏」，而 `ros2 node list` 一切正常。

所以顺序是死规定：发信号 → 等**整组**死透 → 清残留 SHM → 等端口空 → 才起新栈。
本文件把这条顺序的每一步都拆成了独立函数，方便单独读、单独验。
"""



import os                                  # use 读 /proc、发信号
import shlex                               # use 日志里安全打印启动命令
import signal                              # use 优雅停止子进程组
import socket                              # use 探测 Gazebo 端口是否空出
import subprocess                          # use Popen 拉起 launch
import threading                           # use 保护起停状态
import time                                # use 计时/轮询

from ..stack_manager import (              # use 纯逻辑部分（命令拼装、切换决策）
    MODE_NAV, MODE_TEXT, StackError, build_launch_command, plan_switch,
    validate_map_name,
)
from .util import _FASTDDS_ENV             # use 子进程也必须走 UDP



# ══════════════════ 一、栈起停（本进程管理 ROS 栈）══════════════════


class StackRunner:
    """在网关进程里起停整套 ROS 栈，让用户只敲一条命令。

    **为什么要它**
    原来建图/导航要分别敲两条 launch 命令，跟终端控制台"起一次、界面里选模式"
    的体验差很远。现在网关自己 Popen 一个 launch 子进程，网页上点按钮就切换。

    **两个必须做对的细节**
    ① 用 `start_new_session=True` 让 launch 自成一个进程组，停的时候对**整个组**
       发信号。launch 会派生子进程（gzserver/nav2/amcl…），只杀父进程会留下一堆
       孤儿节点抢话题，表现就是"明明停了车还在飘"。
    ② 先 SIGINT 给 Nav2 体面收尾的时间，超时才 SIGKILL。直接 SIGKILL 会让
       Gazebo 留下共享内存段，下次启动报 "Address already in use"。
    """

    def __init__(self, node):
        self.node = node
        self.proc = None             # def launch 子进程
        self.mode = ''               # def 当前模式（''=没起）
        self.since = 0.0             # def 启动时刻
        self.error = ''              # def 最近错误
        self._state_lock = threading.Lock()  # def 保护起停（网页可能连点）
        self._gw_log_thread = None      # def 读子进程输出的线程
        # ★ 另一个锁：把"停旧栈 + 起新栈"整段串起来。
        #   为什么不能只用 _gw_lock：停栈要等 10 秒以上（等整组死+等端口），
        #   而 /api/state 每秒都在抢 _gw_lock 读状态，锁被占住会让网页卡住。
        #   所以 _gw_lock 只管"改几个字段"，_op_lock 管"这一整套动作别并发"。
        #   用 RLock：switch() 里会嵌套调 stop()/start()，普通 Lock 会自锁。
        self._op_lock = threading.RLock()

    def _say(self, msg):             # in 往网页日志写一行（给下面的纯函数当回调）
        self.node.log('info', msg)

    # ── 状态 ──
    def state(self):                 # out 当前状态（给网页）
        with self._state_lock:
            running = self.proc is not None and self.proc.poll() is None
            if not running:
                self.mode = '' if self.proc is None else self.mode
            return {'mode': self.mode,
                    'mode_text': MODE_TEXT.get(self.mode, '未启动'),
                    'running': running,
                    'pid': (self.proc.pid if (self.proc is not None and running) else None),
                    'since': self.since,
                    'error': self.error}

    def is_running(self):            # out 栈是否在跑
        with self._state_lock:
            return self.proc is not None and self.proc.poll() is None

    # ── 起 ──
    def start(self, mode, map_yaml='', waypoint_file='', routes_file='',
              world='', gui=False, rviz=False):   # out (ok, message)
        """起一套栈。已在跑会先拒绝（由调用方走 plan_switch）。"""
        m = str(mode or '').strip().lower()
        ok, reason = validate_map_name(m, map_yaml)
        if not ok:
            return False, reason
        try:
            argv = build_launch_command(
                m, map_yaml=map_yaml, waypoint_file=waypoint_file,
                routes_file=routes_file, world=world, gui=gui, rviz=rviz,
                ws_setup=self._ws_setup())
        except StackError as exc:
            return False, str(exc)

        with self._op_lock:
            # ★ 先判"是不是已经有栈在跑"，再谈清理。
            #   顺序反了的话，连点两下启动按钮时，第二次会白等 6 秒端口、
            #   还往日志里刷一堆"等等等"的噪音，最后才回一句"已有一套在运行"。
            with self._state_lock:
                if self.proc is not None and self.proc.poll() is None:
                    return False, '已有一套栈在运行，请先停止'
            # ★ 起之前再兜一层：扫掉上一套可能没清理干净的东西。
            #   正常换栈已经走 stop()（那边做得更全），这里是给
            #   "上一次崩在半路 / 用户在外面手工 kill 过节点" 兜底的 ——
            #   这两种情况下没有进程组可等，只能靠 SHM 和端口来判活动静。
            purge_stale_shm(log=self._say)
            wait_port_free(log=self._say)      # 两种模式都带 Gazebo，都要等
            with self._state_lock:
                if self.proc is not None and self.proc.poll() is None:
                    return False, '已有一套栈在运行，请先停止'
                try:
                    proc = subprocess.Popen(
                        argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                        start_new_session=True,       # ★ 自成进程组，停时整组收
                        text=True, bufsize=1,
                        env=self._child_env())
                except Exception as exc:
                    self.error = str(exc)
                    return False, f'启动失败：{exc}'
                self.proc = proc
                self.mode = m
                self.since = time.time()
                self.error = ''
                # ★ 把真正执行的命令打进日志。
                #   为什么必须打：map_yaml 曾经被传成一整个 dict 的 str()，
                #   栈照样起得来、日志只有一句"已启动"，从界面上完全看不出
                #   map_server 拿到的是个假路径。只有把 argv 摊开才能一眼看见。
                #   顺带核对：导航模式下地图文件必须真实存在。
                self.node.log('info', '启动命令：' + ' '.join(shlex.quote(a) for a in argv))
                if m == MODE_NAV:
                    if map_yaml and os.path.isfile(map_yaml):
                        self.node.log('info', f'地图文件就绪：{map_yaml}')
                        # ★ 别名说明：中文地图名的路径走完 launch 链会被转义成
                        #   uXXXX 再交给 map_server（→ bad file → 地图全黑，实测）。
                        #   所以传给 launch 的是纯 ASCII 别名，这里把原始地图也打出来，
                        #   免得以后日志里出现一条用户没见过的路径又被当成"路径错了"。
                        try:
                            from agv_common.map_manager import LAUNCH_ALIAS_DIR
                            is_alias = (f'{os.sep}{LAUNCH_ALIAS_DIR}{os.sep}') in map_yaml
                        except Exception:
                            is_alias = False
                        if is_alias:
                            mm = self.node.get_map_manager()
                            real = (mm.map_yaml_file(self.node.current_map)
                                    if mm is not None and self.node.current_map else '')
                            self.node.log('info',
                                          '地图名含非 ASCII 字符，map_server 改用纯 ASCII 别名'
                                          f'（原始地图：{real or "（未知）"}）')
                    else:
                        self.node.log('error',
                                      f'地图文件不存在，map_server 会加载失败：{map_yaml!r}')
        self._gw_log_thread = threading.Thread(
            target=self._pump_output, args=(proc,), daemon=True)
        self._gw_log_thread.start()
        self.node.log('info', f'已启动{MODE_TEXT[m]}（PID {proc.pid}）')
        return True, f'{MODE_TEXT[m]}启动中（Nav2 激活约需 15~40 秒）'

    def _ws_setup(self):             # out 工作区的 setup.bash 路径
        """找本工作区的 install/setup.bash，让子进程能用自建的包。"""
        env = os.environ.get('AGV_WS_SETUP', '').strip()
        if env and os.path.isfile(env):
            return env
        root = self.node.get_workspace_root()
        if root:
            cand = os.path.join(root, 'install', 'setup.bash')
            if os.path.isfile(cand):
                return cand
        return ''

    def _child_env(self):            # out 子进程环境（关掉输出缓冲 + 关 SHM）
        env = dict(os.environ)
        env.setdefault('PYTHONUNBUFFERED', '1')
        # ★ 显式覆盖，不依赖父进程继承：网关可能由 systemd / 别的 shell
        #   拉起，万一父环境里没带这个变量，栈又会走回共享内存。
        env.update(_FASTDDS_ENV)
        return env

    def _pump_output(self, proc):    # in 把子进程输出转到网页日志
        """把 launch 的输出逐行读进网页日志（过滤噪音 + 翻成中文）。

        ★ 必须一直读：PIPE 写满（默认 64KB）后子进程会阻塞在 write 上，
          整个栈就"卡住不动"了——这是这类代码最经典的坑。

        ★ 为什么要过滤和翻译（用户原话："一堆英文日志，根本看不懂"）：
          ROS 栈的原始输出混着大量没信息量的行（launch 样板话、参数回显、
          bond timer…），一条条原样塞进手机上的日志面板，用户只能看到
          一屏英文。这里交给 log_translate：噪音直接丢，已知句式翻中文，
          没识别出的保留原文（宁可留着英文也不能丢信息）。
          过滤/翻译都在**这个读线程**里做，不影响 ROS 回调。
        """
        from agv_web import log_translate as LT
        try:
            for line in proc.stdout:
                s = (line or '').rstrip()
                if not s:
                    continue
                if LT.is_noise(s):
                    continue
                out = LT.translate(s)
                if not out:
                    continue
                lvl = LT.guess_level(s)
                if len(out) > 300:
                    out = out[:300] + '…'
                # 网页日志里不再加 [栈] 前缀：中文前缀里已经带了模块名
                # （[定位] / [路径规划] / [地图服务]…），再加反而更乱
                self.node.log(lvl, out)
        except Exception:
            pass
        finally:
            rc = proc.poll()
            if rc is not None:
                if rc:
                    self.node.log('error',
                                  f'栈异常退出（返回码 {rc}），请看上面几行日志定位原因')
                else:
                    self.node.log('info', '栈已正常退出')

    # ── 停 ──
    def stop(self, timeout=12.0):    # out (ok, message)
        """停掉整套栈，并把它留下的东西清干净（换栈能不能成功全看这一步）。

        顺序不能换，每一步都对应一类真实故障：
          ① SIGINT 整组 —— 给 Nav2 / Gazebo 体面收尾的机会（直接 -9 会留 SHM）；
          ② **等整组死透** —— `proc.wait()` 只保证 launch 父进程退了，
             gzserver 是它的兄弟进程，还要几秒才释放 11345 端口；
          ③ 还没退就 SIGKILL 整组，再等一次；
          ④ 清 /dev/shm 里**无主**的 FastDDS 段 —— 被 Ctrl-C 打断的节点
             几乎都死在 destroy_node() 里，段来不及 unlink，下一个栈
             撞上就是 `Failed init_port fastrtps_port74xx`，然后新旧参与者
             发现不到彼此，网页上表现为"节点未发现 / 收不到 /scan / 地图黑屏"；
          ⑤ 等 Gazebo master 端口空出来 —— gzserver bind 失败是静默的。
        """
        say = self._say
        with self._op_lock:
            with self._state_lock:
                proc = self.proc
                # 先翻状态：从这一刻起网页就该显示"没在跑"，
                # 别让用户在看板上看到一个正在慢慢死的栈。
                self.proc = None
                self.mode = ''
                self.since = 0.0
            if proc is None:
                # 本来就没在跑：仍然清一遍（上一次可能崩在半路，没人收尾）
                purge_stale_shm(log=say)
                return True, '本来就没在跑'
            # start_new_session=True → 子进程自成会话，组号就等于 launch 的 PID
            pgid = proc.pid
            self.node.log('warn', '正在停止栈（最多等 12 秒）…')
            if proc.poll() is None:
                self._signal_group(proc, signal.SIGINT)
                try:
                    proc.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    self.node.log('warn', '栈未在 12 秒内退出，强制结束')
                    self._signal_group(proc, signal.SIGKILL)
                    try:
                        proc.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        self.error = '强制结束也超时'
            # ② launch 退了 ≠ 一切退干净
            if not wait_group_dead(pgid, timeout=6.0, log=say):
                self.node.log('warn', '还有子进程赖着，强制结束整组')
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError, OSError):
                    pass
                wait_group_dead(pgid, timeout=4.0, log=say)
            # ④⑤ 残留 SHM + 端口
            purge_stale_shm(log=say)
            wait_port_free(log=say)
        self.node.log('info', '栈已停止')
        return True, '已停止'

    def _signal_group(self, proc, sig):    # in 给整个进程组发信号
        """对进程组发信号。

        ★ 不能只 proc.send_signal：launch 派生的 gzserver/nav2 不在它的
          PID 上，漏杀会留下孤儿节点抢话题（"车还在飘"就是这么来的）。
        """
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except (ProcessLookupError, PermissionError, OSError):
            try:
                if sig == signal.SIGKILL:
                    proc.kill()
                else:
                    proc.send_signal(sig)
            except Exception:
                pass

    # ── 一次性切换 ──
    def switch(self, target, **kwargs):    # out (ok, message)
        """按 plan_switch 的决策把栈切到 target（'' 表示停）。

        ★ 整段包在 _op_lock 里：换栈是"停旧的 + 起新的"两步，
          中间绝不能被另一个请求插进来（连点两个模式按钮就会）。
        """
        with self._op_lock:
            with self._state_lock:
                cur = self.mode
                running = self.proc is not None and self.proc.poll() is None
            try:
                plan = plan_switch(cur, running, target)
            except StackError as exc:
                return False, str(exc)
            msg = []
            if 'stop' in plan.actions:
                ok, m = self.stop()
                msg.append(m)
                if not ok:
                    return False, '；'.join(msg)
            if 'start' in plan.actions:
                ok, m = self.start(plan.mode, **kwargs)
                msg.append(m)
                if not ok:
                    return False, '；'.join(msg)
            if not plan.actions:
                msg.append(plan.reason)
            return True, '；'.join(msg) or plan.reason


# ══════════════════ 二、真正杀掉一个栈：进程组 / SHM / 端口 ══════════════════
#
# 上半段是"启动时清上次残留"（cleanup_residual），下半段是"换栈前的彻底清理"。
# 两者手段相同、边界不同：启动清理只能猜（没有进程组可等），换栈清理有
# pgid 可精确等待，所以后者更彻底。


# 启动自清理要匹配的模式。
# ★ 每条都必须"长得足够特别"，否则会误杀。这里踩过一次真实事故，记下来：
#   最初写了 'ros2 launch agv_'，想一次匹配所有本项目的 launch。结果
#   `ros2 launch agv_web web_console.launch.py`（就是我自己的父进程！）
#   也含这个字符串，于是网关启动时把自己的 launch 杀了 —— 日志里出现
#   "34 已杀死 ros2 launch agv_web ..."，网页还没起来就没了。
#   所以 launch 那一条必须**排除自己**，见 _CLEANUP_LAUNCH_EXCLUDE。
#
# 其它注意事项：
#   · 绝对不能用 'python3' / 'python' / 'node' 这种宽泛词——会连 WorkBuddy
#     本体、用户别的脚本一起干掉；
#   · 'ros2 run agv_' 匹配不到真正干活的节点（只匹配一次性壳），意义不大，
#     所以直接匹配具体的节点可执行名。
_CLEANUP_PATTERNS = (
    'ros2 launch agv_',            # launch 父进程（子进程随进程组一起收）
    'gzserver', 'gzclient', 'spawn_entity',
    'nav2_', 'slam_toolbox',
    'robot_state_publisher', 'robot_localization',
    'task_manager_node', 'cargo_node', 'waypoint_node',
    'velocity_arbiter', 'agv_lift',
    'teleop_twist_keyboard',
    'rviz2',
)

# 这些 launch **不能杀**：它们是"当前这个网页网关自己"。
# 杀了自己就没网页了——比不清理危险得多。
_CLEANUP_LAUNCH_EXCLUDE = (
    'agv_web',                     # 网页网关本身（本进程就是它拉起来的）
)


def _pid_alive(pid):                    # out 进程是否还活着
    try:
        os.kill(pid, 0)
        return True
    except (ProcessLookupError, ValueError):
        return False
    except PermissionError:
        return True                     # 别人的进程，存在但无权操作


def _match_pids(pattern):               # out 匹配 pattern 的 PID 列表（含自身保护）
    """pgrep -f 的 Python 版，且主动排除"查询命令自己"。

    ★ 为什么要排除自己：`pgrep -f web_gateway` 会把执行这条命令的 shell
      也匹配上（它的命令行里含这个字符串），于是清理逻辑可能把正在
      启动的自己杀掉。所以这里比对 /proc/<pid>/cmdline 时，凡是
      "这条 cmdline 里包含 pgrep/grep 字样"的一律跳过。
    """
    pids = []
    me = os.getpid()
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == me:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                raw = f.read().replace(b'\x00', b' ').decode('utf-8', 'replace')
        except (OSError, PermissionError):
            continue
        if not raw.strip():
            continue
        if pattern not in raw:
            continue
        # 排除"查询动作本身"，避免自杀
        if ('pgrep' in raw or 'grep ' in raw) and pattern not in ('grep ',):
            continue
        pids.append(pid)
    return pids


def _ancestor_pids():                    # out 自己的祖先 PID 集合（含自己）
    """收集"我自己这一条血脉"上的所有 PID。

    ★ 为什么必须排除祖先：网关通常是 `ros2 launch agv_web web_console.launch.py`
      拉起来的，那条命令行里含有 'ros2 launch agv_'。清理逻辑如果不排除它，
      就会在启动时把自己的 launch 杀掉 —— 实测日志：
          SIGINT 34 (ros2 launch agv_)
          34 已杀死  ros2 launch agv_web web_console.launch.py ...
      结果网页根本没起来。这比"不清理"危险得多。
    """
    pids = set()
    pid = os.getpid()
    for _ in range(20):                  # # 最多上溯 20 层，防环
        if pid <= 1 or pid in pids:
            break
        pids.add(pid)
        try:
            with open(f'/proc/{pid}/stat', 'rb') as f:
                fields = f.read().rsplit(b')', 1)[1].split()
            pid = int(fields[1])         # # stat 第 4 个字段是 ppid
        except (OSError, IndexError, ValueError):
            break
    return pids


def cleanup_residual(timeout=6.0, log=None):   # out 清掉上次残留，返回清理条数
    """启动时清掉上次没退干净的 ROS 进程。

    **为什么需要它（用户实际遇到的问题）**
    网页控制台是"关浏览器窗口"就走的，进程往往还挂在后台。下次再启动，
    就出现两套栈的同名节点互抢话题/服务/action，表现是：
      · 「节点没起 / 无响应 / 查询超时」
      · 导航原地转圈、车还在飘
      · 地图加载了但 TF 查不到
    终端版 `agv_console.py` 早就在启动时调 `_pkill_ros()` 了，网页版一直漏了
    这一步 —— 这就是"明明重启了还是有怪问题"的根源。

    **做法**
    先 SIGINT 给 6 秒体面退出（launch/Gazebo 靠它释放 /dev/shm 和 DDS 端口），
    超时才 SIGKILL。直接 -9 会留下共享内存段，下次报 Address already in use。

    **安全边界（三重保险，缺一不可）**
    ① 只匹配本项目独有的可执行名，绝不用 python3/node 这类宽泛词；
    ② 排除自己的整条祖先链（否则会杀掉拉起自己的 launch，见 _ancestor_pids）；
    ③ 排除名字含 agv_web 的 launch（那就是网页网关自己）。
    """
    say = log or (lambda msg: print(msg, flush=True))
    protect = _ancestor_pids()

    found = {}
    for pat in _CLEANUP_PATTERNS:
        for pid in _match_pids(pat):
            if pid in protect:           # # 保险②：别杀自己的血脉
                continue
            found.setdefault(pid, pat)
    if not found:
        return 0

    # 保险③：再按命令行内容过一道，把"网页网关自己"摘出去
    real = {}
    for pid, pat in found.items():
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\x00', b' ').decode('utf-8', 'replace')
        except (OSError, PermissionError):
            continue
        if any(x in cmd for x in _CLEANUP_LAUNCH_EXCLUDE):
            say(f'  跳过 {pid}（是网页网关自己）')
            continue
        real[pid] = pat
    found = real
    if not found:
        return 0

    say(f'清理上次残留进程：{len(found)} 个')
    for pid, pat in found.items():
        say(f'  SIGINT {pid} ({pat})')
        try:
            os.kill(pid, signal.SIGINT)
        except (ProcessLookupError, PermissionError):
            pass

    deadline = time.time() + timeout
    while time.time() < deadline:
        alive = [p for p in found if _pid_alive(p)]
        if not alive:
            say('  已全部退出')
            return len(found)
        time.sleep(0.25)

    alive = [p for p in found if _pid_alive(p)]
    for pid in alive:
        say(f'  SIGKILL {pid}（未在 {timeout:.0f}s 内退出）')
        try:
            os.kill(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
    return len(found)


# ══════════════ 换栈前的"彻底清理"（导航 ⇄ 建图 必须走这套）══════════════
#
# ★ 这一组函数解决的是用户反复报的那类 bug：
#     "切到建图模式地图黑屏、自检说节点都没起来"
#     "切换后老是有遗留，车还在飘 / 看不到激光"
#   根因不在建图代码，而在**换栈时旧的那套没收干净**：
#
#   ① `proc.wait()` 只保证 `ros2 launch` 这个父进程退了。它派生的
#      gzserver / nav2 / slam_toolbox 是**进程组**里的兄弟进程，
#      父进程退出时它们可能还在跑（gzserver 收尾要好几秒）。
#      于是新栈刚起来时，11345 端口还被老 gzserver 占着 ——
#      而 Gazebo 的 bind 失败是**静默**的：新 gzserver 日志照打、
#      世界里却什么都没有，看着像"仿真起了但地图一片黑"。
#
#   ② 被 Ctrl-C 打断的 rclpy 节点几乎都死在 `destroy_node()` 里
#      （日志里那一串 KeyboardInterrupt traceback 就是），
#      于是 FastDDS 的共享内存段来不及 unlink，留在 /dev/shm。
#      下一个栈的所有参与者在 `open_and_lock_file` 上撞到这些残留：
#         [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port74xx
#      本机走的是"同机优先共享内存"这条路，SHM 一废，新旧参与者
#      就发现不到彼此 —— 网页上的表现就是「节点未发现 / 收不到 /scan /
#      地图黑屏」，而终端里节点其实活得好好的。
#
#   所以顺序必须是：发信号 → 等**整组**死透 → 清残留 SHM → 等端口空 → 才起新栈。

# 只碰这两类名字。FastDDS 的段名形如 fastrtps_port7411_xxxx、sem.fastrtps_...
_SHM_STALE_PREFIXES = ('fastrtps_', 'sem.fastrtps_')

# Gazebo Classic 默认 master 端口（gzserver bind 这个）。
GAZEBO_MASTER_PORT = 11345


def _group_members(pgid):               # out 进程组里还活着的 PID 列表
    """列出某进程组里**还没死**的成员（僵尸不算）。

    ★ 为什么不用 `os.killpg(pgid, 0)` 判活：僵尸进程也占着 PID，
      killpg(0) 会一直成功，`while 组还活着` 就永远退不出来。
      容器里 PID 1 不一定帮忙回收孤儿，这个坑很容易踩。
      所以直接读 /proc/<pid>/stat 拿 pgid 和状态，跳过 Z。
    """
    out = []
    try:
        names = os.listdir('/proc')
    except OSError:
        return out
    for name in names:
        if not name.isdigit():
            continue
        try:
            with open(f'/proc/{name}/stat', 'rb') as f:
                raw = f.read().decode('utf-8', 'replace')
        except OSError:
            continue                            # # 进程刚好退了
        # stat 第二列是 comm，可能含空格和括号 → 先切掉再按空格分
        try:
            rest = raw.rsplit(')', 1)[1].split()
            state, pgrp = rest[0], int(rest[2])  # # 0=state 1=ppid 2=pgrp
        except (IndexError, ValueError):
            continue
        if pgrp != pgid or state == 'Z':
            continue
        out.append(int(name))
    return out


def wait_group_dead(pgid, timeout=6.0, log=None):   # out 是否整组已消失
    """等整个进程组真正死光（不只是 launch 父进程）。

    参数：pgid - 进程组号（本项目里 == launch 的 PID，因为 start_new_session=True）
          timeout - 最多等几秒
    返回：True=整组已退出；False=还有活口（调用方该 SIGKILL 了）
    """
    say = log or (lambda msg: None)
    if not pgid:
        return True
    deadline = time.time() + max(0.0, timeout)
    while True:
        left = _group_members(pgid)
        if not left:
            return True
        if time.time() >= deadline:
            say(f'还有 {len(left)} 个进程没退（PID {left[:4]}）')
            return False
        time.sleep(0.2)


def _mapped_shm_names():                # out 正被活进程映射的 /dev/shm 文件名集合
    """扫 /proc/*/maps，收集所有"有主"的共享内存段名。

    ★ 这是删除残留 SHM 的**安全边界**：只有没被任何进程映射的段才允许删。
      网关自己的 FastDDS 段、用户另开的 ROS 程序的段都在这里面，
      因此不会误删（删掉正在用的段会让那个进程直接崩）。
    """
    ref = set()
    try:
        names = os.listdir('/proc')
    except OSError:
        return ref
    for name in names:
        if not name.isdigit():
            continue
        try:
            with open(f'/proc/{name}/maps', 'r', errors='replace') as f:
                for line in f:
                    if '/dev/shm/' not in line:
                        continue
                    seg = line.split('/dev/shm/', 1)[1]
                    seg = seg.split(' (deleted)')[0].strip()
                    if seg:
                        ref.add(seg)
        except OSError:
            continue
    return ref


def purge_stale_shm(shm_dir='/dev/shm', log=None, grace=2.0):   # out 删掉的段数
    """清掉没有任何进程在用的 FastDDS 残留段。

    参数：shm_dir - 共享内存目录（单测传临时目录）
          grace   - 只删"生成超过 grace 秒"的文件，避开正在启动的参与者
    返回：删除条数

    ★ 只删 _SHM_STALE_PREFIXES 里的名字 + 只删无主的，两条同时成立才动。
    """
    say = log or (lambda msg: None)
    try:
        entries = os.listdir(shm_dir)
    except OSError:
        return 0
    ref = _mapped_shm_names()
    now = time.time()
    freed = 0
    for name in entries:
        if not name.startswith(_SHM_STALE_PREFIXES):
            continue
        if name in ref:                     # # 有活进程在用，绝对不能删
            continue
        path = os.path.join(shm_dir, name)
        try:
            if now - os.stat(path).st_mtime < grace:
                continue                    # # 刚生成的，可能是刚要起来的参与者
            os.unlink(path)
            freed += 1
        except OSError:
            continue
    if freed:
        say(f'清理残留共享内存：{freed} 段（上一次被强杀的节点没来得及释放）')
    return freed


def wait_port_free(port=GAZEBO_MASTER_PORT, timeout=6.0, log=None):   # out 端口是否空出
    """等 Gazebo master 端口空出来。

    ★ 为什么必须等：gzserver bind 失败**不打日志、不退出**，
      新栈会"看起来正常启动"，实际世界里没数据 —— 这就是
      "切到建图模式，地图一片黑、自检说节点都没起"的直接成因之一。
    返回：True=端口已空；False=超时仍被占（调用方只告警，不阻塞启动）
    """
    say = log or (lambda msg: None)
    deadline = time.time() + max(0.0, timeout)
    warned = False
    while True:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.bind(('', port))
            return True
        except OSError:
            if time.time() >= deadline:
                say(f'端口 {port} 仍被占用，新栈的 Gazebo 可能起不来'
                    f'（它 bind 失败是静默的，看不出报错）')
                return False
            if not warned:
                say(f'端口 {port} 还被旧栈占着，等它释放…')
                warned = True
            time.sleep(0.2)
        finally:
            sock.close()
