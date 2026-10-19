#!/usr/bin/env python3
"""
agv_console.py —— AGV 简化终端控制台（一个脚本搞定日常测试）
=============================================================================
干什么：把「起仿真 / 建图 / 存点 / 存路线 / 导航 / 手动开 / 顶升 / 急停 /
        查状态 / 任务系统 / 自检」全部收进一个带菜单的终端程序。
        **启动先选模式**，两种模式互不相扰。

两种模式（启动时选）
--------------------
【建图模式】起 仿真 + SLAM，遥控走一圈，命名保存地图（地图与点位/路线一起存）。
    · [k] 键盘遥控小车（新开一个终端跑 teleop，直接驱底盘）
    · [m] 保存当前地图（输入地图名）
    · [d] 删除地图     · [l] 列出已保存的地图
    · [n] 切换到导航模式   · [q] 退出

【导航模式】先选一份已保存地图加载，然后就是原来那套业务功能。
    · 进入时提示：加载地图 / 删除地图 / 列出地图 / 退出
    · 加载某地图后：存点位 / 路线管理 / 导航 / 完整搬运 / 任务系统 /
                    手动 / 顶升下降 / 急停 / 查状态 / 自检 全部可用
    · [0] 返回地图选择（换地图）   · [q] 退出（只有本界面能退出终端）

【页面层级：怎么返回、怎么退出】
    导航主界面（加载地图后那个菜单）
      ├─ [7] 路线管理   ─┐ 子界面里按 q = 回到导航主界面
      ├─ [9] 任务系统   ─┤   （返回后会自动重印主菜单，好知道自己在哪）
      ├─ [6] 手动/ [t] 搬运/ [c] 自检 ─┘
      └─ 在主界面按 q = 退出终端（有任务在跑或有缓冲会先确认）
    ★ 也就是：想退出，先回到导航主界面，再按 q；在功能里按 q 只返回上一层。

点位与路线（都与地图绑定）
--------------------------
每份地图存档是一个目录 `<工作区>/maps/<地图名>/`，里面同时放：
    <地图名>.pgm / <地图名>.yaml —— 栅格与元数据（Nav2 用）
    waypoints.yaml                —— 这份地图专属的**点位**（分搬货点 cargo / 导航点 nav）
    routes.yaml                   —— 这份地图专属的**路线**（若干点位的位姿快照）
    tasks.yaml                    —— 这份地图专属的**任务缓冲**（控制台任务系统用）
加载哪份地图，就用哪份点位/路线/任务；删地图=连它们一起删，不留孤儿数据。

用法
----
1) 交互式（推荐）
     cd <工作区根> && source install/setup.bash
     python3 tools/agv_console.py
2) 直接给子命令
     python3 tools/agv_console.py check     # 一键自检（要业务节点在跑）
     python3 tools/agv_console.py maps      # 只列出地图库
"""
import argparse
import os
import select
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from agv_interfaces.msg import AgvCommand, TaskState
from agv_interfaces.srv import AgvQuery, AgvStore, CargoCmd


# ───────────────────── 点位/路线存储服务的小助手 ─────────────────────
# 合并后所有点位/路线操作都走同一个 /agv/store，靠 resource + action 区分。
# 这两个函数把"建请求"的样板收在一处，调用处只写业务意图。
STORE_SRV = '/agv/store'                 # def 统一存储服务名


def wp_req(action, **kw):                # use 造一个"点位"存储请求
    return AgvStore.Request(resource='waypoint', action=action, **kw)


def rt_req(action, **kw):                # use 造一个"路线"存储请求
    return AgvStore.Request(resource='route', action=action, **kw)

# ───────────────────────────── 外观：颜色与图标 ─────────────────────────────
CLR = {'r': '\033[31m', 'g': '\033[32m', 'y': '\033[33m',
       'b': '\033[34m', 'c': '\033[36m', 'bold': '\033[1m', 'off': '\033[0m'}


def c(text, color):                       # use 给文字上色
    return f"{CLR.get(color, '')}{text}{CLR['off']}"


BANNER = f"""
{c('╔══════════════════════════════════════════════════════════╗', 'c')}
{c('║          AGV 搬运车 · 终端控制台 v3.0                    ║', 'c')}
{c('║          启动先选：建图模式 / 导航模式                   ║', 'c')}
{c('╚══════════════════════════════════════════════════════════╝', 'c')}
"""

# 工作区根目录（本文件在 tools/ 下，向上一级）
WS = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

KIND_CARGO = 'cargo'                      # def 搬货点
KIND_NAV = 'nav'                          # def 导航点
KIND_TEXT = {KIND_CARGO: '搬货点', KIND_NAV: '导航点'}

_EOF = False                              # def 输入流是否已结束（管道/重定向时防死循环）


def _ask(prompt, default=''):             # use 读一行输入（带默认值）
    global _EOF
    try:
        s = input(c(f'  {prompt}', 'y')).strip()
    except EOFError:
        _EOF = True                       # # 输入流结束：调用方应尽快退出菜单
        print()
        return default
    return s or default


def _yn(prompt):                          # use 读一个 是/否
    return _ask(prompt + ' [y/N]: ', 'n').lower().startswith('y')


def _poll_key(timeout=0.0):               # use 非阻塞读一个按键（任务执行中查询用）
    try:
        if select.select([sys.stdin], [], [], timeout)[0]:
            line = sys.stdin.readline()
            if line == '':                # # EOF：当作退出
                global _EOF
                _EOF = True
                return 'q'
            return line.strip().lower()
    except (OSError, ValueError):
        pass
    return None


def _map_manager():                       # use 取地图库管理器（纯逻辑模块）
    if WS not in sys.path:
        sys.path.insert(0, os.path.join(WS, 'src', 'agv_common'))
    from agv_common.map_manager import MapManager, default_maps_root
    return MapManager(default_maps_root(ws=WS))


# ═══════════════════════════ 一、控制台节点 ═══════════════════════════
class Console(Node):
    """控制台节点：持有各发布器/服务客户端，菜单里的动作都调它的方法。"""

    def __init__(self, manage_nodes=False, waypoint_file='', routes_file='',
                 tasks_file=''):
        super().__init__('agv_console')
        self.manage_nodes = manage_nodes       # def 是否托管业务节点
        self.waypoint_file = waypoint_file     # def 业务节点使用的点位文件
        self.routes_file = routes_file         # def 业务节点使用的路线文件
        self.tasks_file = tasks_file           # def 任务缓冲文件（本控制台读写）

        # 发布器
        self.cmd_pub = self.create_publisher(AgvCommand, '/agv/command', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.manual_pub = self.create_publisher(Bool, '/agv/manual_mode', 10)  # 手动模式标志

        # 订阅（缓存最新状态，供菜单显示）
        self.state = None                      # def 最新 TaskState
        self.cargo_busy = None                 # def 最新"正在搬货"标志
        self.create_subscription(TaskState, '/agv/task_state', self._on_state, 10)
        self.create_subscription(Bool, '/agv/cargo/busy', self._on_busy, 10)

        self._srv = {}
        self.procs = []                        # def 托管的外部进程（业务节点/仿真）
        self.stack_log = None                  # def 整套 launch 的日志路径（[l]/自检读取）
        self.logdir = tempfile.mkdtemp(prefix='agv_console.')
        self.task_buffer = []                  # def 任务缓冲（dict: pick/drop/loops/route）

        # ── 自旋：后台执行器线程 ──
        # ★ 必须自旋，否则本节点的服务响应 future 永远不会 done（call() 只 sleep
        #   不 spin），话题回调（/agv/task_state、/agv/cargo/busy）也永不触发。
        self._exec = MultiThreadedExecutor()
        self._exec.add_node(self)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

        if manage_nodes:
            self.start_nodes()

    # ── 订阅回调 ──
    def _on_state(self, msg):
        self.state = msg

    def _on_busy(self, msg):
        self.cargo_busy = bool(msg.data)

    # ── 托管子节点 ──
    def start_nodes(self):                     # out 起三个业务节点
        print(c('  正在启动业务节点（waypoint / cargo / task_manager）…', 'b'))
        extra = ''
        if self.waypoint_file:                 # # 让点位节点用"当前地图专属"的文件
            print(c(f'  点位文件：{self.waypoint_file}', 'b'))
            extra = f'-p waypoint_file:={self.waypoint_file}'
        if self.routes_file:
            print(c(f'  路线文件：{self.routes_file}', 'b'))
            extra = (extra + ' ' if extra else '') + f'-p routes_file:={self.routes_file}'
        self.procs.append(self._spawn('waypoint_node', extra=extra))
        self.procs.append(self._spawn('cargo_node'))
        self.procs.append(self._spawn('task_manager_node'))
        time.sleep(5)

    def _spawn(self, exe, extra=''):           # in 起一个 ros2 run 节点
        pkg = {'waypoint_node': 'agv_waypoint', 'cargo_node': 'agv_lift',
               'task_manager_node': 'agv_task'}[exe]
        cmd = f'ros2 run {pkg} {exe}'
        if extra:
            cmd += f' --ros-args {extra}'
        log = os.path.join(self.logdir, f'{exe}.log')
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        f = open(log, 'w')
        return subprocess.Popen(['setsid', 'bash', '-c', f'exec {cmd}'],
                                stdout=f, stderr=subprocess.STDOUT, env=env)

    def stop_nodes(self):                      # out 收掉托管进程（整进程组）
        for p in self.procs:
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGINT)
                except (ProcessLookupError, PermissionError):
                    pass
        time.sleep(1.0)
        for p in self.procs:
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
        self.procs = []

    # ── 通用：发指令 / 调服务 ──
    # 参数：command - 指令名；args - 参数列表；rid - 请求号（默认按时间生成）；
    #       route - 可选路线名；repeat - 重发次数（防丢用；测去重时传 1）
    # 返回：本次使用的 request_id
    def send_cmd(self, command, args=None, rid=None, route='', repeat=3):
        m = AgvCommand()
        m.source = 'console'
        m.command = command
        m.args = list(args or [])
        m.route = route
        m.request_id = rid or f'con-{int(time.time() * 1000)}'
        m.confidence = 1.0
        for _ in range(max(1, repeat)):
            self.cmd_pub.publish(m)
            time.sleep(0.15)
        return m.request_id

    def call(self, srv_type, name, req, timeout=5.0):   # out 调服务，返回响应/None
        cli = self._srv.get(name)
        if cli is None:
            cli = self.create_client(srv_type, name)
            self._srv[name] = cli
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        return fut.result() if fut.done() else None

    # 后台自旋线程体：跑执行器直到 shutdown
    def _spin(self):                           # in 后台线程
        try:
            self._exec.spin()
        except Exception:
            pass                               # rclpy 已 shutdown / 节点已销毁 → 静默退出

    # 释放节点：先停后台自旋线程，再真正销毁
    def destroy_node(self):                    # out 对外接口
        try:
            self._exec.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        return super().destroy_node()

    # ── 点位 ──
    def do_save(self):
        name = _ask('点位名（如 1号货架 / 走廊A）: ')
        if not name:
            print(c('  已取消：名不能为空', 'y')); return
        kind = KIND_NAV if _yn('  这是"导航点"吗？（只用来组成路线；否=搬货点）') else KIND_CARGO
        try:
            x = float(_ask('x（米）: ', '0')); y = float(_ask('y（米）: ', '0'))
            yaw = float(_ask('yaw（弧度）: ', '0'))
        except ValueError:
            print(c('  坐标必须填数字', 'r')); return
        r = self.call(AgvStore, STORE_SRV,
                      wp_req('save', name=name, x=x, y=y, yaw=yaw, kind=kind))
        print(c(f'  存点结果：{r.message if r else "服务无响应"}',
                'g' if r and r.ok else 'r'))

    def do_list(self):
        r = self.call(AgvStore, STORE_SRV, wp_req('list'))
        if not r:
            print(c('  列点失败（waypoint_node 没起？）', 'r')); return
        if not r.items:
            print(c('  （暂无点位）', 'y')); return
        print(c('  点位表：', 'b'))
        for p in r.items:
            tag = KIND_TEXT.get(p.kind, p.kind)
            print(f'    · {p.name:14s} [{tag}] x={p.x:7.2f}  y={p.y:7.2f}  yaw={p.yaw:6.2f}')

    def do_delete(self):
        name = _ask('要删的点位名: ')
        r = self.call(AgvStore, STORE_SRV, wp_req('delete', name=name))
        print(c(f'  删除结果：{r.message if r else "服务无响应"}',
                'g' if r and r.ok else 'y'))

    # ── 路线 ──
    def list_routes(self):                     # out 取路线名列表（供选择/展示）
        r = self.call(AgvStore, STORE_SRV, rt_req('list'))
        if not r:
            print(c('  列路线失败（waypoint_node 没起？）', 'r')); return []
        if not r.names:
            print(c('  （暂无路线）', 'y')); return []
        print(c('  路线表：', 'b'))
        for n, cnt in zip(r.names, r.counts):
            print(f'    · {n:14s} 途经 {cnt} 个点')
        return list(r.names)

    def choose_route(self):                    # out 让用户挑一条路线（返回名字或 ''）
        names = self.list_routes()
        if not names:
            return ''
        name = _ask('  选哪条路线（写名字，回车=不用路线）: ')
        if not name:
            return ''
        if name not in names:
            print(c(f'  没有路线「{name}」，按不使用路线处理', 'y')); return ''
        return name

    def do_route(self):                        # out 路线管理子菜单
        while True:
            print(ROUTE_MENU)
            ch = _ask('请输入命令: ').lower()
            if _EOF or ch in ('q', 'quit', 'exit', ''):
                return
            try:
                if ch == '1':
                    self._route_save()
                elif ch == '2':
                    self.list_routes()
                elif ch == '3':
                    name = _ask('要看哪条路线的详情: ')
                    if not name:
                        continue
                    r = self.call(AgvStore, STORE_SRV, rt_req('get', name=name))
                    if not r or not r.ok:
                        print(c(f'  {r.message if r else "服务无响应"}', 'y')); continue
                    print(c(f'  路线「{name}」：', 'b'))
                    for i, p in enumerate(r.items, 1):
                        print(f'    {i}. {p.name:14s} x={p.x:7.2f} y={p.y:7.2f} yaw={p.yaw:6.2f}')
                elif ch == '4':
                    name = _ask('要删的路线名: ')
                    r = self.call(AgvStore, STORE_SRV, rt_req('delete', name=name))
                    print(c(f'  {r.message if r else "服务无响应"}',
                            'g' if r and r.ok else 'y'))
                elif ch in ('h', 'help', '?'):
                    print(ROUTE_MENU)
                else:
                    print(c('  未知命令，按 h 看帮助', 'y'))
            except KeyboardInterrupt:
                print(c('  （已中断本次操作）', 'y'))
            except Exception as exc:
                print(c(f'  [出错] {exc}', 'r'))

    def _route_save(self):                     # in 设置/修改一条路线
        r = self.call(AgvStore, STORE_SRV, wp_req('list'))
        if not r or not r.items:
            print(c('  还没有点位：先存几个"导航点"再来设路线', 'y')); return
        navs = [p.name for p in r.items if p.kind == KIND_NAV]
        # 带类型列出，方便挑"导航点"组路线（搬货点也能用，但不参与搬货动作）
        print(c('  可用点位（[导航点] 优先用于组路线）：', 'b'))
        for p in r.items:
            print(f'    {p.name:14s} [{KIND_TEXT.get(p.kind, p.kind)}]  x={p.x:7.2f} y={p.y:7.2f}')
        if not navs:
            print(c('  提示：当前还没有"导航点"，下面只能从搬货点里挑', 'y'))
        name = _ask('路线名（重名即修改）: ')
        if not name:
            print(c('  已取消', 'y')); return
        raw = _ask('按顺序输入途经点名，用 "," 隔开（如 走廊A,走廊B,1号货架）: ')
        names = [s.strip() for s in raw.replace('，', ',').split(',') if s.strip()]
        if not names:
            print(c('  已取消：至少要一个途经点', 'y')); return
        r2 = self.call(AgvStore, STORE_SRV, rt_req('save', name=name, point_names=names))
        print(c(f'  {r2.message if r2 else "服务无响应"}',
                'g' if r2 and r2.ok else 'r'))
        if r2 and r2.ok:
            print(c(f'  路线「{name}」已保存：{len(names)} 个途经点', 'g'))

    # ── 导航 / 搬运 ──
    def do_goto(self):
        name = _ask('去哪个点（点名）: ')
        if not name:
            print(c('  已取消', 'y')); return
        route = self.choose_route() if _yn('  是否使用路线（多点导航）？') else ''
        rid = self.send_cmd('goto', [name], route=route)
        tip = f'（经路线「{route}」）' if route else ''
        print(c(f'  已发送导航指令 →「{name}」{tip}，看状态变化', 'g'))

    def do_goto_xy(self):
        try:
            x = float(_ask('目标 x（米）: '))
            y = float(_ask('目标 y（米）: '))
            w = float(_ask('yaw: ', '0'))
        except ValueError:
            print(c('  坐标必须填数字', 'r')); return
        self.send_cmd('goto', [str(x), str(y), str(w)])
        print(c(f'  已发送导航指令 →({x}, {y})', 'g'))

    def do_start_task(self):
        pick = _ask('取货点名: ')
        if not pick:
            print(c('  已取消：取货点名不能为空', 'y')); return
        drop = _ask('放货点名: ')
        if not drop:
            print(c('  已取消：放货点名不能为空', 'y')); return
        route = self.choose_route() if _yn('  是否使用路线（取货点→路线→放货点）？') else ''
        self.send_cmd('start_task', [pick, drop], route=route)
        tip = f'（中途经路线「{route}」）' if route else ''
        print(c(f'  已发送完整搬运 →「{pick}」⇒「{drop}」{tip}，看状态变化', 'g'))

    # ── 搬货（顶升 / 下降）：只调一个服务 ──
    def do_cargo(self, up=True):
        cmd = 'lift' if up else 'lower'
        r = self.call(CargoCmd, '/agv/cargo', CargoCmd.Request(command=cmd))
        if not r:
            print(c('  搬货服务无响应（cargo_node 没起？）', 'r')); return
        mark = 'g' if r.ok else 'r'
        print(c(f'  {r.message}（正在搬货={r.busy}）', mark))
        time.sleep(0.3)
        print(c(f'  搬货中标志 /agv/cargo/busy = {self.cargo_busy}', 'b'))

    # ── 安全 / 状态 ──
    def do_stop(self):
        self.send_cmd('stop')
        time.sleep(0.5)
        print(c('  已发急停（stop）；看 task_state 应变 PAUSED', 'r'))

    def do_resume(self):
        self.send_cmd('resume')
        time.sleep(0.5)
        print(c('  已发恢复（resume）；看 task_state 应回 IDLE', 'g'))

    def do_query(self):
        for what in ('state', 'pose'):
            r = self.call(AgvQuery, '/agv/query', AgvQuery.Request(what=what))
            if r and r.ok:
                if what == 'pose':
                    print(c(f'  当前位置：x={r.x:.2f} y={r.y:.2f} yaw={r.yaw:.2f}', 'b'))
                else:
                    print(c(f'  {what}: {r.message}', 'b'))
            else:
                print(c(f'  {what}: 无响应（task_manager_node 没起？）', 'y'))
        # 点位表 / 路线表属于"存储内容"，走统一的 /agv/store（不再经 /agv/query）
        r = self.call(AgvStore, STORE_SRV, wp_req('list'))
        print(c(f'  waypoints: {r.message if r and r.ok else "无响应"}', 'b'))
        r = self.call(AgvStore, STORE_SRV, rt_req('list'))
        print(c(f'  routes: {r.message if r and r.ok else "无响应"}', 'b'))

    def do_manual(self):
        # ── 手动/自动互锁：车正在跑任务时先问用户要不要停掉 ──
        active = _active_task_label(self.state)
        if active:
            print(c(f'  检测到小车正在执行任务：{active}', 'y'))
            if not _yn('  是否停止当前任务并清除，然后进入手动控制？'):
                print(c('  已取消手动控制（先处理完当前任务或按 [x] 急停）', 'y'))
                return
            self.manual_pub.publish(Bool(data=True)); time.sleep(0.2)
            self.send_cmd('cancel_task')
            time.sleep(0.8)
            self.send_cmd('resume')
            time.sleep(0.5)
            print(c('  已停止并清除当前任务', 'g'))
        else:
            self.manual_pub.publish(Bool(data=True)); time.sleep(0.2)   # 直接进手动模式

        try:
            v = float(_ask('线速度 m/s（正=前进，负=后退）: ', '0.2'))
            w = float(_ask('角速度 rad/s（正=左转）: ', '0.0'))
            dur = float(_ask('持续几秒: ', '3'))
        except ValueError:
            print(c('  必须填数字', 'r'))
            self.manual_pub.publish(Bool(data=False))   # 退出手动模式
            return
        print(c(f'  手动发送 /cmd_vel 共 {dur} 秒（Ctrl-C 可提前停）…', 'b'))
        tw = Twist(); tw.linear.x = v; tw.angular.z = w
        t0 = time.time()
        try:
            while time.time() - t0 < dur:
                self.vel_pub.publish(tw)
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        self.vel_pub.publish(Twist())
        self.manual_pub.publish(Bool(data=False))       # io 退出手动模式
        print(c('  已停止（已退出手动模式）', 'g'))

    def do_status(self):
        s = self.state
        print(c('  ── 任务状态 /agv/task_state ──', 'b'))
        if s:
            print(f'    状态={s.state}  进度={s.progress:.2f}  '
                  f'目标={s.target_location}  {s.message}')
        else:
            print(c('    （还没收到，task_manager_node 起了吗？）', 'y'))
        print(c('  ── 搬货状态 /agv/cargo/busy ──', 'b'))
        print(f'    {_busy_text(self.cargo_busy)}')
        if self.manage_nodes:
            print(c(f'  子节点日志目录：{self.logdir}', 'b'))

    def do_stack_status(self):
        print(c('  ── 定位/导航节点激活状态 ──', 'b'))
        for n in ('map_server', 'amcl', 'bt_navigator', 'controller_server', 'planner_server'):
            try:
                out = subprocess.run(['ros2', 'lifecycle', 'get', f'/{n}'],
                                     capture_output=True, text=True, timeout=8).stdout.strip()
                # ★ 必须精确匹配：'inactive' 里也含 'active' 子串
                state = 'active' if out.splitlines()[-1].strip() == 'active' else 'inactive'
            except subprocess.TimeoutExpired:
                state = 'timeout'
            except Exception:
                state = 'error'
            if state == 'active':
                mark = c('已激活', 'g')
            elif state == 'timeout':
                mark = c('查询超时（仿真可能刚起，稍后再按 n）', 'y')
            else:
                mark = c('未激活/未起', 'y')
            print(f'    {n:20s} {mark}')
        print(c('  提示：全为「已激活」才能发导航；否则等一会儿再按 n', 'y'))

    # ── 任务缓冲（任务系统用）──
    def load_tasks(self):                      # out 从 tasks.yaml 读任务缓冲
        self.task_buffer = []
        if not self.tasks_file or not os.path.isfile(self.tasks_file):
            return
        try:
            import yaml
            with open(self.tasks_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            for t in (data.get('tasks', []) or []):
                if not isinstance(t, dict):
                    continue
                self.task_buffer.append({
                    'pick': str(t.get('pick', '') or ''),
                    'drop': str(t.get('drop', '') or ''),
                    'loops': max(1, int(t.get('loops', 1) or 1)),
                    'route': str(t.get('route', '') or ''),
                })
        except Exception as exc:
            print(c(f'  任务缓冲读取失败（{exc}），按空处理', 'y'))
            self.task_buffer = []

    def save_tasks(self):                      # out 把任务缓冲写回 tasks.yaml
        if not self.tasks_file:
            return False
        try:
            import yaml
            os.makedirs(os.path.dirname(os.path.abspath(self.tasks_file)), exist_ok=True)
            with open(self.tasks_file, 'w', encoding='utf-8') as f:
                f.write('# 本文件的搬运任务与地图绑定；由控制台"任务系统"读写\n')
                yaml.safe_dump({'tasks': self.task_buffer}, f,
                               allow_unicode=True, sort_keys=False)
            return True
        except Exception as exc:
            print(c(f'  任务缓冲保存失败：{exc}', 'r'))
            return False


# 判断当前是否"有任务在跑"（供手动/自动互锁提示用）
# 参数：state - 最近的 TaskState 消息（或 None）
# 返回：中文描述（空串=空闲，可安全手动）
def _active_task_label(state):
    if state is None:
        return ''
    st = (getattr(state, 'state', '') or '').upper()
    if st in ('IDLE', ''):
        return ''
    busy = {'GO_LOAD', 'LIFTING', 'TRANSPORT', 'LOWERING', 'PAUSED'}
    if st in busy or st == 'ERROR':
        return f'{st}（目标 {getattr(state, "target_location", "") or "-"}）'
    return ''


def _busy_text(busy):
    if busy is None:
        return '（还没收到 /agv/cargo/busy）'
    return '正在搬货（标志=1）' if busy else '空闲（标志=0）'


# ═══════════════════════════ 二、一键自检（不起仿真） ═══════════════════════════
def run_check(console):
    """跑一遍核心功能自检，打印通过/失败。返回失败数。

    原则：① 只用自己的临时名字，跑完清理；② 不做任何会让车动起来的事
    （越界坐标/重复请求天然被门卫挡下）；③ 急停后一定补 resume 复位。
    """
    p = f = 0

    def ok(m):
        nonlocal p; p += 1; print(c(f'  [通过] {m}', 'g'))

    def bad(m):
        nonlocal f; f += 1; print(c(f'  [失败] {m}', 'r'))

    print(c('\n═══ 一键自检（不需要仿真） ═══', 'bold'))
    tmp = f'自检点{int(time.time())}'          # def 本次专用的临时点名
    r = console.call(AgvStore, STORE_SRV, wp_req('save', name=tmp, x=1.0, y=0.0,
                                                 yaw=0.0, kind=KIND_NAV))
    ok('存点（带类型 nav）') if r and r.ok else bad('存点')
    r = console.call(AgvStore, STORE_SRV, wp_req('get', name=tmp))
    ok('查点且坐标/类型正确') if (r and r.ok and r.items
                                and abs(r.items[0].x - 1.0) < 1e-6
                                and r.items[0].kind == KIND_NAV) else bad('查点')
    r = console.call(AgvStore, STORE_SRV, wp_req('get', name='不存在'))
    ok('查不存在的点返回失败而不是崩溃') if r and not r.ok else bad('查不存在点')
    r = console.call(AgvStore, STORE_SRV, wp_req('list'))
    ok('列点含已存点') if r and r.ok and tmp in [p.name for p in r.items] else bad('列点')
    ok('列点响应带 ok 字段') if r and hasattr(r, 'ok') else bad('列点缺 ok')

    rt = f'自检路线{int(time.time())}'         # def 本次专用的临时路线名
    r = console.call(AgvStore, STORE_SRV, rt_req('save', name=rt, point_names=[tmp]))
    ok('存路线（按点名复制位姿）') if r and r.ok and r.count == 1 else bad('存路线')
    r = console.call(AgvStore, STORE_SRV, rt_req('get', name=rt))
    ok('查路线且途经点正确') if (r and r.ok and r.items
                                and [p.name for p in r.items] == [tmp]) else bad('查路线')
    r = console.call(AgvStore, STORE_SRV, rt_req('list'))
    ok('列路线含已存路线') if r and r.ok and rt in list(r.names) else bad('列路线')
    r = console.call(AgvStore, STORE_SRV, rt_req('delete', name=rt))
    ok('删路线') if r and r.ok else bad('删路线')
    r = console.call(AgvStore, STORE_SRV, rt_req('delete', name=rt))
    ok('删不存在的路线返回失败而不是崩溃') if r and not r.ok else bad('删不存在路线')
    r = console.call(AgvStore, STORE_SRV, wp_req('delete', name=tmp))
    ok('删点（清理自检痕迹）') if r and r.ok else bad('删点')

    # 统一存储服务的参数校验：未知资源 / 未知动作要被拒（合并后新增的一道关）
    r = console.call(AgvStore, STORE_SRV,
                     AgvStore.Request(resource='shelf', action='list'))
    ok('未知资源被拒') if r and not r.ok else bad('未知资源未拒')
    r = console.call(AgvStore, STORE_SRV,
                     AgvStore.Request(resource='waypoint', action='rename'))
    ok('未知动作被拒') if r and not r.ok else bad('未知动作未拒')
    # 空路线：不留空壳
    r = console.call(AgvStore, STORE_SRV,
                     rt_req('save', name=f'空路线{int(time.time())}', point_names=[]))
    ok('空路线被拒') if r and not r.ok else bad('空路线未拒')

    r = console.call(CargoCmd, '/agv/cargo', CargoCmd.Request(command='lift'))
    ok('搬货服务：顶升受理且标志置 1') if r and r.ok and r.busy else bad('顶升服务')
    r = console.call(CargoCmd, '/agv/cargo', CargoCmd.Request(command='lower'))
    ok('搬货服务：下降受理且标志归 0') if r and r.ok and not r.busy else bad('下降服务')
    r = console.call(CargoCmd, '/agv/cargo', CargoCmd.Request(command='乱写'))
    ok('搬货服务：未知命令被拒') if r and not r.ok else bad('搬货未知命令')

    # 门卫：未知指令 / 越界坐标 / 重复请求（发给 task_manager，看它日志里是否出现拒绝）
    base = _read_log(console, 'task_manager_node')
    console.send_cmd('fly', rid=f'chk-fly-{int(time.time())}')
    console.send_cmd('goto', ['9999', '9999', '0'], rid=f'chk-oob-{int(time.time())}')
    time.sleep(0.6)
    tail = _read_log(console, 'task_manager_node')[len(base):]
    ok('未知指令被拒') if '未知指令' in tail else bad('未知指令未拒')
    ok('越界坐标被拒') if '超出场地范围' in tail else bad('越界坐标未拒')

    # 去重：同一个 request_id 只受理一次（repeat=1 才不会自己把号先占掉）。
    # ★ 探针必须选"能通过前面所有校验"的指令：越界/未知指令会先被拦下，
    #   根本走不到去重那一层，检查项就永远看不到「重复」。cancel_task 无参、
    #   通过全部校验、且此刻本来就没任务，不会让车动。
    rid = f'chk-dup-{int(time.time())}'
    base = _read_log(console, 'task_manager_node')
    console.send_cmd('cancel_task', rid=rid, repeat=1)
    console.send_cmd('cancel_task', rid=rid, repeat=1)
    time.sleep(0.6)
    tail = _read_log(console, 'task_manager_node')[len(base):]
    ok('重复请求被拒') if '重复' in tail else bad('去重')

    console.send_cmd('stop', rid=f'chk-stop-{int(time.time())}')
    time.sleep(0.6)
    ok('急停生效') if '已急停' in _read_log(console, 'task_manager_node') else bad('急停')
    console.send_cmd('resume', rid=f'chk-resume-{int(time.time())}')   # ★ 复位，别把车留在急停
    time.sleep(0.4)

    r = console.call(AgvQuery, '/agv/query', AgvQuery.Request(what='state'))
    ok('查询服务有应答') if r and r.ok else bad('查询服务')
    r = console.call(AgvStore, STORE_SRV, rt_req('list'))
    ok('存储服务能列路线') if r and r.ok else bad('列路线（存储服务）')

    print(c(f'\n  ═══ 结果：通过 {p} 项，失败 {f} 项 ═══',
            'g' if f == 0 else 'r'))
    return f


def _read_log(console, exe):
    # 托管单节点时读各自的日志；整套 launch 模式下所有节点都写进 stack_log
    candidates = []
    if console.stack_log:
        candidates.append(console.stack_log)
    candidates.append(os.path.join(console.logdir, f'{exe}.log'))
    for path in candidates:
        try:
            with open(path, errors='ignore') as fh:
                text = '\n'.join(l for l in fh
                                 if 'RTPS_TRANSPORT' not in l and 'open_and_lock' not in l)
            if text:
                return text
        except OSError:
            continue
    return ''


# ═══════════════════════════ 三、地图库操作（建图/导航通用） ═══════════════════════════
def _count_in_yaml(path, key):             # use 数一数文件里有多少条（点位/路线）
    if not os.path.isfile(path):
        return 0
    try:
        import yaml
        with open(path, encoding='utf-8') as fh:
            d = yaml.safe_load(fh) or {}
        return len(d.get(key, {}) or {})
    except Exception:
        return 0


def _list_maps(mm):
    names = mm.list_maps()
    print(c(f'  ── 地图库（{mm.root}）──', 'b'))
    if not names:
        print(c('    （还没有已保存的地图）', 'y')); return []
    for n in names:
        paths = mm.map_paths(n)
        npt = _count_in_yaml(paths['waypoints'], 'waypoints')
        nrt = _count_in_yaml(paths['routes'], 'routes')
        print(f'    · {n:16s} 点位 {npt} 个 / 路线 {nrt} 条   {paths["dir"]}')
    return names


def _delete_map(mm):
    name = _ask('要删除的地图名: ')
    if not name:
        print(c('  已取消', 'y')); return
    if not _yn(f'确认删除地图「{name}」（含其点位/路线/任务缓冲）？'):
        print(c('  已取消', 'y')); return
    ok, msg = mm.delete_map(name)
    print(c(f'  {msg}', 'g' if ok else 'r'))


# ═══════════════════════════ 四、建图模式 ═══════════════════════════
MAPPING_MENU = """
  ┌──────────────── 建图模式 ────────────────┐
  │ [k] 键盘遥控小车（另开终端）              │
  │ [m] 保存当前地图（命名）                  │
  │ [l] 列出已保存的地图                      │
  │ [d] 删除地图                              │
  │ [n] 切换到导航模式                        │
  │ [q] 退出                                  │
  └──────────────────────────────────────────┘
"""


def _and_terminal(cmd):                        # use 在新终端窗口跑一条命令（遥控要 TTY）
    """在独立终端窗口里跑 cmd；找不到终端则回退后台运行并提示。"""
    inner = f'{cmd}; echo; echo "[命令已结束，按回车关闭]"; read'
    for term in (['gnome-terminal', '--'], ['x-terminal-emulator', '-e'],
                 ['xterm', '-e']):
        exe = shutil.which(term[0])
        if exe:
            if term[0] == 'gnome-terminal':
                subprocess.Popen(['gnome-terminal', '--', 'bash', '-lc', inner])
            else:
                # ★ xterm 的 -e 后每个参数都要独立：不能塞成一个字符串
                subprocess.Popen([exe, '-e', 'bash', '-lc', inner])
            return True
    # 没有图形终端：退背景运行（键盘收不到，但至少节点起着）
    subprocess.Popen(['setsid', 'bash', '-lc', cmd],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return False


def _start_mapping_stack(console, gui=True, rviz=True):
    """起 仿真 + SLAM + RViz（建图模式）。返回进程对象。"""
    _pkill_ros()                               # ★ 先清残留，防多套栈打架
    time.sleep(1.5)
    log = os.path.join(tempfile.gettempdir(), 'agv_console_mapping.log')
    f = open(log, 'w')
    cmd = (f'ros2 launch agv_navigation mapping.launch.py '
           f'gui:={"true" if gui else "false"} rviz:={"true" if rviz else "false"}')
    p = subprocess.Popen(['setsid', 'bash', '-c', f'exec {cmd}'],
                         stdout=f, stderr=subprocess.STDOUT,
                         env=dict(os.environ, PYTHONUNBUFFERED='1'))
    console.procs.append(p)
    print(c(f'  建图栈已启动（仿真 + SLAM + RViz；日志 {log}）', 'b'))
    print(c('  等 Gazebo 起来约 10~20 秒，再按 [k] 遥控走一圈建图', 'y'))
    return p


def mapping_mode(mm, gui=True, rviz=True):
    """建图模式主循环。自己管理 Console 生命周期。返回 'exit' 或 'nav'。"""
    console = Console(manage_nodes=False)      # 建图模式不托管业务节点（只起仿真+SLAM）
    try:
        _start_mapping_stack(console, gui=gui, rviz=rviz)
        while True:
            print(MAPPING_MENU)
            ch = _ask('请输入命令: ').lower()
            if _EOF:
                return 'exit'
            try:
                if ch in ('q', 'quit', 'exit'):
                    return 'exit'
                elif ch == 'k':
                    print(c('  在新终端窗口启动键盘遥控（直接驱底盘 cmd_vel_raw）…', 'b'))
                    okd = _and_terminal(
                        'source /opt/ros/humble/setup.bash && '
                        'ros2 run teleop_twist_keyboard teleop_twist_keyboard '
                        '--ros-args -r cmd_vel:=cmd_vel_raw')
                    print(c('  遥控窗口已打开' if okd else '  未找到图形终端，已后台起遥控（键盘不可用）',
                            'g' if okd else 'y'))
                elif ch == 'm':
                    _save_current_map(mm)
                elif ch == 'l':
                    _list_maps(mm)
                elif ch == 'd':
                    _delete_map(mm)
                elif ch == 'n':
                    if _yn('切换到导航模式会先关掉建图栈，继续？'):
                        return 'nav'
                else:
                    print(c('  未知命令，按 k/m/l/d/n/q', 'y'))
            except KeyboardInterrupt:
                print(c('  （已中断本次操作，回到菜单）', 'y'))
            except Exception as exc:
                print(c(f'  [出错] {exc}', 'r'))
    finally:
        _close_procs(console)
        console.destroy_node()


def _save_current_map(mm):
    """调 map_saver_cli 存当前 SLAM 地图到地图库（命名）。"""
    name = _ask('给这份地图起个名（如 厂房A）: ')
    if not name:
        print(c('  已取消：名不能为空', 'y')); return
    from agv_common.map_manager import valid_name
    ok, reason = valid_name(name)
    if not ok:
        print(c(f'  {reason}', 'r')); return
    if mm.map_exists(name) and not _yn(f'地图「{name}」已存在，覆盖？'):
        print(c('  已取消', 'y')); return
    tmp = tempfile.mkdtemp(prefix='agv_mapsave.')
    out = os.path.join(tmp, name)
    print(c('  正在保存地图（map_saver_cli）…', 'b'))
    try:
        r = subprocess.run(
            ['bash', '-lc',
             'source /opt/ros/humble/setup.bash && '
             f'ros2 run nav2_map_server map_saver_cli -f {shlex.quote(out)}'],
            capture_output=True, text=True, timeout=40)
    except subprocess.TimeoutExpired:
        print(c('  保存超时：SLAM 还在跑吗？/map 有数据吗？', 'r')); return
    if not os.path.isfile(out + '.pgm'):
        print(c('  保存失败：没生成 pgm。检查建图栈是否在跑、是否走过场地。', 'r'))
        print(c(f'  map_saver 输出：{r.stdout.strip()[-300:]} {r.stderr.strip()[-200:]}', 'y'))
        return
    ok, msg = mm.save_map(name, out + '.pgm', out + '.yaml', overwrite=True)
    print(c(f'  {msg}', 'g' if ok else 'r'))


# ═══════════════════════════ 五、导航模式 ═══════════════════════════
NAV_LOAD_MENU = """
  ┌──────────── 导航模式 · 先加载地图 ────────────┐
  │ [1] 加载地图（输入地图名）                     │
  │ [2] 列出已保存的地图                           │
  │ [3] 删除地图                                   │
  │ [m] 切换回建图模式                             │
  │ [q] 退出                                       │
  └───────────────────────────────────────────────┘
"""

NAV_MENU = """
  ┌──────────────── 操作 ────────────────┐   ┌──────── 查看 ────────┐
  │ [1] 存点位(搬货点/导航点) [4] 点名导航│   │ [s] 状态快照          │
  │ [2] 列出点位        [5] 坐标导航      │   │ [n] 导航节点激活状态  │
  │ [3] 删除点位        [6] 手动开小车    │   │ [l] 看子节点日志尾    │
  │ [7] 路线管理        [8] 查询状态      │   │ [c] 一键自检          │
  │ [t] 完整搬运        [9] 任务系统      │   │ [0] 换地图  [q] 退出  │
  │ [u] 顶升 / [d] 下降 [x] 急停 [r] 恢复 │   │                       │
  └──────────────────────────────────────┘   └───────────────────────┘
"""

# 会进入"自己的子界面/多步问答"的功能键：它们跑完后必须重印导航主菜单，
# 否则用户只看到一个光秃秃的提示符，会以为还在子界面里，再按一次 q 就把终端退掉。
_NAV_NESTED = {
    '6': '手动开小车', '7': '路线管理', '9': '任务系统',
    't': '完整搬运', 'c': '一键自检', '0': '换地图',
}


def _nav_home(console, why=''):            # use 回到导航主界面：印面包屑 + 重印主菜单
    """从子界面返回导航主界面时的统一落点。

    ★ 这就是"返回上一页"的实现：子界面里的 q 一律只回到这里，
      真正退出终端只能在本界面按 q。
    """
    print(c(f'  ── 已返回导航主界面（{why}）──' if why else '  ── 已返回导航主界面 ──', 'g'))
    print(NAV_MENU)


def _confirm_leave_nav(console):           # use 在导航主界面按 q 时确认退出（有活先问清）
    """决定是否真的退出终端。返回 True=退出，False=留在主界面。

    只有导航主界面能退出终端；这里把"退出会不会造成损失"问清楚，
    避免误按 q 就把终端关掉、车却还在动。
    """
    active = _active_task_label(console.state)
    if active:
        print(c(f'  检测到小车正在执行任务：{active}', 'y'))
        if not _yn('  退出会取消当前任务并停车，继续？'):
            print(c('  已取消退出（留在导航主界面）', 'y'))
            return False
        _stop_car(console)
    if console.task_buffer:
        print(c(f'  任务缓冲还有 {len(console.task_buffer)} 条未执行', 'y'))
        if not _yn('  退出会清空任务缓冲，继续？'):
            print(c('  已取消退出（留在导航主界面）', 'y'))
            return False
        console.task_buffer = []
        console.save_tasks()
        print(c('  任务缓冲已清空', 'g'))
    return True


ROUTE_MENU = """
  ┌──────────── 路线管理（与地图绑定） ────────────┐
  │ [1] 设置/修改路线（点名用 "," 隔开）           │
  │ [2] 列出全部路线                               │
  │ [3] 查看某条路线的详情                         │
  │ [4] 删除路线                                   │
  │ [q] 返回                                       │
  └────────────────────────────────────────────────┘
"""

TASK_MENU = """
  ┌──────────── 任务系统（任务缓冲） ────────────┐
  │ [1] 添加搬运任务（取货点/放货点/循环/路线）   │
  │ [2] 删除任务（按序号）                        │
  │ [3] 列出任务缓冲                              │
  │ [4] 清空任务缓冲                              │
  │ [5] 启动任务（依次执行，执行中可 s/l/q）      │
  │ [q] 返回导航主界面（清空缓冲并停车）          │
  └──────────────────────────────────────────────┘
"""


def _start_nav_stack(console, map_yaml, waypoint_file, routes_file,
                     gui=True, rviz=True):
    """起整套仿真+导航栈（走 agv_stack.launch.py with_sim:=true）。

    为什么用 agv_stack 而不是 navigation.launch.py：整套链路里底盘听的是
    /cmd_vel_raw，必须有 velocity_arbiter 把 /cmd_vel 转过去；agv_stack 会一起
    拉起 业务节点 + 速度仲裁 + 导航栈，参数也按所选地图/点位/路线注入。
    """
    _pkill_ros()                               # ★ 先清残留，防多套栈打架
    time.sleep(1.5)                            #    （同名节点会让话题/服务/action 互相抢，
                                               #      表现为「节点没起/无响应/查询超时」）
    log = os.path.join(tempfile.gettempdir(), 'agv_console_nav.log')
    f = open(log, 'w')
    cmd = ('ros2 launch agv_bringup agv_stack.launch.py with_sim:=true '
           f'map_yaml:={shlex.quote(map_yaml)} '
           f'waypoint_file:={shlex.quote(waypoint_file)} '
           f'routes_file:={shlex.quote(routes_file)} '
           f'gui:={"true" if gui else "false"} rviz:={"true" if rviz else "false"}')
    p = subprocess.Popen(['setsid', 'bash', '-c', f'exec {cmd}'],
                         stdout=f, stderr=subprocess.STDOUT,
                         env=dict(os.environ, PYTHONUNBUFFERED='1'))
    console.procs.append(p)
    console.stack_log = log                    # def 供 [l] 看日志用
    print(c(f'  整套栈已启动（业务节点 + 速度仲裁 + 导航栈；日志 {log}）', 'b'))
    print(c('  等 Nav2 全激活约 15~40 秒；按 [n] 查激活状态，全绿再导航', 'y'))
    return p


def navigation_mode(mm, gui=True, rviz=True):
    """导航模式主循环。返回 'exit' 或 'mapping'（换回建图模式）。"""
    while True:
        # ── 先选地图 ──
        print(NAV_LOAD_MENU)
        ch = _ask('请输入命令: ').lower()
        if _EOF:
            return 'exit'
        if ch in ('q', 'quit', 'exit'):
            return 'exit'
        if ch == '2':
            _list_maps(mm); continue
        if ch == '3':
            _delete_map(mm); continue
        if ch in ('m', 'mapping'):
            if _yn('切换回建图模式？'):
                return 'mapping'
            continue
        if ch != '1':
            print(c('  未知命令', 'y')); continue
        name = _ask('输入要加载的地图名: ')
        if not name:
            print(c('  已取消', 'y')); continue
        ok, msg = mm.validate(name)
        if not ok:
            print(c(f'  {msg}', 'r')); continue
        paths = mm.map_paths(name)
        wp_file = mm.waypoints_file(name)
        rt_file = mm.routes_file(name)
        tk_file = mm.tasks_file(name)
        if not os.path.isfile(wp_file):        # 兜底：老地图可能没有点位/路线文件
            mm.write_waypoints(name, initial=True)
        if not os.path.isfile(rt_file):
            mm.write_routes(name)

        # ── 为该地图拉起一套完整环境（业务节点 + 速度仲裁 + 导航栈都在这条 launch 里）──
        print(c(f'\n  加载地图「{name}」…', 'b'))
        console = Console(manage_nodes=False, tasks_file=tk_file)
        console.load_tasks()                   # use 读回这份地图的任务缓冲
        print(c(f'  本次使用点位文件：{wp_file}', 'b'))
        print(c(f'  本次使用路线文件：{rt_file}', 'b'))
        _start_nav_stack(console, paths['yaml'], wp_file, rt_file, gui=gui, rviz=rviz)
        time.sleep(2)
        print(c(f'  当前地图：{name}（按 [0] 可换地图）', 'g'))
        print(NAV_MENU)
        result = _nav_business_loop(console)
        _close_procs(console)                  # 收掉整套栈
        console.destroy_node()
        if result == 'mapping':
            return 'mapping'
        if result == 'exit':
            return 'exit'


def _nav_business_loop(console):
    """导航模式下、已加载某地图后的业务菜单循环。返回 'switch_map'/'exit'/'mapping'。

    ★ 页面层级约定（本次明确）：这里就是「导航主界面」。
      · 各功能键（6 手动 / 7 路线 / 9 任务系统 / t 搬运 / c 自检）进去后，
        在子界面按 q **只回到本界面**，不会关掉终端；
      · 只有在本界面按 q 才退出终端（退出前会确认是否要取消任务/清任务缓冲）。
    """
    while True:
        try:
            ch = _ask('请输入命令: ').lower()
        except KeyboardInterrupt:
            return 'switch_map'
        if _EOF:
            return 'exit'
        if ch in ('q', 'quit', 'exit'):
            # ★ 只有导航主界面能退出终端；有活的先确认，取消则留在本界面
            if _confirm_leave_nav(console):
                return 'exit'
            continue
        if ch == '0':
            return 'switch_map'
        try:
            if ch == '1':
                console.do_save()
            elif ch == '2':
                console.do_list()
            elif ch == '3':
                console.do_delete()
            elif ch == '4':
                console.do_goto()
            elif ch == '5':
                console.do_goto_xy()
            elif ch == '6':
                console.do_manual()
            elif ch == '7':
                console.do_route()
            elif ch == '8':
                console.do_query()
            elif ch == '9':
                task_system(console)
            elif ch == 't':
                console.do_start_task()
            elif ch == 'r':
                console.do_resume()
            elif ch == 'u':
                console.do_cargo(True)
            elif ch == 'd':
                console.do_cargo(False)
            elif ch == 'x':
                console.do_stop()
            elif ch == 's':
                console.do_status()
            elif ch == 'n':
                console.do_stack_status()
            elif ch == 'c':
                run_check(console)
            elif ch == 'l':
                lines = _read_log(console, 'task_manager_node').strip().splitlines()[-15:]
                print(c('  ── 任务栈日志尾 ──', 'b'))
                for line in lines:
                    print(f'    {line}')
            elif ch in ('h', 'help', '?'):
                print(NAV_MENU)
            else:
                print(c('  未知命令，按 h 看帮助', 'y'))
        except KeyboardInterrupt:
            print(c('  （已中断本次操作，回到菜单）', 'y'))
        except Exception as exc:
            print(c(f'  [出错] 本次操作异常：{exc}', 'r'))
        # ★ 从"有自己的子界面"的功能里回来时，重印主菜单当路标。
        #   普通单步功能（存点/急停…）不重印，免得刷屏。
        if ch in _NAV_NESTED:
            _nav_home(console, _NAV_NESTED[ch])


# ═══════════════════════════ 六、任务系统（任务缓冲 + 依次执行） ═══════════════════════════
def _show_tasks(console):                  # use 打印任务缓冲
    ts = console.task_buffer
    if not ts:
        print(c('  （任务缓冲为空）', 'y')); return
    print(c(f'  ── 任务缓冲（共 {len(ts)} 条，总循环 '
            f'{sum(t["loops"] for t in ts)} 次）──', 'b'))
    for i, t in enumerate(ts, 1):
        rt = f' 经路线「{t["route"]}」' if t['route'] else ''
        print(f'    {i}. 完整搬运「{t["pick"]}」⇒「{t["drop"]}」 × {t["loops"]} 次{rt}')


def task_system(console):
    """任务系统菜单：编辑任务缓冲 → 启动任务依次执行。"""
    while True:
        print(TASK_MENU)
        _show_tasks(console)
        ch = _ask('请输入命令: ').lower()
        if _EOF:
            break
        try:
            if ch == '1':                              # 添加任务
                pick = _ask('  取货点名: ')
                drop = _ask('  放货点名: ')
                if not pick or not drop:
                    print(c('  已取消：取货点与放货点都要填', 'y')); continue
                try:
                    loops = int(_ask('  循环次数 [默认 1]: ', '1'))
                except ValueError:
                    loops = 1
                loops = max(1, loops)
                route = console.choose_route() if _yn('  是否使用路线？') else ''
                console.task_buffer.append({'pick': pick, 'drop': drop,
                                            'loops': loops, 'route': route})
                console.save_tasks()
                print(c(f'  已加入任务缓冲（第 {len(console.task_buffer)} 条）', 'g'))
            elif ch == '2':                            # 删除任务
                if not console.task_buffer:
                    print(c('  任务缓冲为空', 'y')); continue
                try:
                    i = int(_ask('  要删除第几条: ', '0'))
                except ValueError:
                    i = 0
                if 1 <= i <= len(console.task_buffer):
                    gone = console.task_buffer.pop(i - 1)
                    console.save_tasks()
                    print(c(f'  已删除：{gone["pick"]}⇒{gone["drop"]}', 'g'))
                else:
                    print(c('  序号不对', 'y'))
            elif ch == '3':                            # 列出
                _show_tasks(console)
            elif ch == '4':                            # 清空
                if _yn('  确认清空任务缓冲？'):
                    console.task_buffer = []
                    console.save_tasks()
                    print(c('  已清空', 'g'))
            elif ch == '5':                            # 启动任务
                if not console.task_buffer:
                    print(c('  任务缓冲为空，先添加任务', 'y')); continue
                run_task_buffer(console)
            elif ch in ('q', 'quit', 'exit', ''):       # 返回导航主界面（不退出终端）
                if console.task_buffer and not _yn('  返回会清空全部任务并停车，继续？'):
                    continue
                _stop_car(console)
                console.task_buffer = []
                console.save_tasks()
                print(c('  已返回导航主界面（任务已清空，小车已停）', 'g'))
                return
            elif ch in ('h', 'help', '?'):
                print(TASK_MENU)
            else:
                print(c('  未知命令，按 h 看帮助', 'y'))
        except KeyboardInterrupt:
            print(c('  （已中断本次操作）', 'y'))
        except Exception as exc:
            print(c(f'  [出错] {exc}', 'r'))


def _stop_car(console):                    # use 让车停下（取消任务 + 直接发零速）
    console.send_cmd('cancel_task')
    time.sleep(0.4)
    console.vel_pub.publish(Twist())
    print(c('  已取消当前任务并发出零速', 'g'))


# 取状态消息里的任务号（没有状态/字段为空时返回空串）
# 参数：st - TaskState 消息或 None
# 返回：str
def _task_id_of(st):                       # use 识别"这一单"
    return (getattr(st, 'task_id', '') or '') if st else ''


def _wait_task_done(console, start_timeout=15.0, hard_timeout=900.0,
                    prev_task_id=None, resend=None, resend_interval=1.0,
                    max_resend=8):
    """等一条搬运任务跑完。执行中可按键：s=状态快照 / l=任务缓冲 / q=退出。

    ★ 判"完成"必须认任务号，不能只看 state：
      /agv/task_state 上报的是"最新一条"状态，上一单的 DONE 会留在缓存里；
      只看 state 就会把上一单的完成当成这一单的完成（假 ✔，明明没跑）。

    ★ 判"起没起来"同样认任务号；迟迟见不到新任务号就用 resend() 重发一次。
      原因：任务层报完 DONE 还有约 0.5s 收尾（等车停稳才 DONE→IDLE），这段时间
      `_can_start()` 的 `_runner.is_alive()` 仍为真，紧跟着下发的 start_task 会被
      拒（"已有任务在执行"）。控制台只看得见 /agv/task_state、看不见拒绝原因，
      只能靠重发（换新 request_id，绕过去重）等它收尾结束。

    参数：console         - 控制台节点（读 console.state）
          start_timeout   - "迟迟没开始"的判据（s）
          hard_timeout    - 单次搬运的总兜底（s）
          prev_task_id    - ★ 下发**之前**的任务号（调用方在 send_cmd 前取好）。
                            只有任务号和它不同，才算"本单真的起来了"
          resend          - 可调用：重发本单 start_task（换新请求号）。None=不重发
          resend_interval - 两次重发之间的最小间隔（s）
          max_resend      - 最多重发几次
    返回：'done' / 'error' / 'quit' / 'timeout'
    """
    t0 = time.time()
    base_id = prev_task_id if prev_task_id is not None else _task_id_of(console.state)
    started = False                        # def 是否已确认本单真的起来了
    last = None                            # def 上次打印过的状态串
    last_resend = time.time()              # def 上次重发时刻
    tries = 0                              # def 已重发次数
    err_since = None                       # def 首次看到故障态的时刻
    while time.time() - t0 < hard_timeout:
        key = _poll_key(0.2)
        if key in ('q', 'quit'):
            print(c('  收到退出：取消当前任务', 'y'))
            _stop_car(console)
            return 'quit'
        if key in ('s', 'status'):
            console.do_status()
        if key in ('l', 'list'):
            _show_tasks(console)
        st = console.state
        state = (getattr(st, 'state', '') or '') if st else ''
        task_id = _task_id_of(st)
        if task_id and task_id != base_id:     # # 任务号变了 = 本单真的起来了
            started = True
        if state and state != last:        # # 状态一变就打印一行，便于盯进度
            last = state
            tgt = getattr(st, 'target_location', '') or '-'
            msg = getattr(st, 'message', '')
            print(c(f'    [任务] {state}  目标={tgt}  {msg}', 'b'))
        if started and state == 'ERROR':
            return 'error'
        if started and state in ('IDLE', 'DONE'):
            return 'done'
        if not started:
            now = time.time()
            # # 任务层卡在故障态：指令根本进不去，早点报出来交给上层 resume
            #   （加 1s 确认，避免把上一单残留的 ERROR 误判成本单失败）
            if state in ('ERROR', 'PAUSED'):
                if err_since is None:
                    err_since = now
                elif now - err_since > 1.0:
                    print(c(f'    [未受理] 任务层处于 {state}，指令进不去'
                            f'（先 resume 解除）', 'y'))
                    return 'error'
            else:
                err_since = None
            # # 迟迟没见到本单起来：重发（换新请求号，绕过去重），等它收尾结束
            if (resend is not None and tries < max_resend
                    and now - last_resend >= resend_interval):
                tries += 1
                last_resend = now
                print(c(f'    [重发] 还没见到任务起来（第 {tries} 次），重新下发', 'y'))
                resend()
            if now - t0 > start_timeout:
                return 'timeout'           # 迟迟没离开 IDLE：多半指令没被受理
    return 'timeout'


def run_task_buffer(console):
    """依次执行任务缓冲里的所有任务（每条按循环次数重复）。"""
    tasks = console.task_buffer
    total = sum(t['loops'] for t in tasks)
    print(c(f'\n  ═══ 启动任务：{len(tasks)} 条任务 / 共 {total} 次搬运 ═══', 'bold'))
    print(c('  执行中可输入：s=状态快照  l=任务缓冲  q=退出任务（会取消当前搬运）', 'y'))
    done = 0
    error = 0
    idx = 0
    for i, t in enumerate(tasks, 1):
        for k in range(t['loops']):
            idx += 1
            rt = f'（经路线「{t["route"]}」）' if t['route'] else ''
            print(c(f'\n  ── [{idx}/{total}] {t["pick"]} ⇒ {t["drop"]}{rt} '
                    f'第 {k + 1}/{t["loops"]} 次 ──', 'b'))
            # ★ 先记下"上一单的任务号"：send_cmd 之后 console.state 会立刻被刷新，
            #   再取就分不清状态里说的是上一单还是本单了。
            prev_id = _task_id_of(console.state)
            console.send_cmd('start_task', [t['pick'], t['drop']],
                             route=t['route'])
            # ★ 重发用新请求号（绕过去重）：任务层报完 DONE 还有约 0.5s 收尾，
            #   这段时间它的后台线程还活着，紧跟的 start_task 会被判"已有任务在执行"。
            outcome = _wait_task_done(
                console, prev_task_id=prev_id,
                resend=lambda: console.send_cmd('start_task',
                                                [t['pick'], t['drop']],
                                                route=t['route']))
            if outcome == 'done':
                done += 1
                print(c(f'    ✔ 第 {idx} 次搬运完成', 'g'))
            elif outcome == 'quit':
                print(c(f'    ⏹ 已退出任务（完成 {done} 次，剩余未执行）', 'y'))
                return
            elif outcome == 'error':
                error += 1
                print(c(f'    ✘ 第 {idx} 次搬运失败（任务状态 ERROR）', 'r'))
                if not _yn('  是否继续执行后面的任务？（建议先查日志/现场）'):
                    console.send_cmd('resume')      # ★ 复位，别把车留在 ERROR/急停
                    time.sleep(0.4)
                    print(c(f'    ⏹ 已停止（完成 {done} 次，失败 {error} 次）', 'y'))
                    return
                console.send_cmd('resume')          # ★ 清 ERROR 才能接下一条
                time.sleep(0.8)
            else:
                error += 1
                print(c('    ✘ 超时：任务迟迟没开始或没结束（看 [l] 日志尾）', 'r'))
                console.send_cmd('cancel_task')
                time.sleep(0.5)
    print(c(f'\n  ═══ 任务结束：成功 {done} 次，失败 {error} 次 ═══',
            'g' if error == 0 else 'y'))


# ═══════════════════════════ 七、入口 ═══════════════════════════
MODE_MENU = """
  ┌──────────── 选择启动模式 ────────────┐
  │ [1] 建图模式（仿真 + SLAM + 遥控建图）│
  │      可遥控、命名保存地图、管理地图   │
  │ [2] 导航模式（选地图 → 导航/搬运/任务）│
  │ [3] 只做自检（不起仿真）             │
  │ [q] 退出                             │
  └──────────────────────────────────────┘
"""


def main():
    ap = argparse.ArgumentParser(description='AGV 终端控制台（建图/导航双模式）')
    ap.add_argument('cmd', nargs='?', default='', choices=['', 'check', 'maps'],
                    help='留空=交互模式选择 / check=一键自检 / maps=列出地图库')
    ap.add_argument('--mode', choices=['mapping', 'nav'], default='',
                    help='跳过模式选择，直接进某模式')
    ap.add_argument('--no-gui', action='store_true', help='起仿真时不开 Gazebo 窗口')
    ap.add_argument('--no-rviz', action='store_true', help='起仿真时不开 RViz 窗口')
    args = ap.parse_args()

    print(BANNER)
    mm = _map_manager()
    gui = not args.no_gui
    rviz = not args.no_rviz

    # `maps` 子命令：只列地图库，不需 ROS
    if args.cmd == 'maps':
        _list_maps(mm)
        return

    rclpy.init()
    # ★ 启动即清理上次残留的仿真/导航/业务进程：
    #   多套栈的同名节点会互相抢话题/服务/action，表现为「节点没起 / 无响应 /
    #   查询超时 / 导航原地转圈」。控制台每次启动都从干净状态开始。
    print(c('  清理上次残留进程…', 'b'))
    _pkill_ros()
    time.sleep(1.0)
    exit_code = 0
    check_console = None
    try:
        if args.cmd == 'check':                # 自检：起业务节点（用默认点位文件）
            check_console = Console(manage_nodes=True)
            time.sleep(1.5)
            exit_code = 1 if run_check(check_console) else 0
        else:
            mode = args.mode or ''
            while True:
                if not mode:                       # 首次进来：先选模式
                    print(MODE_MENU)
                    ch = _ask('请选择: ').lower()
                    if _EOF or ch in ('q', 'quit', 'exit', ''):
                        break
                    if ch == '1':
                        mode = 'mapping'
                    elif ch == '2':
                        mode = 'nav'
                    elif ch == '3':
                        check_console = Console(manage_nodes=True); time.sleep(1.5)
                        exit_code = 1 if run_check(check_console) else 0
                        check_console.stop_nodes(); check_console.destroy_node()
                        check_console = None
                        break
                    else:
                        print(c('  未知选择', 'y')); continue
                # 跑一个模式；返回值决定下一步（换模式 / 结束）
                if mode == 'mapping':
                    nxt = mapping_mode(mm, gui=gui, rviz=rviz)
                    mode = 'nav' if nxt == 'nav' else ''
                else:                                  # mode == 'nav'
                    nxt = navigation_mode(mm, gui=gui, rviz=rviz)
                    mode = 'mapping' if nxt == 'mapping' else ''
                if not mode:                           # 该模式返回"退出"
                    break
    except KeyboardInterrupt:
        pass
    finally:
        print(c('\n  正在收尾…', 'b'))
        if check_console is not None:
            check_console.stop_nodes()
        _pkill_ros()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    print(c('  已退出。', 'g'))
    sys.exit(exit_code)


def _pkill_ros():
    """清掉可能残留的仿真/导航/业务进程（防多套栈打架）。

    ★ 必须覆盖整套栈的所有可执行文件：只杀 nav2_/gzserver 会漏掉
      robot_state_publisher、ekf 等，残留的同名节点照样抢话题/服务/action，
      表现就是「节点没起 / 无响应 / 查询超时 / 导航原地转」。
    ★ 遥控节点也一起收：它被 remap 到 /cmd_vel_raw，不从 /cmd_vel 走，
      残留下来会一直往底盘灌速度。
    """
    for pat in ('ros2 launch agv_',            # launch 父进程（子进程被杀后会自己收尾）
                'gzserver', 'gzclient', 'spawn_entity',
                'nav2_', 'slam_toolbox',
                'robot_state_publisher', 'robot_localization',
                'task_manager_node', 'cargo_node', 'waypoint_node',
                'velocity_arbiter',
                'agv_task', 'agv_lift', 'agv_waypoint', 'agv_hardware',
                'teleop_twist_keyboard',
                'rviz2'):
        subprocess.run(['pkill', '-9', '-f', pat], capture_output=True)


def _close_procs(console):
    """收掉某 console 登记的仿真/导航栈进程（模式切换/换地图时用）。"""
    procs = list(console.procs)
    for p in procs:
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                pass
    time.sleep(1.0)
    for p in procs:
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
    console.procs = []
    _pkill_ros()


if __name__ == '__main__':
    main()
