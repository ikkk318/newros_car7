#!/usr/bin/env python3
"""
gazebo_view.py —— 网页控制小车时，另开一个窗口"看" Gazebo 里的小车
================================================================================
【这个文件是干什么的】
  网页控制台起栈时是 **无头** 的（`stack_manager.build_launch_command(gui=False)`），
  也就是说 gzserver 跑着、物理/激光/里程计全在算，但 **没有 gzclient 窗口**，
  所以你在网页上点点点，屏幕上什么也看不到。

  这个脚本只做一件事：把那个"看不见的 Gazebo"变成"看得见的窗口"。
  新开一个终端执行它，就能看到小车在世界里的真实位置、朝向、周围障碍。

【两种模式（自动判断，也可手动指定）】
  ① 附着 attach（最常用，默认会走这条）
     已经有一个 gzserver 在跑 → 只起一个 gzclient 窗口**挂上去**。
     此模式 **不发任何 ROS 话题、不建任何东西、不碰任何服务**，
     纯粹的"多开一个显示器"，对正在跑的栈零影响。
     ★ 这正是"网页点启动 → 网页控制 → 另开终端看画面"的标准用法。

  ② 镜像 mirror（没有仿真时的兜底）
     一个 gzserver 都没在跑 → 自己起一个**独立**的 Gazebo（端口从 11355 起顺延），
     放一个**没有插件的影子车**，然后让影子车跟着现有 TF（map→base_footprint）
     走。影子车不发 /odom、不发 /scan、不发 /tf、不发 /clock，
     所以不会跟现有栈抢话题。
     用途：栈是纯业务模式（with_sim:=false）或将来接真车时，
     拿 Gazebo 当"三维显示板"来看车在哪儿。
     注意：镜像模式仍然**需要栈在跑**（要读 TF 才知道车在哪），
     没有栈的时候影子车只会停在原地。

【实测踩过的坑（都已在代码里处理，改之前先看这几条）】
  1. **虚拟机里 3D 视口全黑**：窗口能开、左侧面板和底部 FPS 都正常，唯独画面一片黑。
     根因是 VMware SVGA 的 3D 加速与 Gazebo 的 OGRE 渲染管线不兼容。
     解法：`LIBGL_ALWAYS_SOFTWARE=1` 走 Mesa 软件渲染（FPS 约 17，看车足够）。
     本脚本按 `/dev/dri` 是否存在自动判断，无需手动加。
  2. **打开窗口看不到车**：车不会待在原点 —— 实测跑完一轮搬运停在 (-8.69, -7.53)，
     而 gzclient 默认视角对着世界原点，于是只看到空地。解法：起窗口后
     `gz camera -c <相机> -f agv_500` 让镜头跟随（本脚本自动做，`--no-follow` 可关）。
  3. **不能直接 `ros2 launch agv_description sim.launch.py`**：
     那会再起一套差速插件 + 雷达 + IMU + EKF，与正在跑的栈抢 /odom /scan /tf，
     表现为"车同时被两个脑子指挥"、TF 抖动、地图跳变。所以镜像模式必须造
     **无插件**模型。
  4. **镜像模式的 gzserver 不加载 libgazebo_ros_init.so**（那是 /clock 的来源）。
     同 domain 里两个 /clock 发布者会让仿真时间乱跳，Nav2/TF 立刻报警。
  5. **镜像模式的 gzserver 一个插件都不加载**（world 副本里把 <plugin> 全删掉）。
     曾经想用 libgazebo_ros_factory.so + spawn_urdf_model 服务把影子车塞进去，
     实测在 Humble 上服务始终不出来（写进 world 报 incorrect plugin type，
     用 `-s` 加载虽能起来但 ROS 图里没有 spawn 服务），白等 30 秒。
     现在改用 Gazebo 自己的 transport：`gz model -m <名> -f <sdf>` 生成、
     `gz model -m <名> -x -y -z -Y` 挪位。好处是 **完全绕开 ROS**，
     镜像世界连 /clock 都不发，绝不可能跟正在跑的栈抢话题。
  6. **必须先 `gz sdf -p` 转换**：`gz model -f` 吃 SDF，直接喂 URDF 会报
     "No link elements found"（它只认 SDF 语法）。
  7. world 副本与 urdf 都写到 ~/.cache/agv_gazebo_view/，不往项目目录里拉屎。
  8. 探测"服务器在不在"不能用返回码：`gz topic -l` 在没服务器时也返回 0，
     只在输出里写 "An instance of Gazebo is not running."，必须看文本。

  9. **端口被上次残留的 gzserver 占着时不会报错**：新的 gzserver bind 失败也不吭声，
     表现成"等了 60 秒探不到世界"，看着像 world 加载失败。所以镜像模式启动前会
     先试绑端口，被占就自动顺延到 11356/11357…（见 pick_mirror_master）。

【打开窗口后看不到整车？】
  跟随模式会把镜头放在车身边上（几乎贴住车身），所以第一眼往往只看到一堵墙
  或者车的一角。**在 3D 视图里把滚轮往后滚 5~10 格**就能看到整车和周围环境；
  左键拖拽旋转、右键平移。想自己摆机位就加 `--no-follow`。

【视角被锁在车上，想取消跟随】
  跟随本质是"Orbit 视图控制器绑在 agv_500 这个 Visual 上"，不是普通的镜头动画，
  所以**鼠标拖拽解不开**——左键拖动只是在绕着车转。只有两条路：

  ① **彻底取消（推荐）**：关掉窗口，重开时加 `--no-follow`。这条 100% 有效，
     因为跟随命令只在启动时发一次，不加就永远不会锁。
  ② **不关窗口**：把"跟随目标"从车换成静态模型，镜头立刻不再跟着车跑 ——
     在另一个终端执行（GAZEBO_MASTER_URI 要对上，默认 http://127.0.0.1:11345）：
         gz camera -c gzclient_camera -f ground_plane
     换完之后镜头中心变成地面中心，左键/滚轮照常自由摆位。
     注意 `gz camera -f ""`（空名字）**靠不住**：Gazebo 11 是先按名字找 visual，
     找不到就静默放过（实测这次没有任何报错，也没有任何反应），
     别把希望押在空参数上。

【怎么用】
    python3 tools/gazebo_view.py                 # 自动：有仿真就开窗口（最常用）
    python3 tools/gazebo_view.py --mode attach   # 强制：只开窗口
    python3 tools/gazebo_view.py --mode mirror   # 强制：自己造一个看板
    python3 tools/gazebo_view.py --no-follow     # 不自动跟随，自己拖视角
    python3 tools/gazebo_view.py --hw-gl         # 有独立显卡时用硬件渲染
    python3 tools/gazebo_view.py --dry-run       # 只打印要做什么，不真跑
    python3 tools/gazebo_view.py --clean         # 清掉缓存目录

    关窗口 / Ctrl-C 即退出；脚本自己起的进程会一并收拾，不留后台残留。

【独立性承诺】
  本文件 **不被任何 CMakeLists / setup.py / launch 引用**，也不 import 项目内任何
  模块。删掉它（连同 ~/.cache/agv_gazebo_view）对整套系统没有任何影响。
"""

from __future__ import annotations

import argparse
import math
import os
import re
import shlex
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path

# ─────────────────────────── 路径与常量 ───────────────────────────
# 本文件放在 <项目根>/tools/ 下，用自身位置反推项目根，
# 这样无论从哪个目录执行都能找到 urdf/world（不依赖当前工作目录）。
ROOT = Path(__file__).resolve().parents[1]
DEFAULT_XACRO = ROOT / 'src' / 'agv_description' / 'urdf' / 'agv_500.urdf.xacro'
DEFAULT_WORLD = ROOT / 'src' / 'agv_description' / 'worlds' / 'world666.world'

# 缓存目录：只放"生成出来的 world 副本 / 影子 urdf"，删了会自动重建
CACHE_DIR = Path.home() / '.cache' / 'agv_gazebo_view'

# 附着模式连的是默认 Gazebo 主端口（与 gazebo_ros / stack_manager 起的完全一致）
ATTACH_MASTER = 'http://127.0.0.1:11345'
# 镜像模式故意换端口：即使默认端口上真有个"看不见"的服务器（容器/别的
# PID namespace 里跑的那种），也不会两边互相串台。
# 给一串候选是因为**上一次异常退出会留下占着 11355 的残废 gzserver**，
# 新的那个 bind 失败后不会报错，只会静静连不上（见 _port_free 的注释）。
MIRROR_PORT_CANDIDATES = (11355, 11356, 11357, 11358, 11359)

# ROS 环境（镜像模式要用 rclpy + tf2_ros 读 TF，得先 source）
ROS_SETUP = '/opt/ros/humble/setup.bash'

# 影子模型里的实体名。用 agv_500 是为了在 Gazebo 左侧列表里一眼认出是哪台车。
DEFAULT_ENTITY = 'agv_500'

# 影子车"动没动"的判据。小于这两个量就不发命令：
#   5cm / 3° —— 比肉眼能分辨的位移小，又足够滤掉 TF 的微小抖动
#   （TF 是浮点的，车停着也会在 1mm 级来回跳，不设死区会一直发命令）。
MOVE_EPS = 0.05
YAW_EPS = 0.05


# ─────────────────────────── 输出小工具 ───────────────────────────
def say(msg, level='info'):
    """统一的中文进度输出。

    为什么不用 logging：这是个"看一眼就关"的小工具，用户要的是终端里一眼能读懂的
    几行字，不是带时间戳/级别的日志流。
    """
    icon = {'info': '·', 'ok': '✓', 'warn': '!', 'err': '✗', 'step': '→'}.get(level, '·')
    stream = sys.stderr if level in ('warn', 'err') else sys.stdout
    print(f'{icon} {msg}', flush=True, file=stream)


def die(msg):
    """致命错误：说清楚"为什么错、怎么办"，然后退出码 1。

    这类工具失败时最怕只报一句 "failed"，用户根本不知道该改什么。
    """
    say(msg, 'err')
    sys.exit(1)


def _port_free(port):
    """试绑一下这个端口，能绑上就是空闲的。

    ★ 为什么要试绑：上一次异常退出（kill -9，或 Ctrl-C 正好落在 fork 之后）
      会在 11355 上留一个"半死"的 gzserver。新的 gzserver bind 失败时**不报错**，
      只是安静地连不上 master —— 表现出来是"等了 30 秒探不到世界"，
      看着像 world 加载失败，实际是端口被占，极难查。
      试绑一次就能提前躲开这个坑。

    用 0.0.0.0 试而不是 127.0.0.1：gzserver 绑的是通配地址，
    只试回环地址会漏判（绑 0.0.0.0 的端口，绑 127.0.0.1 通常也会 EADDRINUSE，
    但别再依赖这种细节，直接用同一个地址族最稳）。
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind(('', int(port)))
        return True
    except OSError:
        return False
    finally:
        sock.close()


def pick_mirror_master():
    """挑第一个空闲端口当镜像世界的 master 地址。

    ★ 只挑一次（在 import 时算好），写进 MIRROR_MASTER 常量：
      镜像流程里它被引用十来次，做成函数调用会把签名搞得很脏，
      而这个值在整个进程生命周期内都不该变。
    """
    for port in MIRROR_PORT_CANDIDATES:
        if _port_free(port):
            return f'http://127.0.0.1:{port}'
    die('镜像模式找不到空闲端口（试过 ' +
        '、'.join(str(p) for p in MIRROR_PORT_CANDIDATES) + '）。\n'
        '  多半是之前异常退出留下的 gzserver 还占着：pkill -f gzserver 之后再试。')


MIRROR_MASTER = pick_mirror_master()


# ─────────────────────── Gazebo 存活探测 ───────────────────────
def probe_gazebo(master_uri=ATTACH_MASTER, timeout=6.0):
    """问 Gazebo master 要话题列表，判断这个地址上有没有活的服务器。

    返回：list[str] 话题名（服务器在）；[] 服务器不在；None 探测失败（没装 gz 等）。

    ★ 关键坑：`gz topic -l` 在"没有服务器"时 **返回码仍然是 0**，
      只在 stdout/stderr 打 "An instance of Gazebo is not running."
      所以绝对不能靠 returncode 判断，只能看输出文本。
    """
    env = os.environ.copy()
    env['GAZEBO_MASTER_URI'] = master_uri
    try:
        r = subprocess.run(['gz', 'topic', '-l'],
                           capture_output=True, text=True, timeout=timeout, env=env)
    except FileNotFoundError:
        return None                      # 没装 gazebo 工具集
    except subprocess.TimeoutExpired:
        return []                        # 超时按"不在"处理，别让用户干等
    blob = ((r.stdout or '') + (r.stderr or '')).lower()
    if 'not running' in blob:
        return []
    topics = [ln.strip() for ln in (r.stdout or '').splitlines() if ln.strip().startswith('/')]
    return topics


def looks_like_our_sim(topics):
    """话题列表里有没有本项目的模型（用来判断"这是不是我们那套仿真"）。

    仅仅是 /gazebo/default/... 之类的话题还不够 —— 那可能是别的 Gazebo 工程。
    本项目的实体固定叫 agv_500，出现在话题路径里就是铁证。
    """
    return any('agv_500' in t for t in topics or [])


# ─────────────────────── 开窗口（附着模式） ───────────────────────
def gz_env(master_uri):
    """所有 gz* 命令共用的环境变量小工厂。

    集中一处的原因：GAZEBO_MASTER_URI 漏设过一次就会连到默认 11345 上，
    表现是"窗口开了，但看到的不是你要看的那个仿真"，非常难查。
    """
    env = os.environ.copy()
    env['GAZEBO_MASTER_URI'] = master_uri
    return env


def needs_software_gl():
    """判断是否必须强制软件渲染（Mesa llvmpipe）。

    ★ 这是本项目实测出来的坑（VMware 虚拟机）：
      虚拟机的 3D 加速是 VMware SVGA，与 Gazebo 用的 OGRE 渲染管线不兼容，
      窗口能开、FPS 也正常，但 **3D 视口一片纯黑**（左侧面板和底部状态栏都正常，
      所以看着像"世界里什么都没有"，其实是根本没画出来）。
      加 LIBGL_ALWAYS_SOFTWARE=1 走 Mesa 软件渲染后立刻正常（FPS 约 17，够看）。

    判据用 /dev/dri 是否存在，而不是猜显卡型号：
      没有 /dev/dri 说明系统根本没有可用的 DRM 设备节点（虚拟机/无 GPU 直通），
      此时走硬件 GL 只会失败或黑屏，强制软件渲染没有任何损失。
    """
    return not Path('/dev/dri').exists()


def launch_gui(master_uri=ATTACH_MASTER, dry=False, software_gl=None):
    """起一个 gzclient 挂到指定 master 上，返回 Popen（dry 时返回 None）。

    为什么是 gzclient 而不是 gazebo：`gazebo` 会在连不上服务器时**自己起一个**
    gzserver，等于凭空多出一套物理世界；`gzclient` 只做客户端，连不上就退出，
    行为可控。

    software_gl：None=自动判断，True=强制软件渲染，False=强制硬件。
    """
    binary = shutil.which('gzclient') or shutil.which('gazebo')
    if not binary:
        die('找不到 gzclient：本机似乎没装 Gazebo。\n'
            '  安装：sudo apt install gazebo11 libgazebo11-dev')
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        # 没有显示环境时 gzclient 会直接崩在 Qt 初始化，报错很难懂，先拦住
        die('没有图形显示环境（DISPLAY / WAYLAND_DISPLAY 都为空）。\n'
            '  请在虚拟机桌面里的终端执行，不要用 ssh 无 X 转发的方式。')

    env = gz_env(master_uri)
    soft = needs_software_gl() if software_gl is None else software_gl
    if soft:
        env['LIBGL_ALWAYS_SOFTWARE'] = '1'
    if dry:
        tag = 'LIBGL_ALWAYS_SOFTWARE=1 ' if soft else ''
        say(f'[dry-run] 将执行：{tag}GAZEBO_MASTER_URI={master_uri} {binary}', 'step')
        return None
    return subprocess.Popen([binary], env=env)


def follow_model(master_uri, model='agv_500', dry=False, first_wait=6.0):
    """让 GUI 相机跟随模型移动。

    ★ 为什么必须有这一步：小车不会乖乖待在原点。实测网页跑完一轮搬运后，
      车停在 (-8.69, -7.53)，而 gzclient 默认视角对着世界原点 ——
      打开窗口只会看到一片空地，用户第一反应是"没加载出来"。
      设成跟随以后，无论车开到哪儿，镜头都盯着它。

    ★ 为什么要先等几秒：gzclient 从进程拉起到 GUI 真正连上 Gazebo master
      需要时间（软件渲染的机器上要 5 秒以上）。在它连上之前发 camera 命令
      是发不进去的（gz 会报 not running）。所以先等再发，并补发一次兜底。

    相机名不写死：`gz camera -l` 报的是 `gzclient_camera`，
    但不同版本的 gzclient 可能改名，先查再跟随，查不到就用默认名。
    """
    if dry:
        say(f'[dry-run] 将执行：gz camera -l，然后 gz camera -c <相机> -f {model}', 'step')
        return False

    env = gz_env(master_uri)
    say(f'等待 Gazebo 窗口初始化（约 {first_wait:.0f}s）…', 'step')
    time.sleep(first_wait)

    # 查相机名
    cam = 'gzclient_camera'
    try:
        r = subprocess.run(['gz', 'camera', '-l'], capture_output=True,
                           text=True, timeout=5, env=env)
        for ln in (r.stdout or '').splitlines():
            name = ln.strip()
            # 只认"干净的一行"，把 "An instance of Gazebo is not running." 之类过滤掉
            if name and ' ' not in name and name != 'not':
                cam = name
                break
    except Exception:
        pass

    # 发两次：第一次可能撞上 GUI 尚未订阅完，第二次兜底
    for _ in range(2):
        try:
            subprocess.run(['gz', 'camera', '-c', cam, '-f', model],
                           capture_output=True, text=True, timeout=5, env=env)
        except Exception:
            pass
        time.sleep(2.0)

    # 跟随的默认距离比较近（镜头几乎贴着车身，能看到激光扫过的地面和周围障碍，
    # 但车体本身可能在画面外）。这不是 bug —— 滚轮向后拉就能看到整车，
    # 所以提示里把这句话写清楚，免得以为没显示出来。
    say(f'镜头已跟随 {model} —— 若只看到一面墙/车的一角，'
        f'在 3D 视图里把滚轮往后滚 5~10 格即可看到整车', 'ok')
    return True




def wait_gui(proc):
    """守着窗口进程：用户关掉窗口 / Ctrl-C 就返回，让上层统一收尾。

    不能用 proc.wait() 裸等：那样 Ctrl-C 只会打断 Python，子进程变孤儿窗口。
    所以自旋 0.2s 查一次存活，把 KeyboardInterrupt 交给上层处理。
    """
    if proc is None:
        return
    try:
        while proc.poll() is None:
            time.sleep(0.2)
        say('窗口已关闭，退出。', 'ok')
    except KeyboardInterrupt:
        say('收到 Ctrl-C，关闭窗口…')
        stop_proc(proc)


def stop_proc(proc, grace=3.0):
    """温和地收拾子进程：先 terminate，等一会儿，再 kill。

    Gazebo 家族对 SIGTERM 响应良好；但 gzserver 有时因为物理线程卡住不退，
    所以给 3 秒宽限后补一刀，避免"终端退了但 CPU 还被吃掉"。
    """
    if proc is None or proc.poll() is not None:
        return
    try:
        proc.terminate()
        proc.wait(timeout=grace)
    except subprocess.TimeoutExpired:
        try:
            proc.kill()
            proc.wait(timeout=2.0)
        except Exception:
            pass
    except Exception:
        pass


# ─────────────────────── 镜像模式：造影子模型 ───────────────────────
def build_shadow_urdf(xacro_path=DEFAULT_XACRO):
    """把项目 URDF 展开成"纯显示版"：去掉所有插件和传感器。

    为什么要去插件：
      libgazebo_ros_diff_drive 会发 /odom + /tf，
      libgazebo_ros_ray_sensor 会发 /scan，
      libgazebo_ros_imu_sensor 会发 /imu，
      libgazebo_ros_joint_state_publisher 会发 /joint_states。
      这些和正在跑的栈撞车 → 两个 /odom 抢 TF、两条 /scan 让建图抽风。
      影子车只需要"长得像"，不需要"会动"（它动是由 TF 拖着走的）。

    另外关掉重力：影子车不受重力就不会往下掉，位姿每帧由外部覆盖，
    否则物理步进会把它压向地面，视觉上一直"抖"。
    """
    try:
        import xacro                    # xacro 是 ROS 的 python 包，source 后可用
    except Exception as exc:
        die(f'导入 xacro 失败（{exc}）。\n'
            f'  请先 source ROS 环境：source {ROS_SETUP}')

    if not Path(xacro_path).is_file():
        die(f'找不到 URDF：{xacro_path}')

    doc = xacro.process_file(str(xacro_path))
    raw = doc.toxml()

    import xml.etree.ElementTree as ET
    root = ET.fromstring(raw)

    removed = {'plugin': 0, 'sensor': 0}
    # 顶层 <gazebo> 里挂着插件；<gazebo reference="..."> 里挂着 sensor
    for gz in root.findall('gazebo'):
        for child in list(gz):
            tag = child.tag.split('}')[-1]
            if tag in removed:
                removed[tag] += 1
                gz.remove(child)

    # 关重力（见上面注释）
    body = ET.SubElement(root, 'gazebo')
    body.set('reference', 'base_link')
    ET.SubElement(body, 'gravity').text = 'false'

    out = ET.tostring(root, encoding='unicode')
    # ElementTree 有时给未知命名空间加 ns0:/xmlns:ns0 前缀，Gazebo 的 URDF 解析器不认
    out = re.sub(r'\sxmlns:ns\d+="[^"]*"', '', out)
    out = re.sub(r'<(/?)(?:ns\d+:)', r'<\1', out)

    say(f'已生成影子模型：移除 {removed["plugin"]} 个插件 / {removed["sensor"]} 个传感器', 'ok')
    return out


def build_shadow_sdf(xacro_path=DEFAULT_XACRO, out_path=None):
    """把影子模型转成 SDF 文本，供 `gz model -f` 生成实体。

    ★ 为什么要转：`gz model -f` 只认 SDF。直接喂 URDF 会报
      "No link elements found in urdf file"（它会先当 SDF 解析一遍就放弃了）。
      项目里的 `gz sdf -p <urdf>` 正好就是干这个的，比自己拼 XSLT 可靠。

    转换失败不静默吞掉：把 gazebo 的 stderr 一起带出来，否则用户只看到
    "生成失败"四个字，根本不知道是 URDF 写错还是环境没 source。
    """
    out_path = Path(out_path) if out_path else (CACHE_DIR / 'shadow.sdf')
    out_path.parent.mkdir(parents=True, exist_ok=True)

    urdf_text = build_shadow_urdf(xacro_path)
    urdf_tmp = CACHE_DIR / 'shadow.urdf'
    urdf_tmp.write_text(urdf_text, encoding='utf-8')

    try:
        r = subprocess.run(['gz', 'sdf', '-p', str(urdf_tmp)],
                           capture_output=True, text=True, timeout=30)
    except FileNotFoundError:
        die('找不到 `gz` 命令：本机似乎没装 Gazebo。\n'
            '  安装：sudo apt install gazebo11 libgazebo11-dev')
    except subprocess.TimeoutExpired:
        die('`gz sdf -p` 超时（30s）。模型可能过于复杂，请检查 URDF。')

    sdf = r.stdout or ''
    # 判据看内容而不是返回码：gz sdf 转换失败时 returncode 仍然是 0
    if '<sdf' not in sdf or '<model' not in sdf:
        die('URDF 转 SDF 失败：\n' + ((r.stderr or r.stdout or '(无输出)').strip()[:400]))

    out_path.write_text(sdf, encoding='utf-8')
    return out_path


def build_shadow_world(world_path=DEFAULT_WORLD, out_path=None):
    """复制一份 world，并**删掉里面所有插件**。

    镜像模式的 gzserver 是"纯显示器"，必须一个 ROS 插件都不加载：
      · libgazebo_ros_init.so    → 会发 /clock（同 domain 两个 clock 发布者
                                  会让仿真时间乱跳，Nav2/TF 立刻报警）
      · libgazebo_ros_api_plugin → 会注册一堆 /gazebo/* 服务与话题
    删干净以后，这个世界只剩物理引擎和图形，对正在跑的栈零影响。

    ★ 顺带记一笔：曾试图用 libgazebo_ros_factory.so 的
      /gazebo/spawn_urdf_model 服务往里塞车，Humble 上这个服务压根不出现
      （写进 world 被拒、用 `-s` 加载也不注册服务），白白等 30 秒。
      现在改成 `gz model -f`（见 run_mirror），不依赖任何 ROS 插件。
    """
    src = Path(world_path)
    if not src.is_file():
        die(f'找不到 world 文件：{src}')

    import xml.etree.ElementTree as ET
    tree = ET.parse(str(src))
    root = tree.getroot()
    world = root.find('world')
    if world is None:
        die(f'world 文件格式异常（没有 <world> 节点）：{src}')

    dropped = []
    for plug in list(world.findall('plugin')):
        dropped.append(plug.get('name') or plug.get('filename') or '?')
        world.remove(plug)
    if dropped:
        say(f'镜像世界已剔除插件：{", ".join(dropped)}', 'info')

    out_path = Path(out_path) if out_path else (CACHE_DIR / 'shadow.world')
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(out_path), encoding='utf-8', xml_declaration=True)
    return out_path


# ─────────────────────── ROS 环境自举 ───────────────────────
def ensure_ros_env():
    """确保 rclpy 可导入；不行就 source 后重新执行自己。

    为什么需要：用户十有八九是直接 `python3 tools/gazebo_view.py` 就跑了，
    终端里并没有 source 过 setup.bash。附着模式不需要 ROS（纯 gz 命令），
    但镜像模式要 rclpy + tf2_ros（读 TF，不碰 gazebo_msgs）。
    这里用 _AGV_VIEW_REEXEC 环境变量打标记，防止无限自我重启。
    """
    try:
        import rclpy  # noqa: F401
        return
    except Exception:
        pass
    if os.environ.get('_AGV_VIEW_REEXEC') == '1':
        die('已 source 过环境但仍无法导入 rclpy，可能 ROS 安装不完整。')
    if not Path(ROS_SETUP).is_file():
        die(f'找不到 {ROS_SETUP}，无法进入镜像模式。')

    argv = ' '.join(shlex.quote(a) for a in [sys.executable] + sys.argv)
    cmd = (f'source {shlex.quote(ROS_SETUP)} >/dev/null 2>&1 && '
           f'source {shlex.quote(str(ROOT / "install" / "setup.bash"))} >/dev/null 2>&1; '
           f'exec {argv}')
    env = os.environ.copy()
    env['_AGV_VIEW_REEXEC'] = '1'
    say('首次运行：正在 source ROS 环境后重入本脚本…', 'step')
    os.execve('/bin/bash', ['bash', '-c', cmd], env)


# ─────────────────────── 镜像模式：Gazebo transport 小工具 ───────────────────────
# 影子车的一切操作都走 `gz model`（Gazebo 自己的 transport），不碰 ROS 图：
#   生成  gz model -m <名> -f <sdf 文件>
#   挪位  gz model -m <名> -x .. -y .. -z .. -Y ..
#   读位姿 gz model -m <名> -p  →  "x y z roll pitch yaw"
def run_gz(argv, master_uri, timeout=8.0):
    """执行一条 `gz ...` 命令，统一返回 (returncode, stdout, stderr)。

    ★ 为什么不看 returncode 判成功：`gz` 这一系列命令在"目标不存在"时
      往往仍然返回 0，只在文本里写 "Unable to get info on model[...]"。
      调用方必须结合输出文本判断（见 gz_model_pose）。
    """
    try:
        r = subprocess.run(['gz'] + argv, capture_output=True, text=True,
                           timeout=timeout, env=gz_env(master_uri))
        return r.returncode, (r.stdout or ''), (r.stderr or '')
    except FileNotFoundError:
        return 127, '', '找不到 gz 命令（Gazebo 没装？）'
    except subprocess.TimeoutExpired:
        return 124, '', f'gz {" ".join(argv)} 超时'


def gz_model_pose(entity, master_uri):
    """读模型位姿，返回 (x, y, z, roll, pitch, yaw) 的元组；模型不存在返回 None。

    用途有二：① 确认 spawn 真的成功了（生成是异步的，命令返回 ≠ 实体已在世界）
    ② 调试时能直接用脚本里的同一个函数核对。
    """
    rc, out, err = run_gz(['model', '-m', entity, '-p'], master_uri)
    text = (out or '').strip()
    if rc != 0 or not text or 'Unable' in text:
        return None
    parts = text.split()
    if len(parts) < 6:
        return None
    try:
        return tuple(float(v) for v in parts[:6])
    except ValueError:
        return None


def angle_diff(a, b):
    """两个偏角的最小差值（弧度，范围 -pi..pi）。

    直接相减会在 ±pi 处得到 2pi 的假差值，导致"车在原地转了一圈"
    被当成"没动"（或反过来每帧都判成动了）。
    """
    d = (float(a) - float(b) + math.pi) % (2.0 * math.pi) - math.pi
    return d


def run_mirror(args):
    """没有现成仿真时，自建一块"三维显示板"。

    链路：gzserver（独立端口 11355，**不带任何插件**）
          → `gz model -f` 生成影子车 → gzclient 窗口
          → rclpy 节点只读 TF（map→base_footprint），用 `gz model -x -y -Y` 挪影子车

    ★ 为什么一个 ROS 插件都不加载：插件一加载就会发 /clock、注册 /gazebo/*
      服务和话题，而同 ROS domain 里可能正跑着用户那套栈 —— 两个 clock 发布者
      会让仿真时间乱跳，Nav2 直接报错。现在镜像世界对 ROS 图完全隐形，
      只借 TF（只读）来确定车在哪儿，怎么算都污染不到别人。
    """
    ensure_ros_env()

    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    world_out = build_shadow_world(args.world)
    sdf_out = build_shadow_sdf(args.xacro)
    say(f'影子世界：{world_out}', 'ok')
    say(f'影子模型：{sdf_out}', 'ok')

    # ── 1. 起 gzserver ──
    #   --verbose 保留：万一下面哪步失败，用户能直接从终端看到 Gazebo 的报错，
    #   否则只能看到"我这条脚本说失败了"，查不下去。
    gzserver = None
    if args.dry_run:
        say(f'[dry-run] 将执行：GAZEBO_MASTER_URI={MIRROR_MASTER} gzserver {world_out}', 'step')
        say(f'[dry-run] 将执行：gz model -m {args.entity} -f {sdf_out}', 'step')
    else:
        binary = shutil.which('gzserver')
        if not binary:
            die('找不到 gzserver：本机似乎没装 Gazebo。')
        gzserver = subprocess.Popen([binary, '--verbose', str(world_out)],
                                    env=gz_env(MIRROR_MASTER))
        say(f'已启动独立 gzserver（{MIRROR_MASTER}），等待世界就绪…', 'step')

        # ── 2. 等世界真的起来（判据用"有没有话题"）──
        #   不用固定 sleep：虚拟机里 world 加载时间波动很大（几秒到十几秒）。
        deadline = time.time() + args.wait
        while True:
            # timeout 给 6s：`gz topic -l` 正常 1~2s，但虚拟机 CPU 满载时能拖到 3s 以上，
            # 给太紧会被误判成"服务器不在"，然后白等到超时（踩过一次）。
            if probe_gazebo(MIRROR_MASTER, timeout=6.0):
                break
            if gzserver.poll() is not None:
                die('gzserver 启动后立刻退出了，请检查上面的报错。')
            if time.time() > deadline:
                stop_proc(gzserver)
                die(f'等了 {args.wait}s，{MIRROR_MASTER} 上仍探不到世界。\n'
                    '  多半是 world 加载失败，请看上面 gzserver 的输出。')
            time.sleep(0.4)
        say('镜像世界就绪', 'ok')

        # ── 3. 生成影子车 ──
        rc, out, err = run_gz(['model', '-m', args.entity, '-f', str(sdf_out)], MIRROR_MASTER,
                              timeout=20.0)
        if rc != 0:
            stop_proc(gzserver)
            die('生成影子车失败：\n' + ((err or out or '(无输出)').strip()[:400]))
        # 生成是异步的：命令返回 ≠ 实体已经能查到位姿。等它出现再往下走，
        # 否则镜头跟随会跟一个"还不存在"的模型，窗口里空空如也。
        for _ in range(25):
            if gz_model_pose(args.entity, MIRROR_MASTER):
                break
            time.sleep(0.3)
        else:
            stop_proc(gzserver)
            die('影子车生成命令返回成功，但在世界里查不到它。\n'
                f'  可用 GAZEBO_MASTER_URI={MIRROR_MASTER} gz model -m {args.entity} -p 自查。')
        say(f'影子车已放进世界：{args.entity}', 'ok')

    # ── 4. 开窗口 ──
    gui = None if args.no_gui else launch_gui(MIRROR_MASTER, dry=args.dry_run,
                                              software_gl=args.software_gl)
    if not args.no_gui and not args.no_follow and args.follow_model:
        # 影子车也要跟随：否则用户看到的是一台停在原点的车，跟网页上的位置对不上。
        # 这里跟的是 args.entity（自家生成的那个实体），不是 --follow-model。
        follow_model(MIRROR_MASTER, args.entity, dry=args.dry_run)

    if args.dry_run:
        # 干跑到这里就够：world/SDF/命令都验过了，没必要真进跟随循环
        say('[dry-run] 镜像流程检查完毕，不真起进程、不进跟随循环。', 'ok')
        return

    rclpy.init(args=None)
    node = Node('agv_gazebo_view')

    # ── 5. 读 TF，把影子车挪到对应位置 ──
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    # miss = 查不到 TF；err = 挪动影子车失败。分开统计，
    # 否则收尾那句"查不到 TF N 次"会把挪车失败也算进去，误导排查方向。
    stats = {'ok': 0, 'miss': 0, 'err': 0, 'source': ''}
    last_note = [0.0]
    # 上一次真正下发过的位姿。用列表包一层，闭包里改它不需要 nonlocal 声明变量。
    last = {'x': None, 'y': None, 'yaw': None}

    def tick():
        """一次同步：查 TF → 算 yaw → 位姿变了才发命令。

        优先级 map→base_footprint 高于 odom→base_footprint：
          map 系才是"地图上的真实位置"；odom 系会随着里程计累积漂移。
          含 map 的那条查不到（纯 odom 系统）才退回 odom。
        """
        tf = None
        for parent in ('map', 'odom'):
            try:
                # Time() 表示"最新可用"，不要传时间戳：
                # 传 now 会因为 TF 比现在晚几毫秒而报 extrapolation into the future
                tf = tf_buffer.lookup_transform(parent, 'base_footprint', Time())
                stats['source'] = parent
                break
            except Exception:
                continue
        if tf is None:
            stats['miss'] += 1
            # 10 秒只提示一次，避免刷屏（TF 刚起步时查不到是正常的）
            if time.time() - last_note[0] > 10.0:
                last_note[0] = time.time()
                say('暂时查不到 base_footprint 的 TF —— 影子车先停在原地（等你起栈）', 'warn')
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        # 四元数 → 偏航角。gz model 只接受 roll/pitch/yaw，不认四元数，
        # 而车在地面上跑，roll/pitch 恒为 0，只取 yaw 就足够真实。
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # ★ 只有位姿真的变了才发命令：`gz model` 每次都是一次进程启动，
        #   车停着还按 5Hz 无脑发，等于白烧 CPU（虚拟机里尤其明显）。
        if (last['x'] is not None
                and abs(t.x - last['x']) < MOVE_EPS
                and abs(t.y - last['y']) < MOVE_EPS
                and abs(angle_diff(yaw, last['yaw'])) < YAW_EPS):
            return

        rc, out, err = run_gz(
            ['model', '-m', args.entity,
             '-x', f'{t.x:.4f}', '-y', f'{t.y:.4f}',
             # z 抬 2cm：完全贴地会和地面 z-fighting，车底出现闪烁条纹
             '-z', f'{t.z + 0.02:.4f}', '-Y', f'{yaw:.4f}'],
            MIRROR_MASTER, timeout=5.0)
        if rc != 0:
            stats['err'] += 1
            if time.time() - last_note[0] > 10.0:
                last_note[0] = time.time()
                say(f'挪动影子车失败：{(err or out).strip()[:150]}', 'warn')
            return
        last['x'], last['y'], last['yaw'] = t.x, t.y, yaw
        stats['ok'] += 1

    period = 1.0 / max(1.0, float(args.rate))
    say(f'开始跟随（{args.rate:.0f} Hz）—— 网页上操控，这里就能看到车动。Ctrl-C 退出。', 'ok')

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=period)
            tick()
            if gui is not None and gui.poll() is not None:
                say('窗口已关闭，退出。', 'ok')
                break
            if gzserver is not None and gzserver.poll() is not None:
                say('gzserver 已退出，收尾。', 'warn')
                break
    except KeyboardInterrupt:
        say('收到 Ctrl-C，正在收尾…')
    finally:
        stop_proc(gui)
        stop_proc(gzserver)
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        if stats['ok'] or stats['miss'] or stats['err']:
            say(f'同步统计：成功 {stats["ok"]} 帧，查不到 TF {stats["miss"]} 次，'
                f'挪车失败 {stats["err"]} 次，位姿来源 {stats["source"] or "无"}', 'info')


# ─────────────────────── 主流程 ───────────────────────
def clean_cache():
    """删掉缓存目录（生成物），不影响项目。"""
    if CACHE_DIR.exists():
        shutil.rmtree(CACHE_DIR, ignore_errors=True)
        say(f'已清理缓存：{CACHE_DIR}', 'ok')
    else:
        say('缓存目录本来就是空的。', 'info')


def run_attach(args):
    """附着模式：只开一个窗口挂到已经在跑的 gzserver 上。

    这一步**不碰仿真**：不起 server、不 spawn、不发话题、不调服务，
    所以网页那边正在跑的任务一点都不会受影响。纯纯的多开一个显示器。
    """
    proc = launch_gui(ATTACH_MASTER, dry=args.dry_run, software_gl=args.software_gl)
    if not args.no_follow and args.follow_model:
        follow_model(ATTACH_MASTER, args.follow_model, dry=args.dry_run)
    wait_gui(proc)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog='gazebo_view.py',
        description='网页控制小车时，另开窗口看 Gazebo 里的真实情况（独立小工具，可随时删除）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            '示例：\n'
            '  python3 tools/gazebo_view.py                 # 自动：有仿真就开窗口\n'
            '  python3 tools/gazebo_view.py --mode attach   # 只开窗口，不建任何东西\n'
            '  python3 tools/gazebo_view.py --mode mirror   # 没仿真时自建三维看板\n'
            '  python3 tools/gazebo_view.py --dry-run       # 只打印动作，不真跑\n'
            '  python3 tools/gazebo_view.py --clean         # 清缓存\n'
        ))
    ap.add_argument('--mode', choices=('auto', 'attach', 'mirror'), default='auto',
                    help='auto=有仿真就附着、没有就自建镜像（默认）')
    ap.add_argument('--world', default=str(DEFAULT_WORLD), help='镜像模式用的 world（默认 world666）')
    ap.add_argument('--xacro', default=str(DEFAULT_XACRO), help='镜像模式用的机器人描述')
    ap.add_argument('--entity', default=DEFAULT_ENTITY, help='镜像模式里的实体名')
    ap.add_argument('--follow-model', default=DEFAULT_ENTITY,
                    help='让镜头跟随这个模型（默认 agv_500；给空串则不跟随）')
    ap.add_argument('--no-follow', action='store_true', help='不开自动跟随（自己拖视角）')
    ap.add_argument('--rate', type=float, default=5.0,
                    help='镜像模式同步频率 Hz（默认 5；车真动起来时才发命令，静止不耗 CPU）')
    ap.add_argument('--wait', type=float, default=60.0,
                    help='等待镜像世界/服务就绪的秒数（默认 60；虚拟机里装载 world 可能十几秒）')
    ap.add_argument('--no-gui', action='store_true', help='只起服务器不开窗口（调试用）')

    gl = ap.add_mutually_exclusive_group()
    gl.add_argument('--software-gl', dest='software_gl', action='store_true', default=None,
                    help='强制软件渲染（虚拟机 3D 视口全黑时用；默认自动判断）')
    gl.add_argument('--hw-gl', dest='software_gl', action='store_false',
                    help='强制硬件渲染（有独显/GPU 直通时用）')

    ap.add_argument('--dry-run', action='store_true', help='只打印将执行的动作')
    ap.add_argument('--clean', action='store_true', help='清理缓存目录后退出')
    args = ap.parse_args(argv)

    if args.clean:
        clean_cache()
        return 0

    say('探测 Gazebo 服务端…', 'step')
    topics = probe_gazebo(ATTACH_MASTER)

    if topics is None:
        die('找不到 gz 命令行工具：本机似乎没装 Gazebo。\n'
            '  安装：sudo apt install gazebo11 libgazebo11-dev')

    # ── 决定走哪条路 ──
    if args.mode == 'attach':
        if not topics:
            die(f'{ATTACH_MASTER} 上没有 Gazebo 在跑，附着模式无从谈起。\n'
                '  想先看个大概：python3 tools/gazebo_view.py --mode mirror')
        say(f'发现 Gazebo（{len(topics)} 个话题），只开窗口，不动任何东西。', 'ok')
        if looks_like_our_sim(topics):
            say('已确认是本项目仿真（话题里有 agv_500）', 'ok')
        else:
            say('注意：话题里没看到 agv_500，连接的可能是别的 Gazebo 工程。', 'warn')
        return run_attach(args)

    if args.mode == 'auto':
        if topics:
            # 有仿真 → 附着（这条是网页控制时的常态）
            say(f'发现 Gazebo（{len(topics)} 个话题），打开窗口挂上去。', 'ok')
            if looks_like_our_sim(topics):
                say('已确认是本项目仿真（话题里有 agv_500）', 'ok')
            return run_attach(args)
        say(f'{ATTACH_MASTER} 上没有 Gazebo 在跑，转为自建三维看板（镜像模式）。', 'warn')
        say('提示：镜像模式靠读 TF 定位，栈没在跑的话影子车只会停在原地。', 'info')

    # ── 镜像模式 ──
    run_mirror(args)
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        # 顶层也兜一次：/bin/bash 重入那条路上 Ctrl-C 会直接落到这里
        say('已退出。')
        sys.exit(130)
