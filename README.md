# AGV 顶升搬运车 —— ROS 2 Humble 仿真优先工作区

顶升式（Kiva 型）搬运 AGV 的 ROS 2 工作区：**Gazebo Classic 仿真 + slam_toolbox 建图 + Nav2 导航 + 任务层 + 终端控制台**。
搬运策略是「**到点即顶升**」：车停到点位就调搬货服务顶升 / 下降，不做精定位对位。

---

## 一、当前能力边界

### 已经跑通的软件闭环

| 环节 | 说明 |
|---|---|
| 建图 | Gazebo 世界 → slam_toolbox 在线建图 → 命名保存进**地图库**（地图 + 该地图专属的点位/路线放在同一目录） |
| 定位导航 | 已有地图 + AMCL + Nav2：点名导航、坐标导航、按路线多点导航 |
| 指令总线 | `/agv/command` 单一入口：白名单校验、坐标越界拦截、`request_id` 去重、`stop` 快通道 |
| 完整搬运 | 取货点 → 顶升 →（可选路线各点）→ 放货点 → 下降 → 完成回 IDLE |
| 安全 | 急停（PAUSED）、指令超时停车、手动/自动互锁、速度仲裁（近障减速 / 择空 / 限幅 / 上层停发 0.5 s 后停） |
| 交互 | 终端控制台 v3：建图模式 / 导航模式、路线管理、任务系统、一键自检 |

### 留给硬件的口子（只有一处）

- **搬货硬件**：`agv_lift/agv_lift/cargo_node.py` 的 `/agv/cargo` 服务。现阶段它只维护「正在搬货」标志（话题 `/agv/cargo/busy`）并**立即返回成功**，不驱动任何机构。接真实顶升/下降机构时只改这一个文件的服务实现，服务名、字段、话题都不变。
- **底盘**：Gazebo 差速插件听 `/cmd_vel_raw`。实车阶段把 `/cmd_vel_raw` 接到下位机即可；速度仲裁 `velocity_arbiter` 已经是软件侧最后一道处理。

### 本阶段明确不做

- 精定位 / docking（凵槽特征匹配、相对位姿解算、预备位两段式到点）——**已删除**；
- 货物识别 / 货物跟随——**已删除**；
- 精度类量化承诺（成功率、误差指标等）：当前阶段不追求量化精度，后续按需再补。

---

## 二、环境要求

| 项 | 要求 |
|---|---|
| 系统 | Ubuntu 22.04（控制台用到 `setsid` / `killpg` / `pkill` / 终端窗口，**仅 Linux**） |
| ROS 2 | Humble |
| 仿真 | Gazebo Classic 11（`gazebo_ros` / `gazebo_ros_pkgs` / `gazebo_plugins`） |
| 导航 | Nav2（`nav2_bringup` / `nav2_map_server` / `nav2_amcl` / `nav2_planner` / `nav2_controller` / `nav2_bt_navigator` / `nav2_behaviors` / `nav2_lifecycle_manager` …）、`nav2_simple_commander`、`slam_toolbox` |
| 其他 | `robot_localization`（EKF）、`rviz2`、`teleop_twist_keyboard`、`tf_transformations` |
| Python | `PyYAML`（`python3-yaml`）、`pytest`（`python3-pytest`，只跑单测需要） |

---

## 三、目录结构

```
src/
├── agv_interfaces/   接口契约：3 msg（AgvCommand / TaskState / NamedPose）+ 3 srv（AgvStore / AgvQuery / CargoCmd）
├── agv_common/       纯逻辑公共库（无 ROS 节点）：config_loader、map_manager、map_check、goal_check、limits、reactive_avoid、geometry
├── agv_description/  URDF/模型 + Gazebo 世界（worlds/world666.world）+ sim.launch.py
├── agv_navigation/   建图/导航 launch + Nav2 参数 + slam_toolbox 参数 + RViz 配置 + 内置地图（world666map）
├── agv_bringup/      总启动 agv_stack.launch.py + 参数中心 config/default|real
├── agv_waypoint/     统一存储服务节点（waypoint_node，提供 /agv/store）+ 纯逻辑 WaypointStore / RouteStore
├── agv_lift/         搬货服务节点（cargo_node）——**全项目唯一与搬货硬件打交道的地方**
├── agv_task/         任务层（task_manager_node 薄接线 + command_router / task_flow / nav_runner / cargo_client / store_client / state_reporter / TaskManager / GotoController / SafetyMonitor）
└── agv_hardware/     硬件边界：velocity_arbiter 速度仲裁（/cmd_vel → 避障/限幅 → /cmd_vel_raw）
└── agv_web/          网页控制台：web_gateway（rclpy + aiohttp）+ 前端静态页（地图/障碍/位姿/日志）
maps/                 运行时地图库：一份地图一个目录
tools/                agv_console.py（终端控制台 v3）、make_world_map.py（一份布局同时生成 world + 参考栅格图 + 点位）
```

### 地图库与点位绑定

```
maps/                            ← 地图库根（默认 <工作区>/maps，可用环境变量 AGV_MAPS_DIR 覆盖）
└── world666map/                 ← 一份地图 = 一个同名目录（当前项目唯一的地图）
    ├── world666map.pgm          ← 栅格图：594 × 762 格 @ 0.025 m，origin (-10.7, -9.83)
    │                              → 覆盖 x[-10.70, 4.15] y[-9.83, 9.22]
    ├── world666map.yaml         ← 地图元数据（分辨率 / origin）
    ├── waypoints.yaml           ← 这份地图专属的点位（kind: cargo 搬货点 / nav 导航点）
    ├── routes.yaml              ← 这份地图专属的路线（点位位姿快照）
    └── tasks.yaml               ← 这份地图专属的任务缓冲（控制台「任务系统」读写）
```

**世界与地图都统一到 world666**，这是当前唯一的一套场景：

| 产物 | 位置 | 说明 |
|---|---|---|
| Gazebo 世界 | `src/agv_description/worlds/world666.world` | 唯一一份世界真源，由 `tools/make_world_map.py` 生成（含 `<gravity>` / `<physics>` / `<scene>`） |
| 栅格地图（实测为准） | `maps/world666map/world666map.pgm` + `.yaml` | **控制台建图模式实测跑出来的那份**，Nav2 实际加载它 |
| 包内同份地图 | `src/agv_navigation/maps/world666map/` | 随包安装进 `share/`，`navigation.launch.py` 的默认地图取这里 |
| 参考栅格图 | `src/agv_navigation/maps/world666map/world666map_ref.pgm` + `_ref.yaml` | 脚本按几何**推算**出来的参考图，仅供对比；不是 Nav2 加载的那份，脚本也不会覆盖实测地图 |

三处 launch 的默认世界 / 地图已统一：`sim.launch.py`、`mapping.launch.py`、`navigation.launch.py` 的默认世界都是 `world666.world`、默认地图是 `world666map.yaml`；`agv_stack.launch.py` 在没传 `map_yaml` 时也用 `agv_navigation/maps/world666map/world666map.yaml` 兜底（供目标点判据用）。

加载哪份地图，业务节点就用哪份点位/路线/任务文件；**删地图 = 删目录**，点位路线跟着走，不留孤儿数据。

> **任务缓冲语义**：任务按地图存在 `maps/<地图名>/tasks.yaml`；在任务系统里按 `q` 是**返回导航主界面**（会先确认，确认后清空缓冲并停车），缓冲不跨会话保留。

### 速度链路

```
Nav2: controller_server → cmd_vel_nav → velocity_smoother → /cmd_vel
        ↓（任务层急停也发 /cmd_vel 零速；建图模式键盘遥控 / 控制台手动直接发 /cmd_vel_raw）
agv_hardware/velocity_arbiter   ← /scan（近障减速 + 左右择空 + 限幅；0.5 s 收不到新速度即输出零速）
        ↓
/cmd_vel_raw → Gazebo 差速插件（实车：下位机）
```

---

## 四、安装构建

```bash
# 在仓库根目录
colcon build
source install/setup.bash
```

> 改完代码必须重新 `colcon build` 再跑，否则跑的还是旧版本。

---

## 五、快速上手

### 1. 一键起整套（仿真 + 定位导航 + 业务节点 + 速度仲裁）

```bash
ros2 launch agv_bringup agv_stack.launch.py with_sim:=true \
    map_yaml:=maps/world666map/world666map.yaml \
    waypoint_file:=maps/world666map/waypoints.yaml
```

- `routes_file` 不给时会自动按 `waypoint_file` 同目录推导成 `maps/world666map/routes.yaml`。
- 不带 `map_yaml` 也能起：此时 launch 会用 `agv_navigation/maps/world666map/world666map.yaml` 兜底当作目标点判据的依据，Nav2 用它自己的默认地图（同一份几何）。
- 无头运行：加 `gui:=false rviz:=false`。
- 只要业务节点、不带仿真：去掉 `with_sim:=true`（或写 `with_sim:=false`）。
- 切换参数档位：`param_profile:=real`。参数档位**默认是空串 = 只用 `config/default/`**；`config/sim/` 已删除（原本就是空目录，选它等于空转），`config/real/` 目前只有占位 README、还没有实际覆盖文件，所以现在给不给 `param_profile` 实际值都全部来自 `config/default/`。
- 可用参数：`param_profile` / `with_sim` / `gui` / `rviz` / `world_file` / `map_yaml` / `waypoint_file` / `routes_file`。

### 2. 或者用控制台（日常首选）

```bash
python3 tools/agv_console.py            # 启动先选模式：1 建图 / 2 导航 / 3 只做自检 / q 退出
python3 tools/agv_console.py maps       # 只列出地图库（不需要 ROS）
python3 tools/agv_console.py check      # 一键自检（会自己拉起三个业务节点）
python3 tools/agv_console.py --mode nav --no-gui      # 直接进导航模式，不开 Gazebo 窗口
python3 tools/agv_console.py --mode mapping --no-rviz # 直接进建图模式
```

> 除 `maps` 子命令外，控制台每次启动都会**先清理上次残留的仿真/导航/业务进程**，再从干净状态起步（防多套栈打架）。
> 控制台拉起整套栈时用的是默认档位（`param_profile` 空串，即只用 `config/default/`）。

### 2b. 网页控制台（浏览器操作，手机也能用）

不想要终端的话，用网页控制台。它把终端控制台的功能全部搬到浏览器里，并且能像 RViz
一样显示地图、障碍物、小车实时位置和全局路径。

```bash
# 一条命令起网关（建图/导航都在网页里点按钮切换，不用回终端）
ros2 launch agv_web web_console.launch.py token:=agv2024

# 只做网关代理，接已经在跑的栈（栈不归本进程管理）
ros2 launch agv_web web_console.launch.py token:=agv2024 manage_stack:=false
```

> ★ `web_console.launch.py` **只有网关**，不会自己 include 仿真/Nav2 ——
> 栈由网关在收到网页指令后用 `Popen` 拉起（见 `manage_stack`）。
> 可用参数：`token` / `port` / `allow_ips` / `manage_stack` / `ws_root`。

然后用浏览器打开：

```
http://<车机IP>:8080
```

- **手机和电脑在同一局域网都能进**（网关监听 `0.0.0.0`，不是 `127.0.0.1`）。
  手机访问 `http://192.168.x.x:8080` 即可，不用装 App。
- 车机 IP 会变的话，建议在路由器里给车机 MAC 绑定固定 IP。
- **手机端布局**：底部有「地图 / 操作 / 日志」三个页签，按钮都是 ≥44px 的触控尺寸；
  遥控用十字键按住即走，不需要键盘。
- **全程点选，不用打字**：存点位、建路线、加任务、选地图、存地图都是点选的。
  只在「起名字」时才弹输入框，而且**预填好了默认值**（货架1 / 路线1 / 地图10月17日23时），
  直接点确定即可。
  - 建路线的途经点原本要输入"逗号分隔的点位序列"，现在改成**按顺序点选**，
    点错再点一下取消，顺序天然就是点击顺序。
  - 新增接运任务的循环次数是 **+ / − 步进器**，不用敲数字。
  - 所有 `confirm()` 也换成了统一样式的模态框（手机上从底部弹出）。
- **地图交互**：单指拖动平移、双指缩放、单击取位置（用于存点位）、
  **长按地图 = 直接把该点设为导航目标**。
- **控制权锁**：同一时刻只有一台设备能遥控，另一台会显示"被占用"，
  闲置 3 秒自动释放。这是为了防止手机和电脑同时按方向键把车弄抽。
- **口令认证**：`token:=` 参数就是访问口令，登录后存在 Cookie 里。
  不设口令则局域网内任何人都能控制小车，**正式使用务必设置**。

> ★ 网页遥控走的是 `/agv/web_teleop`，由 `task_manager_node` 校验手动模式后转发，
> 不直接写 `/cmd_vel`——否则会绕过手动模式互锁。
>
> ★ 网页与控制台共享同一套状态（`/agv/command`、`/agv/task_state`、`/agv/store`），
> 两边看到的点位、路线、任务缓冲、地图库都是**同一份文件**，不会各说各话。

网页控制台依赖 `aiohttp`：

```bash
sudo apt install python3-aiohttp     # 或 pip3 install aiohttp
```

首次使用需先构建新增的三个包：

```bash
colcon build --packages-select agv_common agv_task agv_web
source install/setup.bash
```

#### 一条命令启动（推荐）

网关自己会起停整套 ROS 栈，**建图/导航在网页里点按钮切换**，不用回终端：

```bash
ros2 launch agv_web web_console.launch.py token:=agv2024
```

浏览器打开 `http://<车机IP>:8080`，然后在「工作模式」卡片里：

| 按钮 | 做什么 |
|---|---|
| 启动导航 | 拉起 `agv_stack`（仿真+AMCL+Nav2+任务层+速度仲裁） |
| 开始建图 | 拉起建图栈（仿真+slam_toolbox），遥控走一圈后点「保存地图」 |
| 停止 | 停掉整套栈（先 SIGINT 等 12 秒，不行再 SIGKILL 整组） |

切模式时会**先停旧的再起新的**——两套栈并存会互相抢话题/服务，表现为原地转、无响应。
已在该模式下再点同一个按钮不会重启（Nav2 激活要 15~40 秒，白等）。

> ★ 为什么网关能这么做：它用 `subprocess.Popen(..., start_new_session=True)` 把
> launch 起成一个独立进程组，停的时候对**整组**发信号。只杀 launch 父进程会留下
> `gzserver`/`nav2` 孤儿节点继续抢话题——"停了车还在飘"就是这么来的。
> 另外网关会持续读子进程输出（不读的话 PIPE 写满 64KB 后整个栈会卡死）。

#### 分模式单独启动（调试用）

需要手工控制时也可以分开起：

```bash
ros2 launch agv_web web_console.launch.py token:=agv2024 manage_stack:=false  # 只做网关
ros2 launch agv_web web_mapping.launch.py token:=agv2024                     # 只建图
```

#### 网页建图流程

1. 浏览器打开 `http://<车机IP>:8080`（手机同局域网也能开）
2. 用十字键遥控走一圈覆盖场地 —— 地图会**实时长出来**（右上角显示格数与米数）
3. 走完点「保存地图」，输入名字即可存进地图库
4. 切到「启动导航」，在弹框里点这张地图

#### 导航：每次启动都会让你选地图（哪怕只有一张）

点「启动导航」后**一定会弹一次选图框**，每张图带副标题：

```
选择导航地图
  地图库里有 2 张图，点一张用来导航
  ✓ world666map（当前加载）   15 个点位 · 含路线 · 2026-10-17 22:10
    workshop_1021             8 个点位 · 2026-10-21 09:30
```

- **只有一张图也弹**：建图和导航两个按钮挨着，误点「启动导航」时如果连地图
  都不问就把整套栈拉起来，人没有任何撤销机会。多这一步就能拦住。
- **点「取消」什么都不发生**：不会启动栈、不会碰正在跑的那套。
- 默认选中**当前正在用的那张**，直接点确定即可。
- 副标题里的「点位 / 路线 / 保存时间」来自 `MapManager.map_info()`，
  地图一多，光看名字（`map1`、`map2`）根本分不清哪张是哪张。

> ★ 地图库为空时才会提示「请先建图」—— 导航**不需要**先建图，
> 只要 `maps/` 下有可用的 `xxx.pgm + xxx.yaml` 就能直接导航。

> ★ 建图模式和导航模式的**遥控通道不一样**：建图栈里没有 `velocity_arbiter`、
> 也没有 `task_manager`，而底盘（URDF 里 `cmd_vel:=cmd_vel_raw` 写死）只听
> `/cmd_vel_raw`。所以建图时遥控直接发 `/cmd_vel_raw`。
> **如果照导航模式发 `/agv/web_teleop`，车轮一动不动。**
>
> ★ 建图时 `slam_toolbox` 每 0.5 秒发一次全量 `/map`，一次 594×762 编码约 0.11 秒。
> 网关按 `map_min_interval`（默认 0.8 s）限流，否则会吃掉大量 CPU 并阻塞 ROS 回调。


### 3. 建图流程

```bash
# 起「仿真 + slam_toolbox + RViz」
ros2 launch agv_navigation mapping.launch.py

# 另开终端遥控走一圈（建图模式下的遥控直接驱底盘 /cmd_vel_raw）
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_raw

# 走完保存地图（当前默认地图名就是 world666map）
ros2 run nav2_map_server map_saver_cli -f /tmp/world666map
# 然后把 /tmp/world666map.pgm、.yaml 放进 maps/<地图名>/ 并改名成 <地图名>.pgm / <地图名>.yaml
```

用控制台更省事：建图模式里按 **k** 遥控、按 **m** 命名保存（自动落到 `maps/<名字>/`，并补齐该地图专属的 `waypoints.yaml` / `routes.yaml`——已存在的不覆盖）。

> 控制台的 **m** 走的就是 `map_saver_cli`：先存到临时目录，成功后再搬进地图库，并把 yaml 里的 `image` 字段改成本目录同名 pgm；地图名已存在会先问是否覆盖。
> 当前项目实际加载的实测地图就是 `maps/world666map/world666map.pgm`（594 × 762 @0.025，origin (-10.7, -9.83)）；重跑 `tools/make_world_map.py` **不会**覆盖它，只会更新 `world666map_ref.pgm`（参考图）。

### 4. 存点位（区分 kind）

点位与路线**共用一个服务** `/agv/store`（`AgvStore`），用请求里的 `resource`（`waypoint` / `route`）+ `action`（`save` / `get` / `list` / `delete`）两个字段分派 —— 不再是一堆一一对应的服务名。

```bash
# 搬货点（完整搬运任务的取货点 / 放货点）
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
    "{resource: 'waypoint', action: 'save', name: '取货点', x: 1.63, y: 2.66, yaw: 0.0, kind: 'cargo'}"
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
    "{resource: 'waypoint', action: 'save', name: '放货点', x: -7.57, y: -7.41, yaw: -3.08, kind: 'cargo'}"

# 导航点（只用来组成路线，不会被当取货/放货点）
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
    "{resource: 'waypoint', action: 'save', name: '走廊A', x: 3.0, y: 1.0, yaw: 0.0, kind: 'nav'}"

# 查 / 列 / 删
ros2 service call /agv/store agv_interfaces/srv/AgvStore "{resource: 'waypoint', action: 'get', name: '取货点'}"
ros2 service call /agv/store agv_interfaces/srv/AgvStore "{resource: 'waypoint', action: 'list'}"
ros2 service call /agv/store agv_interfaces/srv/AgvStore "{resource: 'waypoint', action: 'delete', name: '取货点'}"
```

> `kind` 留空或不合法一律按 `cargo` 处理。
> 响应里 `ok` + `message` 给结果，`items[]`（`NamedPose`：`name/x/y/yaw/kind`）给内容；`list` 另给 `names[]` + `counts[]`（同序，`count` 是条目总数）。**每个动作都有 `ok`**，`list` 也不例外。注意 `route` + `list` 只回名字与个数、`items[]` 为空（要看某条路线的途经点用 `get`）。
> `maps/world666map/waypoints.yaml` 里已预置 5 个搬货点 + 「起始点」（全部 `kind: cargo`），可直接拿来试：
>
> | 点名 | x | y | yaw |
> |---|---|---|---|
> | 1号货架 | 1.63 | 2.66 | 0.00 |
> | 2号货架 | -7.57 | -7.41 | -3.08 |
> | 3号货架 | 1.56 | -6.17 | 0.00 |
> | 4号货架 | -8.62 | 7.02 | 1.55 |
> | 5号货架 | -7.81 | -2.51 | -3.10 |

### 5. 存路线（按点名复制位姿快照）

```bash
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
    "{resource: 'route', action: 'save', name: '巡检A', point_names: ['取货点', '走廊A', '放货点']}"

ros2 service call /agv/store agv_interfaces/srv/AgvStore "{resource: 'route', action: 'get', name: '巡检A'}"
ros2 service call /agv/store agv_interfaces/srv/AgvStore "{resource: 'route', action: 'list'}"
ros2 service call /agv/store agv_interfaces/srv/AgvStore "{resource: 'route', action: 'delete', name: '巡检A'}"
```

> 存路线时会按点名从点位表**复制**位姿，所以之后改名/删点都不影响已存路线；
> 有点名不存在则整条路线拒绝写入，不会留半条。
> 界面里用控制台导航模式 **[7] 路线管理** 更直观（点名用 `,` 隔开）。

### 6. 点名导航 / 坐标导航

```bash
# 点名导航
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
    "{source: 'web', command: 'goto', args: ['取货点'], route: '', request_id: 'r1'}"

# 点名导航 + 走路线（多点）
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
    "{source: 'web', command: 'goto', args: ['放货点'], route: '巡检A', request_id: 'r2'}"

# 坐标导航（[x, y, yaw]，可只给 x y，yaw 按 0）
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
    "{source: 'web', command: 'goto', args: ['3.0', '1.0', '0.0'], request_id: 'r3'}"
```

> `request_id` 相同的指令**只受理一次**（去重）。重发同一条时请改 `request_id`，否则会被判「重复」拒绝。

### 7. 完整搬运 / 顶升 / 下降 / 急停 / 恢复

```bash
# 完整搬运：取货点 → 顶升 →（可选路线）→ 放货点 → 下降 → 完成
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
    "{source: 'web', command: 'start_task', args: ['取货点', '放货点'], route: '', request_id: 't1'}"

# 取消 / 急停 / 恢复
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
    "{source: 'web', command: 'cancel_task', request_id: 'c1'}"
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
    "{source: 'web', command: 'stop', request_id: 's1'}"
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
    "{source: 'web', command: 'resume', request_id: 's2'}"

# 单独试搬货服务（现阶段的实现：只置标志、立即成功，不动机构）
ros2 service call /agv/cargo agv_interfaces/srv/CargoCmd "{command: 'lift'}"
ros2 service call /agv/cargo agv_interfaces/srv/CargoCmd "{command: 'lower'}"
ros2 topic echo /agv/cargo/busy        # Bool：true = 正在搬货
```

> 指令白名单只有 5 个：`goto` / `start_task` / `cancel_task` / `stop` / `resume`。
> 搬货（顶升/下降）**不走指令总线**，由任务层直接调 `/agv/cargo` 服务。

### 8. 看状态 / 问状态

```bash
ros2 topic echo /agv/task_state                                   # 1 Hz 周期 + 变化时上报
ros2 service call /agv/query agv_interfaces/srv/AgvQuery "{what: 'state'}"      # 任务状态
ros2 service call /agv/query agv_interfaces/srv/AgvQuery "{what: 'pose'}"       # 当前位姿
# 点位表 / 路线表不在 /agv/query 里查，走存储服务：
ros2 service call /agv/store agv_interfaces/srv/AgvStore "{resource: 'waypoint', action: 'list'}"
ros2 service call /agv/store agv_interfaces/srv/AgvStore "{resource: 'route', action: 'list'}"
```

`/agv/query` 只管**运行时状态**（`state` / `pose` 两种取值）；点位与路线的列表属于"存储内容"，统一走 `/agv/store` 的 `action='list'`。

`/agv/task_state` 的 `state` 取值：`IDLE` → `GO_LOAD` → `LIFTING` → `TRANSPORT` → `LOWERING` → `DONE`（自动回 `IDLE`）；异常 `ERROR`；急停/停止 `PAUSED`。

---

## 六、控制台操作速查表

**子命令 / 参数**

| 命令 | 用途 |
|---|---|
| `python3 tools/agv_console.py` | 交互式：先选模式 |
| `python3 tools/agv_console.py check` | 一键自检（自己拉起业务节点，不需要仿真） |
| `python3 tools/agv_console.py maps` | 列出地图库（点位/路线条数） |
| `--mode mapping` / `--mode nav` | 跳过模式选择直接进某模式 |
| `--no-gui` / `--no-rviz` | 起仿真时不开 Gazebo 窗口 / 不开 RViz |

**网页控制台**（浏览器操作，无需终端）

| 命令 | 用途 |
|---|---|
| `ros2 launch agv_web web_console.launch.py token:=xxx` | **推荐**：只起网关，网页里切建图/导航 |
| `ros2 launch agv_web web_console.launch.py manage_stack:=false token:=xxx` | 只做代理，接已起好的栈 |
| `ros2 launch agv_web web_mapping.launch.py token:=xxx` | 只建图（调试用，固定建图模式） |
| 浏览器打开 `http://<车机IP>:8080` | 手机/电脑同一局域网都能进 |

**模式选择**：`1` 建图模式　`2` 导航模式　`3` 只做自检（不起仿真）　`q` 退出

**建图模式**

| 键 | 作用 | 键 | 作用 |
|---|---|---|---|
| `k` | 键盘遥控（新终端，直接驱 `/cmd_vel_raw`） | `l` | 列出已保存的地图 |
| `m` | 保存当前地图（命名） | `d` | 删除地图（含其点位/路线/任务） |
| `n` | 切换到导航模式 | `q` | 退出 |

**导航模式 · 先加载地图**：`1` 加载地图（输入地图名）　`2` 列出地图　`3` 删除地图　`m` 切回建图　`q` 退出

**导航模式 · 业务菜单（已加载地图后）**

| 键 | 作用 | 键 | 作用 |
|---|---|---|---|
| `1` | 存点位（搬货点 / 导航点） | `s` | 状态快照 |
| `2` | 列出点位 | `n` | 导航节点激活状态（5 个 lifecycle 节点） |
| `3` | 删除点位 | `l` | 看任务栈日志尾 |
| `4` | 点名导航（可选路线） | `c` | 一键自检 |
| `5` | 坐标导航 | `0` | 换地图（会重起整套栈） |
| `6` | 手动开小车（有任务时会先问是否停止） | `q` | 退出终端（**只有此界面能退出**；有活会先确认） |
| `7` | 路线管理 | `t` | 完整搬运（可选路线） |
| `8` | 查询状态（state/pose 走 `/agv/query`；waypoints/routes 走 `/agv/store`） | `u` / `d` | 顶升 / 下降 |
| `9` | 任务系统 | `x` / `r` | 急停 / 恢复 |

**路线管理**：`1` 设置/修改（点名用 `,` 隔开）　`2` 列出全部　`3` 查看详情　`4` 删除　`q` 返回

**任务系统**（按地图存 `tasks.yaml`）：`1` 添加（取货/放货/循环次数/路线）　`2` 删除（按序号）　`3` 列出　`4` 清空　`5` 启动任务（依次执行，执行中可 `s` / `l` / `q`）　`q` 返回导航主界面（清空缓冲并停车）

> **页面层级（怎么返回 / 怎么退出）**：导航主界面就是加载地图后那个菜单。从 `6` 手动 / `7` 路线管理 / `9` 任务系统 / `t` 完整搬运 / `c` 自检进去后，在功能里按 `q` **只返回导航主界面**（返回后会自动重印主菜单），不会关掉终端；**只有回到导航主界面再按 `q` 才退出终端**（若车在跑任务或缓冲非空，会先确认）。

---

## 七、常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| 服务一直 `waiting for service` | 节点没起，或没 `source install/setup.bash` | 先 `ros2 node list` 确认节点；重开终端记得 source |
| 节点「没起 / 无响应 / 查询超时」 | 上次没退干净，多套栈同名节点互抢话题/服务/action | 用控制台启动（会自动清理残留）；手动时 `pkill -9 -f gzserver; pkill -9 -f nav2_; pkill -9 -f task_manager_node; pkill -9 -f velocity_arbiter` |
| `/scan` 全是 `inf` | 世界没加载起来，车在一个空世界里 | 用 `ros2 launch agv_navigation navigation.launch.py` 或 `agv_stack.launch.py with_sim:=true`（它们自带默认世界 `world666.world`）；要换世界用 `world_file:=/abs/xxx.world`。**注意别改 launch 参数名**：`agv_stack.launch.py` 的对外参数故意叫 `world_file`（空默认值只在显式给了才转发给导航栈），改成 `world` 会让空默认值顶掉导航栈的默认世界 |
| 车不停 / 一直往前跑 | 上层（Nav2/任务层/遥控）已停发速度，但底盘还在跑 | 仲裁节点 `cmd_timeout` 默认 0.5 s：收不到新 `/cmd_vel` 就输出零速。若仍不停，多半是**残留的遥控进程**在直接给 `/cmd_vel_raw` 灌速度（控制台会清 `teleop_twist_keyboard`），手动排查：`pkill -9 -f teleop_twist_keyboard` |
| 急停之后导航全被拒 | 急停进 `PAUSED`，未恢复 | 发一次 `resume`（控制台按 `r`）即回空闲待命，不必重启节点 |
| Nav2 节点显示「未激活」 | 仿真刚起，lifecycle 还没激活完 | 等 15~40 s 再查（控制台按 `n`）；仍不行看控制台 `l` 的日志尾 |
| 换地图后点位/路线「不见了」 | 点线跟地图绑定，换地图就换文件 | 这是设计行为：用控制台 `0` 换地图时它已自动切换到该地图的 `waypoints.yaml` / `routes.yaml` |
| 手动模式下导航被拒 | 手动/自动互锁生效 | 控制台手动控制结束后会自动置回自动模式；手工验证时给 `/agv/manual_mode` 发 `false` |
| 点位贴墙/图外的目标直接被拒 | 目标点判据（`GoalValidator`）会拦：图外、压障碍、净空不足 | 换一个可达点；净空阈值在 `src/agv_bringup/config/default/task.yaml`（`robot_clearance`）；`field_*` 只是**没有地图时**的兜底范围 |
| 日志刷端口告警（`RTPS_TRANSPORT` / `open_and_lock` 之类） | 共享内存/端口提示 | 一般无害，可忽略（控制台读日志时会自动过滤这类行） |
| 任务执行完 / 点了中止，界面还显示「执行中」，之后既不能中止也不能开始新任务 | 三个坑叠加：① `POST /api/tasks` 没有 `list` 分支 → 刷新任务列表永远 400，前端 `taskRunning` 不更新；② 中止只发裸 `cancel_task`，网关的 `_gw_task_stop` 事件没被置位，后台线程要等 300 秒超时；③ `handle_cancel` 改了状态机却没 `publish_state()` | 已修。判据：点「中止」后按钮应**立刻**变回「开始执行」。回归：`bash tools/webtest/run_all.sh task` |
| 日志面板满屏英文，看不懂 | ROS/launch/Nav2 的原始输出直接透传 | 已修：`log_translate.py` 过滤噪音（launch 样板话、参数回显、bond timer、连杆列表…）+ 已知句式翻中文 + 英文节点名映射成中文角色（`[定位]`/`[路径规划]`/`[地图服务]`…）。翻不出来的保留原文，不丢信息 |
| 日志一直在刷「连接断开」 | WebSocket 断线后每 2 秒重连并打一条日志 | 已修：断线只把顶部状态芯片变红，日志只在首次断线提示一次、连续失败 10 次升级为错误；重试间隔 2s→10s 逐步拉长 |

> ⚠️ **不要同时起两套仿真/导航栈**——同名节点会让话题、服务、action 互相抢，看起来就像「导航坏了」。控制台已按模式隔离，一次只跑一套；手动起 launch 时也务必一次只起一套。

---

## 七点五、怎么验证（直接复制粘贴）

全部验证就两条命令：

```bash
cd /home/yuki/agv/newros_car7
bash tools/webtest/run_all.sh
```

会依次跑：编译 → 69 项 Python 单测 → 63 项前端交互（jsdom 真跑 app.js）→
JS 语法 → 原生弹窗残留检查，最后打印「通过 N 组，失败 M 组」。

只要某一组，加参数：

```bash
bash tools/webtest/run_all.sh unit      # 只跑单测（2 秒，改动代码后最常跑）
bash tools/webtest/run_all.sh ui        # 只跑前端交互 + 静态检查
bash tools/webtest/run_all.sh e2e       # 端到端：起注入器+业务节点+网关，走完整 HTTP 链路（约 60 秒）
bash tools/webtest/run_all.sh mapping   # 专项：建图模式遥控是否直发 /cmd_vel_raw
bash tools/webtest/run_all.sh task      # 专项：任务闭环（开始→中止→再启动，约 30 秒）
bash tools/webtest/run_all.sh render    # 出地图渲染预览图 tools/webtest/render_preview.png
```

**看端到端结果时注意第 4 项「自检接口」**：它是**全项 AND**，要求
`task_manager_node` / `velocity_arbiter` / `amcl` / `controller_server` /
`planner_server` 全在。跑单测和前端测试时并没有起这些节点，所以这里显示
`ok: False` 是**正常的**，真正要看的是下面逐项打印里
`地图已加载 / 定位可用 / 激光可用` 是否为 OK —— 这三项 OK 说明数据链路是通的。
上车后起完整栈再跑，这一项才会变 True。

无 Gazebo 环境时，用 `tools/webtest/injector.py` 替代仿真：它按标准 ROS
话题发布 `/map`(120×80) / `/scan`(360°) / `map→odom→base_footprint` TF /
`/plan` / `/agv/task_state`，并订阅 `/cmd_vel_raw` 和 `/agv/web_teleop`
打印收到的帧，用来验证遥控分流。

### 关掉所有后台进程

```bash
cd /home/yuki/agv/newros_car7
bash tools/webtest/stop_all.sh
```

按「SIGINT → 等 12 秒 → SIGKILL 兜底 → 按端口兜底」的顺序清，最后打印
8080~8100 是否全部释放。

**为什么不能直接 `pkill -9 -f python3`**：会把 WorkBuddy 本体和其他 python
进程一起杀掉。脚本里每个模式都限定到具体脚本名/节点名。

**为什么要先 SIGINT**：`ros2 launch` / Gazebo / Nav2 靠 SIGINT 释放 DDS 端口、
注销 `/dev/shm`、删 lifecycle 节点。直接 `-9` 会留僵尸节点和共享内存段，
下次启动就出现「同名节点互抢话题」，看着像导航坏了。

若脚本提示「端口仍在监听但属于别的 PID 命名空间」（典型场景：直接关掉终端
窗口而没按 Ctrl+C），用 `sudo ss -K 'sport = :8095'` 强拆 socket，或注销重登桌面。

### 生命周期：启动自动清理、网页退出关机

**启动时自动清理残留**（和终端版 `agv_console.py` 一致）

网关启动时会先扫一遍上次没退干净的 ROS 进程，先 SIGINT 给 6 秒、超时
才 SIGKILL。不用再手动跑 `stop_all.sh`。

日志里能看到：

```
清理上次残留进程：2 个
  SIGINT 87 (waypoint_node)
  SIGINT 89 (waypoint_node)
  已全部退出
```

三重安全边界，都是踩坑换来的：

1. **只匹配本项目独有的可执行名**——绝不用 `python3` / `node` 这类宽泛词，
   否则会把 WorkBuddy 本体、你自己的脚本一起杀掉；
2. **排除自己的整条祖先链**——最初写 `ros2 launch agv_` 想一把匹配所有本项目
   launch，结果网关自己就是 `ros2 launch agv_web ...` 拉起来的，启动瞬间把
   自己杀了（日志：`34 已杀死 ros2 launch agv_web ...`），网页根本没起来。
   现在用 `_ancestor_pids()` 上溯父进程链，凡是祖先一律不动；
3. **`manage_stack=false` 时完全不清理**——那种模式是"你自己在别的终端起了栈，
   网页只当遥控器"，清掉等于杀掉你正在跑的任务。
   临时禁用清理：`AGV_NO_CLEANUP=1 ros2 launch agv_web ...`。

**网页上点「退出并关闭车」**

停掉整套栈 + 关掉网关，端口和进程一起释放。和「停止」的区别：
「停止」只停栈、网关还在（还能再点启动）；「退出」是连网关一起走。

**关浏览器窗口不会自动停车**，这是有意的。因为按 F5 刷新页面也会触发
`beforeunload`，如果顺手停栈，你每刷一次车就关一次。所以：
- 刷新 / 切后台 → 只断开连接、交还控制权，栈继续跑；
- 真要收工 → 点「退出并关闭车」。

---

## 七点六、控制权为什么不用反复点

以前「取得控制权」后原地站 3 秒就丢了，走一步要点一次锁。原因是把
「把车停住」和「把锁还回去」混成了同一个超时：

| | 该多快 | 谁来保证 |
|---|---|---|
| 车停住 | **0.35 秒**（必须短，安全底线） | `TeleopWatchdog` + ROS 的 `cmd_timeout 0.5s` |
| 锁释放 | **120 秒**（慢了完全无害） | `ControlLease` |

没锁的设备本来也发不动车（发过来直接 409），所以把锁留久一点没有安全风险。
现在的行为：

- 默认闲置 **120 秒**才释放，可在 launch 里用 `lease_idle_sec:=` 改；
- 持有期间每 **20 秒**自动续期一次（页面在后台时不续，避免白占着车）；
- 手机切回前台立刻补一次续期；
- 丢掉控制权时**明确提示**（`控制权已被「xxx」接管`），不会静默失效；
- 关页面时用 `sendBeacon` 自动交还，不会一直占着。

---

## 八、已知限制与待办

- **精度类指标不承诺**：当前阶段到点即停，不做对位，也不给成功率/误差之类的量化承诺；后续按需再补（补的时候再引入指标与门禁工具）。
- **搬货硬件未接**：`cargo_node` 只置位「正在搬货」标志并立即返回成功，不驱动机构；接硬件只改 `agv_lift/agv_lift/cargo_node.py`。
- **地图坐标系与 Gazebo 世界只靠地图 origin 对齐**：没有额外的 TF 校验环节。换 world 或改场景后必须重新建图（或重跑 `tools/make_world_map.py` 让 world 与参考图同源重生成），否则 AMCL 对不上、车走不动。
- **目标点只有一个判据，且已与地图对齐（原风险已解决）**：目标点能不能去，全项目只有 `agv_common/goal_check.py` 的 `GoalValidator` 一个判据 —— **有地图时以实测地图为准**（图外 / 压障碍 / 净空不足都拒），没地图（如纯逻辑测试）时才退化为 `src/agv_bringup/config/default/task.yaml` 的 `field_*` 矩形兜底。原先"门卫按 `field_*` 判、预检按地图判"是两套判据、两个范围（`field_*` 曾是 `-11.30 / 10.45 / -10.42 / 9.91`，比实测地图大一圈），会出现「门卫放行、预检拒绝」的同坐标两种结论；现在 ① `field_*` 已改成实测地图范围 `-10.70 / 4.15 / -9.83 / 9.22`，② 判据收敛为一个对象，`precheck_enabled` 开关删除 —— `field_*` 从"第二个判据"降级为"没有地图时的兜底"，二者不可能再互相矛盾。仍要留意的是**场景覆盖**：实测地图 x 只到 4.15，而 world666 墙体跨到 x≈9.95，东侧不在实测图内，重新建图后务必复核覆盖范围。
- **场景几何只改一处**：`tools/make_world_map.py` 顶部参数区是唯一真源。跑一次它会重写 `src/agv_description/worlds/world666.world`、`src/agv_navigation/maps/world666map/`（`world666map_ref.pgm/.yaml`、`waypoints.yaml`、`routes.yaml`），并把点位/路线同步到运行时地图库 `maps/world666map/`；**实测地图 `maps/world666map/world666map.pgm` 由控制台建图产生，脚本不覆盖**。脚本会打印地图尺寸、原点与"场地范围（供 `task.yaml` 的 `field_*` 用）"。
- **控制台仅 Linux**：依赖 `setsid` / `killpg` / `pkill` / 终端窗口（`gnome-terminal` 或 `x-terminal-emulator` 或 `xterm`）跑遥控。**网页控制台不受此限制**（纯浏览器）。
- **网页控制台换地图需重启导航栈**：`map_server` 在 ROS 2 里换图要重启（点位/路线是按地图名注入 launch 参数的，热切会不一致），所以网页上"切换地图"只是记录选择并提示重启，不会热生效。
- **网页建图不做地图编辑**：能在网页里遥控建图、实时看地图长出来、命名保存；但**不能手动擦除/绘制栅格**（RViz 的 Map 编辑插件功能未移植）。需要修图时仍用图像工具处理 pgm。
- **点位文件只保留 `x / y / yaw / kind`**：`WaypointStore` 只读这四个字段，其余字段加载时丢弃、下次保存该点位时不再写回。
- 待办：接真实顶升/下降机构；`config/real/` 实车参数填充；语音入口（原计划 C5，可裁剪；网页入口已完成）。
- 本机没有 ROS 2 环境，以上内容为静态核对结果，**未在仿真或实车上跑通验证**。

---

## 九、文档索引

| 文档 | 内容 |
|---|---|
| [`README.md`](README.md) | 本文件：能力边界、构建、快速上手、控制台速查、常见问题、已知限制 |
| [`TESTING.md`](TESTING.md) | 测试策略、单测清单、一键自检与人工验收清单、调试手段 |
| [`AGV_PLAN.md`](AGV_PLAN.md) | 方案总纲 |
| [`AGV_INTERFACES.md`](AGV_INTERFACES.md) | 功能接口清单（逐模块参数 / 返回值 / 接口） |
| [`搬运场景几何契约.md`](搬运场景几何契约.md) | 改 world / 槽位尺寸前必读的几何契约 |
| [`GIT使用.md`](GIT使用.md) | git 白话说明 |

> 改场景几何的正确姿势：只改 `tools/make_world_map.py` 顶部参数区，再跑一次
> `python3 tools/make_world_map.py`，让 world、参考栅格图、建议点位三者同源生成；
> 然后按打印出的"场地范围"同步 `task.yaml` 的 `field_*`。
> **实测地图 `maps/world666map/world666map.pgm` 不会被脚本覆盖**——场景换了要重新建图。
