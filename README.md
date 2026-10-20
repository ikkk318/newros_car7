# AGV 顶升搬运车（ROS 2 Humble + Gazebo）

> 一台顶升式（Kiva 型）搬运 AGV 的完整软件：网页控制台、终端控制台、任务状态机、建图/导航栈、速度仲裁。
> 仿真优先，当前阶段接的是"假顶升"（服务立即返回成功）。

---

## 能做什么

- **网页控车**：手机/电脑同一局域网打开浏览器，看地图、看激光、按方向键、点地图设目标
- **建图**：点「开始建图」→ 遥控走一圈 → 地图实时长出来 → 保存进地图库
- **导航**：点名导航、坐标导航、带路线多点导航
- **搬运**：取货点 → 顶升 →（路线）→ 放货点 → 下降，全程状态可视
- **任务序列**：网页里排多单，依次执行，随时中止
- **安全**：急停、手动/自动互锁、指令超时停车、近障减速刹车、速度限幅

## 不做什么（当前阶段）

- 不做精定位 / 二次对位（docking）、货物跟随
- 不做量化精度验收（"误差 ≤4cm"这类指标不作为承诺）
- 搬货服务里不驱动任何真实机构——只维护"搬货中"标志

---

## 环境

Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11 + Python 3.10（系统 python）。

```bash
sudo apt install python3-aiohttp python3-yaml python3-numpy python3-pil
```

★ 启动前确认 `which python3` 是 `/usr/bin/python3`。非系统 Python 会让
`spawn_entity.py` 找不到 ROS 的 numpy（cp310），表现为**机器人根本没生成**。

---

## 三条命令上手

```bash
# 1. 构建
cd /home/yuki/agv/newros_car7
source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash

# 2. 起网页控制台（唯一入口，栈由网页按钮拉起）
ros2 launch agv_web web_console.launch.py token:=agv2024

# 3. 浏览器打开 http://<车机IP>:8080 → 登录 → 点「启动导航」→ 选地图
```

想用终端：

```bash
python3 tools/agv_console.py          # 交互式（建图/导航双模式）
python3 tools/agv_console.py check    # 一键自检
```

停所有进程：

```bash
bash tools/stop_all.sh
```

★ 示例口令 `agv2024` 请务必改掉——不设口令则局域网内任何人都能控车。

---

## 目录

```
src/
├── agv_interfaces/   3 个消息 + 3 个服务
├── agv_common/       纯逻辑底座：几何、目标点判据、地图库、避障策略、参数加载、鉴权
├── agv_description/  URDF 车体、Gazebo 世界、仿真 launch
├── agv_navigation/   Nav2 参数、建图/导航 launch、包内地图
├── agv_bringup/      参数中心（default/real）+ 业务栈总装 launch
├── agv_waypoint/     waypoint_node：点位/路线统一存储
├── agv_task/         task_manager_node：指令门卫、搬运状态机、导航执行、安全
├── agv_lift/         cargo_node：搬货服务（唯一硬件口子）
├── agv_hardware/     velocity_arbiter：近障减速 + 择空 + 限幅
└── agv_web/          网页网关（gateway/ 8 模块）+ 4 个前端脚本 + 栈管理

tools/
├── agv_console.py    终端控制台（建图/导航/点位/路线/任务/自检）
├── gazebo_view.py    网页无头栈时另开 gzclient 看车
├── stop_all.sh       一键停所有 AGV 进程
└── make_world_map.py 场景生成器（改世界/地图时用）

maps/                 地图库（每张图一个目录：pgm + yaml + 点位 + 路线 + 任务）
docs/                 文档
docs/archive/         历史原文（逐项证据表、踩坑记录）
```

只有 6 个运行入口：`web_console.launch.py`、`agv_stack.launch.py`、`navigation.launch.py`、
`mapping.launch.py`、`sim.launch.py`、`tools/agv_console.py`（外加 `gazebo_view.py` 与 `stop_all.sh`）。

---

## 文档索引

| 文档 | 讲什么 |
|---|---|
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | 分层、数据流、线程模型、启动链、编码约束、技术债 |
| [`docs/INTERFACES.md`](docs/INTERFACES.md) | 话题/服务/消息字段、指令语义与拒绝规则、参数默认值、文件格式 |
| [`docs/OPERATIONS.md`](docs/OPERATIONS.md) | 构建、启动矩阵、建图、点位/路线/任务、控制台速查、排障、验收 |
| [`docs/GEOMETRY.md`](docs/GEOMETRY.md) | 车体/传感器、世界与地图范围、槽位、15 个点位、净空判据 |
| [`docs/WEB_CONSOLE.md`](docs/WEB_CONSOLE.md) | 界面交互、27 条 API、栈管理、位姿/地图缓存、日志管道 |
| [`docs/CHANGELOG.md`](docs/CHANGELOG.md) | 三轮修复史与本次重构 |
| [`docs/VERSIONING.md`](docs/VERSIONING.md) | 版本号与标签、推 GitHub 后为什么"看起来被覆盖"、回滚与后悔药 |

历史原文（逐项证据表、踩坑记录）在 `docs/archive/`。

---

## 三个关键设计（先看这三条再改代码）

1. **统一指令总线**：所有入口只发 `/agv/command`，由 `task_manager_node` 一门受理。
   校验只有一处，状态只有一处可查。
2. **速度在硬件边界收口**：上层只发 `/cmd_vel`（想走多快），`velocity_arbiter` 统一避障与限幅后才发
   `/cmd_vel_raw`。底盘**只听** `/cmd_vel_raw`（URDF 写死）。
3. **网页只是客户端**：网关发指令、调服务、起停栈；它不绕过任务层直接控车。

改代码前必读 `docs/ARCHITECTURE.md` §12「编码约束」——那十条每一条都踩过。

---

## 已知限制

| # | 限制 | 说明 |
|---|---|---|
| 1 | 中止时货物可能仍在车上 | 中止在运输段需人工确认货物状态 |
| 2 | 单机单任务 | 多任务靠网页侧串行下发，无队列调度 |
| 3 | 搬货是假的 | `cargo_node` 只置标志，接真实机构要改这个文件 |
| 4 | 择向是规则式 | `velocity_arbiter` 不做路径规划，复杂障碍会犹豫 |
| 5 | 恢复行为较慢 | `progress_checker` 放宽到 0.20 m/30 s，普通路段卡住要 20 秒才触发 |
| 6 | 只有仿真验证 | 真实底盘、真实顶升机构未接 |
