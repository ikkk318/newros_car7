# 架构

> 本文件说明**为什么这样分层、各层边界在哪、哪些约束一改就坏**。
> 接口与参数的数值不在这里（见 `INTERFACES.md`），启动命令不在这里（见 `OPERATIONS.md`）。

平台：Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11（仿真优先）。
工作区：标准 `src/` 布局，10 个包。

---

## 1 一句话定位

顶升式（Kiva 型）搬运 AGV：车开到货架 → **到点即顶升** → 导航运输 → 到放货点 → 下降。

当前阶段只把「到点导航 + 统一指令 + 任务/状态上报」这条主干跑通、跑稳；
搬货动作只留一个服务口子（`/agv/cargo`），里面暂不做真实动作、立即返回成功。
**不追求量化精度，也不做精定位/二次对位/货物跟随**——那些方案在早期存在过，现已从代码删除。

三个核心取舍：

1. **统一指令总线**——所有入口都只发 `/agv/command`，由 `task_manager_node` 一门受理。
   校验只有一处，状态只有一处可查，指令不会互相打架。
2. **搬货只留一个服务口子**——任务层不碰硬件、只调 `/agv/cargo`。
   "换硬件" = 改 `agv_lift/cargo_node.py` 一个文件，主干一行不改。
3. **速度在硬件边界收口**——上层只发 `/cmd_vel`（想走多快），
   由 `velocity_arbiter` 统一避障与限幅后才发 `/cmd_vel_raw`。限速与防撞只有一处实现。

---

## 2 分层与数据流

```
 入口层   tools/agv_console.py（终端）        agv_web（网页网关 + 静态前端）
              │ 发 /agv/command                    │ 发 /agv/command、/agv/web_teleop
              │ 直调服务                            │ 直调服务、起停栈
              ▼                                     ▼
 任务层   ┌──────────────────────────────────────────────────┐
          │ task_manager_node                                 │
          │  CommandRouter（门卫）+ TaskManager（搬运状态机）  │
          │  + GotoController + NavRunner + SafetyMonitor      │
          │  + GoalValidator（目标点唯一判据）                 │
          └──┬────────────────────┬──────────────────────────┘
             │ Nav2 动作           │ 服务调用
             ▼                     ▼
 服务层   Nav2 栈              cargo_node          waypoint_node
         AMCL/规划/控制         唯一硬件对接点       /agv/store 统一存储
             │ /cmd_vel           /agv/cargo         YAML 持久化
 硬件边界    ▼                     │
       ┌───────────────────────────────────────┐
       │ velocity_arbiter  ← /scan             │ 近障减速 + 择空 + 缓回 + 后方保护 + 限幅
       │ /cmd_vel ─────────► /cmd_vel_raw      │ → Gazebo 差速插件（实车：下位机）
       └───────────────────────────────────────┘
```

**建图模式是另一条链**：没有 arbiter、没有 task_manager，
`web_gateway`（或键盘）直接把速度发 `/cmd_vel_raw`，`slam_toolbox` 消费 `/scan` 建图。

---

## 3 包职责

| 包 | 职责 | 关键模块 |
|---|---|---|
| `agv_interfaces` | 接口定义，3 msg + 3 srv | `AgvCommand`、`TaskState`、`NamedPose`、`AgvStore`、`AgvQuery`、`CargoCmd` |
| `agv_common` | **纯逻辑**，不依赖 ROS 运行时（可单独测） | `geometry`、`goal_check`、`map_check`、`map_manager`、`reactive_avoid`、`limits`、`config_loader`、`web_guard` |
| `agv_description` | 车体模型、世界、仿真 launch | `urdf/*.xacro`、`worlds/world666.world`、`launch/sim.launch.py` |
| `agv_navigation` | Nav2 参数、建图/导航 launch、包内地图 | `launch/{mapping,navigation}.launch.py`、`params/nav2_params.yaml` |
| `agv_bringup` | 参数中心 + 业务栈总装 | `config/default/*.yaml`、`launch/agv_stack.launch.py` |
| `agv_waypoint` | 点位/路线存储 | `waypoint_node`、`store_dispatch`（纯逻辑分发）、`waypoint_store`、`route_store` |
| `agv_task` | 任务层（最厚） | `task_manager_node`、`command_router`、`task_manager`、`task_flow`、`nav_runner`、`goto_controller`、`goto_pose`、`slot_approach`、`safety_monitor`、`state_reporter`、`store_client`、`cargo_client` |
| `agv_lift` | 搬货服务 | `cargo_node`（唯一硬件口子） |
| `agv_hardware` | 速度仲裁 | `velocity_arbiter` |
| `agv_web` | 网页控制台 | `gateway/`（见 `WEB_CONSOLE.md`）、`static/`、`stack_manager`、`status_board`、`log_translate`、`grid_image` |

`agv_common` 是全项目的"纯逻辑底座"：几何、判据、避障策略、参数加载、鉴权都在这里，
所以这些模块**不需要 ROS 也能跑**——这是它们能被反复验证的原因。

---

## 4 节点职责

| 节点 | 包 | 订阅 | 发布 | 提供 | 调用 |
|---|---|---|---|---|---|
| `waypoint_node` | `agv_waypoint` | — | — | `/agv/store` | — |
| `cargo_node` | `agv_lift` | — | `/agv/cargo/busy` | `/agv/cargo` | — |
| `task_manager_node` | `agv_task` | `/agv/command`、`/agv/web_teleop`、`/cmd_vel`、`/agv/manual_mode` | `/agv/task_state`、`/cmd_vel`（急停/转发） | `/agv/query` | `/agv/store`、`/agv/cargo`、`/navigate_to_pose` |
| `velocity_arbiter` | `agv_hardware` | `/cmd_vel`、`/scan` | `/cmd_vel_raw` | — | — |
| `web_gateway` | `agv_web` | `/map`、`/scan`、`/plan`、`/agv/task_state`、`/cmd_vel`、`/rosout`、TF | `/agv/command`、`/agv/web_teleop`、`/cmd_vel_raw`（建图）、`/initialpose` | HTTP/WS | `/agv/store`、`/agv/cargo`、`/agv/query` + 起停栈 |
| `agv_console`（脚本） | `tools/` | `/agv/task_state`、`/agv/cargo/busy` | `/agv/command`、`/cmd_vel`、`/agv/manual_mode` | — | 同上网关的业务服务 + 起整栈 |

---

## 5 指令总线：门卫的五道关

`CommandRouter.handle()` 的判定顺序**不能换**：

1. **`stop` 快通道**——白名单与去重之前直接执行。安全通道必须永远可用。
2. **白名单**——`goto`/`start_task`/`cancel_task`/`stop`/`resume`，其余一律 `未知指令「X」`。
3. **参数检查**——`goto` ≥1 个、`start_task` ≥2 个；点名非空；坐标可解析。
4. **坐标合法性**——交给 `GoalValidator`（有地图用地图，没地图用 `field_*` 兜底）。
5. **`request_id` 去重** → **`_can_start()` 前置检查**（急停 / 手动 / 已有任务 / 状态机不空闲）。

设计要点：**拒绝不抛异常、不回复**，只留一行 `[指令] … → 拒绝：<原因>`。
网页侧靠 `on_rosout` 抓这行原文，任务受理失败时如实转述——**不要臆测原因**。

---

## 6 搬运流程与两段式进槽

```
start_task(A, B)
  → GO_LOAD: 导航到 A
        └─ 若 A 是槽内点且 slot_approach=true：先到「X号货架」对位 → 再直行进槽
  → LIFTING: /agv/cargo lift
  → TRANSPORT: （路线各点）→ 导航到 B
        └─ 同样两段式
  → LOWERING: /agv/cargo lower
  → DONE → IDLE
```

**为什么槽内要两段式**（这是几何决定的，不是策略偏好）：
凵型槽净宽 0.80 m，车体 0.50×0.50（半对角 0.354 m），四周只剩 0.19 m——**原地转身物理上做不到**。
而 Nav2 的 `SimpleGoalChecker` 要求航向误差 ≤0.20 rad，转不动就判无进展 → 恢复行为 → 越转越偏 → 超时。
实测进槽到位后航向差 138°。

所以：**纠偏挪到槽外**（槽口对位），槽内最后 1.37 m 是直的；槽内到点判据以位置为主
（0.12 m / 0.35 rad / 稳定 0.6 s），判到就 `cancelTask()` 再搬货。

**软到点必须带 hold**：0.5 m/s 的车穿过 0.12 m 的圈只要 0.48 s，不要求"连续 0.6 s 在圈内"
就会路过即到点。判到后必须 `cancelTask()`（否则顶升与转动叠加）+ 短暂 `sleep`。

配套：`progress_checker` 不能是 0.5 m / 10 s（槽内每点位移都小于它），现为 0.20 m / 30 s。

---

## 7 速度链路与四道安全

```
Nav2 controller → cmd_vel_nav → velocity_smoother → /cmd_vel ─┐
task_manager 急停/超时零速 ────────────────────────────────────┼─► velocity_arbiter ─► /cmd_vel_raw
网页遥控（建图）→ /cmd_vel_raw                                  │      ↑ /scan
网页遥控（导航）→ /agv/web_teleop → task_manager → /cmd_vel ───┘
```

四道安全，各管一段，**不要互相代劳**：

| # | 机制 | 管什么 | 阈值 |
|---|---|---|---|
| 1 | `SafetyMonitor` | 速度心跳超时 → 补零速；急停 | `cmd_timeout_ms=500` |
| 2 | `ControlLease` | 控制权归属（谁在遥控） | `lease_idle_sec=120` |
| 3 | `TeleopWatchdog` | 没新指令就发零速 | `teleop_stale_sec=0.35` |
| 4 | `velocity_arbiter` | 近障减速 / 刹停 / 择向 / 限幅 | `stop=0.30`、`slow=0.55`、`take=0.45` |

★ **停车和还锁是两件事**：看门狗只负责"没新指令发零速"，**绝不能**调 `lease.force_release()`。
曾经在 `on_tick` 里这么写，等于把 120 秒的控制权降级成 0.35 秒——用户松手就掉锁（实测 4 秒被收回）。

★ **`/agv/web_teleop` 必须有人订阅**。导航模式网页把速度发到这里，由 `task_manager_node` 校验后转发；
全项目曾经没有任何订阅者，导致"按方向键车一动不动、零报错"。这是导航遥控的唯一入口，
也是急停/手动互锁的唯一闸门。

---

## 8 线程与并发模型

`task_manager_node` 里有四个上下文，**共享状态必须明确归属**：

| 上下文 | 谁在里面跑 | 注意 |
|---|---|---|
| rclpy 执行器线程 | 所有 ROS 回调（`on_command`/`on_web_teleop`/定时器） | 回调里不做长阻塞 |
| 任务后台线程 | `TaskFlow.run_*`（导航、搬货、等状态） | 一次只允许一个（`_runner.is_alive()` 拒第二个） |
| aiohttp 事件循环（`agv_web`） | HTTP handler、WebSocket | handler 里的阻塞调用要 `run_in_executor` |
| 栈子进程（`StackRunner`） | `ros2 launch` 进程组 | stdout 由独立线程 `_pump_output` 读取 |

★ **受理即建单**：`handle_start_task` 里**当场** `manager.start(pick, drop)`（纯状态机，微秒级），
再 `_spawn` 线程。以前 start 放在后台线程、两次查点之后才跑（服务未就绪时最长 12 秒），
`task_id` 迟迟不生成，网关 6 秒等不到就判"没能启动"并重发 → 三连失败。

★ **中止绝不能落 ERROR**：`ERROR` 是 `_can_start()` 的硬拒绝态，落了就"之后既不能终止也不能启新任务"。
中止走 `_stop_quietly()`（回 IDLE + 停车 + 上报）。

★ **`waitUntilNav2Active()` 必须可中止**：它阻塞 3~10 秒，期间点中止是空操作，
就绪后照样 `goToPose` → "日志说停车了车又开出去"。

---

## 9 启动链

```
web_console.launch.py（唯一网页入口，只起网关）
   └─ 网关按网页按钮 Popen：
      ├─ 导航：agv_stack.launch.py with_sim:=true
      │     ├─ waypoint_node / cargo_node / task_manager_node / velocity_arbiter
      │     └─ include navigation.launch.py
      │           ├─ include sim.launch.py（Gazebo + spawn + EKF + RViz）
      │           └─ include nav2_bringup
      └─ 建图：mapping.launch.py
            ├─ include sim.launch.py
            ├─ include slam_toolbox
            └─ waypoint_node（草稿点位）
```

`agv_stack.launch.py` 也可以单独跑（终端控制台就是这么用的）。

★ **换栈必须五步清干净**（`StackRunner.stop()`）：
① SIGINT 整组 → ② 等整组死透 → ③ 不死再 SIGKILL → ④ 清 `/dev/shm` 里**无主**的 `fastrtps_*` → ⑤ 等 11345 端口空出。

原因：`proc.wait()` 只等 launch 父进程，`gzserver` 是兄弟进程还占着 11345，bind 失败是**静默**的
（日志照打，世界里没数据）；Ctrl-C 打断的节点死在 `destroy_node()`，
FastDDS 的 SHM 段没 unlink，新参与者发现不到老参与者——症状是"节点全在、话题全在，彼此收不到"。

★ **全链路关 SHM**：`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`，设在
`web_gateway.main()`（`rclpy.init()` **之前**）、`StackRunner._child_env()`、两个 launch 的 `additional_env`。

---

## 10 位姿与地图缓存

★ **tf2 Buffer 缓存 10 秒历史**：发布者死后 `lookup_transform` **不报错**，仍返回最后一个变换（stamp 很旧）。
判位姿失效**必须看 stamp 年龄**，不能只看能不能查到。

★ 换栈时 `tf_buffer.clear()` + 位姿/扫描/路径/地图全部作废，不能指望"被新数据覆盖"（没数据就永远不覆盖）。

★ **`map_epoch`（换栈才变）≠ `map_version`（建图 0.8 s 一变）**。混用会让前端每次建图更新都重载 PNG。
epoch 一个动作只能 +1（谁判断"模式真变了"谁负责清）。

★ **中文地图名不能直接交给 map_server**：走完 `agv_stack → navigation → nav2_bringup` 后
中文路径会变成 `u5730u56FE…`（`\uXXXX` 转义且反斜杠丢失）→ `bad file` → bringup 中止
→ 没有 `/map` → 网页全黑只剩起始点。传 launch 的 `map_yaml` 必须过
`MapManager.launch_map_yaml()`（非 ASCII 时给 `maps/.launch/<sha1>/map.yaml` + 软链）。

---

## 11 日志两条通道

同一件事会进来两遍，去重规则见 `WEB_CONSOLE.md` §6。这里只记设计约束：

- 通道 ①：子进程 stdout → `_pump_output` → `log_translate` → `log()`（带 `[节点名] ` 前缀）
- 通道 ②：`/rosout` → `on_rosout`（无前缀，但知道"顶升成功=绿"这种语义）

`_norm_log()` 只剥"身份前缀"，`[顶升] `、`[1/5] ` 这类**语义前缀必须保留**，否则不同事件会被折叠成一条。
重复时按 `_LEVEL_RANK` 合并成更醒目的级别，不能直接丢。

---

## 12 编码约束（一改就坏的十条）

1. **`_gw_lock` 是非重入 `Lock`**，`log()` 内部会取锁 → 锁内只改字段，绝不调 `log()`。
2. **`MODE_TEXT[mode]` 是故意直接下标**（非法模式要抛 `KeyError`），不许改 `.get()`。
3. **`api_tasks` 的 add 分支不做 `or 1` 归一化**，与 replace/`_load_tasks` 故意不同。
4. **`main()` 必须在 `rclpy.init()` 之前设 FASTDDS 环境**，反了 transport 已固定。
5. **mixin 不得定义 `__init__`**（会吞掉 `Node.__init__`），`super().__init__` 只调一次。
6. **前端顶层符号只能声明一次**（重复 `const` 整页白屏）。
7. **`install/` 是普通拷贝**，改 Python/launch/static 都要 `colcon build` 才生效。
8. **`ROS_DOMAIN_ID` 上限 232**（233 FastDDS 直接退）。
9. **从脚本启动 ROS 要洗 PATH**：受管 Python 排在 `/usr/bin` 前会让 `spawn_entity.py` 找不到
   ROS 的 numpy（cp310）→ 机器人根本没生成。`export PATH=/usr/bin:/bin:/usr/sbin:/sbin && unset PYTHONPATH` 再 source。
10. **`/tmp` 在本机不可靠**：日志、备份别放那。

---

## 13 依赖方向

```
agv_interfaces  ←（所有 ROS 节点）
agv_common      ← 所有包（纯逻辑，不反向依赖任何包）
agv_task        → agv_common、agv_interfaces
agv_waypoint    → agv_common、agv_interfaces
agv_lift        → agv_interfaces
agv_hardware    → agv_common、agv_interfaces
agv_web         → agv_common（map_manager、web_guard）、agv_interfaces
agv_bringup     → 参数 + launch（不写代码逻辑）
```

`agv_common` **不得** import 任何其它 agv 包；`agv_web` 不得 import `agv_task`（两者只通过话题/服务通信）。

---

## 14 技术债（已知、暂不修）

| # | 项 | 影响 |
|---|---|---|
| 1 | 中止时货物悬挂未处理 | 中止在 `TRANSPORT` 时货物仍在车上，需人工确认 |
| 2 | 网页口令 `agv2024` 写死在 launch 示例 | 部署前必须换 |
| 3 | `velocity_arbiter` 的择向策略是规则式，不是规划 | 复杂障碍会来回犹豫 |
| 4 | 单机单任务，无队列调度 | 多任务靠网页侧串行下发 |
| 5 | `progress_checker` 放宽到 0.20 m/30 s | 普通路段真卡住会晚 20 秒才触发恢复 |
