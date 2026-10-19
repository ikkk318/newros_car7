# 接口契约（话题 / 服务 / 参数 / 文件格式）

> 本文件是**接口事实的唯一真源**。字段名、话题名、默认值、状态串、回执文案都逐个核对过源码。
> 其它文档只写"见 INTERFACES §x"，不复制数值。
>
> 核对基线：`d8b5831` + 本次重构。自定义接口共 **3 个消息 + 3 个服务**（`src/agv_interfaces/`）。

---

## 1 总览

```
   控制台 agv_console ──┐
   网页 / 语音 / 调度 ──┴──► /agv/command (AgvCommand) ──► task_manager_node
                                                          │  门卫 CommandRouter
   网页遥控（导航模式）──► /agv/web_teleop (Twist) ────────►│  + 搬运状态机
                                                          │  + GotoController
   /agv/manual_mode (Bool) ───────────────────────────────►│  + SafetyMonitor
                                                          │  + GoalValidator
                          ┌───────────────────────────────┘
                          ├──► /agv/task_state (TaskState)   1 Hz + 变化即发
                          ├──► /cmd_vel  ──► velocity_arbiter ──► /cmd_vel_raw ──► 底盘
                          └──► 服务 /agv/store  /agv/cargo  /agv/query
```

四条链路的分工：

| 链路 | 途径 | 特点 |
|---|---|---|
| 指令下发 | `/agv/command`（话题） | 广播式，**不回答**；受理与否只体现在日志与 `/agv/task_state` |
| 状态上报 | `/agv/task_state`（话题） | 1 Hz 周期 + 状态变化时立即发 |
| 存储与查询 | `/agv/store`、`/agv/cargo`、`/agv/query`（服务） | 一问一答，每个响应都有 `ok` 与中文 `message` |
| 导航执行 | `/navigate_to_pose`（Nav2 动作） | 任务层内部调用，不是对外接口 |

---

## 2 话题

### 2.1 业务自定义话题（7 个）

| 话题 | 类型 | 方向 | 发布者 | 订阅者 | 频率 | 说明 |
|---|---|---|---|---|---|---|
| `/agv/command` | `AgvCommand` | 入 | 控制台、网页、语音、调度 | `task_manager_node` | 事件 | 统一指令总线，白名单 5 条（§5） |
| `/agv/task_state` | `TaskState` | 出 | `task_manager_node` | 控制台、网页 | 1 Hz + 变化 | 状态、进度、中文提示 |
| `/agv/web_teleop` | `Twist` | 入 | `web_gateway`（导航模式） | `task_manager_node` | 10 Hz | ★ **导航模式网页遥控的唯一入口**：经互锁校验后转 `/cmd_vel`。建图模式网关直接发 `/cmd_vel_raw`，不走本话题 |
| `/cmd_vel` | `Twist` | 双向 | Nav2 `velocity_smoother`、`task_manager_node`（急停零速）、`task_manager_node`（转发遥控） | `velocity_arbiter` | 20 Hz | 上半层"想走的速度" |
| `/cmd_vel_raw` | `Twist` | 出 | `velocity_arbiter`（导航）/ `web_gateway`（建图） | Gazebo 差速插件（实车：下位机） | 20 Hz | ★ **硬件边界**：真正驱动底盘 |
| `/agv/manual_mode` | `Bool` | 入 | 控制台 | `task_manager_node` | 事件 | `true`=手动，期间拒导航（互锁） |
| `/agv/cargo/busy` | `Bool` | 出 | `cargo_node` | 控制台、网页 | 变化即发 + 1 Hz | `true`=正在搬货 |

> ★ **建图模式没有 arbiter**，网关把速度直发 `/cmd_vel_raw`；导航模式有 arbiter，走 `/agv/web_teleop`。
> 分流逻辑在 `stack_manager.teleop_topic_for()`。

### 2.2 仿真 / 导航栈提供的话题（本项目只消费）

| 话题 | 类型 | 发布者 | 频率 | 说明 |
|---|---|---|---|---|
| `/scan` | `LaserScan` | Gazebo 射线插件 | 15 Hz | 360° 全向（361 点），量程 0.05–8.0 m，`frame=laser` |
| `/odom` | `Odometry` | Gazebo 差速插件 | 50 Hz | 轮式里程计 |
| `/imu` | `Imu` | Gazebo IMU 插件 | 100 Hz | 角速率用于纠偏 |
| `/joint_states` | `JointState` | Gazebo 关节插件 | 50 Hz | 补全左右轮 TF |
| `/tf`、`/tf_static` | `TFMessage` | rsp / AMCL / EKF / 差速插件 | — | `map→odom→base_footprint→base_link→laser/imu_link` |
| `/map` | `OccupancyGrid` | `map_server`（导航）或 `slam_toolbox`（建图） | — | 定位/规划栅格图 |
| `/robot_description` | `String` | `robot_state_publisher` | — | `spawn_entity.py` 取 URDF 的来源 |

---

## 3 服务（3 个）

| 服务 | 类型 | 提供者 | 调用者 | 请求 | 响应 |
|---|---|---|---|---|---|
| `/agv/store` | `AgvStore` | `waypoint_node` | `task_manager_node`、控制台、网页 | `resource, action, name, x, y, yaw, kind, point_names[]` | `ok, message, items[], names[], counts[], count` |
| `/agv/cargo` | `CargoCmd` | `cargo_node` | `task_manager_node`、控制台、网页 | `command, request_id` | `ok, busy, message` |
| `/agv/query` | `AgvQuery` | `task_manager_node` | 控制台、网页 | `what` | `ok, state, x, y, yaw, message` |

### 3.1 `/agv/store` — 点位 / 路线统一存储

`resource` 与 `action` 先 `strip().lower()`，**大小写与空格不敏感**；非法值返回 `ok=false` 不抛异常。

| 字段 | 类型 | 含义 |
|---|---|---|
| `resource` | string | `waypoint` / `route` |
| `action` | string | `save` / `get` / `list` / `delete` |
| `name` | string | 资源名（`list` 忽略） |
| `x` `y` `yaw` | float64 | 仅 `waypoint`+`save` |
| `kind` | string | 仅 `waypoint`+`save`：`cargo` / `nav`，**留空按 cargo** |
| `point_names[]` | string[] | 仅 `route`+`save`：途经点名，至少 1 个 |

响应字段的有效性矩阵：

| resource + action | `items` | `names` | `counts` | `count` |
|---|---|---|---|---|
| `waypoint` + save | — | — | — | 0 |
| `waypoint` + get | 1 项 | — | — | 1 |
| `waypoint` + list | **全部点位** | 点名 | 全 1 | 条目数 |
| `waypoint` + delete | — | — | — | 0 |
| `route` + save | — | — | — | 写入途经点数 |
| `route` + get | N 项 | — | — | 途经点数 |
| `route` + list | **空** | 路线名 | 各路线点数 | 条目数 |
| `route` + delete | — | — | — | 0 |

> ⚠️ `waypoint`+`list` 的 `items` **有值**，`route`+`list` 的 `items` **为空**（只给名字与点数）。
> 要路线的坐标，对每条路线再发一次 `route`+`get`。

动作语义：

| 动作 | 语义 | 规则 |
|---|---|---|
| `save` | 存 / 重名覆盖 | `route` 按点名从点位表**复制快照**，之后改名删点不影响已存路线；点名有任一不存在则**整条拒绝**，不写半条 |
| `get` | 按名查一个 | 不存在 → `ok=false`，不报错 |
| `list` | 列出全部 | 按名 `sorted()` 排序，可复现 |
| `delete` | 删一个 | 不存在 → `ok=false`，不报错 |

主要回执（`message`）：

| 情况 | ok | message |
|---|---|---|
| waypoint save / get / list / delete | ✓ | `已保存点位「X」（cargo）` / `点位「X」已找到` / `共 N 个点位` / `已删除点位「X」` |
| route save | ✓ | `已保存路线「X」，共 N 个途经点` |
| route save 点名不存在 | ✗ | `这些点位不存在：A、B` |
| 未知资源 / 动作 | ✗ | `未知资源「X」，应为 waypoint 或 route` / `未知动作「X」，应为 save / get / list / delete` |
| 名单为空 | ✗ | `点名不能为空` / `路线名不能为空` |

`task_manager_node` 内部用它做"点名→坐标"（`store_client.py`），服务 3 s 不可用即按"查点失败"处理并取消导航。

### 3.2 `/agv/cargo` — 顶升 / 下降

★ 全项目**唯一**与搬货机器打交道的地方，也是接真实硬件的唯一改动点。当前只维护 `busy` 标志并立即返回成功。

| 语义 | 接受的写法 | 响应 |
|---|---|---|
| 顶升 | `lift` / `up` / `顶升` | `ok=true, busy=true, "已顶升，开始搬货"` |
| 下降 | `lower` / `down` / `放货` / `下降` | `ok=true, busy=false, "已下降，放货完成（空闲）"` |
| 其它 | — | `ok=false, "未知命令「X」：只支持 lift（顶升）/ lower（下降）"` |

受理后立即向 `/agv/cargo/busy` 广播。任务层等待上限 `cargo_timeout_s`（5 s），超时按失败处理并置 `ERROR`。

### 3.3 `/agv/query` — 运行时状态 / 位姿

| `what` | 行为 | 典型 message |
|---|---|---|
| `pose` | 查 `map→base_footprint` TF | `当前位姿 x=1.00 y=2.00 yaw=0.00`；失败 `ok=false`，`暂时取不到位姿（定位/TF 未就绪）` |
| 其它任何值 | 读状态机 | `GO_LOAD（前往取货点）目标 1号货架 进度 15%，导航 IDLE` |

★ 只答运行时状态。点位表 / 路线表走 `/agv/store`，**不在这里查**（旧取值 `waypoints`/`routes` 已删除，传了不报错但会按 `state` 返回）。

---

## 4 消息字段

### 4.1 `AgvCommand`

| 字段 | 类型 | 说明 |
|---|---|---|
| `source` | string | `web` / `voice` / `scheduler` / `console` |
| `command` | string | 白名单：`goto` / `start_task` / `cancel_task` / `stop` / `resume` |
| `args` | string[] | 点名，或坐标 `[x,y]` / `[x,y,yaw]`（按字符串传） |
| `route` | string | 可选路线名，**空串=不用路线** |
| `confidence` | float32 | 语音 0~1，其它填 1.0 |
| `request_id` | string | 去重与追踪用，**空串=匿名不去重** |

### 4.2 `TaskState`

| 字段 | 类型 | 说明 |
|---|---|---|
| `task_id` | string | `task-0001`；**单点导航为空串** |
| `state` | string | 见 §6 |
| `target_location` | string | 当前目标点名；坐标导航为 `(x.xx, y.yy)` |
| `progress` | float32 | 0~1 |
| `message` | string | 可直接显示的中文 |

### 4.3 `NamedPose`

用在 `/agv/store` 的 `items[]`，**不在任何话题上传输**。

| 字段 | 说明 |
|---|---|
| `name` | 点名；路线里是快照时的点名 |
| `x` `y` `yaw` | map 坐标（m）与朝向（rad，逆时针为正） |
| `kind` | `cargo` / `nav`；**路线途经点为空串** |

> 为什么不用 5 个平行数组：某个数组短一位就会把 A 点名字配到 B 点坐标上，不报错、车却开错地方。
> 改成 `NamedPose[]` 后这类错位在类型层面就不存在。

---

## 5 指令语义与拒绝规则

### 5.1 白名单

`VALID_COMMANDS = ('goto', 'start_task', 'cancel_task', 'stop', 'resume')`。
**搬货不在总线里**——由任务层直接调 `/agv/cargo`。

| command | args | 语义 |
|---|---|---|
| `goto` | ≥1 | 点名或坐标导航，可带 `route` |
| `start_task` | ≥2 | 取货点 → （路线各点）→ 放货点，中途顶升/下降 |
| `cancel_task` | 忽略 | 取消并停车，状态机回 `IDLE` |
| `stop` | 忽略 | 急停：零速 + 中止线程 + 任务置 `PAUSED` |
| `resume` | 忽略 | 清急停、清任务、回 `IDLE` |

`goto` 的 args 解析：1 个 → **一律当点名**（纯数字也是点名）；2 个 → 坐标 `x,y`（yaw=0）；≥3 → `x,y,yaw`（第 4 个起忽略）。

### 5.2 拒绝规则

拒绝不抛异常、不回复，只留日志 `[指令] <command> from=<source> → 拒绝：<原因>`。
校验顺序：**stop 快通道 → 白名单 → 参数 → 坐标合法性 → 去重 → `_can_start` 前置检查**。

| 规则 | 拒绝串片段 |
|---|---|
| `stop` 无限受理 | —（绕白名单与去重） |
| 不在白名单 | `未知指令「fly」，已拒绝` |
| `goto` 缺参数 | `指令「goto」缺少参数（需目标点名或坐标）` |
| `start_task` 缺参数 | `指令「start_task」缺少参数（需起始点与目标点）` |
| 点名空 | `goto 目标点名不能为空` |
| 坐标非数字 | `goto 坐标格式非法，应为 [x, y] 或 [x, y, yaw]` |
| 坐标越界/压障/净空不足 | 见 §5.3 |
| 同 `request_id` 重复 | `请求「r1」重复，已拒绝` |
| 前置检查不过 | `处于急停状态，请先恢复（resume）` / `当前是手动模式，禁止导航` / `已有任务在执行，请先取消或等它完成` / `搬运状态为 X，无法接新任务` |

### 5.3 目标点合法性：只有地图一个判据

`agv_common.goal_check.GoalValidator` 是**唯一**判据，门卫与预检问的是同一个对象。

| 条件 | 判据 |
|---|---|
| `map_yaml` 可读 | 实测栅格地图：图外拒 / 压障碍拒 / 净空不足拒（`robot_clearance` 默认 0.30 m） |
| `map_yaml` 空 | `field_*` 矩形兜底（只在没地图时生效） |

拒绝串示例：`目标点 (99.00, 99.00) 在地图范围外 (x∈[-10.7, 4.2], y∈[-9.8, 9.2])，已拒绝`、
`目标点 (x,y) 距障碍仅 0.18 m，小于车身所需净空 0.30 m，已拒绝`。
（已知外观瑕疵：门卫会在末尾再拼一次"，已拒绝"，日志会出现重复尾巴，拦截行为正确。）

### 5.4 点位预检

从点位表取出的点在导航前过一遍 `GoalValidator.check()`；不过则 `点位「X」预检未通过：<原因>`，任务失败。
没有"关闭预检"的开关。

### 5.5 去重与前置检查

- 只有 `request_id` 非空才去重；**无论受理与否都记**（防重试风暴）；上限 1000 条，FIFO 淘汰。
- `stop` 绕过去重（安全通道）。
- 测去重的探针必须能通过前面全部校验，用 `cancel_task`（自检就是这么做的）。
- `_can_start()` 顺序：`rclpy.ok()` → 急停 → 手动模式 → 后台线程在跑 → 状态机不在 `IDLE`/`DONE`。
  `cancel_task` / `stop` / `resume` **不做**这些检查。

---

## 6 状态机与状态串

```
IDLE ──start_task──► GO_LOAD ──► LIFTING ──► TRANSPORT ──► LOWERING ──► DONE ──► IDLE
                       │            │            │             │
                       └────────────┴────────────┴─────────────┴──► ERROR
                       任何活动状态 ──stop──► PAUSED ──resume──► IDLE
```

`SEQUENCE = ('GO_LOAD','LIFTING','TRANSPORT','LOWERING','DONE')`。**没有 ALIGN 阶段**——到点即顶升。
单点导航只用 4 态：`IDLE → GO_LOAD → DONE`，失败 `ERROR`。

| state | progress（搬运） | message（搬运） | progress（单点） | message（单点） |
|---|---|---|---|---|
| `IDLE` | 0.0 | 空闲，等待任务 | 0.0 | 空闲，等待任务 |
| `GO_LOAD` | 0.15 | 前往取货点 | **0.5** | 导航中 |
| `LIFTING` | 0.40 | 顶升中 | — | — |
| `TRANSPORT` | 0.60 | 运输中 | — | — |
| `LOWERING` | 0.85 | 下降放货中 | — | — |
| `DONE` | 1.0 | 搬运完成 | 1.0 | 导航完成 |
| `ERROR` | 0.0 | 任务异常，需人工复位 | 0.0 | 导航失败 |
| `PAUSED` | 0.0 | 已暂停（急停/停止） | 0.0 | — |

> ⚠️ 同一个 `GO_LOAD` 两套进度（0.15 / 0.5）来自两张表。用 `task_id` 是否为空串区分来源。

`publish_state()` 组装优先级（`state_reporter.compose()`）：
搬运态非 IDLE → 报搬运；否则导航态非 IDLE → 报导航；都空闲 → IDLE；**最后**若急停则强制覆盖为 `PAUSED`，
`message='已急停（安全停止）'`（急停显示优先于一切）。

---

## 7 参数

参数中心在 `src/agv_bringup/config/`：`default/` 打底，`param_profile` 指定的档位覆盖。
**`param_profile` 默认空 = 只用 default**；实车差异写 `config/real/*.yaml` 后传 `param_profile:=real`。

### 7.1 `task.yaml`（task_manager_node）

| 参数 | 默认 | 作用 |
|---|---|---|
| `field_min` / `field_max` | -10.70 / 4.15 | **兜底**坐标范围 x（仅无地图时生效） |
| `field_min_y` / `field_max_y` | -9.83 / 9.22 | 兜底 y |
| `waypoint_frame` | `map` | 目标坐标系 |
| `nav_print_period` | 1.0 | 导航反馈打印周期（s） |
| `nav_timeout_s` | 180.0 | **单段**导航超时（s） |
| `cargo_timeout_s` | 5.0 | 搬货服务等待上限（s） |
| `robot_clearance` | 0.30 | 目标点所需净空（m） |
| `map_yaml` | `""` | 判据地图；留空退化为 `field_*`（由 launch 注入） |

### 7.2 代码内声明、未写入 yaml 的 4 个参数（`task_manager_node.py`）

| 参数 | 默认 | 作用 |
|---|---|---|
| `slot_approach` | `True` | 槽内点位两段式进槽：先到「X号货架」对位再直行进槽；关掉则退回直接进槽 |
| `slot_arrive_xy` | 0.12 | 槽内软到点位置容差（m） |
| `slot_arrive_yaw` | 0.35 | 槽内软到点角度容差（rad ≈20°） |
| `slot_arrive_hold` | 0.6 | 判到点需连续稳定（s）★ 不设 hold 会"路过即到点" |

### 7.3 `safety.yaml`

| 参数 | 默认 | 作用 |
|---|---|---|
| `cmd_timeout_ms` | 500 | 多久没新 `/cmd_vel` 判超时停车（ms） |
| `estop_budget_ms` | 200 | 急停后发停车的时限（ms），用于达标判定 |

### 7.4 `cargo.yaml`

| 参数 | 默认 | 作用 |
|---|---|---|
| `service_name` | `/agv/cargo` | 搬货服务名 |
| `busy_topic` | `/agv/cargo/busy` | 搬货中标志 |
| `publish_period` | 1.0 | 标志补发周期（s，内部下限 0.1） |

### 7.5 `waypoint.yaml`

| 参数 | 默认 | 作用 |
|---|---|---|
| `waypoint_file` | `/tmp/agv_waypoints.yaml` | 点位文件（控制台加载地图时覆盖） |
| `routes_file` | `/tmp/agv_routes.yaml` | 路线文件（同上） |
| `frame_id` | `map` | 坐标系（文件里有则以文件为准） |

### 7.6 `hardware.yaml`（velocity_arbiter，共 18 个）

| 参数 | 默认 | 作用 |
|---|---|---|
| `stop_distance` | 0.30 | 距障 ≤ 此值直接刹停（防撞底线，勿动） |
| `slow_distance` | 0.55 | 距障 ≤ 此值开始按比例减速 |
| `take_over_distance` | 0.45 | 距障 ≤ 此值且低速时进入"择向接管" |
| `takeover_max_speed` | 0.25 | 进入接管所需车速上限（m/s） |
| `slow_min_speed` | 0.10 | 减速过渡段最小线速度（m/s） |
| `front_half_angle_deg` | 45.0 | 前方扇区半张角（近障减速，宽覆盖） |
| `takeover_half_angle_deg` | 20.0 | ★ 接管闸门锥半张角，**别开大**：45° 会扫到槽侧壁（近侧壁距车心 0.250，45° 命中 0.354 ≤ 0.45）→ 闸门被骗开、槽内被反复夺向。解析临界 27°，实测 30~35° |
| `side_max_angle_deg` | 90.0 | 侧方统计到此角为止 |
| `rear_half_angle_deg` | 20.0 | ★ 后方扇区（倒车看车尾），**别开大**：±25° 会擦到槽侧壁（0.54 m）→ "能进不能出"复发 |
| `yaw_handback_rate` | 0.30 | 远离障碍后角速度交还步长（rad/s/次） |
| `max_linear` / `max_angular` | 0.5 / 1.0 | 最终速度上限 |
| `scan_topic` / `cmd_in` / `cmd_out` | `/scan` / `/cmd_vel` / `/cmd_vel_raw` | 话题接线 |
| `cmd_timeout` | 0.5 | 上层速度超时即输出零速 |
| `scan_timeout` | 0.5 | 激光超时则原样放行（只限幅） |
| `publish_rate` | 20.0 | 仲裁输出频率（Hz，内部下限 1.0） |

### 7.7 launch 参数（`agv_bringup/agv_stack.launch.py`）

| 参数 | 默认 | 作用 |
|---|---|---|
| `param_profile` | `""` | 档位目录；**勿改名 `profile`**（会被 gzserver 当 `--profile` 吃掉） |
| `with_sim` | `false` | 是否同时拉起导航栈（Gazebo + AMCL + Nav2 + RViz） |
| `gui` / `rviz` | `true` / `true` | 窗口开关 |
| `world_file` | `""` | 世界文件；**勿改名 `world`**（空默认值会顶掉导航栈默认世界） |
| `map_yaml` | `""` | 所选地图 yaml，同时决定预检用的地图；留空兜底 `share/agv_navigation/maps/world666map/world666map.yaml` |
| `waypoint_file` / `routes_file` | `""` | 与地图绑定的点位/路线；`routes_file` 留空则用点位同目录的 `routes.yaml` |

业务节点 `use_sim_time = (with_sim == 'true')`——不带 Gazebo 时用系统时钟，否则没有 `/clock` 定时器不触发。

---

## 8 数据文件格式

点位/路线/任务**与地图绑定**：一份地图一个目录，删地图一起删，不留孤儿。

```
<工作区>/maps/<地图名>/            （地图库根可用 AGV_MAPS_DIR 覆盖）
   ├── <地图名>.pgm              栅格图
   ├── <地图名>.yaml             元数据（分辨率/原点）
   ├── waypoints.yaml            这份地图专属点位
   ├── routes.yaml               这份地图专属路线
   └── tasks.yaml                任务缓冲（控制台按需生成，节点不读）
```

当前地图库两张：`world666map`（初始，15 个点位 + 2 条路线）与 `地图10月19日16时`（实测新建，只有起始点）。

### 8.1 `waypoints.yaml`

```yaml
frame_id: map
waypoints:
  1号货架: {kind: cargo, x: 1.63, y: 2.66, yaw: 0.0}
  1号槽内: {kind: cargo, x: 3.0, y: 2.692, yaw: 0.0}
  1点:     {kind: nav,   x: 0.0, y: 0.0, yaw: 0.0}
```

- 必须同时含 `x/y/yaw` 且能转 float，否则**整条丢弃**（不让节点崩）
- `kind` 缺失或非法 → 按 `cargo`
- 文件有 `frame_id` 则以文件为准；文件不存在或解析失败 → 空表不报错
- 写回是整体重写、`sort_keys=True`、中文不转义，**只保留 `x/y/yaw/kind` 四个字段**（手工加的其他字段会被丢掉）

### 8.2 `routes.yaml`

```yaml
frame_id: map
routes:
  1号:
    points:
      - {name: 1点, x: 0.0, y: 0.0, yaw: 0.0}
      - {name: 2点, x: -7.0, y: 2.4, yaw: 0.0}
```

坐标是**存路线那一刻从点位表复制的快照**。清洗后为空的路线不入表。

### 8.3 `tasks.yaml`

```yaml
tasks:
- {pick: 1号货架, drop: 2号货架, loops: 3, route: 1号}
```

`loops` 读取时下限 1；`route` 空串=不用路线。控制台按 `loops` 发 `start_task`，等回 `IDLE`/`DONE` 再发下一条。

---

## 9 可直接复制的示例

前提：`source install/setup.bash` 且业务节点已起。

```bash
# 存点位（cargo 搬货点 / nav 导航点）
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'waypoint', action: 'save', name: '取货点', x: 1.0, y: 0.0, yaw: 0.0, kind: 'cargo'}"
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'waypoint', action: 'save', name: '走廊A', x: 0.5, y: -2.0, yaw: 1.571, kind: 'nav'}"

# 存路线（按点名复制位姿）
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'route', action: 'save', name: '巡检A', point_names: ['取货点', '走廊A', '放货点']}"

# 列点位 / 列路线 / 查一条路线
ros2 service call /agv/store agv_interfaces/srv/AgvStore "{resource: 'waypoint', action: 'list'}"
ros2 service call /agv/store agv_interfaces/srv/AgvStore "{resource: 'route', action: 'list'}"
ros2 service call /agv/store agv_interfaces/srv/AgvStore "{resource: 'route', action: 'get', name: '巡检A'}"

# 点名导航 / 带路线多点导航
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'goto', args: ['取货点'], request_id: 'r1'}"
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'goto', args: ['放货点'], route: '巡检A', request_id: 'r2'}"

# 坐标导航（两参数时 yaw=0）
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'goto', args: ['3.0', '1.2', '0.0'], request_id: 'r3'}"

# 完整搬运：取货 → 顶升 → 运输 → 放货 → 下降
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'start_task', args: ['取货点', '放货点'], route: '巡检A', request_id: 't1'}"

# 急停 / 恢复 / 取消
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand "{source: 'web', command: 'stop', request_id: 's1'}"
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand "{source: 'web', command: 'resume', request_id: 's2'}"
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand "{source: 'web', command: 'cancel_task', request_id: 's3'}"

# 手动顶升/下降（调试搬货服务）
ros2 service call /agv/cargo agv_interfaces/srv/CargoCmd "{command: 'lift', request_id: 'm1'}"
ros2 service call /agv/cargo agv_interfaces/srv/CargoCmd "{command: 'lower'}"

# 状态与位姿
ros2 topic echo /agv/task_state
ros2 service call /agv/query agv_interfaces/srv/AgvQuery "{what: 'state'}"
ros2 service call /agv/query agv_interfaces/srv/AgvQuery "{what: 'pose'}"

# 手动/自动互锁
ros2 topic pub --once /agv/manual_mode std_msgs/msg/Bool "{data: true}"
```

被拒绝的样子（看 `task_manager_node` 日志）：

```bash
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand "{source: 'web', command: 'lift'}"
#   → 拒绝：未知指令「lift」，已拒绝
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand "{source: 'web', command: 'goto', args: ['9999','9999','0']}"
#   → 拒绝：在地图范围外
```

---

## 10 速查

| # | 能力 | 接口 | 类型 |
|---|---|---|---|
| 1 | 点名/坐标导航 | `/agv/command` `goto` | 话题入 |
| 2 | 完整搬运 | `/agv/command` `start_task` | 话题入 |
| 3 | 取消 / 急停 / 恢复 | `/agv/command` `cancel_task`/`stop`/`resume` | 话题入 |
| 4 | 状态上报 | `/agv/task_state` | 话题出 |
| 5 | 点位与路线 CRUD | `/agv/store` | 服务（1 个顶 8 个） |
| 6 | 顶升 / 下降 | `/agv/cargo` | 服务 |
| 7 | 搬货中标志 | `/agv/cargo/busy` | 话题出 |
| 8 | 运行时状态 / 位姿 | `/agv/query` | 服务 |
| 9 | 手动/自动互锁 | `/agv/manual_mode` | 话题入 |
| 10 | 网页遥控（导航模式） | `/agv/web_teleop` | 话题入 |
| 11 | 速度下达 / 到底盘 | `/cmd_vel` / `/cmd_vel_raw` | 话题 |
| 12 | 导航执行（内部） | `/navigate_to_pose` | Nav2 动作 |

**接口文件 6 个**：`msg/` = `AgvCommand`、`TaskState`、`NamedPose`；`srv/` = `AgvStore`、`AgvQuery`、`CargoCmd`。

---

## 11 已移除的接口（只供追溯）

代码里这些名字**已全部不存在**，不要在客户端使用。变更背景见 `docs/CHANGELOG.md`。

| 已移除 | 替代 |
|---|---|
| 8 个服务 `Waypoint{Save,Get,List,Delete}` / `Route{Save,Get,List,Delete}` | `/agv/store` + `resource`/`action` |
| 服务 `Dock`、消息 `DockConfig`/`DockDeviation`/`LiftStatus`、整包 `agv_docking` | 无替代：改为"到点即顶升" |
| 话题 `/agv/lift/*`、`/agv/dock/*`、`/agv/cargo/active` | `/agv/cargo/busy` |
| 总线指令 `lift` / `lower` / `dock` / `query` | 服务 `/agv/cargo`、`/agv/query` |
| 状态 `ALIGN_LOAD` / `ALIGN_UNLOAD` | `SEQUENCE` 只剩 5 态 |
| `/agv/query` 的 `what=waypoints`/`routes` | `/agv/store` 的 `action=list` |
| 参数 `precheck_enabled` | 判据自动降级，没有开关 |
| 参数档位 `config/sim/` | 已删；`param_profile` 空 = 只用 default |
| 世界 `warehouse.world`、`new_world.world`、地图 `maps/warehouse/` | 统一为 `world666.world` / `maps/world666map/` |
| 点位里的 `dock_*` / `cargo_model` / `slot_*` 字段 | 点位只存 `x/y/yaw/kind` |
