# AGV 顶升搬运车 · 接口文档（ROS 2 Humble）

> 本文描述**当前代码实际存在**的全部接口：3 个自定义消息、3 个自定义服务、6 个业务自定义话题，
> 以及仿真/导航栈提供的话题。字段名、服务名、话题名、状态串、默认参数值均逐个核对过源码，
> 可直接照着写代码或发指令。
>
> 与上一版最大的区别：**点位/路线的 8 个同构 CRUD 服务已合并成 1 个 `/agv/store`**，
> 并新增 `NamedPose` 消息统一表示"带名字的位姿"——**接口文件从 12 个降到 6 个**（见 §9.2）。
> 目标点合法性的判据也收敛为**唯一一个**（`GoalValidator`），`field_*` 降级为无地图时的兜底。
>
> 更早的一次大改动是**删除"精定位 / docking + 货物跟随"整套方案**，
> 改为"**到点即顶升**"的简单实现；搬货动作只走服务，不走指令总线。被删除的接口清单见 §9.1。

接口包位置：`src/agv_interfaces/`（`msg/` 3 个、`srv/` 3 个，见 `CMakeLists.txt` 的 `rosidl_generate_interfaces`）。
数据文件位置：`maps/world666map/`（`waypoints.yaml` / `routes.yaml` / `tasks.yaml`，格式见 §8）。

---

## 1 接口总览（谁发谁收）

```
                        ┌──────────────────────────────────────┐
   控制台 agv_console ──┤ /agv/command      AgvCommand  话题   │
   网页 / 语音 / 调度 ──┤   （唯一指令入口，白名单 5 条）      │
                        └───────────────┬──────────────────────┘
                                        │ 收
                                        ▼
                        ┌──────────────────────────────────────┐
   /cmd_vel  ◄──────────┤        task_manager_node             │
   /agv/manual_mode ───►│  门卫 CommandRouter + 搬运状态机      │
                        │  GotoController + SafetyMonitor      │
                        │  GoalValidator（目标点唯一判据）      │
                        └──┬────────┬────────┬─────────┬───────┘
              /agv/task_state│        │        │         │ 客户端调用
                (TaskState) │        │        │         │
                            ▼        │        │         ▼
   控制台 / 语音播报 ◄────────┘        │        │   /agv/store （点位 + 路线）
                                      │        │   /agv/cargo （搬货）
                     /agv/query ◄─────┘        │   /navigate_to_pose (Nav2 动作)
                     (AgvQuery 服务)           │
                                               ▼
                        ┌──────────────────────────────────────┐
                        │ waypoint_node                        │
                        │  /agv/store   AgvStore               │
                        │   resource=waypoint|route            │
                        │   action=save|get|list|delete        │
                        └──────────────────────────────────────┘
                                               ▲ 存/查/列/删（控制台直调）
                        ┌──────────────────────────────────────┐
                        │ cargo_node   /agv/cargo（CargoCmd）   │
                        │              /agv/cargo/busy (Bool)   │
                        └──────────────────────────────────────┘

   速度链路（硬件边界）：
     Nav2 velocity_smoother ─┐
     task_manager_node 急停 ─┼─► /cmd_vel ─► velocity_arbiter ─► /cmd_vel_raw ─► Gazebo 差速插件
     控制台手动遥控 ─────────┘        （订阅 /scan，近障减速 + 择空 + 限幅）
```

四条链路各司其职：

| 链路 | 途径 | 说明 |
|---|---|---|
| 指令下发 | `/agv/command`（话题） | 广播式，**不回答**；受理与否只体现在 task_manager_node 日志与 `/agv/task_state` |
| 状态上报 | `/agv/task_state`（话题） | 1 Hz 周期 + 状态变化时立刻发 |
| 点位/路线/搬货/运行时查询 | ROS 服务（`/agv/store`、`/agv/cargo`、`/agv/query`） | 一问一答，**每个响应都有 `ok` 与中文 `message`** |
| 导航执行 | `/navigate_to_pose`（Nav2 动作） | 任务层内部的 `goto_pose()` 调用，不是对外接口 |

> 点位/路线只有 `/agv/store` **一个**入口：新增一种资源（如"工位"）只需扩 `resource` 取值，
> 不必再加 4 个服务。

---

## 2 话题表

### 2.1 业务自定义话题

| 话题 | 类型 | 方向 | 发布者 | 订阅者 | 频率 | 说明 |
|---|---|---|---|---|---|---|
| `/agv/command` | `agv_interfaces/AgvCommand` | 入 | 控制台 `agv_console`、网页、语音、调度 | `task_manager_node` | 事件驱动 | **统一指令总线**，白名单见 §5 |
| `/agv/task_state` | `agv_interfaces/TaskState` | 出 | `task_manager_node` | 控制台、语音播报 | 1 Hz 周期 + 状态变化时立即发 | 任务状态、进度、中文提示 |
| `/cmd_vel` | `geometry_msgs/Twist` | 双向 | Nav2 `velocity_smoother`（声明 20 Hz）、`task_manager_node`（急停/超时补零速）、控制台手动遥控（10 Hz 发送） | `velocity_arbiter`、`task_manager_node`（速度心跳） | 速度平滑器 20 Hz | 上半层"想走的速度"；也是急停零速通道 |
| `/cmd_vel_raw` | `geometry_msgs/Twist` | 出 | `velocity_arbiter` | Gazebo 差速插件 `libgazebo_ros_diff_drive`（实车阶段：下位机） | 20 Hz（`publish_rate`） | **硬件边界**：真正驱动底盘的话题 |
| `/agv/manual_mode` | `std_msgs/Bool` | 入 | 控制台 `agv_console` | `task_manager_node` | 事件驱动 | `true`=手动。手动期间拒导航（互锁） |
| `/agv/cargo/busy` | `std_msgs/Bool` | 出 | `cargo_node` | 控制台 `agv_console` | 变化时立即 + 1 Hz 补发（`publish_period`） | `true`=正在搬货（已顶升未放下） |

### 2.2 由仿真 / 导航栈提供的话题（本项目不定义，仅消费）

| 话题 | 类型 | 发布者 | 主要订阅者 | 频率 | 说明 |
|---|---|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo 射线传感器插件 | `velocity_arbiter`、AMCL、Nav2 代价地图、slam_toolbox | 15 Hz | 360° 全向（361 点，1°/点，±π 首尾重合）。量程 0.05–8.0 m，`frame_name=laser` |
| `/odom` | `nav_msgs/Odometry` | Gazebo 差速插件 | `robot_localization` EKF、AMCL、Nav2 | 50 Hz | 轮式里程计 |
| `/imu` | `sensor_msgs/Imu` | Gazebo IMU 插件 | EKF（`ekf_filter_node`） | 100 Hz | 角速率用于纠偏 |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo 关节状态插件 | `robot_state_publisher` | 50 Hz | 补全左右轮 TF |
| `/tf`、`/tf_static` | `tf2_msgs/TFMessage` | `robot_state_publisher` / AMCL / EKF / 差速插件 | 全体 | — | `map→odom→base_footprint→base_link→laser/imu_link` |
| `/map` | `nav_msgs/OccupancyGrid` | `map_server`（`navigation.launch.py` 加载 `map:=` 指定的 yaml，默认 `world666map.yaml`）或 slam_toolbox（建图模式） | AMCL、Nav2 全局代价地图 | — | 定位/规划用的栅格地图 |
| `/robot_description` | `std_msgs/String` | — | `spawn_entity.py`（`sim.launch.py` 以 `-topic /robot_description` 取 URDF） | — | URDF/xacro 文本，机器人模型来源 |

> `/cmd_vel` 是 Nav2 内部链路的**最后一环**：`controller_server → cmd_vel_nav → velocity_smoother → /cmd_vel`。
> 本项目一个字都没改 Nav2 链路，只在 `/cmd_vel` 之后插了 `velocity_arbiter`。
> 建图模式下键盘遥控经 `--ros-args -r cmd_vel:=cmd_vel_raw` **直连底盘**，不经过仲裁。

---

## 3 服务表与逐服务说明

### 3.1 服务总表

| 服务名 | 类型 | 提供者 | 调用者 | 请求字段 | 响应字段 |
|---|---|---|---|---|---|
| `/agv/store` | `AgvStore` | `waypoint_node` | `task_manager_node`、控制台、网页 | `resource,action,name,x,y,yaw,kind,point_names[]` | `ok,message,items[],names[],counts[],count` |
| `/agv/cargo` | `CargoCmd` | `cargo_node` | `task_manager_node`、控制台 | `command,request_id` | `ok,busy,message` |
| `/agv/query` | `AgvQuery` | `task_manager_node` | 控制台、网页 | `what` | `ok,state,x,y,yaw,message` |

**只有这 3 个服务。** 原先点位 4 个 + 路线 4 个同构的 CRUD 服务已合并进 `/agv/store`，
被合并服务的名称已不复存在，清单见 §9.1。

> ✅ **每个响应都有 `ok`**。曾经列表类服务的响应没有 `ok`（要靠数组是否为空判断），
> 这个不一致随合并一并消失了——`AgvStore` 的所有动作都返回 `ok` 与中文 `message`。

---

### 3.2 `/agv/store` — 点位 / 路线统一存储

类型 `agv_interfaces/srv/AgvStore`　提供者 `waypoint_node`

**为什么是一个服务而不是八个**：点位就是"只有 1 个位姿的资源"，路线就是"有 N 个位姿的资源"，
两者本来是同一个东西——都是"给个名字，对一个带名字的位姿列表做增删查列"。
合并后校验与回执只有一处实现（纯 Python 的 `store_dispatch.dispatch()`），
不会出现"这个服务有 `ok`、那个没有"；调用方从 8 个客户端降到 1 个。

#### 3.2.1 请求字段

| 字段 | 类型 | 含义 |
|---|---|---|
| `resource` | `string` | 资源类型：`waypoint`（点位）/ `route`（路线） |
| `action` | `string` | 动作：`save` / `get` / `list` / `delete` |
| `name` | `string` | 资源名（`save` / `get` / `delete` 用；`list` 忽略） |
| `x` | `float64` | **仅 `waypoint` + `save`**：位置 x（map 坐标系，m） |
| `y` | `float64` | **仅 `waypoint` + `save`**：位置 y（m） |
| `yaw` | `float64` | **仅 `waypoint` + `save`**：朝向（rad，REP-103 逆时针为正） |
| `kind` | `string` | **仅 `waypoint` + `save`**：`cargo`（搬货点）/ `nav`（导航点）。**留空按 `cargo` 处理** |
| `point_names` | `string[]` | **仅 `route` + `save`**：途经点名列表（按行进顺序，至少 1 个） |

`resource` 与 `action` 都先做 `strip().lower()` 再比对，**大小写与空格不敏感**。
非法 `resource` / `action` 返回 `ok=false`，不抛异常。

#### 3.2.2 响应字段

| 字段 | 类型 | 含义 |
|---|---|---|
| `ok` | `bool` | 是否成功（**每个动作都给**） |
| `message` | `string` | 中文回执，可直接打印 |
| `items` | `NamedPose[]` | `get`：该资源的位姿列表；`waypoint`+`list`：全部点位；其余动作**空** |
| `names` | `string[]` | `list`：资源名列表（按名排序，与 `counts` 同序）；其余动作**空** |
| `counts` | `uint32[]` | `list`：各资源的位姿个数（与 `names` 同序）。`waypoint`+`list` 恒为全 `1` |
| `count` | `uint32` | `get`：位姿个数；`list`：条目总数；`route`+`save`：实际写入的途经点数；其余 `0` |

**按动作区分哪些字段有值**（其余字段为空集/0，读取前先看 `ok`）：

| `resource` + `action` | `ok` | `message` | `items` | `names` | `counts` | `count` |
|---|---|---|---|---|---|---|
| `waypoint` + `save` | ✓ | ✓ | — | — | — | `0` |
| `waypoint` + `get` | ✓ | ✓ | 1 项 | — | — | `1` |
| `waypoint` + `list` | ✓ | ✓ | **全部点位** | 点名列表 | 全 `1` | 条目数 |
| `waypoint` + `delete` | ✓ | ✓ | — | — | — | `0` |
| `route` + `save` | ✓ | ✓ | — | — | — | 写入途经点数 |
| `route` + `get` | ✓ | ✓ | N 项 | — | — | 途经点数 |
| `route` + `list` | ✓ | ✓ | — | 路线名列表 | 各路线点数 | 条目数 |
| `route` + `delete` | ✓ | ✓ | — | — | — | `0` |

> ⚠️ `waypoint`+`list` 的 `items` **有值**，而 `route`+`list` 的 `items` **为空**（路线列表只给名字与点数）。
> 需要路线的途经点坐标时，对每条路线再发一次 `route`+`get`。

#### 3.2.3 四个动作的语义

| 动作 | 语义 | 关键规则 |
|---|---|---|
| `save` | 存 / **覆盖**（重名覆盖） | `waypoint` 用 `x/y/yaw/kind`；`route` 用 `point_names` |
| `get` | 按名字查一个 | 不存在 → `ok=false`，**不报错** |
| `list` | 列出全部 | 按名排序（`sorted()`，中文按码位序，稳定可复现） |
| `delete` | 删一个 | 不存在 → `ok=false`，**不报错** |

`route` + `save` 的两个特别规则：

- **按点名从点位表复制位姿**，所以路线文件里存的是坐标**快照**；之后改名/删点**不影响**已存路线。
- **有点名不存在就整条拒绝**，不写半条路线；`name` 与 `point_names` 里的空字符串都会被过滤。

#### 3.2.4 回执一览（`message` 逐条）

`waypoint` 类：

| 情况 | `ok` | `message` |
|---|---|---|
| save 成功 | `true` | `已保存点位「<点名>」（cargo\|nav）` |
| save 时点名为空 | `false` | `点名不能为空` |
| save 时坐标非数字 / 写盘失败 | `false` | `保存失败：坐标非法或无法写入文件` |
| get 找到 | `true` | `点位「<点名>」已找到` |
| get 不存在 | `false` | `点位「<点名>」不存在` |
| list 成功 | `true` | `共 N 个点位` |
| delete 成功 | `true` | `已删除点位「<点名>」` |
| delete 时点名为空 | `false` | `点名不能为空` |
| delete 不存在 | `false` | `点位「<点名>」不存在` |

`route` 类：

| 情况 | `ok` | `message` |
|---|---|---|
| save 成功 | `true` | `已保存路线「<路线名>」，共 N 个途经点` |
| save 时路线名为空 | `false` | `路线名不能为空` |
| save 时过滤后无途经点 | `false` | `路线至少要有一个途经点` |
| save 时有点名不存在 | `false` | `这些点位不存在：A、B`（顿号连接） |
| save 写盘失败 | `false` | `保存失败：无法写入文件` |
| get 找到 | `true` | `路线「<路线名>」共 N 个途经点` |
| get 不存在 | `false` | `路线「<路线名>」不存在` |
| list 成功 | `true` | `共 N 条路线` |
| delete 成功 | `true` | `已删除路线「<路线名>」` |
| delete 时路线名为空 | `false` | `路线名不能为空` |
| delete 不存在 | `false` | `路线「<路线名>」不存在` |

资源 / 动作本身非法：

| 情况 | `ok` | `message` |
|---|---|---|
| `resource` 不在 `(waypoint, route)` | `false` | `未知资源「<原样>」，应为 waypoint 或 route` |
| `action` 不在 `(save, get, list, delete)` | `false` | `未知动作「<原样>」，应为 save / get / list / delete` |

> `route` + `get` 查不到时等价于"不存在"——**空路线不会入表**，所以不存在与查不到是同一件事。
> 删点**不影响已存的路线**（路线存的是快照）。`kind` 规整规则：`strip().lower()` 后不在
> `(cargo, nav)` 内的一律按 `cargo` 落盘。

#### 3.2.5 八个动作的调用示例

前提：已 `source install/setup.bash` 且 `waypoint_node` 在跑。

```bash
# ── 点位：存 / 查 / 列 / 删 ──────────────────────────────────
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'waypoint', action: 'save', name: '取货点', x: 1.0, y: 0.0, yaw: 0.0, kind: 'cargo'}"

ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'waypoint', action: 'save', name: '走廊A', x: 0.5, y: -2.0, yaw: 1.571, kind: 'nav'}"

ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'waypoint', action: 'get', name: '取货点'}"

ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'waypoint', action: 'list'}"

ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'waypoint', action: 'delete', name: '取货点'}"

# ── 路线：存 / 查 / 列 / 删 ──────────────────────────────────
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'route', action: 'save', name: '巡检A', point_names: ['取货点', '走廊A', '放货点']}"

ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'route', action: 'get', name: '巡检A'}"

ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'route', action: 'list'}"

ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'route', action: 'delete', name: '巡检A'}"
```

> **`list` 不带 `name`**：`list` 动作忽略 `name` 字段，写了也不报错。
> `waypoint` 存点时 `x/y/yaw` 必须写全；`route` 存点时不看 `x/y/yaw/kind`，只看 `point_names`。

`task_manager_node` 内部用它做"点名 → 坐标"（`store_client.py`），
服务不可用（3 s 等不到）时按"查点失败"处理，导航取消。

---

### 3.3 `/agv/cargo` — 搬货服务（顶升 / 下降二合一）

类型 `agv_interfaces/srv/CargoCmd`　提供者 `cargo_node`

> ★ 这是全项目**唯一**与"搬货机器"打交道的地方，也是后续接真实硬件的唯一改动点。
> 现阶段只维护"正在搬货"标志并立即返回成功，**不驱动任何机器**。

| 方向 | 字段 | 类型 | 含义 |
|---|---|---|---|
| 请求 | `command` | `string` | `lift`=顶升（开始搬货）/ `lower`=下降（放货完成） |
| 请求 | `request_id` | `string` | 请求号（追踪用，**可为空**） |
| 响应 | `ok` | `bool` | 是否受理成功 |
| 响应 | `busy` | `bool` | 执行后的"搬货中"状态 |
| 响应 | `message` | `string` | 中文回执 |

`command` 会被 `strip().lower()` 后再判断，接受的写法：

| 语义 | 接受的写法 | `busy` | `ok` | `message` |
|---|---|---|---|---|
| 顶升 | `lift` / `up` / `顶升` | `true` | `true` | `已顶升，开始搬货` |
| 下降 | `lower` / `down` / `放货` / `下降` | `false` | `true` | `已下降，放货完成（空闲）` |
| 其它 | 任意 | 不变 | `false` | `未知命令「<原样 command>」：只支持 lift（顶升）/ lower（下降）` |

受理后立即向 `/agv/cargo/busy` 广播新的 `busy` 值；`task_manager_node` 调本服务时的等待上限为
`cargo_timeout_s`（默认 5 s），超时按"搬货失败"处理并把任务置 `ERROR`。

```bash
ros2 service call /agv/cargo agv_interfaces/srv/CargoCmd "{command: 'lift', request_id: 'm1'}"
ros2 service call /agv/cargo agv_interfaces/srv/CargoCmd "{command: 'lower'}"
```

---

### 3.4 `/agv/query` — 查询运行时状态 / 当前位置

类型 `agv_interfaces/srv/AgvQuery`　提供者 `task_manager_node`

> `/agv/command` 是话题，广播发出去**收不到回答**；想知道"现在什么状态、车在哪"必须用本服务。

| 方向 | 字段 | 类型 | 含义 |
|---|---|---|---|
| 请求 | `what` | `string` | `state`（任务状态）/ `pose`（当前位置） |
| 响应 | `ok` | `bool` | 是否查询成功 |
| 响应 | `state` | `string` | 任务状态串（**任何 `what` 都填**，取 `TaskManager.state()`） |
| 响应 | `x` | `float64` | 当前位置 x（`what=pose` 时；map 坐标系，m） |
| 响应 | `y` | `float64` | 当前位置 y |
| 响应 | `yaw` | `float64` | 当前朝向（rad） |
| 响应 | `message` | `string` | 中文提示 |

`what` 先做 `strip().lower()`，然后分支：

| `what` | 行为 | 典型 `message` |
|---|---|---|
| `pose` | 查 `map→base_footprint` TF | 成功：`当前位姿 x=1.00 y=2.00 yaw=0.00`；失败：`ok=false`，`暂时取不到位姿（定位/TF 未就绪）` |
| `state`（**含其它任何值**） | 读搬运状态机 | `GO_LOAD（前往取货点）目标 取货点 进度 15%，导航 IDLE` |

> ★ **本服务只回答"运行时状态"**。点位表 / 路线表**不在这里查**——它们属于"存储内容"，
> 统一走 `/agv/store`（`resource=waypoint|route, action=list`）。职责单一，
> 避免同一个东西两个服务都能查。
>
> ⚠️ 旧的 `what=waypoints` / `what=routes` 取值**已删除**。现在传这两个值**不会报错**，
> 而是落进"其它任何值"分支、按 `state` 返回——所以拿旧写法查点位会得到状态串而非点位表。
> 写客户端时请直接用 `/agv/store`。

注意 `pose` 之外的 `what` 都返回 `ok=true`（`message` 里说明情况即可），只有 `pose` 取不到 TF 时返回 `ok=false`。

```bash
ros2 service call /agv/query agv_interfaces/srv/AgvQuery "{what: 'state'}"
ros2 service call /agv/query agv_interfaces/srv/AgvQuery "{what: 'pose'}"
```

---

## 4 消息字段表

### 4.1 `agv_interfaces/AgvCommand` — 统一指令总线

发到 `/agv/command`，由 `task_manager_node` 消费。

| 字段 | 类型 | 含义 |
|---|---|---|
| `source` | `string` | 指令来源：`web` / `voice` / `scheduler` / `console` |
| `command` | `string` | 动作名，白名单：`goto` / `start_task` / `cancel_task` / `stop` / `resume` |
| `args` | `string[]` | 参数：点名，或坐标 `[x, y]` / `[x, y, yaw]`（**按字符串传入**） |
| `route` | `string` | 可选：本次要经过的路线名（**空串 = 不用路线**） |
| `confidence` | `float32` | 置信度（语音用 0~1；其它入口填 `1.0`） |
| `request_id` | `string` | 请求唯一号：用于去重与追踪（**空串 = 匿名，不去重**） |

`route` 的两种效果：

- `goto` → 路线各点 → 目标点；
- `start_task` → 取货点 → 路线各点 → 放货点。

### 4.2 `agv_interfaces/TaskState` — 任务状态

`task_manager_node` 发布到 `/agv/task_state`。

| 字段 | 类型 | 含义 |
|---|---|---|
| `task_id` | `string` | 任务唯一号，形如 `task-0001`（**单点导航时为空串**）；一次完整搬运一个 |
| `state` | `string` | 状态串，取值见 §6 |
| `target_location` | `string` | 当前目标点名（取货点 / 路线点 / 放货点）；坐标导航时为 `(x.xx, y.yy)` |
| `progress` | `float32` | 任务进度 0~1，取值见 §6 |
| `message` | `string` | 中文提示，可直接显示 |

### 4.3 `agv_interfaces/NamedPose` — 带名字的位姿（本次新增）

**为什么加它**：原先点位/路线的查询结果用「5 个平行数组」返回
（`names[]` / `xs[]` / `ys[]` / `yaws[]` / `kinds[]`），调用方必须自己保证下标对齐；
一旦某个数组短一位，就会**静默地把 A 点名字配到 B 点坐标上**——不报错、不崩溃，
车却开去别的地方。改成一个数组、每个元素自带全部字段后，**这类对齐错误在类型层面就不存在了**。

| 字段 | 类型 | 含义 |
|---|---|---|
| `name` | `string` | 名字：点位名；作为路线途经点时是"复制当时的点位名"（仅便于人看） |
| `x` | `float64` | 位置 x（map 坐标系，m） |
| `y` | `float64` | 位置 y（m） |
| `yaw` | `float64` | 朝向（rad，REP-103：逆时针为正） |
| `kind` | `string` | 类型：`cargo` 搬货点 / `nav` 导航点；**路线途经点留空** |

两种用法：

| 场合 | `name` | `x` / `y` / `yaw` | `kind` |
|---|---|---|---|
| 点位（`waypoint`） | 点名 | 全部有效 | `cargo` / `nav` |
| 路线途经点（`route`） | 快照时的点名 | 全部有效 | **空串** |

> `NamedPose` 只出现在 `/agv/store` 的 `items[]` 里，**不在任何话题上传输**。

---

## 5 指令语义与拒绝规则

### 5.1 白名单与参数要求

白名单常量 `VALID_COMMANDS = ('goto', 'start_task', 'cancel_task', 'stop', 'resume')`。
**搬货（顶升/下降）不在白名单里**——它由任务层直接调 `/agv/cargo` 服务，不走总线。

| `command` | `args` 要求 | 语义 | 受理后 |
|---|---|---|---|
| `goto` | 至少 1 个 | 点名导航或坐标导航，可带 `route` | 起后台线程导航；受理回执 `已下发导航（点名 X）` / `已下发导航（坐标）` |
| `start_task` | 至少 2 个 | 完整搬运：取货点名 → 放货点名，可带 `route` | 起后台线程跑搬运；回执 `已开始搬运：A → B` |
| `cancel_task` | 忽略 | 取消当前任务并停车（清 Nav2 目标、状态机回 `IDLE`、补零速） | `已取消任务并停车`；本就空闲时 `当前无任务可取消` |
| `stop` | 忽略 | **急停**：进急停态、立刻发零速、中止后台线程、任务置 `PAUSED` | `已急停（停车耗时 XXms，达标/超标）` |
| `resume` | 忽略 | 恢复：清急停、清任务、回 `IDLE` 待命 | `已恢复，回到空闲待命` |

`goto` 的 `args` 解析规则（`command_router.py`）：

| `len(args)` | 解析 | 备注 |
|---|---|---|
| 1 | **一律当点名**（`args[0]`） | 纯数字也当点名，如点名「666」合法 |
| 2 | 当坐标 `x, y`，`yaw` 缺省 `0.0` | 坐标合法性交给 `GoalValidator` 判（见 §5.2） |
| ≥3 | 当坐标 `x, y, yaw` | 第 4 个及以后忽略；`yaw` 也必须能转成 float |

`start_task` 只取 `args` 前两个非空项作为取货点、放货点。

### 5.2 拒绝规则与日志字样

拒绝时**不抛异常、不回复**，只在 `task_manager_node` 日志里留一条：

```
[指令] <command> from=<source> → 拒绝：<原因>
```

受理时对应的是 `→ 受理：<原因>`。`reason` 是中文原因串，关键判据：

| # | 规则 | 实现位置 | 拒绝时 `reason` 片段 |
|---|---|---|---|
| 1 | `stop` 无限受理 | 门卫最前面，绕过白名单与去重 | —（永远受理） |
| 2 | `command` 必须在白名单 | `VALID_COMMANDS` | `未知指令「fly」，已拒绝` |
| 3 | 白名单内必须已注册处理器 | `_handlers` | `指令「goto」暂未实现，已拒绝` |
| 4 | `goto` 至少 1 个参数 | `_NEEDS_ARGS` | `指令「goto」缺少参数（需目标点名或坐标）` |
| 5 | `start_task` 至少 2 个参数 | `_NEEDS_TWO_ARGS` | `指令「start_task」缺少参数（需起始点与目标点）` |
| 6 | `goto` 单参数点名非空 | `_check_goto_args` | `goto 目标点名不能为空` |
| 7 | `goto` 坐标可解析为 float | `_check_goto_args` | `goto 坐标格式非法，应为 [x, y] 或 [x, y, yaw]` |
| 8 | **坐标交给唯一判据 `GoalValidator`** | `_check_goto_args` → `_in_field` | 图外 / 压障碍 / 净空不足的具体串，见下 |
| 9 | 同 `request_id` 只受理一次 | `_seen` 去重表 | `请求「r1」重复，已拒绝` |
| 10 | 处理器抛异常不拖垮路由 | `_dispatch` | `指令「goto」执行异常：<异常>` |
| 11 | 前置检查（见下） | `_can_start` | `处于急停状态，请先恢复（resume）` / `当前是手动模式，禁止导航` / `已有任务在执行，请先取消或等它完成` / `搬运状态为 X，无法接新任务` / `系统正在关闭` |

### 5.3 目标点合法性：唯一的判据是地图

★ **规则 8 已收敛**：坐标能不能去，全项目只有**一个**判据 —— `agv_common.goal_check.GoalValidator`，
由 `task_manager_node` 构造并注入 `CommandRouter`，**门卫不再自己按矩形范围判**。
`GoalValidator` 的行为：

| 条件 | 判据 | 说明 |
|---|---|---|
| `map_yaml` 给了且能读 | **实测栅格地图**（权威） | 图外拒 / 压障碍拒 / 净空不足拒（`MapChecker.check_point()`） |
| `map_yaml` 空或读不到 | `field_*` 矩形范围（**兜底**） | 只在没有地图时生效（纯逻辑测试、或没走加载地图流程） |

地图判据下的实际拒绝串（`robot_clearance` 默认 0.30 m）：

| 情况 | `message` |
|---|---|
| 图外 | `目标点 (99.00, 99.00) 在地图范围外 (x∈[-10.7, 4.2], y∈[-9.8, 9.2])，已拒绝` |
| 压在障碍/墙上 | `目标点 (x.xx, y.yy) 落在障碍物/墙体上，已拒绝` |
| 净空不足 | `目标点 (x.xx, y.yy) 距障碍仅 C.CC m，小于车身所需净空 0.30 m，已拒绝` |
| 通过 | `目标点 (1.63, 2.66) 合法，净空 1.23 m` |

无地图时的兜底串：

```
目标点 (99.00, 99.00) 超出场地范围 x[-10, 4] y[-10, 9]，已拒绝
```

> ⚠️ 已知**外观**瑕疵（不影响拦截正确性）：`CommandRouter` 会在判据返回的串后面再拼一次
> `，已拒绝`，而地图判据的串本身已以`，已拒绝`结尾，于是日志里会出现
> `……，已拒绝，已拒绝`。拦截行为正确，只是尾巴重复。

`goto` 的**点名**不发坐标，所以点名导航**不做**坐标校验——点位的合法性由
`GoalValidator` 在任务流程里预检（`task_flow.precheck()`，见 §5.4）。
`_in_field(x, y)` 的直接委托写法保证"门卫"与"预检"用的是**同一个对象**，
不可能再出现"门卫放行、预检拒绝"这种同一坐标两种结论的情况。

### 5.4 点位预检（`task_flow.precheck()`）

从点位表取出的点（取货点 / 路线途经点 / 放货点）在导航前会过一遍 `GoalValidator.check()`：

- 预检不过 → 日志 `点位「<点名>」预检未通过：<原因>`，任务判失败；
- `validator is None`（没配判据）→ 不拦；
- **不再有预检开关**：判据是"有地图用地图、没地图用兜底"，
  没有"关掉预检"这个状态（旧的预检开关参数已删除，见 §9.1）。

### 5.5 `request_id` 去重细节

- 只有 `request_id` **非空**才去重；空串视为匿名请求，不去重。
- 去重表记为"见过"，**无论受理与否都记**——这是为了防重试风暴（被拒的越界请求重发也是拒）。
- 表上限 `max_seen = 1000`，超限**淘汰最旧的**（按插入顺序 FIFO）。
- `stop` **绕过去重**：同一个 `request_id` 连发多次 `stop` 也每次都受理（安全通道）。
- 控制台 `send_cmd()` 默认连发 3 次防丢包，因此**每次发送都换新的 `request_id`**；
  自检里要测去重时才用 `repeat=1` 手动复用同一个号。
- ★ **测去重时，探针必须能通过前面的白名单/参数/越界校验**。校验顺序是
  `stop 快通道 → 白名单 → 参数 → goto 坐标越界 → 去重`，所以拿**越界坐标**（如
  `goto 9999 9999`）当去重探针是测不到的——两次都在"越界"那一步被拦下，日志里
  只会出现"超出场地范围"，永远不会出现"重复"。自检用的探针是 `cancel_task`
  （无参、通过全部校验、此刻无任务故不会让车动）。

### 5.6 `_can_start()` 前置检查（`goto` / `start_task` 共用）

按顺序判断，任一不过就拒绝：

1. `rclpy.ok()` 为假 → `系统正在关闭`
2. `safety.estopped()` 为真 → `处于急停状态，请先恢复（resume）`
3. 手动模式 → `当前是手动模式，禁止导航`
4. 后台线程还在跑 → `已有任务在执行，请先取消或等它完成`（**任务串行执行**）
5. `manager.state()` 不在 `('IDLE', 'DONE')` → `搬运状态为 X，无法接新任务`

`cancel_task` / `stop` / `resume` **不做**这些检查，永远受理。

---

## 6 状态机与状态串取值

### 6.1 完整搬运状态机（`TaskManager`）

```
IDLE ──start_task──► GO_LOAD ──► LIFTING ──► TRANSPORT ──► LOWERING ──► DONE ──► IDLE
                       │            │            │             │
                       └────────────┴────────────┴─────────────┴──► ERROR（导航失败 / 搬货服务失败 / 异常）
                       任何活动状态 ──stop──► PAUSED ──resume──► IDLE
```

`SEQUENCE = ('GO_LOAD', 'LIFTING', 'TRANSPORT', 'LOWERING', 'DONE')`，每完成一步调一次
`step_success()` 推进一格；`DONE` 再推一次自动回 `IDLE`（可接下一单）。
**没有 ALIGN_LOAD / ALIGN_UNLOAD**——到点即顶升，没有对位阶段。

单点导航（`GotoController`）只用到 4 个态：`IDLE → GO_LOAD → DONE`，失败走 `ERROR`。

### 6.2 状态串取值表

| `state` | 阶段 | `progress`（搬运） | `message`（搬运） | `progress`（单点导航） | `message`（单点导航） |
|---|---|---|---|---|---|
| `IDLE` | 空闲，可接新任务 | `0.0` | `空闲，等待任务` | `0.0` | `空闲，等待任务` |
| `GO_LOAD` | 前往取货点（带路线时依次经过路线各点） | `0.15` | `前往取货点` | `0.5` | `导航中` |
| `LIFTING` | 顶升中（调 `/agv/cargo` 搬货服务） | `0.40` | `顶升中` | — | — |
| `TRANSPORT` | 运输中：已顶升，前往放货点（带路线时经过路线各点） | `0.60` | `运输中` | — | — |
| `LOWERING` | 下降放货中 | `0.85` | `下降放货中` | — | — |
| `DONE` | 任务完成（随后自动回 `IDLE`） | `1.0` | `搬运完成` | `1.0` | `导航完成` |
| `ERROR` | 异常（导航失败 / 搬货服务失败），需人工复位 | `0.0` | `任务异常，需人工复位` | `0.0` | `导航失败` |
| `PAUSED` | 暂停：收到 `stop`（急停）后冻结，可 `resume` 恢复 | `0.0` | `已暂停（急停/停止）` | `0.0` | — |

> ⚠️ 同一个 `GO_LOAD` 在两个来源里 `progress` 不同：**完整搬运是 0.15，单点导航是 0.5**。
> 这是两套表（`TaskManager.STATE_PROGRESS` 与 `state_reporter.GOTO_PROGRESS`）造成的，
> 控制台若按进度画进度条要注意区分来源（`task_id` 为空串即单点导航）。

`publish_state()` 的组装优先级（组装逻辑在 `agv_task/state_reporter.py` 的 `compose()`，节点只负责发消息）：

1. 搬运态 != `IDLE` → 报搬运态（带 `task_id`、`target_location`、搬运进度）；
2. 否则单点导航态 != `IDLE` → 报导航态（`task_id=''`，`target_location` = 目标点名或 `(x.xx, y.yy)`）；
3. 都空闲 → `task_id=''`, `target_location=''`, `progress=0.0`, `state='IDLE'`, `message='空闲，等待任务'`；
4. **最后**：若 `safety.estopped()` 为真，**强制覆盖**为 `state='PAUSED'`，
   `message='已急停（安全停止）'`，`progress=0.0`（急停显示优先于一切）。

`target_location` 的取值：`GO_LOAD`/`LIFTING` 给取货点名，`TRANSPORT`/`LOWERING` 给放货点名，其余为空串。

---

## 7 参数表

参数中心在 `src/agv_bringup/config/`：`default/` 打底，`param_profile` 指定的档位目录覆盖。
**`param_profile` 默认空串 = 只用 `default`**；实车差异写好 `config/real/*.yaml` 后传 `param_profile:=real`。
`config/sim/` 目录**已删除**（仿真与默认值没有差异，不需要单独一层）。

### 7.1 `task_manager_node` —— `default/task.yaml`

| 参数 | 默认值 | 作用 |
|---|---|---|
| `field_min` | `-10.70` | **兜底**坐标范围 x 下界（m）＝实测地图左边界。**仅无地图时生效** |
| `field_max` | `4.15` | 兜底范围 x 上界（m）＝实测地图右边界 |
| `field_min_y` | `-9.83` | 兜底范围 y 下界（m）＝实测地图下边界 |
| `field_max_y` | `9.22` | 兜底范围 y 上界（m）＝实测地图上边界 |
| `waypoint_frame` | `map` | 点位/导航目标坐标系 |
| `nav_print_period` | `1.0` | 导航反馈打印周期（s），避免刷屏 |
| `nav_timeout_s` | `180.0` | **单段**导航总超时（s）：超时取消目标并判失败 |
| `cargo_timeout_s` | `5.0` | 调 `/agv/cargo` 的等待上限（s） |
| `robot_clearance` | `0.30` | 目标点所需净空（m）：车宽 0.5 → 半宽 0.26 + 余量 |
| `map_yaml` | `""` | 目标点判据用的地图 yaml；**留空则退化为 `field_*` 兜底**（由 launch 注入） |

> 四个 `field_*` 现在**与实测地图一致**（不再是旧参考图的包围盒），所以"门卫放行、判据拒绝"
> 这类矛盾已经不存在了。详见 §5.3 与 §9.4。

### 7.2 `task_manager_node` —— `default/safety.yaml`

| 参数 | 默认值 | 作用 |
|---|---|---|
| `cmd_timeout_ms` | `500` | 多久没收到新 `/cmd_vel` 就判超时停车（ms） |
| `estop_budget_ms` | `200` | 急停后发出停车的时限（ms），用于 `estop_timely()` 达标判定 |

### 7.3 `cargo_node` —— `default/cargo.yaml`

| 参数 | 默认值 | 作用 |
|---|---|---|
| `service_name` | `/agv/cargo` | 搬货服务名 |
| `busy_topic` | `/agv/cargo/busy` | "正在搬货"标志话题（`std_msgs/Bool`，`true`=搬货中） |
| `publish_period` | `1.0` | 标志补发周期（s）：晚加入的订阅者也能看到当前状态（内部下限 0.1 s） |

### 7.4 `waypoint_node` —— `default/waypoint.yaml`

| 参数 | 默认值 | 作用 |
|---|---|---|
| `waypoint_file` | `/tmp/agv_waypoints.yaml` | 点位文件路径（**控制台加载地图时会覆盖成该地图的 `waypoints.yaml`**） |
| `routes_file` | `/tmp/agv_routes.yaml` | 路线文件路径（同上，覆盖成该地图的 `routes.yaml`） |
| `frame_id` | `map` | 点位/路线坐标系（文件里有 `frame_id` 时以文件为准） |

### 7.5 `velocity_arbiter` —— `default/hardware.yaml`

| 参数 | 默认值 | 作用 |
|---|---|---|
| `stop_distance` | `0.30` | 距障 ≤ 此值(m)：直接刹停（防撞底线） |
| `slow_distance` | `0.55` | 距障 ≤ 此值(m)：开始按比例减速 |
| `take_over_distance` | `0.45` | 距障 ≤ 此值(m) 且低速：进入"择向接管" |
| `takeover_max_speed` | `0.25` | 进入接管所需的车速上限(m/s) |
| `slow_min_speed` | `0.10` | 减速过渡段的最小线速度(m/s) |
| `front_half_angle_deg` | `45.0` | 前方扇区半张角（度）：用于"近障减速"，保持宽覆盖 |
| `takeover_half_angle_deg` | `20.0` | 接管闸门锥半张角（度）：只管"要不要夺向"。★ 别开大——前锥 45° 的边界会扫到平行于车身的槽侧壁（近侧壁距车心 0.250，45° 命中 0.354 ≤ take 0.45）→ 闸门被骗开、车在槽内被反复夺向。解析临界 27.0°，实测临界 30~35° |
| `side_max_angle_deg` | `90.0` | 侧方统计到该角度为止（度） |
| `rear_half_angle_deg` | `20.0` | 后方扇区半张角（度）：倒车时按此窄锥看车尾。★ 别开大——槽内两侧平行壁会进入锥内（实测 ±25° 就擦到侧壁 0.54 m），倒车被自己压住 → "能进不能出"复发 |
| `yaw_handback_rate` | `0.30` | 远离障碍后角速度交还导航的步长(rad/s/次) |
| `max_linear` | `0.5` | 最终线速度上限(m/s) |
| `max_angular` | `1.0` | 最终角速度上限(rad/s) |
| `scan_topic` | `/scan` | 激光话题 |
| `cmd_in` | `/cmd_vel` | 订阅 Nav2/任务层的最终速度 |
| `cmd_out` | `/cmd_vel_raw` | 发给底盘（Gazebo 差速插件 / 实车下位机） |
| `cmd_timeout` | `0.5` | 上层速度超时(s)：超时即输出零速（防"最后一条速度被无限重发"） |
| `scan_timeout` | `0.5` | 激光超时(s)：超时则原样放行（只限幅） |
| `publish_rate` | `20.0` | 仲裁输出频率(Hz)，内部下限 1.0 |

> `hardware.yaml` 现在**显式列出全部 16 个参数**（`cmd_timeout` / `publish_rate` 过去只写在
> 代码的 `declare_parameter` 里，现已补进 YAML，配置来源唯一）。

### 7.6 其他节点参数（非 `default/*.yaml` 管理，来自各自 `declare_parameter`）

| 节点 | 参数 | 默认值 | 作用 |
|---|---|---|---|
| `robot_pose`（可选工具节点） | `target_frame` | `map` | 查询目标坐标系 |
| | `base_frame` | `base_footprint` | 本体坐标系 |
| | `rate` | `1.0` | 查询频率 Hz（下限 0.1） |

### 7.7 launch 参数 —— `agv_bringup/launch/agv_stack.launch.py`

| 参数 | 默认值 | 作用 |
|---|---|---|
| `param_profile` | `""`（空） | 参数中心档位目录名：**留空 = 只用 `config/default`**；实车差异写好 `config/real/*.yaml` 后传 `real`。**勿改名 `profile`**——会被 gzserver 当 `--profile` 吃走 |
| `with_sim` | `false` | 是否同时拉起 `agv_navigation` 导航栈（仿真 + AMCL + Nav2 + RViz） |
| `gui` | `true` | Gazebo 窗口 |
| `rviz` | `true` | RViz 窗口 |
| `world_file` | `""` | 可选 Gazebo world 绝对路径（**留空则用 `agv_navigation` 的默认世界 `world666.world`**）。**勿改名 `world`**——空默认值会顶掉导航栈的默认世界 |
| `map_yaml` | `""` | 可选：所选地图的 yaml 路径（控制台按地图库文件注入），同时决定点位预检用的地图。**留空则 launch 兜底用 `share/agv_navigation/maps/world666map/world666map.yaml`** |
| `waypoint_file` | `""` | 可选：与所选地图绑定的点位文件路径 |
| `routes_file` | `""` | 可选：与所选地图绑定的路线文件路径（**留空则用点位文件同目录的 `routes.yaml`**） |

本 launch 拉起 4 个节点：`waypoint_node`、`cargo_node`、`task_manager_node`、`velocity_arbiter`
（`with_sim:=true` 时再 include `agv_navigation/navigation.launch.py`）。
业务节点时钟：`use_sim_time = (with_sim == 'true')`——不带 Gazebo 时用系统时钟，否则没有 `/clock` 定时器不会触发。

```bash
ros2 launch agv_bringup agv_stack.launch.py                                  # 只起业务节点（只用 default 参数）
ros2 launch agv_bringup agv_stack.launch.py with_sim:=true                    # 再带上仿真+导航
ros2 launch agv_bringup agv_stack.launch.py with_sim:=true gui:=false rviz:=false
ros2 launch agv_bringup agv_stack.launch.py with_sim:=true param_profile:=real   # 切实车参数层

# 显式绑定 world666 世界与它的地图/点位/路线（多份地图时才需要写全）
ros2 launch agv_bringup agv_stack.launch.py with_sim:=true \
  world_file:=$PWD/src/agv_description/worlds/world666.world \
  map_yaml:=$PWD/maps/world666map/world666map.yaml \
  waypoint_file:=$PWD/maps/world666map/waypoints.yaml \
  routes_file:=$PWD/maps/world666map/routes.yaml
```

`agv_navigation/launch/navigation.launch.py` 的参数：`world`（默认 `agv_description/worlds/world666.world`）、
`map`（默认 `share/agv_navigation/maps/world666map/world666map.yaml`）、`nav2_params_file`、`gui`、
`rviz`、`use_sim_time`（默认 `true`）。**勿改名 `params_file`**——会被 gzserver 当 `--params-file` 吃走。
`mapping.launch.py` 的默认世界同为 `world666.world`。雷达视场角固定 360°（写在 URDF 里，
无 launch 参数），建图与导航共用同一份扫描数据。

---

## 8 数据文件格式

点位/路线/任务三者**与地图绑定**：一份地图一个目录，删地图就一起删，不留孤儿数据。

当前项目只用**一份世界 + 一份地图**：

| 角色 | 真源路径 | 说明 |
|---|---|---|
| Gazebo 世界（唯一一份） | `src/agv_description/worlds/world666.world` | 含 `<gravity>` / `<physics type='ode'>` / `<scene>` |
| 栅格地图（**已实测验证，以此为准**） | `maps/world666map/world666map.pgm` + `world666map.yaml` | **594 × 762 格 @0.025 m/格，origin (-10.7, -9.83)** → 覆盖 x ∈ [-10.70, 4.15]、y ∈ [-9.83, 9.22]（详见 §9.4） |
| 包内同份副本 | `src/agv_navigation/maps/world666map/` | `colcon build` 后由 `share/agv_navigation/maps/` 提供 |
| 生成脚本的参考图 | `src/agv_navigation/maps/world666map/world666map_ref.pgm` + `_ref.yaml` | 870 × 813 @0.025，origin (-11.304, -10.423)，**仅供参照，导航不用** |

三处 launch 的默认世界/地图已统一指向 `world666.world` / `world666map.yaml`：
`agv_description/sim.launch.py`、`agv_navigation/navigation.launch.py`、`agv_navigation/mapping.launch.py`；
`agv_stack.launch.py` 的 `map_yaml` 兜底为 `share/agv_navigation/maps/world666map/world666map.yaml`。

```
<工作区>/maps/<地图名>/            （地图库根可用环境变量 AGV_MAPS_DIR 覆盖）
   ├── <地图名>.pgm              栅格图（Nav2 用）
   ├── <地图名>.yaml             地图元数据（分辨率/原点）
   ├── waypoints.yaml            ★ 这份地图专属的点位
   ├── routes.yaml               ★ 这份地图专属的路线
   └── tasks.yaml                ★ 任务缓冲（控制台"任务系统"读写，按需生成）
```

当前实际内容（`maps/world666map/`）：`world666map.pgm`、`world666map.yaml`、`waypoints.yaml`、`routes.yaml`；
`tasks.yaml` 由控制台按需生成。本项目当前只有这一份地图，`maps/warehouse/` 与
`src/agv_navigation/maps/warehouse/` **已删除**（世界 `warehouse.world` / `new_world.world` 也已删除，
`world666.world` 是唯一一份）。

### 8.1 `waypoints.yaml`

结构：`frame_id` + `waypoints{点名: {x, y, yaw, kind}}`，`kind ∈ cargo|nav`。
下面是当前 `maps/world666map/waypoints.yaml` 的**实际内容**（5 个搬货点 + 起始点，全部 `kind: cargo`）：

```yaml
# 由 make_world_map.py 生成（用户 world666 场景）
# 每个"X号货架"= 搬货点（kind: cargo）：停在槽口外约 0.9 m、车头正对槽口，
# 到达后由任务层调用 /agv/cargo 服务触发顶升 / 下降（服务里怎么动由硬件决定）。
frame_id: map
waypoints:
  起始点: {x: 0.0, y: 0.0, yaw: 0.0, kind: cargo}
  1号货架:
    x: 1.630
    y: 2.660
    yaw: 0.000
    kind: cargo
  2号货架:
    x: -7.570
    y: -7.410
    yaw: -3.080
    kind: cargo
  3号货架:
    x: 1.560
    y: -6.170
    yaw: 0.000
    kind: cargo
  4号货架:
    x: -8.620
    y: 7.020
    yaw: 1.550
    kind: cargo
  5号货架:
    x: -7.810
    y: -2.510
    yaw: -3.100
    kind: cargo
```

一个含 `nav` 点的例子（`nav` 点只用来组成路线，不会被当成取货点/放货点）：

```yaml
frame_id: map
waypoints:
  起始点: {x: 0.0, y: 0.0, yaw: 0.0, kind: nav}
  走廊A:
    x: 0.500
    y: -2.000
    yaw: 1.571
    kind: nav
```

读取规则（`WaypointStore._load`）：

- 某点必须同时含 `x`/`y`/`yaw` 且能转 float，否则**整条丢弃**（不让节点崩）；
- `kind` 缺失或非法 → 按 `cargo` 处理；
- 文件里有 `frame_id` 时以文件为准，否则用节点参数 `frame_id`；
- 文件不存在或 YAML 解析失败 → 降级为空表，不报错。

写回规则（`WaypointStore._save`）：整体重写文件，`sort_keys=True`，`allow_unicode=True`（中文不转义），
每个点只写 `{x, y, yaw, kind}` 四个字段——**手工在文件里加的额外字段在下次经服务存点时会被丢掉**。

### 8.2 `routes.yaml`

结构：`frame_id` + `routes{路线名: {points: [{name, x, y, yaw}, ...]}}`。
点的坐标是**存路线那一刻从点位表复制的快照**。
当前 `maps/world666map/routes.yaml` 是**空表**（还没存过路线）：

```yaml
# 本文件的路线与地图绑定（路线由若干点位的位姿快照组成）
frame_id: map
routes: {}
```

由 `route` + `save` 存过路线之后长这样：

```yaml
frame_id: map
routes:
  巡检A:
    points:
      - {name: 1号货架, x: 1.63, y: 2.66, yaw: 0.0}
      - {name: 2号货架, x: -7.57, y: -7.41, yaw: -3.08}
  空跑:
    points:
      - {name: 5号货架, x: -7.81, y: -2.51, yaw: -3.1}
```

读取规则：`points` 里每个元素必须含 `x`/`y`/`yaw`（`name` 可空），非法项丢弃；
**清洗后为空的路线不入表**（避免留下空壳）。写回同样是整体重写、键排序、中文不转义。

### 8.3 `tasks.yaml`

控制台"任务系统"的任务缓冲，**节点不读它**（`task_manager_node` 完全不关心）。
结构：`tasks: [{pick, drop, loops, route}]`。

```yaml
# 本文件的搬运任务与地图绑定；由控制台"任务系统"读写
tasks:
- pick: 1号货架
  drop: 2号货架
  loops: 3
  route: 巡检A
- pick: 3号货架
  drop: 4号货架
  loops: 1
  route: ''
```

字段语义：`pick` 取货点名、`drop` 放货点名、`loops` 循环次数（读取时下限 1）、
`route` 可选路线名（空串=不用路线）。控制台执行时对每条任务按 `loops` 次数发 `start_task`
（带 `route`），并等 `TaskState` 回到 `IDLE`/`DONE` 才发下一条。

---

## 9 兼容性 / 变更记录

### 9.1 已移除的接口（历史，仅供追溯）

> 本节是**唯一**允许出现已删除接口名的地方。**代码里这些名字已全部不存在**，
> 不要在客户端里使用；这里保留只为追溯旧版本、理解迁移方向。

**本次合并移除的 8 个服务**（点位 4 + 路线 4）：

| 已移除的服务 | 类型 | 原来干什么 | 现在怎么办 |
|---|---|---|---|
| `/agv/waypoint/save` | `WaypointSave` | 存/覆盖一个点位 | `/agv/store`，`resource=waypoint, action=save` |
| `/agv/waypoint/get` | `WaypointGet` | 按名查一个点位 | `/agv/store`，`resource=waypoint, action=get` |
| `/agv/waypoint/list` | `WaypointList` | 列出全部点位 | `/agv/store`，`resource=waypoint, action=list` |
| `/agv/waypoint/delete` | `WaypointDelete` | 删除一个点位 | `/agv/store`，`resource=waypoint, action=delete` |
| `/agv/route/save` | `RouteSave` | 存/覆盖一条路线 | `/agv/store`，`resource=route, action=save` |
| `/agv/route/get` | `RouteGet` | 查路线全部途经点 | `/agv/store`，`resource=route, action=get` |
| `/agv/route/list` | `RouteList` | 列出全部路线 | `/agv/store`，`resource=route, action=list` |
| `/agv/route/delete` | `RouteDelete` | 删除一条路线 | `/agv/store`，`resource=route, action=delete` |

**更早一次删除**（方案改为"到点即顶升"后已彻底移除）：

| 已删除的接口 | 原来干什么 | 现在怎么办 |
|---|---|---|
| 服务 `agv_interfaces/Dock`（`Dock.srv`） | 到预备位后闭环精定位，收敛到槽口 | **已移除**，无替代；改为导航直接到搬货点，**到点即顶升** |
| 消息 `agv_interfaces/DockConfig` | 点位里的精定位配置 | **已移除**；点位只存 `x/y/yaw/kind` |
| 消息 `agv_interfaces/DockDeviation` | 精定位实时偏差 | **已移除**，无替代 |
| 消息 `agv_interfaces/LiftStatus` | 顶升到位/货物在位/限位/故障码 | **已移除**，改用 `/agv/cargo/busy`（`std_msgs/Bool`）+ `/agv/cargo` 服务的 `busy` 回执 |
| 包 `agv_docking`（含 `dock_node`） | 凵槽激光特征匹配 + 位姿解算 + 点镇定闭环 | **整包已移除** |
| 话题 `/agv/lift/*`、`/lift/status` | 顶升状态上报 | **已移除**，改用 `/agv/cargo/busy` |
| 话题 `/agv/cargo/active` | 告知本次搬哪个货物实体（多货物跟随） | **已移除**，无替代（单货物假设） |
| 话题 `/agv/dock/*`（`/agv/dock/active`、`/agv/dock/deviation`） | 精定位期间置位 / 实时偏差 | **已移除**，无替代 |
| 总线指令 `lift`、`lower` | 经 `/agv/command` 触发顶升/下降 | **已移除**；搬货**只走服务** `/agv/cargo`（`CargoCmd`） |
| 总线指令 `dock` | 经总线触发精定位 | **已移除**，无替代 |
| 总线指令 `query` | 经总线问状态 | **已移除**；`/agv/command` 是话题、收不到回答，查询改用服务 `/agv/query`（`AgvQuery`） |
| 状态 `ALIGN_LOAD`、`ALIGN_UNLOAD`、两段式到点（`pre_dock` 预备位偏移） | 到预备位 → 对位 → 再插入 | **已移除**；`SEQUENCE` 现在只有 `GO_LOAD → LIFTING → TRANSPORT → LOWERING → DONE` |
| `/agv/query` 的 `what=waypoints` / `what=routes` 取值 | 经查询服务列出点位/路线 | **已移除**；改走 `/agv/store` 的 `action=list`。现在传旧值不报错，会按 `state` 返回（见 §3.4） |
| 参数 `precheck_enabled` | 点位预检开关 | **已移除**；判据改为"有地图用地图、没地图用 `field_*` 兜底"，没有"关闭预检"这个状态 |
| 参数档位目录 `config/sim/` | 仿真参数覆盖层 | **已删除**；仿真与默认值无差异，`param_profile` 默认空 = 只用 `config/default`。`config/real/` 保留（现只有 README 占位） |
| 量化指标（"成功率 98%""误差 ≤4cm"等） | — | **不再作为接口承诺**，文档中不出现 |
| 世界 `warehouse.world`、`new_world.world` | 旧场景世界 | **已删除**；统一为 `src/agv_description/worlds/world666.world`（唯一一份） |
| 地图 `maps/warehouse/`、`src/agv_navigation/maps/warehouse/` | 旧场景栅格图 + 点位 | **已删除**；统一为 `maps/world666map/world666map.{pgm,yaml}` |
| 点位里的 `dock_enabled`/`dock_method`/`slot_*`/`cargo_model`/`cargo_present`/`dock_*_tol` 字段 | 精定位与货物跟随的配置 | **已删除**；`maps/world666map/waypoints.yaml` 现在只有 `x`/`y`/`yaw`/`kind` |

对 `/agv/command` 的影响：白名单从 8 条收窄为 **5 条**
（`goto | start_task | cancel_task | stop | resume`）。
现在往总线发 `lift` / `lower` / `dock` / `query` 会得到 `未知指令「…」，已拒绝`
（有单测 `test_removed_commands_rejected` 守着这个行为）。

### 9.2 接口合并：12 个文件 → 6 个

**接口文件从 12 个降到 6 个**，减半，但**能力一条没少**。两处改动：

| 改动 | 原来 | 现在 | 为什么 |
|---|---|---|---|
| 合并服务 | 8 个：`Waypoint{Save,Get,List,Delete}` + `Route{Save,Get,List,Delete}`（`.srv`） | 1 个：`AgvStore.srv`（`resource` + `action` 二维分发） | 8 个服务**结构完全相同**，都是"给个名字，对一个带名字的位姿列表做增删查列"。点位＝只有 1 个位姿的资源，路线＝有 N 个位姿的资源，**本来是同一个东西**，合并是去掉重复而不是硬凑 |
| 新增消息 | 无：查询结果用 5 个平行数组 `names[]/xs[]/ys[]/yaws[]/kinds[]` | `NamedPose.msg`：1 个数组、每项自带全部字段 | **消灭一类静默 bug**：平行数组靠调用方自觉保证下标对齐，某个数组短一位就会把 A 点名字配到 B 点坐标上——**不报错、不崩溃，车开去别的地方**。改用 `NamedPose[]` 后这类错位在类型层面就不存在了 |

合并带来的其它好处：

- **校验与回执只有一处实现**（`store_dispatch.dispatch()`），不会再出现"这个服务有 `ok`、
  那个没有"——旧的列表类服务缺 `ok` 的不一致正是被这样消掉的；
- **调用方从 8 个客户端降到 1 个**（`agv_task/store_client.py`）；
- **新增一种资源（如"工位"）只需扩 `resource` 取值**，不用再加 4 个服务；
- 调度逻辑抽成纯 Python 纯函数，**可单测**，不必起 ROS 才能验证。

字段演进（相对再早的 3 msg + 3 srv）：

| 接口 | 变化 | 说明 |
|---|---|---|
| `NamedPose.msg` | **新增** | 带名字的位姿，取代 5 个平行数组 |
| `AgvStore.srv` | **新增** | 取代 8 个同构 CRUD 服务 |
| `AgvQuery.srv` | `what` 取值**收窄为** `state` / `pose` | 点位/路线列表改走 `/agv/store`，本服务只答运行时状态 |
| `AgvCommand.msg` | **新增** `route` 字段 | 可选路线名，实现"多点导航"；空串=不用路线 |
| `TaskState.msg` | （无变化） | — |
| `CargoCmd.srv` | **新增服务** | 搬货（顶升/下降）二合一，替代总线 `lift`/`lower` |
| — | **新增话题** `/agv/cargo/busy` | 搬货中标志 |
| — | **新增话题** `/agv/manual_mode` | 手动/自动互锁 |

**当前接口清单（6 个文件）**：

| 类别 | 文件 | 用途 |
|---|---|---|
| msg | `NamedPose.msg` | 带名字的位姿（`/agv/store` 的 `items[]`） |
| msg | `AgvCommand.msg` | 统一指令总线 |
| msg | `TaskState.msg` | 任务状态上报 |
| srv | `AgvStore.srv` | 点位/路线统一存储 |
| srv | `AgvQuery.srv` | 运行时状态 / 位姿查询 |
| srv | `CargoCmd.srv` | 搬货（顶升 / 下降） |

### 9.3 接口包旁证文件

| 文件 | 状态 |
|---|---|
| `src/agv_interfaces/CMakeLists.txt` | ✅ 已同步：`rosidl_generate_interfaces` 只列 6 个文件，并在注释里说明"原先是 12 个" |
| `src/agv_interfaces/package.xml` | ✅ 已同步：描述为"三条消息 + 三个服务"；`<depend>std_msgs</depend>` 已删（6 个接口只用 ROS 2 内置基础类型） |
| `src/agv_waypoint/agv_waypoint/waypoint_node.py` | ✅ 已同步：只注册 `/agv/store` 一个服务；节点头注说明"为什么只有 1 个服务" |
| `src/agv_waypoint/agv_waypoint/store_dispatch.py` | ✅ 已同步：纯逻辑调度，`RESOURCES`/`ACTIONS` 常量与本文 §3.2 一致 |
| `src/agv_task/agv_task/store_client.py` | ✅ 已同步：单个 `StoreClient`（不再是 8 个客户端） |
| `src/agv_task/README.md` | ✅ 已同步：白名单只 5 条，明确 `lift`/`lower`/`dock` 不在白名单，搬货走服务 |
| `src/agv_interfaces/README.md` | ✅ 已同步：按"3 msg + 3 srv"重写，含 `NamedPose` 与 `/agv/store` 专节，旧服务名只出现在其"已移除的接口"表中 |

### 9.4 坐标范围：`field_*` 已与实测地图对齐

曾经存在的真实矛盾：`field_*`（旧值 x ∈ [-11.30, 10.45]、y ∈ [-10.42, 9.91]）
是按**生成脚本的参考图**包围盒定的，比实测地图（x ∈ [-10.70, 4.15]、y ∈ [-9.83, 9.22]）宽一圈，
x 正方向差得最多（10.45 vs 4.15）——于是同一个坐标会"门卫放行、预检拒绝"。

**现已修好**，两层措施：

1. **判据收敛**：坐标合法性只有 `GoalValidator` 一个判据（§5.3），
   门卫与预检问的是**同一个对象**，不可能再各判各的；
2. **兜底值对齐**：`task.yaml` 的四个 `field_*` 已改为与实测地图**完全一致**
   （`-10.70` / `4.15` / `-9.83` / `9.22`），所以即便退化到矩形兜底，结论也与地图判据一致。

| 项 | 数值 | 来源 |
|---|---|---|
| 栅格地图实际覆盖 = 兜底范围 | x ∈ [-10.70, **4.15**]，y ∈ [-9.83, 9.22] | `maps/world666map/world666map.yaml` 的 `origin: [-10.7, -9.83]` + `world666map.pgm` 的 **594 × 762** 格 @0.025 m/格 |
| `field_*` 兜底范围（**已对齐**） | x ∈ [-10.70, 4.15]，y ∈ [-9.83, 9.22] | `config/default/task.yaml` |
| 参考图（**仅供参照，不参与判据**） | x ∈ [-11.304, 10.446]，y ∈ [-10.423, 9.902] | `world666map_ref.yaml`，870 × 813 @0.025 |

> `maps/world666map/waypoints.yaml` 的 5 个搬货点与「起始点」（x ∈ [-8.62, 1.63]，
> y ∈ [-7.41, 7.02]）都在上述范围内，**现有点位不受任何边界差异影响**。

---

## 10 可直接复制的完整示例

前提：先 `source install/setup.bash`，并已 `ros2 launch agv_bringup agv_stack.launch.py ...` 拉起业务节点。

### 10.1 存两个搬货点 + 一个导航点

```bash
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'waypoint', action: 'save', name: '取货点', x: 1.0, y: 0.0, yaw: 0.0, kind: 'cargo'}"

ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'waypoint', action: 'save', name: '放货点', x: 3.0, y: 1.0, yaw: 0.0, kind: 'cargo'}"

ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'waypoint', action: 'save', name: '走廊A', x: 0.5, y: -2.0, yaw: 1.571, kind: 'nav'}"
```

### 10.2 存一条路线（按点名复制位姿）

```bash
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'route', action: 'save', name: '巡检A', point_names: ['取货点', '走廊A', '放货点']}"
```

### 10.3 查点位 / 查路线

```bash
# 查一个点位的坐标与类型
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'waypoint', action: 'get', name: '取货点'}"

# 列出全部点位（items 有值，names/counts 也有值）
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'waypoint', action: 'list'}"

# 查一条路线的全部途经点
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'route', action: 'get', name: '巡检A'}"

# 列出全部路线（只有 names/counts，items 为空）
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'route', action: 'list'}"
```

### 10.4 点名导航（带路线做多点导航）

```bash
# 单点：去「取货点」
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'goto', args: ['取货点'], route: '', confidence: 1.0, request_id: 'r1'}"

# 多点：先经路线「巡检A」，再到「放货点」
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'goto', args: ['放货点'], route: '巡检A', confidence: 1.0, request_id: 'r2'}"
```

### 10.5 坐标导航

```bash
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'goto', args: ['3.0', '1.2', '0.0'], request_id: 'r3'}"

# 两参数：yaw 按 0 处理
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'goto', args: ['3.0', '1.2'], request_id: 'r4'}"
```

### 10.6 完整搬运（取货点 → 顶升 → 运输 → 放货点 → 下降）

```bash
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'start_task', args: ['取货点', '放货点'], route: '', confidence: 1.0, request_id: 't1'}"

# 带路线：取货点 → 路线各点 → 放货点
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'start_task', args: ['取货点', '放货点'], route: '巡检A', request_id: 't2'}"
```

### 10.7 看状态 / 问状态

```bash
ros2 topic echo /agv/task_state

ros2 service call /agv/query agv_interfaces/srv/AgvQuery "{what: 'state'}"
ros2 service call /agv/query agv_interfaces/srv/AgvQuery "{what: 'pose'}"
```

> 想看点位表 / 路线表，用 §10.3 的 `/agv/store`（`action=list`），**不要**再用
> `/agv/query` 的 `what=waypoints` / `what=routes`（已删除）。

### 10.8 急停与恢复

```bash
# 急停：任何状态都受理，绕过去重
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'stop', request_id: 's1'}"

# 恢复：清急停、清任务、回 IDLE
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'resume', request_id: 's2'}"

# 取消当前任务
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'cancel_task', request_id: 's3'}"
```

### 10.9 单独手动顶升 / 下降（调试搬货服务）

```bash
ros2 service call /agv/cargo agv_interfaces/srv/CargoCmd "{command: 'lift', request_id: 'm1'}"
ros2 topic echo /agv/cargo/busy          # 应变 true
ros2 service call /agv/cargo agv_interfaces/srv/CargoCmd "{command: 'lower', request_id: 'm2'}"
ros2 topic echo /agv/cargo/busy          # 应变 false
```

### 10.10 手动模式互锁

```bash
ros2 topic pub --once /agv/manual_mode std_msgs/msg/Bool "{data: true}"    # 手动：禁止导航
ros2 topic pub --once /agv/manual_mode std_msgs/msg/Bool "{data: false}"   # 自动：允许导航
```

### 10.11 验证被拒绝的指令（看 task_manager_node 日志）

```bash
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'lift', request_id: 'x1'}"        # → 拒绝：未知指令「lift」
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'goto', args: ['9999', '9999', '0'], request_id: 'x2'}"   # → 拒绝：在地图范围外
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'goto', args: ['取货点'], request_id: 'x3'}"
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'goto', args: ['取货点'], request_id: 'x3'}"   # → 拒绝：重复
```

### 10.12 验证 `/agv/store` 的失败分支

```bash
# 未知资源
ros2 service call /agv/store agv_interfaces/srv/AgvStore "{resource: 'station', action: 'list'}"
#   → ok=false, message='未知资源「station」，应为 waypoint 或 route'

# 未知动作
ros2 service call /agv/store agv_interfaces/srv/AgvStore "{resource: 'waypoint', action: 'fly'}"
#   → ok=false, message='未知动作「fly」，应为 save / get / list / delete'

# 路线里有点名不存在：整条拒绝，不写半条
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'route', action: 'save', name: '坏路线', point_names: ['取货点', '没有这个点']}"
#   → ok=false, message='这些点位不存在：没有这个点'

# 查不存在的点位：ok=false，但不报错
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'waypoint', action: 'get', name: '没有这个点'}"
#   → ok=false, message='点位「没有这个点」不存在'
```

---

## 11 接口清单速查

| # | 能力 | 接口 | 类型 | 关键字段 |
|---|---|---|---|---|
| 1 | 点名/坐标导航 | `/agv/command` | 话题（入） | `goto` + `args` + 可选 `route` |
| 2 | 完整搬运 | `/agv/command` | 话题（入） | `start_task` + `[取货点, 放货点]` |
| 3 | 取消 / 急停 / 恢复 | `/agv/command` | 话题（入） | `cancel_task` / `stop` / `resume` |
| 4 | 状态上报 | `/agv/task_state` | 话题（出） | `state, progress, target_location, message` |
| 5 | 存/查/列/删点位与路线 | `/agv/store` | **服务 ×1** | `resource`(`waypoint`/`route`) + `action`(`save`/`get`/`list`/`delete`) → `ok,items(NamedPose[]),names,counts,count` |
| 6 | 顶升 / 下降 | `/agv/cargo` | 服务 | `command`(`lift`/`lower`) → `ok,busy` |
| 7 | 搬货中标志 | `/agv/cargo/busy` | 话题（出） | `Bool` |
| 8 | 查询运行时状态 / 位姿 | `/agv/query` | 服务 | `what`(`state`/`pose`) → `ok,state,x,y,yaw,message` |
| 9 | 手动/自动互锁 | `/agv/manual_mode` | 话题（入） | `Bool` |
| 10 | 速度下达 | `/cmd_vel` | 话题（双向） | `Twist` |
| 11 | 速度到底盘 | `/cmd_vel_raw` | 话题（出） | `Twist` |
| 12 | 导航执行（内部） | `/navigate_to_pose` | Nav2 动作 | `PoseStamped` |

合计：**自定义消息 3 个**（`AgvCommand`、`TaskState`、`NamedPose`）、
**自定义服务 3 个**（`AgvStore`、`AgvQuery`、`CargoCmd`）、
**业务自定义话题 6 个**（`/agv/command`、`/agv/task_state`、`/cmd_vel`、`/cmd_vel_raw`、
`/agv/manual_mode`、`/agv/cargo/busy`）。

> 接口文件共 **6 个**（`msg/` 3 + `srv/` 3），原先为 12 个（`msg/` 2 + `srv/` 10）。
> 变化与理由见 §9.2。
