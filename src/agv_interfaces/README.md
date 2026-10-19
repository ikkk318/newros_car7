# agv_interfaces —— 接口契约包（自定义 msg / srv）

本包是全项目共用的**接口契约**：仿真、任务层、点位管理、控制台、语音、调度都通过这里的
消息与服务通信。**字段以 `msg/`、`srv/` 下的文件为准，改动必须同步所有调用方。**

> 具体由哪个节点提供哪个接口，见下表；接口语义的完整说明分别见
> `src/agv_task/README.md` 与 `src/agv_waypoint/` 的节点头注。

| 接口 | 提供方 |
|---|---|
| `/agv/command`、`/agv/task_state`、`/agv/query` | `agv_task` 的 `task_manager_node` |
| `/agv/store` | `agv_waypoint` 的 `waypoint_node` |
| `/agv/cargo`、`/agv/cargo/busy` | `agv_lift` 的 `cargo_node` |

**共 6 个接口文件：3 个 msg + 3 个 srv。**（原先是 12 个：点位/路线的 8 个同构 CRUD
服务已合并为 1 个 `AgvStore`，并新增 `NamedPose` 统一表示"带名字的位姿"。）

---

## 一、消息（msg，共 3 个）

### AgvCommand —— 统一指令总线（发到 `/agv/command`）

| 字段 | 类型 | 含义 |
|---|---|---|
| `source` | `string` | 指令来源：`"web"` / `"voice"` / `"scheduler"` |
| `command` | `string` | 动作名：`goto` \| `start_task` \| `cancel_task` \| `stop` \| `resume` |
| `args` | `string[]` | 参数：点名，或坐标 `[x, y, yaw]`（按字符串传入） |
| `route` | `string` | 可选：本次要经过的路线名（空 = 不用路线） |
| `confidence` | `float32` | 置信度（语音用 0~1；其他入口填 1.0） |
| `request_id` | `string` | 请求唯一号：用于去重与追踪 |

- `args` 语义：`goto` = `[点名]` 或 `[x, y]` 或 `[x, y, yaw]`；`start_task` = `[取货点名, 放货点名]`。
- `route` 给了就做多点导航：`goto` 走"路线各点 → 目标点"，`start_task` 走"取货点 → 路线各点 → 放货点"。
- `stop` 是最高优先级安全通道：永远受理、绕过去重。

### TaskState —— 任务状态（发到 `/agv/task_state`）

| 字段 | 类型 | 含义 |
|---|---|---|
| `task_id` | `string` | 任务唯一号（一次完整搬运一个） |
| `state` | `string` | 状态串：`IDLE` / `GO_LOAD` / `LIFTING` / `TRANSPORT` / `LOWERING` / `DONE` / `ERROR` / `PAUSED` |
| `target_location` | `string` | 当前目标点名（取货点 / 路线点 / 放货点） |
| `progress` | `float32` | 任务进度 0~1 |
| `message` | `string` | 中文提示，可直接显示 |

- 发布时机：1 Hz 周期 + 状态变化时。
- 进度表：`GO_LOAD` 0.15、`LIFTING` 0.40、`TRANSPORT` 0.60、`LOWERING` 0.85、`DONE` 1.0。

### NamedPose —— 带名字的位姿

| 字段 | 类型 | 含义 |
|---|---|---|
| `name` | `string` | 名字（点名；作路线途经点时是复制当时的点名，仅便于人看） |
| `x` / `y` | `float64` | 位置（map 坐标系，m） |
| `yaw` | `float64` | 朝向（rad，REP-103：逆时针为正） |
| `kind` | `string` | `"cargo"` 搬货点 / `"nav"` 导航点；路线途经点留空 |

> **为什么要这个类型**：原先点位/路线的查询结果用「5 个平行数组」返回
> （`names[] / xs[] / ys[] / yaws[] / kinds[]`），调用方必须自己保证下标对齐；
> 一旦某个数组短一位，就会**静默地把 A 点名字配到 B 点坐标上**——不报错、
> 不崩溃，车却开去别的地方。改成"一个数组、每个元素自带全部字段"后，
> 这类对齐错误在类型层面就不存在了。

## 二、服务（srv，共 3 个）

### AgvStore —— 点位 / 路线统一存储（`/agv/store`）

原先有 8 个服务：`Waypoint{Save,Get,List,Delete}` + `Route{Save,Get,List,Delete}`，
两两结构完全相同，都是"给一个名字，对一个带名字的位姿列表做增删查列"。
**点位就是只有 1 个位姿的资源，路线就是有 N 个位姿的资源**——本来就是同一个东西，
所以合并成这一个：用 `resource` 选资源、用 `action` 选动作。

| 请求字段 | 类型 | 含义 |
|---|---|---|
| `resource` | `string` | `"waypoint"` 点位 / `"route"` 路线 |
| `action` | `string` | `"save"` / `"get"` / `"list"` / `"delete"` |
| `name` | `string` | 资源名（`save`/`get`/`delete` 用；`list` 忽略） |
| `x` / `y` / `yaw` | `float64` | 仅 `waypoint + save`：位置与朝向 |
| `kind` | `string` | 仅 `waypoint + save`：`"cargo"` / `"nav"`（留空按 `cargo`） |
| `point_names` | `string[]` | 仅 `route + save`：途经点名列表（按行进顺序，至少 1 个） |

| 响应字段 | 类型 | 含义 |
|---|---|---|
| `ok` | `bool` | 是否成功（**每个动作都会给，包括 `list`**） |
| `message` | `string` | 中文回执，可直接打印 |
| `items` | `NamedPose[]` | `get`：该资源的位姿列表；`waypoint + list`：全部点位；`route + list` **故意为空**（见下）；其余为空 |
| `names` | `string[]` | 仅 `list`：资源名列表（与 `counts` 同序） |
| `counts` | `uint32[]` | 仅 `list`：各资源的位姿个数（与 `names` 同序） |
| `count` | `uint32` | `get` = 位姿个数；`save(route)` = 实际写入的途经点数；`list` = 条目总数 |

各动作语义：

| resource | action | 语义 |
|---|---|---|
| `waypoint` | `save` | 存 / 覆盖一个点位（重名覆盖）。坐标非数字则 `ok=false`，不写盘。 |
| `waypoint` | `get` | 按名查一个点位，结果在 `items[0]`。不存在时 `ok=false`，不报错。 |
| `waypoint` | `list` | 列出全部点位，`items` 按点名排序（顺序稳定）。 |
| `waypoint` | `delete` | 删一个点位，不存在则 `ok=false`。**不影响已存路线**（路线存的是位姿快照）。 |
| `route` | `save` | 存 / 覆盖一条路线。按 `point_names` 从点位表**复制位姿**，所以路线里是快照，之后改点位不影响已存路线。**有点名不存在就整条拒绝，不写半条**；空路线也拒绝。`count` = 实际写入的途经点数。 |
| `route` | `get` | 查一条路线的全部途经点，`items` 与 `point_names` 顺序一致，`kind` 留空。 |
| `route` | `list` | 列出全部路线，`names` 与 `counts` 同序。**`items` 为空**——`items` 里每项的 `name` 是"途经点名"，一个扁平数组表达不了"哪条路线的哪些点"这层归属；要看某条路线的途经点请用 `route + get`。 |
| `route` | `delete` | 删一条路线，不存在则 `ok=false`。 |

```bash
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'waypoint', action: 'save', name: '取货点', x: 1.0, y: 0.0, yaw: 0.0, kind: 'cargo'}"
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'waypoint', action: 'get', name: '取货点'}"
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'waypoint', action: 'list'}"
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'waypoint', action: 'delete', name: '取货点'}"

ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'route', action: 'save', name: '巡检A', point_names: ['走廊1', '取货点']}"
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'route', action: 'get', name: '巡检A'}"
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'route', action: 'list'}"
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
  "{resource: 'route', action: 'delete', name: '巡检A'}"
```

> ★ 本服务**不做**"点位是否可达"的判断——那是目标点判据 `GoalValidator`
> （`agv_common/goal_check.py`）的事，`goto` 坐标与导航前都用它。

### AgvQuery —— 运行时查询（`/agv/query`）

| 请求字段 | 类型 | 含义 |
|---|---|---|
| `what` | `string` | `"state"` 任务状态 / `"pose"` 当前位置 |

| 响应字段 | 类型 | 含义 |
|---|---|---|
| `ok` | `bool` | 是否查询成功 |
| `state` | `string` | 任务状态串（`what=state` 时） |
| `x` / `y` / `yaw` | `float64` | 当前位置与朝向（`what=pose` 时） |
| `message` | `string` | 中文回执 |

- `/agv/command` 是话题，广播发出去收不到回答，所以查状态必须走这个服务。
- **点位表 / 路线表不在这里查**：它们属于"存储内容"，统一走 `AgvStore`
  （`resource=waypoint|route, action=list`）。本服务只回答"运行时状态"，职责单一。
  （旧版本的 `what=waypoints|routes` 取值已移除。）

### CargoCmd —— 搬货（`/agv/cargo`）

| 请求字段 | 类型 | 含义 |
|---|---|---|
| `command` | `string` | `"lift"`（顶升，开始搬货）/ `"lower"`（下降，放货完成） |
| `request_id` | `string` | 请求号，仅用于日志追踪 |

| 响应字段 | 类型 | 含义 |
|---|---|---|
| `ok` | `bool` | 是否受理 |
| `busy` | `bool` | **执行后**的"搬货中"状态：`lift` 成功 → true，`lower` 成功 → false |
| `message` | `string` | 中文回执 |

> ★ 全项目唯一与搬货硬件打交道的地方；现阶段服务内不做任何动作，
> 只改标志后立即返回成功。接真实硬件时改 `agv_lift/agv_lift/cargo_node.py` 即可，
> 接口不变。

## 三、编译与依赖

```bash
colcon build --packages-select agv_interfaces
source install/setup.bash

ros2 interface show agv_interfaces/msg/AgvCommand
ros2 interface show agv_interfaces/srv/AgvStore
ros2 interface list | grep agv_interfaces
```

`CMakeLists.txt`：

| 项 | 内容 |
|---|---|
| `find_package` | `ament_cmake`、`rosidl_default_generators` |
| `rosidl_generate_interfaces` | 列出全部 6 个文件：3 个 `msg/*.msg` + 3 个 `srv/*.srv` |
| 结尾 | `ament_package()`（必须放在文件末尾） |

`package.xml`：

| 项 | 内容 |
|---|---|
| `buildtool_depend` | `ament_cmake`、`rosidl_default_generators` |
| `exec_depend` | `rosidl_default_runtime` |
| `member_of_group` | `rosidl_interface_packages`（接口包固定写法） |

> 实际情况说明：**所有 6 个接口都只用 ROS 2 内置基础类型**（`string` / `bool` / `float32` /
> `float64` / `uint32` 及其数组），没有任何 `geometry_msgs` 或 `std_msgs` 字段。

## 四、已移除的接口

| 已移除 | 原用途 | 替代方案 |
|---|---|---|
| `srv/WaypointSave.srv`、`WaypointGet`、`WaypointList`、`WaypointDelete` | 点位的增删查列（4 个服务） | 合并进 **`srv/AgvStore.srv`**：`resource='waypoint'` + `action` |
| `srv/RouteSave.srv`、`RouteGet`、`RouteList`、`RouteDelete` | 路线的增删查列（4 个服务） | 合并进 **`srv/AgvStore.srv`**：`resource='route'` + `action` |
| `AgvQuery` 的 `what=waypoints` / `what=routes` | 把点位表 / 路线表序列化进 `message` | 改用 `AgvStore` 的 `action='list'`，返回结构化的 `items`（点位）/ `names`+`counts`（路线），不必再解析文本 |
| `srv/Dock.srv` | 精定位服务：到预备位后调它做插入对接 | 不再需要：方案改为**到点即停**，导航终点就是搬货点；无替代服务 |
| `msg/DockConfig.msg` | 一个点位的精定位配置（是否启用 / 方法 / 凵槽几何 / 各项容差） | 删除。点位只存 `name / x / y / yaw / kind` |
| `msg/DockDeviation.msg` | 精定位解算出的相对位姿偏差（横向 / 航向 / 深度） | 删除，无替代 |
| `msg/LiftStatus.msg` | 顶升状态（`lifted` / `cargo_present` / 上下限位 / 故障码），发 `/lift/status` | 改为 **`CargoCmd` 服务 `/agv/cargo`**（顶升 / 下降二合一，回执 `ok` / `busy` / `message`）<br>+ **话题 `/agv/cargo/busy`**（`std_msgs/Bool`，`true` = 正在搬货） |

同时移除的总线指令：`lift` / `lower` / `dock` / `query` —— 搬货不再走 `/agv/command`，
由任务层直接调 `/agv/cargo` 服务；查运行时状态改用 `/agv/query` 服务（`AgvQuery`）。
`AgvCommand.command` 的合法取值只剩 `goto` / `start_task` / `cancel_task` / `stop` / `resume`。

> 本包接口**只删不增字段级兼容层**：`AgvStore` 已不含任何 `dock_*` 字段，
> 也不返回货物的"在位"信息。
