# AGV 顶升搬运车 · 方案总纲（当前实现版）

> 本文档描述**当前代码真实的样子**，是队友/答辩读者的入口文档。平台（冻结）：**Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11**（仿真优先）；工作区为标准 `src/` 布局（示例路径 `/home/yuki/new_ros_car`，以本机实际路径为准）。
> 内容全部来自**静态核对代码**（逐文件读源码与配置）。本机没有 ROS 2 运行环境，故**不声称**任何仿真或实车验证结论，也不写成功率、精度之类的量化指标。

## 1. 一句话定位与当前阶段目标

做一台**顶升式（Kiva 型）搬运 AGV**：车开到货架（搬货点）→ **到点即顶升** → 导航运输 → 到放货点 → 下降放货。

**当前阶段目标（务实版）**：先把「**到点导航 + 统一指令 + 任务/状态上报**」这条主干跑通、跑稳；搬货动作只留一个**服务口子**（`/agv/cargo`），里面暂时不做真实动作、立即返回成功。这样软件主干能独立推进与验收，等真实顶升机构就位只补一个文件（§10）。

**本阶段明确不做**：精定位 / 二次对位（docking）、货物/货架跟随、两段式到点（预备位）、总线上的 `lift`/`lower`/`dock` 指令、量化精度验收指标。这些在早期方案里存在过，现已从代码删除；**当前阶段不追求量化精度，后续按需补**。

## 2. 系统分层与数据流

```
 入口层  tools/agv_console.py（控制台 v3.0：建图/导航双模式，发指令、调服务、拉起整栈）
              │ 发布 /agv/command (AgvCommand)     │ 直接调服务
              ▼                                     ▼
 任务层   ┌────────────────────────────┐   /agv/store  /agv/cargo
          │ task_manager_node          │◄─ 作为客户端调统一存储/搬货服务
          │ 任务层唯一执行者：门卫      │
          │ 搬运状态机 + 单点导航 + 安全 │
          └──┬───────────────────┬─────┘
             │ Nav2 动作接口      │ 服务调用 /agv/cargo(lift|lower)
             ▼                   ▼
 服务层   Nav2 导航栈         cargo_node           waypoint_node
         (AMCL/规划/控制)      唯一硬件对接点        统一存储 /agv/store
             │ /cmd_vel          /agv/cargo/busy     YAML 持久化
 硬件边界    ▼                   │（现阶段不动硬件）
       ┌──────────────────────────────────┐
       │ velocity_arbiter  ← /scan        │ 近障减速 + 左右择空 + 缓回 + 后方保护 + 限幅
       │ /cmd_vel ──────────► /cmd_vel_raw│ → Gazebo 差速插件（实车阶段：下位机）
       └──────────────────────────────────┘
 安全链路：急停与超时补发的零速也发到 /cmd_vel，因此自然经过 velocity_arbiter 转发到底盘。
```

三个核心取舍：① **统一指令总线**——所有入口都只发 `/agv/command`，由 `task_manager_node` 一门受理，指令不打架、校验只有一处、状态只有一处可查；② **搬货只留一个服务口子**——任务层不碰硬件、只调 `/agv/cargo`，"换硬件"= 改一个文件、主干一行不改；③ **速度在硬件边界收口**——上层只发 `/cmd_vel`（想走多快），由 `velocity_arbiter` 统一避障与限幅后发给底盘，限速与防撞只有一处实现。

## 3. 各节点职责

### 3.1 节点、话题、服务

| 节点 | 包 / 可执行文件 | 订阅 | 发布 | 服务（提供 / 调用） | 参数 |
|---|---|---|---|---|---|
| `waypoint_node` | `agv_waypoint` / `waypoint_node` | — | — | **提供** `/agv/store`（`AgvStore`，点位与路线的**统一**增删查列） | `waypoint_file`、`routes_file`、`frame_id` |
| `cargo_node` | `agv_lift` / `cargo_node` | — | `/agv/cargo/busy`（`std_msgs/Bool`） | **提供** `/agv/cargo`（`CargoCmd`） | `service_name`、`busy_topic`、`publish_period` |
| `task_manager_node` | `agv_task` / `task_manager_node` | `/agv/command`、`/cmd_vel`、`/agv/manual_mode` | `/agv/task_state`、`/cmd_vel`（急停/超时零速） | **提供** `/agv/query`；**调用** `/agv/store`（`resource=waypoint` 取坐标、`resource=route` 取途经点）、`/agv/cargo` | 见 §3.2 |
| `velocity_arbiter` | `agv_hardware` / `velocity_arbiter` | `/cmd_vel`、`/scan` | `/cmd_vel_raw` | — | 见 §3.3 |

**控制台**（脚本，非包）：`tools/agv_console.py`，节点名 `agv_console`；订阅 `/agv/task_state`、`/agv/cargo/busy`；发布 `/agv/command`、`/cmd_vel`、`/agv/manual_mode`；另直接调统一存储/搬货/查询服务，并负责起整栈、管地图库、读写任务缓冲、一键自检。

### 3.2 `task_manager_node` 参数

| 参数 | 默认 | 作用 |
|---|---|---|
| `field_min` / `field_max` / `field_min_y` / `field_max_y` | -50 / 50 / -50 / 50 | **没有地图时的兜底**场地边界（m），x 与 y 各自独立校验。`config/default/task.yaml` 已把它对齐实测地图范围 `-10.70 / 4.15 / -9.83 / 9.22` |
| `cmd_timeout_ms` / `estop_budget_ms` | 500 / 200 | 速度心跳超时(ms) / 急停发出停车的时限(ms) |
| `waypoint_frame` | map | 点位/目标坐标系 |
| `nav_print_period` / `nav_timeout_s` | 1.0 / 180.0 | 导航反馈打印周期(s) / 单段导航总超时(s，超时取消并判失败) |
| `cargo_timeout_s` | 5.0 | 等待 `/agv/cargo` 应答的上限（s） |
| `robot_clearance` | 0.30 | 目标点所需净空(m)（仅在地图判据下生效） |
| `map_yaml` | ""（launch 注入） | 目标点判据用的地图 yaml；**给了就用地图判，为空才退化为上面的 `field_*` 矩形** |

> **目标点合法性只有一个判据**：`agv_common/goal_check.py` 的 `GoalValidator`（见 §9.1）。原先"门卫按 `field_*` 判、预检按地图判"两套判据已收敛为一套，`precheck_enabled` 开关随之删除 —— 判据是必做的，不再可关。

### 3.3 `velocity_arbiter` 参数

| 参数 | 默认 | 作用 |
|---|---|---|
| `stop_distance` / `slow_distance` / `take_over_distance` | 0.30 / 0.90 / 0.60 | 刹停距离 / 减速起点 / 择向接管距离（m） |
| `takeover_max_speed` / `slow_min_speed` | 0.25 / 0.10 | 允许接管的车速上限 / 减速过渡段最小线速度（m/s） |
| `front_half_angle_deg` / `side_max_angle_deg` / `yaw_handback_rate` | 45.0 / 90.0 / 0.30 | 前方扇区半角 / 侧方统计最大角(度) / 角速度交还步长(rad/s) |
| `takeover_half_angle_deg` | 20.0 | 接管闸门锥半角（度）：只管"要不要夺向"，比前扇区窄。★ 别开大（前锥 45° 边界会扫到平行槽侧壁，骗开闸门 → 槽内反复夺向） |
| `rear_half_angle_deg` | 20.0 | 后方扇区半角（度）：倒车时按此窄锥看车尾，太近则刹停/减速。★ 别开大（槽内平行侧壁会进锥，倒车被自己压住） |
| `max_linear` / `max_angular` | 0.5 / 1.0 | **最终速度限幅（安全约束唯一收口）** |
| `scan_topic` / `cmd_in` / `cmd_out` | `/scan` / `/cmd_vel` / `/cmd_vel_raw` | 话题名 |
| `scan_timeout` / `cmd_timeout` / `publish_rate` | 0.5 / 0.5 / 20.0 | 激光超时(s，超时只限幅放行) / 上层速度超时(s，超时即零速) / 周期重发频率(Hz) |

## 4. 指令总线与任务状态机

### 4.1 指令总线 `/agv/command`（`AgvCommand`）

字段：`source`、`command`、`args[]`、`route`、`confidence`、`request_id`。**白名单（其余一律拒绝）**：`goto`、`start_task`、`cancel_task`、`stop`、`resume`。

| command | 参数 | 语义 |
|---|---|---|
| `goto` | `args=[点名]` 或 `[x, y]` 或 `[x, y, yaw]`；`route` 可选 | 去目标点；给了 `route` 就先走路线各点，**再到目标点** |
| `start_task` | `args=[取货点名, 放货点名]`；`route` 可选 | 完整搬运一轮；路线点在**顶升之后、放货点之前** |
| `cancel_task` | 无 | 取消当前任务、取消 Nav2 目标、发零速、状态回空闲 |
| `stop` | 无 | 急停：**任何状态都受理，绕过去重**（安全通道） |
| `resume` | 无 | 解除急停、清任务、回空闲待命 |

门卫（`CommandRouter`）四条规则：① **`stop` 永远受理**，在白名单校验与去重之前直接执行；② **`request_id` 去重**——非空 `request_id` 只受理一次，重复的回复"重复，已拒绝"，被拒绝的请求号也记下（防重试风暴），去重表最多 1000 条、超出淘汰最旧；③ **坐标校验**——`goto` 传 1 个参数当**点名**（点名可以是纯数字），传 2 个当 `[x, y]`（yaw=0），传 ≥3 个当 `[x, y, yaw]`，三者都须能解析成数字且 `x`/`y` **各自**在场地边界内，越界拒绝；④ **前置检查**（`goto`/`start_task`）——急停中拒绝并提示先 `resume`，手动模式拒绝，已有任务在跑拒绝（串行执行）。

### 4.2 搬运状态机（`TaskManager`，纯逻辑可单测）

```
IDLE ──start──► GO_LOAD ──► LIFTING ──► TRANSPORT ──► LOWERING ──► DONE ──► IDLE
                  │            │             │             │
  任一步失败 ──────┴────────────┴─────────────┴─────────────┴────► ERROR
  收到 stop（有活动任务）───────────────────────────────────────► PAUSED
```

| 状态 | 进度 | 中文提示 | 说明 |
|---|---|---|---|
| `IDLE` | 0.00 | 空闲，等待任务 | 可接新任务 |
| `GO_LOAD` | 0.15 | 前往取货点 | 搬运流程中取货点直连；`goto` 语义下路线点在它之前 |
| `LIFTING` | 0.40 | 顶升中 | 调 `/agv/cargo`，`command="lift"` |
| `TRANSPORT` | 0.60 | 运输中 | 先走路线各点，再到放货点 |
| `LOWERING` | 0.85 | 下降放货中 | 调 `/agv/cargo`，`command="lower"` |
| `DONE` | 1.00 | 搬运完成 | 随后自动回 `IDLE`（可接下一单） |
| `ERROR` | 0.00 | 任务异常，需人工复位 | 导航失败 / 搬货服务失败 |
| `PAUSED` | 0.00 | 已暂停（急停/停止） | 收到 `stop` 且当时有活动任务 |

迁移细节（与代码一致）：`step_success()` 沿 `GO_LOAD → LIFTING → TRANSPORT → LOWERING → DONE` 逐格推进，在 `DONE` 上再调一次即回 `IDLE`；`IDLE`/`ERROR`/`PAUSED` 三态**不可推进**。`on_estop()` 只在有活动任务时进 `PAUSED`，**不记**"暂停前那一格"——急停会取消 Nav2 目标并中止后台线程，那一段导航已作废；恢复（节点 `resume` 处理器）走"清急停 + 清任务 + 回 `IDLE`"，即**回空闲待命、重新配任务**，不做续跑（`TaskManager` 里已不再提供 `resume()` 方法，避免两套语义并存）。**单点导航状态**（`GotoController`）复用同一套状态串，只用 4 个：`IDLE →(start)→ GO_LOAD →(成功)→ DONE`，失败 → `ERROR`，`cancel()` 任意态 → `IDLE`；它在 `/agv/task_state` 里的进度是 `GO_LOAD` 0.5、`DONE` 1.0、`ERROR` 0.0。

### 4.3 状态上报 `/agv/task_state`（`TaskState`）与查询服务 `/agv/query`（`AgvQuery`）

`TaskState` 字段：`task_id`、`state`、`target_location`、`progress`、`message`。

- **发布时机**：1 Hz 周期 + 状态变化时（指令受理后、手动模式切换、各步推进、失败/急停时）。
- **优先级**：搬运任务非 `IDLE` 就报搬运状态；否则单点导航非 `IDLE` 就报导航状态；都空闲报 `IDLE`；**急停优先** —— 急停生效时一律显示 `PAUSED / 已急停（安全停止）`。
- `task_id` 形如 `task-0001`（一次完整搬运一个，自增）；`target_location` 在 `GO_LOAD`/`LIFTING` 阶段是取货点，在 `TRANSPORT`/`LOWERING` 阶段是放货点。

`AgvQuery` 是**运行时状态查询**，只回答两件事（`what` 按 `state` / `pose` 取值；其余值走默认分支，等同 `state`）：`state`（也是默认值）→ `state` + `message` 里的"状态（提示）目标 进度 导航状态"；`pose` → `x`/`y`/`yaw`（来自 `map → base_footprint` 的 TF，取不到时 `ok=false`）。为什么要有它：`/agv/command` 是**话题**，广播出去收不到回答，想知道"现在什么状态"必须用服务。

> **点位表 / 路线表不在这里查**（旧版的 `what=waypoints` / `what=routes` 已删除）：它们属于"存储内容"，统一走 `/agv/store` 的 `action=list`。这样同一个东西不会有两个服务都能查。

## 5. 点位 / 路线 / 任务缓冲

三者关系：**点位** = 一个坐标 + 一个名字 + 一个类型；**路线** = 一串点位位姿的**快照**；**任务缓冲** = 控制台自己的"待办搬运清单"（只有控制台读写，节点完全不关心）。

```
点位 waypoints.yaml ──(按点名复制位姿 = 快照)──► 路线 routes.yaml
   ▲                                              │ route 字段
   └────── /agv/command 的 args（点名）────────────┘
                                                  ▼
                     控制台任务系统任务缓冲 tasks.yaml（pick / drop / loops / route）
```

### 5.1 点位（`/agv/store` 的 `resource=waypoint`）

点位分两类（`kind`）：`cargo` = **搬货点**（完整搬运的取货点/放货点）；`nav` = **导航点**（只用来组成路线，单独 `goto` 到它也不会触发搬货服务）。`kind` 留空或非法 → **按 `cargo` 处理**。

点位与路线**共用一个服务** `/agv/store`（`AgvStore`），靠请求里的 `resource` + `action` 两个字段分派 —— 旧版那 8 个一一对应的服务（`Waypoint{save,get,list,delete}`、`Route{save,get,list,delete}`）**已全部删除**。完整请求/响应字段见下表：

| 请求字段 | 用途 |
|---|---|
| `resource` | `waypoint` / `route` |
| `action` | `save` / `get` / `list` / `delete` |
| `name` | 资源名（`save`/`get`/`delete` 用；`list` 忽略） |
| `x`、`y`、`yaw` | 仅 `waypoint` + `save`：位置与朝向 |
| `kind` | 仅 `waypoint` + `save`：`cargo` / `nav`（留空按 `cargo`） |
| `point_names[]` | 仅 `route` + `save`：途经点名列表（按行进顺序，至少 1 个） |

| 响应字段 | 用途 |
|---|---|
| `ok` | 是否成功 —— **每个动作都会给，包括 `list`**（旧版 `list` 缺 `ok` 的不一致已消除） |
| `message` | 中文回执，可直接打印 |
| `items[]` | `NamedPose` 列表：`get` = 该资源的位姿列表（点位 1 条 / 路线 N 条）；`waypoint`+`list` = 全部点位；**`route`+`list` 为空**（只回名字与个数）；`save`/`delete` = 空 |
| `names[]` | 仅 `list`：资源名列表（与 `counts` 同序） |
| `counts[]` | 仅 `list`：各资源的位姿个数（与 `names` 同序；点位恒为 1） |
| `count` | `get` = 取到的位姿数；`save(route)` = 实际写入的途经点数（失败为 0）；`list` = 条目总数 |

按 `action` 展开（等价于旧 8 个服务的能力）：

| resource / action | 请求 | 响应要点 |
|---|---|---|
| `waypoint` / `save` | `name`、`x`、`y`、`yaw`、`kind` | `ok`、`message`（重名覆盖） |
| `waypoint` / `get` | `name` | `ok`、`items[0]`（`name/x/y/yaw/kind`）；不存在 `ok=false`，不报错 |
| `waypoint` / `list` | 空 | `ok`、`items[]`、`names[]`、`counts[]`（按点名排序） |
| `waypoint` / `delete` | `name` | `ok`、`message`（不存在 `ok=false`） |
| `route` / `save` | `name`、`point_names[]` | `ok`、`count`（实际写入点数）、`message` |
| `route` / `get` | `name` | `ok`、`items[]` |
| `route` / `list` | 空 | `ok`、`names[]`、`counts[]`（同序） |
| `route` / `delete` | `name` | `ok`、`message` |

`NamedPose` 字段：`name`、`x`、`y`、`yaw`、`kind`（作为路线途经点返回时 `kind` 留空、`name` 是保存当时的点名，仅供人看）。

```yaml
frame_id: map
waypoints:
  起始点:  {x: 0.0,  y: 0.0,  yaw: 0.0,  kind: cargo}
  1号货架: {x: 1.63, y: 2.66, yaw: 0.0,  kind: cargo}
```

> 上面是 `maps/world666map/waypoints.yaml` 的真实形制（`make_world_map.py` 生成的 5 个"X号货架" + 起始点全部是 `kind: cargo`）；`nav` 类型由控制台存点时按需指定，用于只走路线、不触发搬货的中间点。

读取容错：缺 `x`/`y`/`yaw` 或坐标不是数字的点**直接丢弃**；文件读坏/格式错 → 降级为空表，不让节点崩。

### 5.2 路线（`/agv/store` 的 `resource=route`）

路线 = **一串按行进顺序排列的点位**。保存时**按点名从点位表复制位姿**，所以文件里存的是**快照**（之后改点位名/删点位**不影响已存路线**）。请求/响应字段见 §5.1 的统一表，动作语义与点位完全同构（`save`/`get`/`list`/`delete`）。

两防呆规则：**路线至少要有一个途经点**（空路线拒绝）；**任一点名不存在则整条路线拒绝保存**（`message` 列出缺失点名），不会写进半条路线。

```yaml
frame_id: map
routes:
  巡检A:
    points:
      - {name: 走廊1, x: 1.0, y: 2.0, yaw: 0.0}
      - {name: 走廊2, x: 3.0, y: 2.0, yaw: 0.0}
```

**路线语义（最容易搞混，务必记住）**：`goto` + `route` → 路线点在**目标点之前**；`start_task` + `route` → 路线点在**顶升之后、放货点之前**（取货 → 顶升 → 路线 → 放货 → 下降）。选了路线却取不到（服务不可用 / 路线不存在）→ **直接判失败**，不会偷偷退化成单点导航。

### 5.3 任务缓冲（`tasks.yaml`，只有控制台读写，没有对应节点）

```yaml
tasks:
- pick: 1号货架      # 取货点名
  drop: 2号货架      # 放货点名
  loops: 3           # 循环次数（≥1）
  route: 巡检A       # 可选路线名（空串=不用路线）
```

启动任务 = 按缓冲顺序逐条下发 `start_task`（每条按 `loops` 重复），执行中可输入 `s` 查状态、`l` 看缓冲、`q` 退出任务；失败会问是否继续，继续时自动 `resume` 清 `ERROR`。**任务缓冲语义（已确认）**：缓冲**按地图保存**在 `maps/<地图名>/tasks.yaml`；在任务系统里按 `q` 是**返回导航主界面**（不是退出终端），会先确认"返回会清空全部任务并停车"，确认后清空缓冲、存盘、发停。**页面层级**：功能子界面（`6` 手动 / `7` 路线管理 / `9` 任务系统 / `t` 搬运 / `c` 自检）里按 `q` 只回到导航主界面，返回时自动重印主菜单；**只有导航主界面按 `q` 才退出终端**，退出前若有任务在跑或缓冲非空会先确认。

### 5.4 文件绑定（一份地图一份数据）

```
<工作区>/maps/<地图名>/          ← 一份地图 = 一个同名目录
├── <地图名>.pgm                 栅格图（Nav2 用）      ├── waypoints.yaml   ★ 专属点位
├── <地图名>.yaml                地图元数据(分辨率/原点) ├── routes.yaml      ★ 专属路线
└── tasks.yaml                   ★ 专属任务缓冲（按需生成）
```

地图库根默认 `<工作区>/maps`，可用环境变量 **`AGV_MAPS_DIR`** 覆盖；**删地图 = 删整个目录**（点位/路线/任务缓冲一起走，不留孤儿数据）。没有控制台加载流程时，`waypoint_node` 兜底用配置里的 `/tmp/agv_waypoints.yaml` 与 `/tmp/agv_routes.yaml`。

## 6. 地图与建图流程

### 6.1 两条命令线

| 要做的事 | 命令 |
|---|---|
| 建图（仿真 + SLAM + RViz） | `ros2 launch agv_navigation mapping.launch.py` |
| 在已建好的地图上导航 | `ros2 launch agv_bringup agv_stack.launch.py with_sim:=true` |

`mapping.launch.py` 起：`agv_description` 仿真（Gazebo + spawn 车，发 `/scan`、`/odom`、TF）+ `slam_toolbox` 异步在线建图（`/scan` + TF → `/map` + `map→odom`）+ RViz；参数 `world`（默认 `agv_description/worlds/world666.world`）、`gui`、`rviz`、`use_sim_time`（默认 true）、`slam_params_file`（雷达视场角固定 360°，写在 URDF 里，无 launch 参数）。`agv_stack.launch.py` 起业务节点 + `velocity_arbiter`，`with_sim:=true` 时再 include `navigation.launch.py`（仿真 + `map_server` + AMCL + 全套 Nav2 + RViz，`slam:=False`、`autostart:=true`）。

### 6.2 建图 → 存图 → 用图（推荐全走控制台）

1. 启动控制台选 **1 建图模式** → 自动起「仿真 + SLAM + RViz」。
2. 按 **`k`** 另开终端键盘遥控（remap 到 `/cmd_vel_raw`，**直驱底盘**）走一圈；建图时雷达是全向 360°（车没顶升货架，后方无遮挡）。
3. 按 **`m`** 保存地图并**起名**：底层调 `map_saver_cli` 落到临时目录，再拷进 `maps/<地图名>/`，同时补一份专属 `waypoints.yaml`（预置"起始点"）与空 `routes.yaml`。
4. 以后要用：控制台选 **2 导航模式** → 选地图加载，点位/路线/任务缓冲自动跟着这张地图走。

### 6.3 场景真源：world666 + world666map

**世界只有一份真源**：`src/agv_description/worlds/world666.world`，由 `tools/make_world_map.py` 按脚本顶部的参数化布局生成（墙 + 5 个凵型槽位 + 2 个坐在槽顶的货物方块，含 `<gravity>`、`<physics type='ode'>`、`<scene>`）。改场景就改脚本再重跑，**不要手改 world**。

`make_world_map.py` 落包时写出这些产物：

| 产物 | 路径 | 说明 |
|---|---|---|
| Gazebo 世界 | `src/agv_description/worlds/world666.world` | 唯一世界真源 |
| **参考**栅格地图 | `.../maps/world666map/world666map_ref.pgm + .yaml` | 脚本按布局算出的理论地图（870×813 @0.025，origin -11.304/-10.423） |
| 点位 | `.../world666map/waypoints.yaml` | 5 个"X号货架" + 起始点，全部 `kind: cargo` |
| 空路线 | `.../world666map/routes.yaml` | 路线由控制台按点名组织 |
| 运行时地图库同步 | `maps/world666map/waypoints.yaml`、`routes.yaml` | 点位/路线同步；**栅格图不覆盖** |

**以哪份地图为准**：`maps/world666map/world666map.pgm + .yaml` 是控制台建图模式**实测**出来的那份（594×762 @0.025，origin `-10.7/-9.83`），**日常导航以它为准**；同一份也装进包内 `src/agv_navigation/maps/world666map/`。脚本生成的 `*_ref` 只是同布局的**参考版**，用于对照与重建场景，**不会覆盖实测那份**（脚本里也明确打印了这一点）。四处默认已统一指向 world666map：`sim.launch.py`、`mapping.launch.py`、`navigation.launch.py`（`default_world` + `default_map`），以及 `agv_stack.launch.py` 的 `map_yaml` 兜底（= `agv_navigation/maps/world666map/world666map.yaml`）。

脚本还会用 `agv_common.map_check.MapChecker` 自检每个停留点净空（≥0.30 m）与"起点→各点"的最宽通道瓶颈，并打印**场地范围**（供 `task.yaml` 的 `field_min/max` 用）。用法：`python3 tools/make_world_map.py`（`--out-dir` 换输出目录，`--dry-run` 只生成不落包）。

> **实测地图范围与 `field_*` 已对齐（原不一致问题已解决）**：按 PGM 头部实测，**实测地图** 594×762 @0.025、origin(-10.7, -9.83) → 覆盖 **x[-10.70, 4.15]、y[-9.83, 9.22]**；`task.yaml` 的 `field_min/max` 现已改成同一组数（`-10.70 / 4.15 / -9.83 / 9.22`），原先那组跟**参考地图**（870×813）一致、比实测地图大一圈的值（`-11.30 / 10.45 / -10.42 / 9.91`）已废弃。
>
> 更根本的是判据本身收敛了：目标点能不能去，全项目只有 **`agv_common/goal_check.py` 的 `GoalValidator`** 一个判据 —— 有地图（`map_yaml`）时**以实测地图为准**（图外 / 压障碍 / 净空不足都拒），没地图时才退化为 `field_*` 矩形。因此**不可能再出现"门卫放行、预检拒绝"的同坐标两种结论**；`field_*` 从"第二个判据"降级为"没有地图时的兜底"，并与实测地图保持一致。
>
> 仍要注意的是**场景覆盖**：实测地图 x 只到 4.15，而 world666 墙几何跨到 x≈9.95，**东侧那片不在实测地图内**（建图范围与场景范围本身没铺满）。换场景或重新建图后**必须复核这张图的覆盖范围**，别让车去图上没有的地方。

### 6.4 数据卫生说明

- 点位文件现在只带 `x`/`y`/`yaw`/`kind` 四个字段（历史遗留的 `dock_*`/`slot_*`/`cargo_*` 等**已废弃字段**及其旧点位文件均已删除）；若从别处拷入这类文件，`WaypointStore` 只取上述四字段、**静默忽略其余**，并会在下次经服务保存该点位时把它们丢弃（属正常收敛）。
- `tools/make_world_map.py` 的注释与打印文案**已同步为当前方案**（`cargo_follow`、精定位、`dock`、`warehouse`、`new_world` 等旧提法均已清零）；脚本输出为 `kind: cargo`、不带旧字段。

## 7. 控制台使用（`tools/agv_console.py` v3.0）

```bash
cd <工作区> && source install/setup.bash
python3 tools/agv_console.py                      # 交互式：先选模式
python3 tools/agv_console.py --mode nav           # 直接进导航模式
python3 tools/agv_console.py --no-gui --no-rviz   # 不起 Gazebo 窗口 / 不起 RViz
python3 tools/agv_console.py check                # 一键自检（自己起 3 个业务节点）
python3 tools/agv_console.py maps                 # 只列地图库（不需要 ROS）
```

启动菜单：**`1` 建图模式 / `2` 导航模式 / `3` 只做自检 / `q` 退出**。每次启动会先 `pkill` 清理上次残留的仿真/导航/业务/遥控进程 —— 多套栈的同名节点会互抢话题/服务/action，表现就是"节点没起 / 无响应 / 查询超时 / 原地转圈"。

### 7.1 建图模式：想干 X 就按 Y

`k` 遥控车走一圈（另开终端跑 `teleop_twist_keyboard`，remap 到 `/cmd_vel_raw` 直驱底盘）｜`m` 保存当前地图（输入地图名 → `map_saver_cli` → 落进 `maps/<名>/`，点位/路线文件一起建）｜`l` / `d` 列出地图（地图名 + 点位/路线条数 + 目录）/ 删整个目录（需确认）｜`n` / `q` 切到导航模式 / 退出（切模式会先关掉建图栈）。

### 7.2 导航模式：两段菜单

**第一段 · 选地图**：`1` 加载地图（输入地图名）/ `2` 列出 / `3` 删除 / `m` 回建图模式 / `q` 退出。输入地图名后：校验 pgm+yaml 是否齐全 → 缺 `waypoints.yaml`/`routes.yaml` 就补一份 → **拉起整套栈**（`agv_stack.launch.py with_sim:=true`，注入该地图的 `map_yaml`、`waypoint_file`、`routes_file`）→ 等 Nav2 激活约 15~40 秒（按 `n` 查，全绿再导航）。

**第二段 · 已加载地图后的业务菜单**

| 想干的事 | 按键 | 想干的事 | 按键 |
|---|---|---|---|
| 存点位（选搬货点/导航点） | `1` | 查询状态（状态/位姿走 `/agv/query`，点位/路线走 `/agv/store`） | `8` |
| 列出点位 / 删除点位 | `2` / `3` | 任务系统（任务缓冲） | `9` |
| 点名导航（可选路线） | `4` | 完整搬运（取货→顶升→运输→下降） | `t` |
| 坐标导航（x / y / yaw） | `5` | 手动顶升 / 下降 | `u` / `d` |
| 手动开小车（自动进/出手动模式） | `6` | 急停 / 恢复 | `x` / `r` |
| 路线管理 | `7` | 状态快照（含 `/agv/cargo/busy`） | `s` |
| 看日志尾（任务栈最后 15 行） | `l` | 导航节点激活状态（5 个 lifecycle 节点） | `n` |
| 换地图（回地图选择） | `0` | 一键自检 / 退出控制台 | `c` / `q` |

**手动开小车（`6`）的互锁**：若检测到正在跑任务，先问"是否停止当前任务并清除"；确认后发 `/agv/manual_mode=true` → `cancel_task` → `resume`，再让你填线速度/角速度/持续秒数，速度发到 `/cmd_vel`（经速度仲裁到底盘），结束后发零速并 `manual_mode=false` 退出手动模式。

### 7.3 路线管理子菜单（按 `7`）

`1` 设置/修改路线（先打印可用点名；**重名即修改**；途经点名用 **`,`** 或中文 `，` 隔开）｜`2` / `3` 列出全部路线 / 查看详情（途经点数 / 序号+点名+坐标）｜`4` / `q` 删除路线 / 返回。

### 7.4 任务系统子菜单（按 `9`，任务缓冲按地图存 `tasks.yaml`）

`1` 添加任务（取货点 / 放货点 / 循环次数，默认 1 / 是否用路线）｜`2` / `3` / `4` 删除任务（按序号）/ 列出（显示"总循环次数"）/ 清空（需确认）｜`5` 启动任务（按缓冲顺序逐条 `start_task`，每条按 `loops` 重复）｜`q` **返回导航主界面**（= 清空全部任务并把车停下，需确认；不会关掉终端）。

启动任务后可输入 `s` 状态快照 / `l` 看任务缓冲 / `q` 退出任务（会取消当前搬运）。失败（`ERROR`）会问是否继续，继续时自动发 `resume` 清零 `ERROR`。

> **判"完成"认任务号，不认状态字**：`/agv/task_state` 上报的是"最新一条"，上一单的 `DONE` 会留在缓存里；只看 `state` 会把上一单的完成当成这一单的完成。
> **"迟迟没开始"（15 秒）先重发再判超时**：任务层报完 `DONE` 还有约 0.5 s 收尾（等车停稳才 `DONE → IDLE`），这段时间 `_can_start()` 的串行闸门仍不放行，紧跟下发的 `start_task` 会被拒（"已有任务在执行"）。控制台只看得见 `/agv/task_state`、看不见拒绝原因，所以每 1 s **换新请求号重发一次**（绕过去重），最多 8 次，等它收尾结束。真没起来才判 `timeout` 并 `cancel_task`。

### 7.5 一键自检（`c` 或 `check` 子命令）

原则：**只用自己的临时名字、跑完清理、不做任何让车动起来的事**（越界坐标/重复请求天然被门卫挡下，急停后**一定补 `resume` 复位**）。检查内容：**点位/路线**（统一走 `/agv/store`）—— 存点（带 `kind`）→ 查点（坐标与类型正确）→ 查不存在的点返回失败而不崩 → 列点包含 → 存路线（`count` 正确）→ 查路线 → 列路线 → 删路线 → 删不存在的路线返回失败 → 删点清理痕迹；**搬货服务** —— `lift` 受理且 `busy=true`、`lower` 受理且 `busy=false`、未知命令被拒；**门卫** —— 未知指令被拒、越界坐标被拒、重复 `request_id` 被拒（读任务栈日志里的拒绝文案）；**安全** —— 急停生效（日志出现"已急停"）后补 `resume`；**查询服务** —— `state` 与 `pose` 有应答。自检需要业务节点在跑：`check` 子命令会自己起 3 个业务节点（用配置默认点位文件），在导航模式里按 `c` 则复用当前整套栈。

## 8. 配置中心与参数机制

```
src/agv_bringup/config/
├── default/           打底（与硬件无关的公共默认值，唯一有实际值的层）
│   ├── task.yaml      任务层：场地范围、超时、目标点净空  ├── cargo.yaml     搬货服务：服务名/busy 话题/补发周期
│   ├── safety.yaml    安全层：cmd_timeout_ms、estop_budget_ms
│   ├── hardware.yaml  硬件边界：velocity_arbiter 全部参数  └── waypoint.yaml 点位路线：文件路径（兜底）、坐标系
└── real/              实车覆盖（当前只有一份占位 README.md，还没有实际覆盖文件）
```

> `config/sim/` **已删除**：它原本是个空目录，选与不选都没有任何文件参与合并，属于"看着像有档位、实际是空转"的假开关。

合并逻辑（`agv_common.config_loader.load_config`）：**同一文件名逐层叠加，`default` 在前、覆盖层在后；字典递归合并，标量/list 直接覆盖**；某层文件不存在就跳过。

**怎么选档**：`agv_stack.launch.py` 的对外参数 **`param_profile`**（默认**空串**）就是档位，启动期用 `OpaqueFunction` 读参数中心合并成节点参数再注入。**空串 = 只用 `config/default`**。

```bash
ros2 launch agv_bringup agv_stack.launch.py with_sim:=true                        # 默认：只用 config/default
ros2 launch agv_bringup agv_stack.launch.py with_sim:=true param_profile:=real   # 叠一层实车覆盖
```

注意：`real/` 目前只有占位 README、**还没有实际覆盖文件**，所以现在选 `real` 与不选覆盖层**效果相同**（都用 `default/`）；控制台起栈时**不传** `param_profile`，走的就是默认空串。

### 8.1 `agv_stack.launch.py` 全部对外参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `param_profile` | 空串 | 参数档位，**空=只用 `config/default`**（**勿改名 `profile`**：会被 gzserver 当成 `--profile` 吃走） |
| `with_sim` | `false` | 是否再拉起导航栈（仿真 + AMCL + Nav2 + RViz） |
| `gui` / `rviz` | `true` / `true` | Gazebo 窗口 / RViz 窗口 |
| `world_file` | 空 | 可选 Gazebo world 绝对路径（**勿改名 `world`**：空默认值会顶掉导航栈默认世界）；留空则用 `agv_description/worlds/world666.world` |
| `map_yaml` | 空 | 所选地图的 yaml（控制台按地图库注入；**也是目标点判据的依据**）；留空则兜底 `agv_navigation/maps/world666map/world666map.yaml` |
| `waypoint_file` / `routes_file` | 空 | 与地图绑定的点位 / 路线文件；**`routes_file` 留空则取点位文件同目录的 `routes.yaml`** |

三个"为什么"值得记住：① `map_yaml` 留空时 launch 兜底用 `agv_navigation/maps/world666map/world666map.yaml` 当**目标点判据**的参照，并一起传给 Nav2 的 `map`；② 业务节点 `use_sim_time = with_sim`，**只在带仿真时为 true**（不带仿真还开仿真时钟，没有 `/clock` 会让"安全推进/1Hz 上报"定时器根本不触发）；③ 参数名带后缀（`param_profile`/`world_file`/`nav2_params_file`）是为了躲开 ROS 2 launch **全局 context 无作用域隔离**的问题 —— 名字撞车就会互相污染。

## 9. 安全设计与已知脆弱点

### 9.1 安全设计（四道）

| 机制 | 实现位置 | 行为 |
|---|---|---|
| **急停** | `task_manager_node`（`handle_stop` + `SafetyMonitor`） | `stop` 绕过白名单与去重；立即发零速到 `/cmd_vel`（经仲裁到底盘）；取消 Nav2 目标；中止后台线程；任务转 `PAUSED`；记录"多久发出停车"并与 `estop_budget_ms`(200ms) 比较，日志/回执给出"达标/超标" |
| **指令超时** | `SafetyMonitor.tick` + `on_cmd_vel` | 超过 `cmd_timeout_ms`(500ms) 没收到新 `/cmd_vel` 就判"上层已不再发速度"，补发一帧零速（每 2 秒最多告警一次） |
| **手动/自动互锁** | `task_manager_node` + 控制台 | 订阅 `/agv/manual_mode`；手动模式或急停中，`goto`/`start_task` 一律拒绝；控制台进手动前先问是否停掉当前任务 |
| **速度限幅收口** | `VelocityLimits` + `velocity_arbiter` | 发给底盘的速度最后都过 `clamp_twist`（线 ≤ `max_linear`、角 ≤ `max_angular`）；仲裁节点还做近障减速、左右择空、缓回导航；激光超时（`scan_timeout`）时不做避障但**仍然限幅** |

**目标点合法性（`GoalValidator`，唯一判据）**：导航/搬运前对目标点做判断 —— 有地图时**以实测地图为准**：是否在地图内、是否落在障碍上、到最近障碍的净空是否 ≥ `robot_clearance`（默认 0.30 m）；没给地图（如纯逻辑测试）时**退化为 `field_*` 矩形兜底**。不合格直接拒绝并报中文原因。它是**必做**的一环（`precheck_enabled` 开关已删除）；地图加载失败时降级为矩形判据并在日志里说明，不让判据本身成为新的故障点。

> 这条设计的由来：原先"门卫按 `field_*` 判、预检按地图判"是**两套判据**，范围不一致（`field_*` 比实测图大一圈），同一个坐标会出现「门卫放行、预检拒绝」——同一件事两种结论。现已收敛成一个对象，`field_*` 不再能与地图冲突。

### 9.2 已知脆弱点 / 风险与待办

| # | 风险 / 待办 | 说明与建议 |
|---|---|---|
| 1 | **实测地图 `origin` 必须与 world666 世界坐标系对齐** | 栅格图与 world 是两份产物：world 由 `make_world_map.py` 生成，而**日常用的是实测地图** `maps/world666map/world666map.yaml`（origin `-10.7/-9.83`，594×762 @0.025）。手改这份 yaml 的 `origin`/`resolution`、或换 world 不重建地图 → AMCL 整体错位、车走不动。**别手改地图 yaml 的几何字段。** |
| 2 | **点位坐标是"生成口径"，与实测地图未逐点复核** | `waypoints.yaml` 里 5 个货架点来自 `make_world_map.py` 的 `STOP_POINTS`（如 1 号 `1.63, 2.66`），而槽位实测落点是 `2.53, 2.69`——**两者不是同一套坐标**（前者是槽口外约 0.9 m 的停车点，后者是槽中心）。请按"停车点"理解并现场核对；换场景重跑脚本后**务必用 `GoalValidator` 判据 + 实跑复核**。 |
| 3 | ~~实测地图范围与 `field_*` 配置范围不一致~~（**已修**） | 原先是两套判据、两个范围：`field_*` 是 `-11.30 / 10.45 / -10.42 / 9.91`（跟生成脚本的**参考地图**一致，比实测地图大一圈），于是"配置内有、实测图外"的坐标会被门卫放行。现已 ① 把 `field_*` 对齐实测地图范围 `-10.70 / 4.15 / -9.83 / 9.22`，② 把判据收敛为 `agv_common/goal_check.py` 的 `GoalValidator` 一个对象 —— **有地图时以地图为准，`field_*` 只在没有地图时兜底**，从根上消除"同坐标两种结论"。仍保留的注意点：实测地图 x 只到 4.15、而 world666 墙体跨到 x≈9.95，**东侧不在实测图内**，重新建图后务必复核覆盖范围。见 §6.3。 |
| 4 | 定位初始位姿是"标称"值 | `nav2_params.yaml` 里 AMCL `set_initial_pose: true`、`initial_pose=(0,0,0)`，与仿真出生点一致；改出生点必须**两处同步**，否则导航不动。 |
| 5 | 急停响应时间只是"本进程计时" | `estop_budget_ms` 量的是"从触发到本节点发出零速"，不含仲裁转发、Gazebo 执行、底盘机械延迟。**不当端到端指标**；实车阶段需重新定义测点。 |
| 6 | ~~`resume` 语义有两套~~（**已修**） | 原先 `TaskManager.resume()` 声称能回"暂停前那一格"，而节点 `resume` 处理器调的是 `reset()`。现已删掉 `TaskManager.resume()` 与 `_paused_from`，只保留"恢复 = 清零回 `IDLE`"一套语义（急停已取消导航、线程已中止，续跑本就无法兑现）。 |
| 7 | ~~`task_manager_node` 同时发布和订阅 `/cmd_vel`~~（**已修**） | 它发零速时**自己也会收到**，把"上层还在发速度"的计时清零 —— 等于自己解除自己的超时。现已修：发之前调 `SafetyMonitor.note_self_publish()` 记账，收到自己的回声就冲销、不计心跳；并且加了 `timeout_edge()`，超时停车帧只在**刚进入**超时态时发一帧，不再每 100 ms 重复发。 |
| 8 | 无真实硬件动作 | `/agv/cargo` 现阶段只改标志、立即返回成功，**"顶升"没有物理含义**，`start_task` 会"秒完成"顶升/下降步；现在跑通的是**逻辑链路**，不是搬运物理。 |
| 9 | 点位/路线无版本号、无并发保护 | 存储是"内存 + 每次写盘"，多进程同时写同一个 yaml 会互相覆盖。**同一份地图不要同时开两个控制台/两个 waypoint_node。** |
| 10 | 未做量化验收 | 本阶段**不追求**成功率/对接偏差之类的数字指标，因此也没有对应统计脚本与门禁；后续按需补（届时再定指标与测法）。 |

## 10. 硬件接入指引（**只改 `cargo_node` 一个文件**）

设计上全项目**唯一**与搬货机器打交道的地方是 `src/agv_lift/agv_lift/cargo_node.py` —— 接真机只改这一个文件。外部接口**保持不变**，所以任务层、控制台、状态机都不用动：

| 接口 | 语义 |
|---|---|
| 服务 `/agv/cargo`（`CargoCmd`） | 请求 `command="lift"` 顶升开始搬货 / `command="lower"` 下降放货完成；`request_id` 追踪用；响应 `ok`（是否受理成功）、`busy`（执行后的搬货中状态）、`message`（中文回执） |
| 话题 `/agv/cargo/busy`（`std_msgs/Bool`） | `true`=正在搬货，`false`=空闲/放货完成。**变化时立即发，另外每秒补发一次**（晚加入的订阅者也能看到当前状态） |
| 参数 | `service_name`、`busy_topic`、`publish_period`（可在 `config/default/cargo.yaml` 覆盖） |

**改法**：只改 `CargoNode.on_cargo()`，把"改标志"换成"驱动机器 + 等到位判断"。

```python
def on_cargo(self, request, response):
    cmd = str(request.command or '').strip().lower()
    if cmd in LIFT_WORDS:                      # 顶升
        ok, msg = self._do_lift()              # ← 新增：驱动顶升机构，等到上限位 / 超时
        self.busy = bool(ok)
    elif cmd in LOWER_WORDS:                   # 下降
        ok, msg = self._do_lower()             # ← 新增：驱动下降到下限位 / 超时
        self.busy = False if ok else self.busy
    else:
        ok, msg = False, f'未知命令「{request.command}」：只支持 lift / lower'
    response.ok, response.message, response.busy = ok, msg, self.busy
    self.pub_busy.publish(Bool(data=self.busy))
    return response
```
要点：① `LIFT_WORDS`/`LOWER_WORDS` 已兼容中英文写法（`lift`/`up`/`顶升`、`lower`/`down`/`放货`/`下降`），新硬件协议在这里映射；② **到位判断要给出确定的 `ok`**，失败/超时必须 `ok=false`，任务层才会判 `ERROR` 并停车；③ 硬件串口/超时等参数**不要写死**，加到 `config/default/cargo.yaml` 并在 `__init__` 里 `declare_parameter` 读入。

**与任务层的时序约定（关键）**：任务层等待上限是 `cargo_timeout_s`（默认 **5.0 s**）。动作 ≤ 5 s 就直接阻塞到到位再返回 `ok`（最简单）；> 5 s 则二选一：① 把 `cargo_timeout_s` 调大到动作时长 + 余量；② 服务内立即返回受理成功、用 `/agv/cargo/busy` 表示"进行中"，等真到位再置位（此时任务层会先往下走，需自行确认语义可接受）。推荐先用 ①，保持"服务返回 = 动作完成"的简单语义。

**改完自己先跑这三步验收**：① 服务级 —— 控制台按 `u`/`d`，看回执 `ok`、`message` 与 `/agv/cargo/busy` 是否与实际机构状态一致，发乱写命令确认被拒；② 自检 —— 按 `c`（或 `agv_console.py check`），搬货三项应全过；③ 链路级 —— `t` 完整搬运或任务系统 `9`，盯 `/agv/task_state` 的 `GO_LOAD → LIFTING → TRANSPORT → LOWERING → DONE`，故意让硬件超时一次，确认任务进 `ERROR` 且车停下。另注：实车下位机应听 **`/cmd_vel_raw`**（而不是 `/cmd_vel`），这样急停、避障、限幅才都还在链路上。

## 11. 目录结构说明与常用命令速查

```
<工作区>/
├── AGV_PLAN.md              本文档（方案总纲，当前实现版）
├── README.md / AGV_INTERFACES.md / TESTING.md / 搬运场景几何契约.md / GIT使用.md
├── maps/                    ★ 运行时地图库：<地图名>/{<地图名>.pgm,yaml, waypoints.yaml, routes.yaml, tasks.yaml}
│                            （现有一份实测地图 world666map/：594×762 @0.025，origin -10.7/-9.83）
├── src/
│   ├── agv_description/     机器人 URDF/Xacro + Gazebo 世界（worlds/world666.world，唯一世界真源）+ 仿真 launch
│   ├── agv_interfaces/      接口契约：3 msg + 3 srv（只放定义，不放逻辑）
│   ├── agv_common/          纯 Python 公共库：geometry/limits/reactive_avoid/map_check/map_manager/config_loader/goal_check
│   ├── agv_bringup/         参数中心（config/default|real）+ 总启动 agv_stack.launch.py
│   ├── agv_waypoint/        waypoint_node（统一存储服务 /agv/store）+ WaypointStore / RouteStore
│   ├── agv_lift/            cargo_node（★ 唯一硬件对接点：/agv/cargo + /agv/cargo/busy）
│   ├── agv_hardware/        velocity_arbiter（速度仲裁：/cmd_vel → /cmd_vel_raw）
│   ├── agv_task/            task_manager_node（薄接线）+ 分模块：command_router/task_flow/nav_runner/cargo_client/store_client/state_reporter + 纯逻辑 task_manager/goto_controller/safety_monitor
│   └── agv_navigation/      建图与导航 launch + Nav2/SLAM 参数 + 包内地图（maps/world666map/）
└── tools/                   agv_console.py（控制台 v3.0）、make_world_map.py（场景生成器：world + 参考地图 + 点位）
```

纯逻辑单测（无需仿真）：`python3 -m pytest src/agv_common/test src/agv_waypoint/test src/agv_task/test -q`

```bash
# 构建
colcon build && source install/setup.bash

# 建图（+ gui:=false rviz:=false 可无头）
ros2 launch agv_navigation mapping.launch.py
# 业务栈：不带仿真 / 带仿真（map_yaml、waypoint_file 留空即用默认 world666map）
ros2 launch agv_bringup agv_stack.launch.py
ros2 launch agv_bringup agv_stack.launch.py with_sim:=true \
    map_yaml:=maps/world666map/world666map.yaml waypoint_file:=maps/world666map/waypoints.yaml

# 控制台 / 场景重新生成
python3 tools/agv_console.py            # 交互（选模式）；maps 列地图库；check 一键自检
python3 tools/make_world_map.py         # 改脚本顶部参数区后重跑（重出 world666.world + 参考地图 + 点位）

# 手写接口（不开控制台时的最小用法）
# 存点位 / 存路线：统一走 /agv/store，用 resource + action 区分
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
    "{resource: 'waypoint', action: 'save', name: '取货点', x: 1.0, y: 0.0, yaw: 0.0, kind: 'cargo'}"
ros2 service call /agv/store agv_interfaces/srv/AgvStore \
    "{resource: 'route', action: 'save', name: '巡检A', point_names: ['走廊1', '取货点']}"
ros2 service call /agv/store agv_interfaces/srv/AgvStore "{resource: 'waypoint', action: 'list'}"
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand "{source: 'web', command: 'goto', args: ['取货点'], request_id: 'r1'}"
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand "{source: 'web', command: 'start_task', args: ['取货点','放货点'], route: '巡检A', request_id: 'r2'}"
ros2 topic echo /agv/task_state; ros2 service call /agv/query agv_interfaces/srv/AgvQuery "{what: 'state'}"
ros2 service call /agv/cargo agv_interfaces/srv/CargoCmd "{command: 'lift', request_id: 't1'}"
ros2 topic echo /cmd_vel_raw            # 底盘真正听的话题，用来核对速度链路
```

### 11.1 配套文档索引

各文档均已按新方案重写、口径与本文档一致：`AGV_INTERFACES.md`（接口总表：3 msg + 3 srv 逐字段说明 + **旧接口删除对照表**）、`README.md`（上手流程、地图库约定、常见问题排查）、`TESTING.md`（测试组织与跑法）、`搬运场景几何契约.md`（墙/槽位/车/雷达几何与地图分辨率等硬约束）、`GIT使用.md`（协作/提交约定），以及 `src/agv_interfaces/README.md`、`src/agv_task/README.md`、`src/agv_navigation/README.md`（各包内部说明）。

> **唯一需要留意的口径**：以上文档里出现的 `docking`、`DockConfig`、`ALIGN_LOAD`、`warehouse`、`new_world` 等字样，**都是"旧方案已删除"对照表里的记录**，不是现存功能 —— 看到它们时对照同表格的"现在怎么办"一列即可。现存接口以 §3、§4.3、§5.1 为准：`agv_interfaces` 共 **3 个 msg**（`AgvCommand`、`TaskState`、`NamedPose`）+ **3 个 srv**（`AgvStore`、`AgvQuery`、`CargoCmd`）。旧版那 8 个点位/路线 CRUD 服务（`Waypoint{save,get,list,delete}`、`Route{save,get,list,delete}`）已合并为**一个** `/agv/store` —— 见 §5.1。
