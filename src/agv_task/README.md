# agv_task —— 任务层（唯一执行者）

本包是 AGV 的**任务层**：所有业务指令的唯一入口与唯一执行者。
它把「收到指令 → 门卫校验 → 导航 / 调搬货服务 → 状态上报」串成一条链。

- **可执行节点**：`task_manager_node`（本包唯一的 `console_scripts` 入口，注册在 `setup.py` 里）。
  它现在只是一个**薄接线层**：读参数、建收发、起线程；业务逻辑全在下述模块里。
- **纯逻辑模块**（不 import rclpy，可脱离 ROS 单测）：`command_router` / `safety_monitor` /
  `task_manager` / `goto_controller` / `state_reporter`。
- **流程与客户端模块**（import ROS 消息，但逻辑自成一体的可读单元）：
  `task_flow`（两条流程）/ `nav_runner`（导航执行）/ `cargo_client`（搬货调用）/
  `store_client`（点位路线查询）。
- **函数库**：`goto_pose` / `robot_pose` / `init_robot_pose`。

> 任务层自己不碰搬货硬件：顶升 / 下降一律通过 `/agv/cargo` 服务完成，本包只管"什么时候调"。

---

## 一、模块职责

| 模块 | 一句话职责 |
|---|---|
| `task_manager_node.py` | 节点接线层：读参数、建收发与客户端、注册指令处理器；**回调只受理，导航/搬运跑在后台线程**。 |
| `task_flow.py` | 两条业务流程：单点导航（目标 →（路线各点）→ 目标点）、完整搬运（取货 → 顶升 → 运输 → 下降）。 |
| `nav_runner.py` | 导航执行器：单段 / 多点串联、可中止等待、超时收敛；持 `BasicNavigator`。 |
| `cargo_client.py` | 搬货客户端：调 `/agv/cargo`，处理"服务不在 / 超时 / ok=false"三种失败。 |
| `store_client.py` | 点位 / 路线查询：把 `/agv/store` 的调用与等待收拢到一处。 |
| `state_reporter.py` | 状态组装（纯逻辑）：按 **急停 > 搬运任务 > 单点导航 > 空闲** 的优先级产出上报内容。 |
| `task_manager.py` | 完整搬运状态机（纯逻辑）：`IDLE → GO_LOAD → LIFTING → TRANSPORT → LOWERING → DONE → IDLE`，含 `ERROR` / `PAUSED`。 |
| `command_router.py` | 指令门卫（纯逻辑）：白名单、参数、`request_id` 去重、分派；坐标交给唯一判据 `GoalValidator`。 |
| `goto_controller.py` | 单点导航状态机（纯逻辑）：`IDLE → GO_LOAD → DONE / ERROR`，只维护状态，不亲自导航。 |
| `goto_pose.py` | 单点导航执行：`goto_pose(navigator, x, y, yaw, ...)` 走 Nav2 `/navigate_to_pose` 动作，节流打印反馈并返回 `TaskResult`。 |
| `robot_pose.py` | 实时位姿：从 TF 查 `map → base_footprint`，`get_current_pose(node)` 返回 dict 或 None。 |
| `safety_monitor.py` | 安全层（纯逻辑）：急停（记录发出停车耗时，预算 200 ms）、`/cmd_vel` 指令超时、手动/自动互锁、**自发帧回声抑制**。 |

> `init_robot_pose.py` 也在包里（等价 RViz 的 2D Pose Estimate），但 **`task_manager_node` 没有用到它**：
> 仿真里 AMCL 由 `nav2_params.yaml` 的 `set_initial_pose: true` 自带初始位姿 (0,0,0)。

## 二、指令总线

指令从 `/agv/command`（`AgvCommand` 话题）进来，全部经 `CommandRouter.route()` 处理：

| 指令 | 参数 | 语义 |
|---|---|---|
| `goto` | `[点名]` 或 `[x, y]` 或 `[x, y, yaw]` | 前往点名 / 坐标；`route` 非空时先走路线各点。 |
| `start_task` | `[取货点名, 放货点名]` | 完整搬运；`route` 非空时在顶升后、放货点前走路线各点。 |
| `cancel_task` | 空 | 取消当前任务、停 Nav2 目标、发零速、状态机回空闲。 |
| `stop` | 空 | 急停。**永远受理，绕过去重**。 |
| `resume` | 空 | 解除急停并清空任务，回到空闲待命。 |

门卫规则（`command_router.py`）：

- 白名单之外一律拒绝；白名单内但无人注册也拒绝。**`lift` / `lower` / `dock` 不在白名单**（搬货走服务）。
- `goto` 单参数一律当点名（点名可以是纯数字，如 `666`）；2 个参数按 `[x, y]`（yaw 缺省 0）；3 个当 `[x, y, yaw]`。
- 坐标合法性只有**一个判据** `GoalValidator`（`agv_common/goal_check.py`）：
  有 `map_yaml` 时以地图为准（图外 / 压障碍 / 净空不足直接拒），没地图才退回 `field_*` 矩形兜底。
  这样不会出现"门卫放行、别处又拒绝"的两种结论。
- `request_id` 非空才去重，同一号只受理一次（受理与否都记账，防重试风暴）；去重表上限 `max_seen=1000`，超限淘汰最旧。
- 校验顺序：`stop 快通道 → 白名单 → 参数 → goto 坐标判据 → 去重`。**测去重要用能过前置校验的指令**
  （自检用 `cancel_task`）；用越界坐标当探针会先在"坐标"被拦下，永远测不到去重。

**处理器约定**：`handler(args, msg) -> (bool, str)`，即 `(accepted, reason)`。
`args` 是参数列表，`msg` 是原始指令（处理器可读 `route`）。只返回 bool 也兼容；
处理器抛异常由路由器兜住，不拖垮路由。

## 三、状态机与进度

`task_manager.py` 的 `SEQUENCE`，每调一次 `step_success()` 走一格：

| 状态 | 进度 | 中文提示 | 说明 |
|---|---|---|---|
| `IDLE` | 0.00 | 空闲，等待任务 | 可接新任务 |
| `GO_LOAD` | 0.15 | 前往取货点 | 导航到取货点 |
| `LIFTING` | 0.40 | 顶升中 | 调 `/agv/cargo` 的 `lift` |
| `TRANSPORT` | 0.60 | 运输中 | 已顶升，前往放货点 |
| `LOWERING` | 0.85 | 下降放货中 | 调 `/agv/cargo` 的 `lower` |
| `DONE` | 1.00 | 搬运完成 | 随后自动回 `IDLE` |
| `ERROR` | 0.00 | 任务异常，需人工复位 | 导航失败 / 搬货服务失败 |
| `PAUSED` | 0.00 | 已暂停（急停/停止） | 急停后冻结 |

- 单点导航（`goto`）只走 `IDLE / GO_LOAD / DONE / ERROR`，进度分别是 0 / 0.5 / 1.0 / 0。
- `idle` 时若急停，状态**不进 PAUSED**（没有活动任务可暂停）；有活动任务时才记住来处。
- 任务态非 `IDLE` 时 `/agv/task_state` 报搬运态，否则报单点导航态；急停优先报 `PAUSED`。

## 四、路线语义

路线是**点位位姿的快照**：存路线时按点名从点位表复制 x/y/yaw，之后改点位不影响已存路线；
点名缺失则整条路线拒绝，不写半条。

| 指令 | 路线各点的位置 |
|---|---|
| `goto`（带 route） | **目标点之前**：先依次走路线各点，再去目标点。 |
| `start_task`（带 route） | **顶升之后、放货点之前**：取货点 → 顶升 → 路线各点 → 放货点 → 下降。 |

- `route` 为空串 = 不用路线，仍是单点 / 两点导航。
- 选了路线却取不到（服务不可用 / 不存在），**直接判失败，不会偷偷退化成单点导航**。

## 五、搬货接口

任务层调搬货**只有一个入口**：`task_flow.py` → `cargo_client.step()` → 服务 `/agv/cargo`（`CargoCmd`）。

| 时机 | 请求 | 期望回执 |
|---|---|---|
| 到取货点之后 | `command="lift"` | `ok=true`、`busy=true`（正在搬货） |
| 到放货点之后 | `command="lower"` | `ok=true`、`busy=false`（放货完成） |

- `request_id` 填 `f'{cmd}-{毫秒时间戳}'`，仅用于追踪。
- 等应答上限由参数 `cargo_timeout_s` 决定；服务不可用（`wait_for_service` 3 s 超时）或无应答，都判该步失败 → `ERROR`。
- 服务实现在 `agv_lift/agv_lift/cargo_node.py`：**现阶段只把"正在搬货"标志置 1/0 后立即返回成功，不做任何动作**。
  接真实搬货硬件时改那一个文件即可，外部接口不变。

## 六、节点接口

| 接口 | 类型 | 方向 | 用途 |
|---|---|---|---|
| `/agv/command` | `AgvCommand` 话题 | 收 | 统一指令总线（唯一业务入口） |
| `/cmd_vel` | `Twist` 话题 | 收 | 速度心跳：判断"上层还在发指令" |
| `/cmd_vel` | `Twist` 话题 | 发 | 急停 / 超时 / 取消 / 任务失败时补一帧零速 |
| `/agv/manual_mode` | `Bool` 话题 | 收 | 手动模式互锁：手动时禁止导航 |
| `/agv/task_state` | `TaskState` 话题 | 发 | 状态上报：1 Hz 周期 + 状态变化时 |
| `/agv/query` | `AgvQuery` 服务 | 收 | 查询运行时状态：`what` = `state` / `pose` |
| `/agv/store` | `AgvStore` 服务 | 调 | 点位与路线的统一增删查列（`resource` + `action`） |
| `/agv/cargo` | `CargoCmd` 服务 | 调 | 顶升 / 下降（唯一搬货入口） |

导航：`nav2_simple_commander.BasicNavigator`，**首次导航时才创建**，所以没起 Nav2 也能先把节点拉起来。

> ★ `/cmd_vel` 同时是"收"和"发"：本节点发的停车帧会绕回自己的心跳订阅。
> 若不处理，这帧回声会把刚判定的"指令超时"清掉——自己解除自己的超时保护。
> 现在发之前调 `SafetyMonitor.note_self_publish()` 记账，收到回声就冲销；
> 并用 `timeout_edge()` 保证停车帧只在刚超时那一次发，不每 100 ms 重发。

## 七、参数

来源：`src/agv_bringup/config/default/task.yaml` + `safety.yaml`，由 `agv_stack.launch.py` 合并后注入
（`config/<档位>` 可覆盖，档位参数名 `param_profile`，**默认留空 = 只用 default**）。
全部由 `task_manager_node` 的 `declare_parameter` 声明：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `field_min` / `field_max` | −10.70 / 4.15 | **兜底**场地 x 边界（m）＝实测地图边界；仅在没有地图时生效 |
| `field_min_y` / `field_max_y` | −9.83 / 9.22 | **兜底**场地 y 边界（m）＝实测地图边界 |
| `waypoint_frame` | `map` | 点位与导航目标坐标系 |
| `nav_print_period` | 1.0 | 导航反馈打印周期（s），防刷屏 |
| `nav_timeout_s` | 180.0 | **单段**导航总超时（s）：超时取消目标并判失败 |
| `cargo_timeout_s` | 5.0 | 调 `/agv/cargo` 的等应答上限（s） |
| `robot_clearance` | 0.30 | 目标点所需净空（m）：车宽 0.5 → 半宽 0.26 + 余量 |
| `map_yaml` | `""` | 目标点判据用的地图 yaml；**留空则退化为上面的兜底矩形**（launch 会注入默认地图） |
| `cmd_timeout_ms` | 500 | 安全层：多久没收到新 `/cmd_vel` 就判超时停车（ms） |
| `estop_budget_ms` | 200 | 安全层：急停后发出停车的时限（ms），用于达标判定 |

> `cmd_timeout_ms` / `estop_budget_ms` 实际来自 `safety.yaml`，节点里合并进同一份参数。
> 判据器在地图读不到时**不抛异常**：记录原因后退化为矩形兜底，并在启动日志里说明当前用的是哪种判据。

## 八、运行

```bash
colcon build --packages-select agv_task
source install/setup.bash

# 只起任务层（需配合 agv_waypoint 的点位/路线服务与 agv_lift 的搬货服务）
ros2 run agv_task task_manager_node

# 或用参数中心一次起全套（waypoint_node + cargo_node + task_manager_node + 速度仲裁）
ros2 launch agv_bringup agv_stack.launch.py
ros2 launch agv_bringup agv_stack.launch.py with_sim:=true     # 再带上导航栈
ros2 launch agv_bringup agv_stack.launch.py with_sim:=true gui:=false rviz:=false
```

## 九、测试

三个纯逻辑单测，都不依赖 ROS，可直接 `pytest`：

```bash
pytest src/agv_task/test -v
```

| 文件 | 覆盖 |
|---|---|
| `test/test_command_router.py` | 24 条：白名单、坐标判据（矩形兜底 + 自定义判据注入）、nan/inf 被拒、缺参数、点名 / 坐标解析、`request_id` 去重、`stop` 绕过、route 透传、处理器异常隔离、已移除指令被拒 |
| `test/test_task_manager.py` | 10 条：主序列完整走通、状态无 `ALIGN_*`、按阶段报目标点名、执行中拒绝新任务、失败 / 取消 / 急停 / 恢复、`ERROR` 不可推进 |
| `test/test_safety_monitor.py` | 12 条：急停达标（≤200 ms）与超标、指令超时停车与新指令解除、手动互锁、急停后任务状态清理、**自发零速回声不清超时**、超时边沿只报一次 |
| `test/test_state_reporter.py` | 8 条：状态优先级（急停 > 搬运 > 导航 > 空闲）、进度随状态推进、解除急停回到真实态 |

> 上述只描述测试覆盖范围，**本机没有 ROS 2，未在仿真或实车上验证过**。

## 十、实现要点（踩过的坑）

1. **指令回调里绝不能同步等服务**：点名 → 坐标、路线取点都在**后台线程**里做，节点用
   `MultiThreadedExecutor`，查点 / 查路线的客户端挂**可重入回调组**。否则"等应答"与"处理应答"
   互相卡死，表现为点明明存在却查不到。
2. **回调只受理，不干活**：`handle_*` 只做前置检查并起后台线程（daemon），导航与搬运全在线程里跑；
   `_abort_task` 事件供取消 / 急停打断。
3. **日志不要在 `info` / `warn` 之间复用缓存方法**：rclpy 会抛
   `Logger severity cannot be changed between calls`，按结果分别调用。
4. **自发帧不能喂自己的心跳**：节点既发 `/cmd_vel` 又订阅它当心跳，发出去的零速会绕回来
   把"指令超时"清掉。发之前记账（`note_self_publish`），收回声时冲销。

## 十一、待接项

- **接真实搬货硬件**：改 `agv_lift/agv_lift/cargo_node.py` 的服务实现（加真正的顶升 / 下降动作与到位判断），
  服务名、字段、话题均不变，`agv_task` 无需改动。
