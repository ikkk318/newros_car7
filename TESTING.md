# 测试指南 —— 怎么验证 AGV 项目

**测试策略：纯逻辑单元测试为主，节点级 / 仿真级靠人工验收。**

- **纯逻辑单测**（不需要 ROS、不需要仿真，秒级）：各包 `test/` 下的 pytest，覆盖存储、路由、状态机、安全判定、地图预检、避障策略等"算得对不对"的部分。
- **人工验收**（需要 ROS，多数需要仿真）：控制台一键自检 + 场景演练 + 安全验证，覆盖"接得通不通、状态对不对、能不能停下"。
- ROS 节点级 / 仿真级自动化测试**已删除**（依赖仿真环境，跑起来慢且不稳定），当前只保留纯逻辑单测。

> 改完代码先 `colcon build && source install/setup.bash`，否则跑的还是旧版本。
>
> 本机没有 ROS 2 环境，本文档的命令与预期现象均为**静态核对结果，未实际执行验证**。

---

## 一、怎么跑测试

### 1. 用 colcon 跑（推荐，符合 ROS 2 惯例）

```bash
# 全量
colcon test
colcon test-result --verbose

# 只跑有单测的包
colcon test --packages-select agv_common agv_waypoint agv_task
colcon test-result --verbose
```

### 2. 直接跑 pytest（最快）

```bash
# 三个包一起
python3 -m pytest src/agv_common/test src/agv_waypoint/test src/agv_task/test -q

# 单个文件
python3 -m pytest src/agv_task/test/test_command_router.py -q
```

> 直接跑 pytest 时若报 `ModuleNotFoundError: agv_common`，把包的源码目录加进 `PYTHONPATH`：
> `PYTHONPATH=src/agv_common:src/agv_waypoint:src/agv_task python3 -m pytest src/*/test -q`

### 3. 一键自检（需要 ROS）

```bash
python3 tools/agv_console.py check     # 自己拉起三个业务节点，不需要仿真
```

覆盖 23 项检查（点位/路线增删查、存储服务的资源/动作校验、搬货服务、指令门卫、急停、查询服务），结尾会打印 `通过 N 项，失败 M 项`；有失败时退出码为 1。

---

## 二、测试清单（逐个文件）

三个包、十四个测试文件，共 **164 个测试函数**。

### `agv_common`（纯逻辑工具库）

| 测试文件 | 测什么 |
|---|---|
| `src/agv_common/test/test_geometry.py` | 数值限幅 `clamp`（含上下界传反）、角度归一化 `wrap_to_pi`、角度差 `angle_diff`、两点距离、yaw↔四元数互转往返一致 |
| `src/agv_common/test/test_goal_check.py` | **目标点唯一判据 `GoalValidator`**：无地图时用矩形兜底（界内受理 / 界外拒绝 / 缺省兜底等于不拦 / 只给部分 field 也正常）、地图读不到时**退化为兜底并记录原因**（不抛异常）、坐标非数字拒绝、`describe()` 能说明当前判据；**真实实测地图验收回归**：地图外坐标（x=6/−20、y=±15、9999）**全部拒绝**，而**五个槽内作业点与六个搬货点全部放行**（收紧判据不得误拦"进槽作业"）、槽内最小净空确实 ≥ 0.30 m、`task.yaml` 的 `field_*` 与实测地图对齐且已无 `precheck_enabled` |
| `src/agv_common/test/test_limits_config.py` | `VelocityLimits` 线/角速度对称限幅；`deep_merge` 标量覆盖 + 嵌套递归且不改原字典；`load_yaml` 空文件返回 `{}`；`load_config` 的 `default → 覆盖层` 合并；整层文件都不存在时跳过 |
| `src/agv_common/test/test_map_check.py` | 用合成小地图（P5 pgm + yaml）测：PGM 尺寸/内容解析、地图元数据解析、空地中央点合法、落在墙上/图外/离墙太近（净空不足）三种拒绝并给出中文原因、离墙越远净空越大、走廊瓶颈宽度计算（开阔 / 窄缝 / 完全不连通返回 None） |
| `src/agv_common/test/test_map_manager.py` | `valid_name` 合法性与目录穿越拦截；建目录 / 存地图 / 列表 / 删除（整目录含点位一起走）；源文件不存在与删不存在地图的失败分支；`yaml` 的 `image` 字段被修正为同名 pgm；只有 pgm 没 yaml 不算有效地图 |
| `src/agv_common/test/test_reactive_avoid.py` | 反应式避障三层逻辑：前方远 → 原样放行；中距 → 按比例减速；太近 → 线速度归零；**上层要停（v_nav=0）时减速下限不得把它抬成前进速度**；减速下限不超过上层速度；左空右堵 → 左偏（w>0）、右空左堵 → 右偏（w<0）；障碍远离后角速度分步交还（不一步跳回）；持续远离后彻底交还并解除接管；**★ 倒车（v_nav<0）时不得夺走转向**（否则车在槽内直退会因横向偏移符号翻转而变成原地左右摇摆、退不出来），且左右对称偏移给出相同转向、倒车不因前方近而刹停；**★ 到点原地对准（v_nav=0）时不得夺走转向、停车（v=w=0）时不得凭空注入旋转**（否则槽内前方恒为槽底挡板 → 航向永远收敛不了 → `goToPose` 不返回 → 任务超时，即"明明到了却以为没到"），同时**真在往前挪时择向接管仍有效**（防改死功能）；`reset` 清状态；inf/nan/0 等无效读数不影响判断 |
| `src/agv_common/test/test_reverse_capability.py` | **倒车能力跨包配置守卫**（读 agv_navigation/agv_bringup 的配置，不 import 其它包）：DWB `min_vel_x < 0`（否则控制器根本不生成倒车轨迹 → 车"能进不能出"）；倒车速度不超过仲裁器 `max_linear` 且 ≥ −0.30（后方盲区要保守）；`velocity_smoother.min_velocity[0] < 0`（否则倒车在此被削成 0）；`behavior_server` 仍保留 `backup` 恢复动作；局部代价地图 `track_unknown_space: false`（否则车后盲区变障碍、倒车被堵死）；全局地图仍为 `true`（全局路径不得穿未知区）|

### `agv_waypoint`（点位 / 路线存储）

| 测试文件 | 测什么 |
|---|---|
| `src/agv_waypoint/test/test_waypoint_store.py` | 存点后能查到（含 `kind`）、`nav` 类型存取与按类型筛名、非法 `kind` 回退成 `cargo`、坐标非数字拒绝写盘、重名覆盖、删/查不存在返回 False/None 不抛异常、空表列点、重启后点位与类型仍在、返回值是副本（外部改动不污染内存）、坏 YAML 与文件不存在都不崩、**老点位文件（无 kind 字段）向后兼容** |
| `src/agv_waypoint/test/test_route_store.py` | 存路线后能查且途经点顺序保持、重名覆盖、空路线名/空路线拒绝、删/查不存在返回 False/None、列名与各条途经点数、重启后路线还在、返回值是副本、坏 YAML 与文件不存在都不崩 |
| `src/agv_waypoint/test/test_store_dispatch.py` | 统一存储服务 `/agv/store` 的调度逻辑（取代原先 8 个同构服务各自的校验）：未知资源/未知动作拒绝、**每类响应都带 `ok`（含 `list`，修掉原先 `WaypointList`/`RouteList` 缺 `ok` 的缺陷）**、点位存查删列、重名覆盖、不存在不报错、列表按名排序且字段齐、路线存的是**位姿快照**（改点位不影响已存路线）、空路线拒绝、**有点名不存在则整条拒绝不写半条**、列表给名字与点数且同序、途经点 `kind` 留空、同名点位与路线互不干扰；**响应字段取值矩阵（防契约漂移）**：`route+list` 的 `items` 刻意为空（无法表达"哪条路线的哪些点"，要看途经点用 `route+get`）、`waypoint+list` 才填 `items`、`get` 填 `items` 与 `count`、`save`/`delete` 不填 `items`、任何动作的 `ok` 恒为 `bool` 且 `message` 非空 |

### `agv_task`（任务层纯逻辑）

| 测试文件 | 测什么 |
|---|---|
| `src/agv_task/test/test_command_router.py` | 白名单与参数校验：未知指令拒绝、**已删除的 `lift`/`lower`/`dock` 一律按未知指令拒绝**、点名（含纯数字点名）受理、两参数按坐标处理（yaw 缺省 0）、**`nan`/`inf` 被判格式非法拒绝**、`goto` 缺参数拒绝、`start_task` 需两个参数、同 `request_id` 只受理一次（去重）、`stop` 绕过去重与参数校验永远受理、处理器能拿到 `route` 字段、白名单内但未注册的指令拒绝、处理器抛异常被隔离不影响路由；**坐标判据只有一份**：矩形兜底时 x/y 独立边界生效、未注入判据则不拦坐标、换成自定义判据后路由按它判（不再看 `field_*`）、两参数坐标同样过判据、`GoalValidator` 能说明当前判据、地图读不到时退化为矩形而不抛异常 |
| `src/agv_task/test/test_safety_monitor.py` | 急停进急停态且禁导航、急停停车耗时 ≤200 ms 判达标、超过预算判不达标、指令超时停车（累计 dt 到阈值）、新速度指令解除超时、手动模式禁导航（互锁）、急停后任务状态需外部清理、人工复位清急停；**★ 自发零速的回声不得清掉自己的超时**（否则节点发停车帧会"自解锁"）、真上层指令仍能解除超时、回声记账只冲销一帧不误吞后续真指令、超时边沿只报一次（不每 tick 重发停车帧） |
| `src/agv_task/test/test_state_reporter.py` | 状态组装优先级：都空闲报 `IDLE`、搬运任务优先于单点导航、无任务时报单点导航、导航完成/失败、**急停显示优先于一切**、解除急停后回到真实状态、搬运进度随状态推进（0.15/0.40/0.60/0.85）、未知状态不崩 |
| `src/agv_task/test/test_task_manager.py` | **搬运主序列不含 `ALIGN_*`（精定位状态已删，有专门的用例断言）**且严格等于 `GO_LOAD → LIFTING → TRANSPORT → LOWERING → DONE`；正常走完自动回 IDLE；按阶段返回取货点/放货点；执行中拒绝新任务（串行）；缺取货/放货点名拒绝；失败进 ERROR 且 `reset` 回 IDLE；急停进 PAUSED 且 `reset` 清任务回 IDLE（不再提供「续跑暂停前那一格」的 `resume()`）；空闲时急停不产生 PAUSED；`cancel` 清目标；ERROR 态不可推进 |

---

## 三、纯逻辑测试怎么补充

**约定：新增一个纯逻辑模块（不含 ROS import、可脱离仿真运行）时，就在同包 `test/` 下配一个 `test_<模块名>.py`。**

三条规矩：

1. **模块必须真的能脱离 ROS 跑**——只依赖标准库 / PyYAML / 本包内其他纯逻辑模块；ROS 只出现在 `*_node.py` 里。
2. **一个行为一个测试函数**，函数名说清"测什么"，失败信息用中文（参考现有文件里 `assert ok, reason` 的写法，把服务/函数的理由直接当断言消息）。
3. **临时文件一律用 pytest 的 `tmp_path`**，不要写到工作区里；不要依赖网络、仿真、时间真实流逝（`dt` 当参数传进去）。

样例骨架：

```python
# src/agv_<包名>/test/test_my_logic.py
"""my_logic 模块单元测试：<一句话说明测什么>。"""
from agv_<包名>.my_logic import MyLogic          # use 被测对象


def test_normal_case(tmp_path):                  # # 正常路径
    m = MyLogic(str(tmp_path / 'x.yaml'))        # use 用临时目录，不污染工作区
    assert m.do(1.0) is True


def test_bad_input_rejected(tmp_path):           # # 非法输入：拒绝且不抛异常
    m = MyLogic(str(tmp_path / 'x.yaml'))
    assert m.do('不是数字') is False
    assert m.count() == 0


def test_persistence(tmp_path):                  # # 持久化：重建对象后内容还在
    path = str(tmp_path / 'x.yaml')
    MyLogic(path).do(1.0)
    assert MyLogic(path).count() == 1
```

补完之后跑一遍：

```bash
python3 -m pytest src/agv_<包名>/test -q
```

---

## 四、人工验收清单

纯逻辑单测证明不了「接得通、状态对、能停下」，下面三块必须人工跑一遍。

### A. 一键自检（不需要仿真）

```bash
python3 tools/agv_console.py check
```

预期结尾：`通过 23 项，失败 0 项`。逐项对照（按代码执行顺序）：

- [ ] 存点（带类型 `nav`）
- [ ] 查点：坐标与类型正确
- [ ] 查不存在的点：返回失败而不是崩溃
- [ ] 列点：包含刚存的点
- [ ] 列点响应带 `ok` 字段（原 `WaypointList` 缺 `ok`，已修）
- [ ] 存路线（按点名复制位姿，`count == 1`）
- [ ] 查路线：途经点正确
- [ ] 列路线：包含刚存的路线
- [ ] 删路线
- [ ] 删不存在的路线：返回失败而不是崩溃
- [ ] 删点（清理自检痕迹）
- [ ] 存储服务：未知资源（`resource='shelf'`）被拒
- [ ] 存储服务：未知动作（`action='rename'`）被拒
- [ ] 存储服务：空路线被拒（不留空壳）
- [ ] 搬货服务：顶升受理且「正在搬货」标志置 1
- [ ] 搬货服务：下降受理且标志归 0
- [ ] 搬货服务：未知命令被拒
- [ ] 未知指令（`fly`）被拒
- [ ] 越界坐标（`9999, 9999`）被拒
- [ ] 重复请求（同一 `request_id`）被拒 —— 探针是 `cancel_task`：它无参、通过全部校验，
      且此时本来就没任务，不会让车动。**别改回越界坐标**：`goto 9999 9999` 会在
      "坐标"那一步就被拦下，走不到去重，这一项会恒判失败。
- [ ] 急停生效
- [ ] 查询服务有应答
- [ ] 存储服务能列路线

- [ ] **自检自己会收尾**：临时点位 `自检点<时间戳>` 与临时路线 `自检路线<时间戳>` 在结束时已被删除（结束后用 `[2] 列出点位` / `[7] 路线管理 → [2]` 复核，列表里不应留下这两个名字）。
- [ ] **自检会把车 resume 回空闲**：自检内部发了 `stop` 之后会补一次 `resume`，结束时 `[s] 状态快照` 应显示 `IDLE`，而不是 `PAUSED`。

### B. 场景演练（需要仿真）

按顺序做，每步记下"该看到什么"：

- [ ] **1 建图**：控制台进建图模式（或 `ros2 launch agv_navigation mapping.launch.py`）→ 按 `k` 遥控走一圈。
      通过现象：RViz 里 `map` 逐渐长出与场地一致的轮廓；走完后按 `m` 命名保存，控制台提示已保存，`maps/<地图名>/` 下出现 `<地图名>.pgm`、`<地图名>.yaml` 与（首次保存时补齐的）`waypoints.yaml`、`routes.yaml`。
- [ ] **2 存点**：切导航模式加载该地图，按 `1` 存至少 2 个「搬货点」（取货点、放货点）和 1 个「导航点」。
      通过现象：按 `2` 列点能看到名字、类型（搬货点/导航点）、坐标。
- [ ] **3 存路线**：按 `7` → `1`，点名用 `,` 隔开（如 `取货点,走廊A`），再按 `7` → `3` 看详情。
      通过现象：保存提示途经点数正确；详情里顺序与输入一致、坐标是快照值。
- [ ] **4 点名导航**：先按 `n` 确认 5 个 lifecycle 节点（`map_server` / `amcl` / `bt_navigator` / `controller_server` / `planner_server`）全部「已激活」，再按 `4` 输入点名。
      通过现象：`[s]` 里状态从 `IDLE` 变 `GO_LOAD`（进度 0.5、提示「导航中」），车朝目标点走，到位后回 `IDLE`。
- [ ] **5 完整搬运**：按 `t` 填取货点、放货点（可选路线）。
      通过现象：按 `[s]` 观察状态逐格推进 `GO_LOAD → LIFTING → TRANSPORT → LOWERING → DONE → IDLE`；`[s]` 里「搬货状态」在 `LIFTING` 期间显示「正在搬货（标志=1）」，`LOWERING` 后回到「空闲（标志=0）」。
- [ ] **6 任务系统跑多条任务**：按 `9` → `1` 添加 2~3 条任务（可设循环次数与路线）→ `3` 核对列表 → `5` 启动。任务按地图存在 `maps/<地图名>/tasks.yaml`。
      通过现象：逐条打印 `── [i/总数] 取货点 ⇒ 放货点 第 k/循环数 次 ──`，完成的打印 `✔ 第 i 次搬运完成`、失败的打印 `✘ 第 i 次搬运失败（任务状态 ERROR）`；全部跑完打印 `任务结束：成功 N 次，失败 M 次`。中间可输入 `s` 看状态、`l` 看任务缓冲、`q` 退出（会取消当前搬运）。
      连跑多条时若恰好撞上上一单的收尾期，会打印 `[重发] 还没见到任务起来（第 N 次），重新下发`，**属正常自愈**，最终仍应是 `✔`；若看到 `[重发]` 后跟着 `✘ 超时` 且日志尾没有本单的 `GO_LOAD`，才是指令真没被受理。
- [ ] **任务系统返回导航主界面（不是退出终端）**：按 `9` 进任务系统 → 按 `q`。
      通过现象：控制台提示 `已返回导航主界面（任务系统）` 并**重新打印导航主菜单**；此时提示符仍在等命令，**终端没有关闭**。必须再按一次 `q`（在导航主界面上）才会退出终端。
- [ ] **任务系统返回即清空并停车**（承上）：上一步按 `q` 时若缓冲非空，会先问"返回会清空全部任务并停车"。
      通过现象：确认后控制台发出零速；重新进 `9` 时 `[3] 列出任务缓冲` 为空（任务不跨会话保留）。答 `n` 则留在任务系统、缓冲不动。
- [ ] **各功能子界面都能返回上一层**：分别进 `7` 路线管理、`6` 手动、`9` 任务系统，在里面按 `q`。
      通过现象：每次都回到导航主界面并重印主菜单（提示 `已返回导航主界面（…）`），终端不退出；只有导航主界面按 `q` 才退出。
- [ ] **导航主界面按 q 有活时会确认**：让车在跑任务（或留几条缓冲）时在导航主界面按 `q`。
      通过现象：先提示检测到任务在跑/缓冲非空并询问；答 `n` 留在主界面且**车不被停、缓冲不清**；答 `y` 才取消任务、清空缓冲并退出终端。
- [ ] **7 换地图**：按 `0` 回到地图选择、加载另一份地图。
      通过现象：整套栈重起，点位列出的点位来自新地图的文件（不会串到上一份地图的点位）。当前项目只有 `world666map` 一份地图，要验证换图需先用建图模式再存一份（如 `测试图A`）。

### C. 安全验证

- [ ] **急停后车停**：车在跑时按 `x`（或发 `stop`）。
      通过现象：立刻发出零速，车在 0.5 s 内停下（速度仲裁的 `cmd_timeout` 兜底，正常是立即停）。
- [ ] **状态变 PAUSED**：急停后 `[s]` 显示 `PAUSED`，消息为「已急停（安全停止）」。
- [ ] **PAUSED 期间导航被拒**：急停状态下再发 `goto` / `start_task`，日志出现 `处于急停状态，请先恢复（resume）`，车不动。
- [ ] **resume 回 IDLE**：按 `r`（或发 `resume`）后 `[s]` 显示 `IDLE`，随后能重新下指令，不必重启节点。
- [ ] **手动模式下拒绝导航**：按 `6` 进入手动控制期间（或手工把 `/agv/manual_mode` 置 `true`），发 `goto` 应被拒（`当前是手动模式，禁止导航`）；手动控制结束后控制台会自动置回自动模式。
- [ ] **上层停发速度后 0.5 s 内停**：
      ```bash
      ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
      # Ctrl-C 停掉上一条命令后观察
      ros2 topic echo /cmd_vel_raw
      ```
      通过现象：Ctrl-C 后约 0.5 s 内 `/cmd_vel_raw` 输出变成全 0（仲裁节点超时停车），车停下。
- [ ] **地图外 / 压障碍的坐标一律被拒（已收敛为一个判据）**：目标点合法性只由 `GoalValidator` 判——有 `map_yaml` 时以地图为准（图外 / 压障碍 / 净空不足直接拒），没地图才退回 `field_*` 矩形兜底。`task.yaml` 的 `field_*` 已与实测地图对齐（`-10.70 / 4.15 / -9.83 / 9.22`），不会再出现"门卫放行、别处拒绝"的两种结论。
      ```bash
      # 这个坐标落在地图外（x=6 > 4.15），应被直接拒绝
      ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
          "{source: 'test', command: 'goto', args: ['6.0', '0.0', '0'], request_id: 'oob-map-1'}"
      ```
      通过现象：日志出现 `超出场地范围`／`在地图范围外`，指令被拒，车不动。
      另外可对照：槽内作业点（如 `2.53, 2.69`）净空约 0.38 m，**仍应放行**——收紧判据不能把正常进槽作业一起拦掉。

---

## 五、调试手段

| 要看什么 | 命令 / 操作 |
|---|---|
| 有哪些节点在跑 | `ros2 node list` |
| 话题有没有数据、频率对不对 | `ros2 topic hz /scan`、`ros2 topic hz /cmd_vel_raw`、`ros2 topic hz /agv/task_state` |
| 速度链路走到哪一步 | `ros2 topic echo /cmd_vel`（Nav2/任务层输出）对比 `ros2 topic echo /cmd_vel_raw`（底盘实际收到） |
| 任务栈日志尾 | 控制台导航模式按 **`l`**（读整套栈日志的最后 15 行） |
| Nav2 激活状态 | 控制台导航模式按 **`n`**；或 `ros2 lifecycle get /amcl`（`bt_navigator` / `controller_server` / `planner_server` / `map_server` 同理）。注意必须精确匹配 `active`——`inactive` 里也含 `active` 子串 |
| 定位链是否通 | `ros2 run tf2_ros tf2_echo map base_footprint`（有输出说明 AMCL 已发布 `map→odom`） |
| 点位 / 路线内容 | `ros2 service call /agv/store agv_interfaces/srv/AgvStore "{resource: 'waypoint', action: 'list'}"`（路线把 `resource` 换成 `'route'`）；也可以直接看 `maps/world666map/waypoints.yaml` |
| 当前地图几何 | `maps/world666map/world666map.yaml`（`resolution` / `origin`）；PGM 头两行给尺寸（594 × 762） |
| 目标点判据是哪一种 | 看 `task_manager_node` 启动日志里的「目标点判据 …」：`判据=实测地图 <路径>（… 所需净空 0.30 m）` 表示以地图为准；出现 `判据=矩形兜底` 说明地图没读到，此时 `field_*` 才生效 |
| 搬货是否在忙 | `ros2 topic echo /agv/cargo/busy` |
| 子节点各自的日志 | 控制台启动时会打印子节点日志目录（`manage_nodes` 托管模式下为 `waypoint_node.log` / `cargo_node.log` / `task_manager_node.log`） |

> 读控制台日志时：`RTPS_TRANSPORT` / `open_and_lock` 这类共享内存端口告警无害，控制台会自动过滤。

---

## 六、验收标准

**功能正确 + 状态正确 + 能停下**，三项都满足即通过：

1. **功能正确**：指令被正确受理/拒绝，点位与路线增删查列结果正确，搬货服务对 `lift` / `lower` / 未知命令给出正确回执。
2. **状态正确**：`/agv/task_state` 的状态序列与阶段推进符合预期（`GO_LOAD → LIFTING → TRANSPORT → LOWERING → DONE → IDLE`），急停进 `PAUSED`、`resume` 回 `IDLE`。
3. **能停下**：急停立即停；上层停发速度后 0.5 s 内归零；任务失败/取消时发出零速。

> 本阶段**不设精度类指标**（到位误差、成功率之类）：当前是「到点即顶升」，不做对位，故不做量化承诺；后续需要时再引入。
