# 网页控制台

> 本文件是**网页侧机制的唯一真源**：界面交互、HTTP/WS 路由、栈管理、缓存语义、遥控与租约、任务缓冲、日志管道。
> 启动命令见 `OPERATIONS.md`，接口字段见 `INTERFACES.md`。

代码：`src/agv_web/agv_web/`（后端 `gateway/`）、`static/`（前端）。

---

## 1 组成

```
浏览器 ──HTTP/WS──► agv_web 网关（aiohttp）
                      ├─ gateway/state.py     状态快照、位姿、地图、日志缓存
                      ├─ gateway/commands.py  遥控、指令、任务、地图库
                      ├─ gateway/stack.py     起停栈、五步清理
                      ├─ gateway/web_app.py   路由表 + 中间件 + base/sys handler
                      ├─ gateway/handlers_*.py 业务 API
                      ├─ stack_manager.py     纯逻辑：该敲什么命令、能不能切
                      ├─ status_board.py      8 槽仪表 + 日志限流
                      ├─ log_translate.py     日志中文化
                      └─ grid_image.py        OccupancyGrid → PNG
```

网关是**唯一的网页进程**，它是 ROS 节点（订阅 `/map`、`/scan`、`/plan`、`/agv/task_state`、`/cmd_vel`、`/rosout` 与 TF），
同时是 HTTP 服务器，还负责起停整套 ROS 栈。

```bash
ros2 launch agv_web web_console.launch.py token:=agv2024
# 浏览器 http://<车机IP>:8080
```

`token` 是访问口令，登录后存 Cookie。不设口令则局域网内任何人都能控车。

---

## 2 界面与交互

| 区域 | 操作 |
|---|---|
| 地图 | 拖动平移、双指/滚轮缩放、单击取位置（存点位）、**长按 = 设导航目标** |
| 十字键 | 按住持续走、松手即停；速度档位 3 挡 |
| 工作模式 | 启动导航 / 开始建图 / 停止 |
| 任务卡片 | 添加、删除、改路线、开始执行、中止；列表 5 秒刷新一次 |
| 地图库 | 换图、删图、存当前地图 |
| 顶部 | 状态芯片（任务/模式/连接）+ 红色急停按钮 |

### 2.1 地图三态判定的坑

`bindMap()` 用「**按下时长 + 位移**」判三态：

| 手势 | 条件 | 动作 |
|---|---|---|
| 单击 | ≤500 ms 且 ≤6 px | 选点（存点位用） |
| 拖动 | 位移 >6 px | 平移地图 |
| 长按 | 650 ms 不松手 | 设导航目标 |

★ **不能用 `longPress === null` 间接推断**：鼠标点击时它恒为 `null`，
会把单击误判成"长按已触发"，导致选点永不生效。

### 2.2 小车图标

正方形车体 + 车头深色条 + 中轴指向线 + 「车头/车尾」文字。
★ **文字不跟车体旋转**（旋转后中文会歪，读不出来）。

### 2.3 不能有原生弹窗

全部换成统一模态框（手机上从底部弹出）。`uiConfirm/uiPrompt/uiPick/uiMulti/uiNumber` 五个封装。

★ 曾经用 `prompt()` 收数字，手机上会被浏览器拦截或体验极差；现在所有输入都是可点控件
（循环次数是 `+ / −` 步进器，不是敲数字）。

---

## 3 HTTP / WebSocket API（27 条）

26 个 handler + 1 条静态挂载。全部返回 JSON（`/api/map.png` 除外）。

| # | 方法 | 路径 | 请求 | 说明 |
|---|---|---|---|---|
| 1 | GET | `/` | — | 主页面 |
| 2 | GET | `/login` | — | 登录页 |
| 3 | POST | `/api/login` | `{token}` | 校验口令，种 Cookie |
| 4 | GET | `/api/state` | — | 全量状态快照（见 §8） |
| 5 | GET | `/api/map.png` | — | 当前地图 PNG；无图 404 |
| 6 | GET | `/api/logs` | `?since=` | 日志增量 |
| 7 | POST | `/api/lease` | `{action, who}` | `acquire` / `renew` / `release` |
| 8 | POST | `/api/teleop` | `{who, v, w}` | 遥控一帧 |
| 9 | POST | `/api/stop` | — | 急停（零速 + 强制释放 + 发 `stop`） |
| 10 | POST | `/api/command` | `{command, args, route}` | 发 `/agv/command` |
| 11 | POST | `/api/store` | `{resource, action, ...}` | 代理 `/agv/store` |
| 12 | POST | `/api/cargo` | `{command}` | 代理 `/agv/cargo` |
| 13 | POST | `/api/relocalize` | `{x, y, yaw}` | 发 `/initialpose` |
| 14 | GET/POST | `/api/maps` | — / `{action}` | 地图库列表 |
| 15 | POST | `/api/map_switch` | `{name}` | 换图（重起栈） |
| 16 | POST | `/api/map_delete` | `{name}` | 删图（含点位/路线/任务） |
| 17 | GET/POST | `/api/tasks` | POST `{action}` | 任务缓冲 CRUD |
| 18 | GET/POST | `/api/routes` | POST `{action}` | 路线 CRUD |
| 19 | GET/POST | `/api/check` | — | 8 项自检 |
| 20 | POST | `/api/resume` | — | 清急停 |
| 21 | POST | `/api/map_save` | `{name}` | 保存当前地图 |
| 22 | GET/POST | `/api/stack` | — | 栈状态 |
| 23 | POST | `/api/stack_start` | `{mode, map}` | 起栈（`nav` / `mapping`） |
| 24 | POST | `/api/stack_stop` | — | 停栈 |
| 25 | POST | `/api/shutdown` | `{who, stop_stack}` | 关网关（可顺带停栈） |
| 26 | GET | `/ws` | — | WebSocket，4 Hz 推状态 |
| 27 | GET | `/static/*` | — | 静态资源 |

### 3.1 双方法兼容

★ **前端 `api()` 助手固定发 POST**。后端只写 `add_get` 会 405，catch 后表现为"数据空"。
所以 `maps`/`tasks`/`routes`/`check`/`stack` 都同时注册 GET 与 POST。

反过来，处理器必须**容忍空 body**：GET 没有 body，`await request.json()` 会抛
"Expecting value" → 500。现有做法是 GET 时从查询串取参数、默认 `list`。

### 3.2 `/api/tasks` 的 action

| action | 请求 | 说明 |
|---|---|---|
| `list` / 空 | — | 返回 `{tasks, running}`。★ **POST 也必须支持 list**，否则前端 5 秒一次刷新全 400 → `taskRunning` 不更新 → 界面永远"执行中" |
| `add` | `{pick, drop, loops, route}` | 追加一条 |
| `delete` | `{index}` | 按序号删 |
| `replace` | `{tasks[]}` | 整表覆盖（改某单路线用，保顺序）；执行中返回 409 |
| `clear` | — | 清空 |
| `start` | — | 起后台线程逐条执行；已有任务返回 409 |
| `stop` | — | **必须走这里**：同时置 `_gw_task_stop` 事件 + 发 `cancel_task`。只发裸 `cancel_task` 会让后台线程空转到 300 秒超时 |

### 3.3 `/api/lease` 的语义

| action | 返回 |
|---|---|
| `acquire` | 被别人占 → 409 + `{code:LOCKED_BY_OTHER, owner}` |
| `renew` | 续期失败**返回 200 + ok:false**（后台定时调用，409 会刷满日志） |
| `release` | 释放 + 发零速 + 看门狗 idle |

★ 为什么要单独 `renew`：只靠发速度续期的话，用户拿到锁后"想一下往哪走"就会超时掉锁，
下次按方向键必须重新点一次「取得控制权」——每走一步点一次锁，没法用。
有了 `renew`，锁的语义是"我还在用这台车"，而不是"我这毫秒在发速度"。

---

## 4 栈管理

### 4.1 按钮与模式

| 按钮 | 行为 |
|---|---|
| 启动导航 | 弹选图框 → `stack_start(mode=nav, map=...)` |
| 开始建图 | `stack_start(mode=mapping)`（无需选图） |
| 停止 | `stack_stop`（五步清理） |

- **只有一张图也弹框**：建图/导航按钮挨着，误点时如果连地图都不问就把整套栈拉起来，人没有撤销机会
- 点取消什么都不发生
- 默认选中当前正在用的那张
- 已在某模式下再点同一按钮不重启（Nav2 激活要 15~40 秒，白等）
- 切模式**先停旧的再起新的**（两套栈会互抢话题/服务）

### 4.2 五步清理（`StackRunner.stop()`）

① SIGINT 整组 → ② 等整组死透 → ③ 不死再 SIGKILL → ④ 清 `/dev/shm` 无主 `fastrtps_*` → ⑤ 等 11345 端口空出。

为什么必须这么严：

- `proc.wait()` **只等 launch 父进程**，`gzserver` 是兄弟进程，还占着 11345
- 端口被占时 `gzserver` bind 失败是**静默的**（日志照打，世界里没数据）
- Ctrl-C 打断的节点死在 `destroy_node()`，FastDDS 的 SHM 段没 unlink → 新参与者发现不到老参与者
  → "节点全在、话题全在，彼此收不到"

`start()` 之前也会先补一次清理；`_op_lock`（RLock）串行 start/stop/switch，
`_gw_lock` 只改字段（否则停栈十几秒会堵住 `/api/state`）。

### 4.3 建图 vs 导航的遥控通道

| 模式 | 网关发到 | 链路 |
|---|---|---|
| 导航 | `/agv/web_teleop` | task_manager 校验互锁 → `/cmd_vel` → arbiter → `/cmd_vel_raw` |
| 建图 | `/cmd_vel_raw` | 直驱底盘（建图栈没有 arbiter / task_manager） |

分流在 `stack_manager.teleop_topic_for()`。
★ **照导航模式发 `/agv/web_teleop`，建图时车轮一动不动**。

### 4.4 启动自清理

网关启动时扫一遍上次没退干净的 ROS 进程，SIGINT 给 6 秒 → SIGKILL → 清 SHM → 等端口。
排除自己的祖先链与名字含 `agv_web` 的 launch（否则会自杀）。

★ 启动自清理的模式表**不能包含 `agv_web`**。

---

## 5 位姿与时基

### 5.1 tf2 Buffer 会缓存 10 秒历史

发布者死后 `lookup_transform` **不报错**，仍返回最后一个变换（stamp 很旧）。
所以判位姿失效**必须看 stamp 年龄**（`_tf_age`），不能只看能不能查到。

### 5.2 换栈必须主动作废

`tf_buffer.clear()` + 位姿/扫描/路径/地图/日志全部清空，不能指望"被新数据覆盖"——没数据就永远不覆盖。

### 5.3 时基必须一致

`use_sim_time` 与 TF 时间戳必须同源。曾经网关用系统时钟、Gazebo 用仿真时钟，
导致位姿被判"陈旧 57 年"，界面一直显示定位丢失。
网关默认 `use_sim_time=true`（跟着 Gazebo），无仿真时用系统时钟。

### 5.4 重定位

`/api/relocalize` 发 `/initialpose`。

- 发布前必须确认有订阅者（`get_subscription_count()`），为 0 就**如实报失败**，不要假装成功
- 前端**不能把当前位姿原样发回**（等于"我在原地"，AMCL 什么都不会改）
- 发布后本地位姿缓存立刻跟上，否则界面要等下一次 TF 才动
- 失败有三个常见原因：没有订阅者 / `map` 帧不存在 / 时间戳与 TF 不同源

---

## 6 地图缓存与分发

### 6.1 两个版本号别混

| 字段 | 什么时候变 | 用途 |
|---|---|---|
| `map_epoch` | **换栈/切模式才 +1** | 前端靠它判断"要不要丢掉旧图" |
| `map_version` | 建图时 0.8 秒一变 | 前端靠它判断"有没有新帧" |

★ 混用会让前端在每次建图更新时重载 PNG（闪、卡）。epoch 一个动作只能 +1
（谁判断"模式真变了"谁负责清）。

### 6.2 限流

`slam_toolbox` 每 0.5 秒发一次全量 `/map`，594×762 编码约 0.11 秒。
网关按 `map_min_interval`（默认 0.8 s）限流，否则吃满 CPU 并阻塞 ROS 回调。

### 6.3 中文地图名必须转 ASCII

★ 中文路径传进 `nav2_bringup` 会变成 `u5730u56FE…`（`\uXXXX` 转义且反斜杠丢失）
→ `bad file` → bringup 中止 → 没有 `/map` → **网页全黑只剩起始点**。

`MapManager.launch_map_yaml(name)`：路径全 ASCII 就原样返回；否则生成
`maps/.launch/<sha1>/map.yaml`（`image: map.pgm`）+ `map.pgm` 软链，把这条纯 ASCII 路径传给 launch。
`world666map` 因此零变化。

### 6.4 地图库

- `maps/<名字>/` 下放 `<名字>.pgm` + `<名字>.yaml` + `waypoints.yaml` + `routes.yaml` + `tasks.yaml`
- `maps/.launch/` 是生成物（已 gitignore），可以删，按需重建
- `maps/_mapping/` 是建图草稿（地图名未定），保存时并入正式地图
- 导航**不需要先建图**，有 pgm+yaml 就能导

---

## 7 遥控、租约与急停

三层，各管一段：

| 层 | 管什么 | 参数 |
|---|---|---|
| `ControlLease` | 控制权归属（谁在遥控） | `lease_idle_sec=120` |
| `TeleopWatchdog` | 没新指令就发零速 | `teleop_stale_sec=0.35` |
| `SafetyMonitor`（任务层） | 速度心跳超时 / 急停 | `cmd_timeout_ms=500` |

★ **停车和还锁是两件事**。`on_tick` 里看到门狗超时**绝不能** `lease.force_release()`——
那等于把 120 秒的锁降级成 0.35 秒，用户一松手就掉锁（实测 15:58:29 取得 → 15:58:33 收回）。
现在超时只发零速，日志写"控制权保留"。

★ **松手 ≠ 掉线**：`api_teleop` 收到 (0,0) 是用户松手，走 `watchdog.idle()`；
非零才 `feed()`。把零速也 feed 进去会让 0.35 秒后误报一次"心跳超时"。

★ **急停是唯一允许 `force_release()` 的地方**（`/api/stop`）。

---

## 8 状态快照与仪表盘

`snapshot()` 一次取全：位姿、扫描点、路径、地图版本/epoch、日志尾、任务、避障状态、控制权、栈状态。

### 8.1 8 个固定槽位

`GAUGES` 固定 8 槽，**位置恒定**：

| # | 槽 | 取值来源 |
|---|---|---|
| 1 | 任务 | 状态机 `state` + 进度 |
| 2 | 动作 | 导航/搬运（靠 `task_id` 区分） |
| 3 | 速度 | `/cmd_vel` 最新帧 |
| 4 | 避障 | `LogFilter.avoid_engaged()` 续命 3 秒 |
| 5 | 定位 | 位姿新鲜度 + 误差 |
| 6 | 地图 | 加载状态 + 格数 |
| 7 | 模式 | `nav` / `mapping` |
| 8 | 连接 | WebSocket 状态 |

★ **面板值必须来自状态量，不能来自日志文本**。旧实现会"固定死"：避障一停就再没新日志，
那格永远停在上次内容。现在 `snapshot()` 组 `gstate` → `build_gauges()`，日志只是旁路。

★ 前端 `BOARD_SKELETON` 是同一份清单，改一处必须两边一起改。

### 8.2 CSS 约束

`.bv` 必须 `word-break: break-word`；**不能有 `line-clamp` / `max-height` / `overflow-y`**
（这三样正是"日志长看不到后面"的元凶）；移动端 ≤860px 两列。

---

## 9 日志管道

两条通道，同一件事会进来两遍：

| 通道 | 路径 | 特点 |
|---|---|---|
| ① 子进程 stdout | `_pump_output` → `log_translate` → `log()` | 带 `[节点名] ` 前缀 |
| ② `/rosout` | `on_rosout` | 无前缀，但知道"顶升成功=绿"这类语义 |

### 9.1 去重

`_norm_log()` **只剥"身份前缀"**，`[顶升] `、`[1/5] ` 这类语义前缀必须保留，否则不同事件会被折叠。
重复时按 `_LEVEL_RANK` 合并成更醒目的级别，不能直接丢。

### 9.2 限流顺序

**限流表 → 事件词 → 兜底折叠**，顺序不能反。

- 限流表：`Passing new path to controller`(30s)、`Planner loop missed its desired rate`(60s)、
  `Behavior Tree tick rate`(60s) 这类碎碎念
- 事件词：`[1/5]`、`顶升`、`拒绝`、`完成` 必须放行（兜底归一化会把它们变得一样）
- 兜底折叠：剩下的按归一化文本去重

★ 加噪音规则**必须排除级别**：前缀要写 `(?!.*\[(?:ERROR|WARN|WARNING|FATAL)\])`，
否则 `[ERROR] … Navigating to goal aborted` 会被 "Navigating to goal" 误杀。

### 9.3 拒绝原因要可见

`_RELAY_WANT` 放行 `\[指令\].*?拒绝`；`on_rosout` 把最近一条拒绝原文缓存到 `_gw_last_reject`，
任务启动失败时**如实转述**。
★ **别臆测原因**——旧文案"路线「」不存在"在没选路线时必然是个空名字，纯误导。

### 9.4 WebSocket 短抖

手机 WiFi 每 10~26 秒掉一次、2~3 秒回来。断开满 5 秒才提示（`_wsDownTimer`），
且只有提示过断开才补"重新连上"。重连间隔 2s→10s 逐步拉长。

---

## 10 前端模块

4 个经典脚本，按序加载（**不是 ES module，不加 `type="module"`**）：

| 文件 | 内容 |
|---|---|
| `core.js` | 全局状态 `S`、`$`、`api()`、模态框五个封装、`uiPose`、日志渲染 |
| `map.js` | canvas、坐标变换、`draw*`、地图加载/缓存、拖拽缩放、仪表盘 `renderBoard` |
| `console.js` | WebSocket / `applyState`、遥控方向盘、点位、路线 |
| `ui.js` | 任务、地图库、自检、栈控制、模式切换、登录、`bindUI`、`DOMContentLoaded` |

规则：

- **每个顶层符号只能声明一次**（重复 `const` 整页 `SyntaxError` 白屏）
- `canvas` 在 `map.js` 顶层取 DOM → 4 个 script 必须在 `</body>` 前
- `app.css` 单文件，不拆
- 改完 `node --check` 逐个过，再浏览器实测

坐标约定（写在 `app.js` 头注，拆分后仍在 `map.js`）：

```
ROS 世界：x 右、y 上，单位米
Canvas：  x 右、y 下
地图 origin 是左下角
px = (x - origin_x)/res
py = height - (y - origin_y)/res
```

---

## 11 自检 8 项

`/api/check` 按模式分支：

| 模式 | 节点 | 服务 |
|---|---|---|
| `nav` | `task_manager_node`、`velocity_arbiter`、`amcl`、`controller_server`、`planner_server` | `/agv/query`、`/agv/store`、`/agv/cargo` |
| `mapping` | `slam_toolbox`、`robot_state_publisher`、`waypoint_node` | `/agv/store` |

加「地图已加载」「定位可用」「激光可用」三项数据链路检查。

★ 建图模式**不能**照导航模式查那 3 个服务与 Nav2 节点——建图栈里根本没有，
会显示三个假红。这是"建图自检一片红"的根因。

★ 地图加载失败时，自检要**转述 map_server 的原文**（`_gw_map_error`），而不是只说"未收到 /map"。

---

## 12 冒烟（改完网页代码至少跑一遍）

```bash
ros2 launch agv_web web_console.launch.py token:=smoke manage_stack:=false port:=8099
```

```bash
B=http://127.0.0.1:8099
curl -s -c /tmp/c -X POST $B/api/login -H 'Content-Type: application/json' -d '{"token":"smoke"}'
curl -s -b /tmp/c $B/api/state | head -c 200
curl -s -b /tmp/c -o /dev/null -w '%{http_code}\n' $B/api/map.png        # 无栈 404
curl -s -b /tmp/c -X POST $B/api/tasks -H 'Content-Type: application/json' -d '{"action":"list"}'
curl -s -b /tmp/c -o /dev/null -w '%{http_code}\n' $B/static/app.css     # 200
```

浏览器打开：登录 → 地图区有网格 → 方向键无 JS 报错 → 任务卡片能刷新 → 点「停止」不报错。
