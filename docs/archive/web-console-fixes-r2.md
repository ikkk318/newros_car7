# 网页控制台 · 第二轮真机问题修复报告

> 承接《网页控制台验证报告.md》。那份记录的是"从零到能跑"的验证；
> 这份记录的是**真机实际使用中暴露出来的 7 个问题**及其根因。
>
> 日期：2026-10-18　范围：`/home/yuki/agv/newros_car7`

---

## 一、用户报的问题与根因

| # | 现象 | 根因 | 影响面 |
|---|---|---|---|
| 1 | 放多个任务，只执行第一个 | 上一单的 DONE 残帧被当成本单已完成；task_manager 收尾期拒绝新指令 | 任务系统完全不可用 |
| 2 | 任务不能选路线执行 | 后端 `add` 有 `route` 字段，前端**没做入口** | 路线功能形同虚设 |
| 3 | 已保存的路线进网页不显示，得新建一条才"刷新"出来 | ① `refreshRoutes()` 只在 `boot()` 调一次，栈没起时 503 后再不重试<br>② `waypoint_node` 读入内存后不再重读文件 | 看起来像"路线丢了" |
| 4 | 点地图选点没任何反应 | `endDrag` 用 `longPress === null` 判"长按已触发"，鼠标点击时它恒为 null → 选点被跳过 | 存点位功能不可用 |
| 5 | 小车是三角形，要求正方形 + 车头车尾标注 | 三角形缩小时三点糊成一点，看不出朝向 | 观感 / 可读性 |
| 6 | 中止任务后仍显示"执行中"，之后既不能终止也不能启新任务 | ① `cancel` 后没立刻 `publish_state`<br>② 中止被当**失败** → 状态机落 ERROR，而 ERROR 是硬拒绝态<br>③ 后台线程还活着，`_can_start` 的串行锁不放 | 任务系统死锁 |
| 7 | 日志刷"连接断开/读取任务失败"和一堆英文 | `POST /api/tasks` 缺 `list` 分支 → 400；ROS 原始日志直出 | 无法排障 |

---

## 二、修复要点

### 1. 多任务只跑第一个

`_wait_task_outcome()` 现在必须确认"**这一单真的开始了**"才开始等结果：

- 判据：`task_id` 与下发前不同（task_manager 每受理一单就 +1）
  **且** 状态进入活跃态；`goto` 单点导航（无 task_id）只认活跃态。
- 看到 `DONE` 后还要等状态回 `IDLE` 才交差 —— task_manager 收尾是
  「报 DONE → 睡 0.5s → 回 IDLE」，只等 DONE 会被 `_can_start()` 里的
  `_runner.is_alive()` 当场拒掉。
- 被拒绝（没启动）做 3 次有限重试，最终失败如实报出，不再假装成功。

**硬证据**：`tools/webtest/task_multi.sh 3` 实跑，日志出现
`task-0001 / task-0002 / task-0003` 三单，假 Nav2 收到 14 个导航目标
（两单走路线各 6 个目标 + 一单直连 2 个）。

### 2. 任务选路线

前端新增第 3 步 `pickRouteForTask()`（点选，带途经点数副标题），
任务卡片显示「经 路线名 / 直连」，点整行可改路线（走 `replace` 整表覆盖）。

### 3. 路线不显示

- `refreshRoutes()` 加 15 秒定时器；失败后 **2 秒快速重试 3 次**（新增 `_retryRoutesSoon`）。
  定时器是 15 秒一次，而"栈刚起来"窗口只有几秒，干等 15 秒体感就是"不显示"。
- `RouteStore` / `WaypointStore` 新增 `_reload_if_changed()`：每次对外读写前比对
  文件 mtime，变了就重读。
  **这不只是"看不到"，是数据丢失级**：旧实现在别人改过文件后，
  网页再保存会用旧内存整表覆盖，把终端刚加的路线**静默删除**。
- `/api/routes` 从 `add_post` 改成 `add_route('*')`，且处理器容忍空 body
  （GET 没有 body，`await request.json()` 会抛 → 500）。

### 4. 点地图选点没反应

`bindMap()` 重写：用「按下到抬起的时长 + 位移」判断，
不再依赖 `longPress` 是否为 null 这种间接推断。鼠标和触摸走同一条逻辑。

- 单击/轻触（≤500ms、位移 ≤6px）= 选点（`setPicked`）
- 拖动 = 平移视图
- 长按 650ms = 设导航目标
- `pointerleave` 只清拖动状态，**不**吞掉选点

### 5. 小车图标

正方形车体 + 车头侧深色条 + 中轴指向线 + 「车头/车尾」文字
（文字不跟车体旋转，斜着走也能读）。

`render_preview.py` 同步改为 `fill_poly` 逐像素还原：
原来用外接矩形画旋转后的色带，会糊成一大片歪色块，
看着像"色带跑到车尾去了"——那是**预览脚本**画错，不是前端。

### 6. 中止后卡死（本轮最隐蔽）

三处必须一起修，缺一不可：

**① `goto_pose` 增加 `abort_event` 参数**

原来 `waitUntilNav2Active()` 是阻塞的（真机 3~10 秒）。用户在这段里点中止：
`handle_cancel` 已经打了"已取消任务并停车"，但那时还没有导航目标，`cancel()` 是空操作；
等 `waitUntilNav2Active()` 返回，`goto_pose` **照样** `goToPose` ——
日志刚说完停车，20 毫秒后车又开出去。

修法：把等待放旁路线程轮询，每个关键节点复查标志，命中即返回 `CANCELED`。

**② `TaskFlow` 中止不再算失败**

`nav.step()` 返回 False 原来一律 `task_fail()` → `manager.fail()` → ERROR。
而 `_can_start()` 明确拒绝 ERROR（"搬运状态为 ERROR，无法接新任务"）——
于是用户点了中止，之后再也开不了新任务，只能重启节点。

修法：`_aborted()` 为真时走 `_stop_quietly()`（回 IDLE + 停车 + 上报），
不进 ERROR。**谁看到中止，谁负责把状态机摆回 IDLE** ——
取消是另一个线程置的标志，流程线程可能几百毫秒后才反应过来，
这期间状态机停在 GO_LOAD/LIFTING，让流程自己收尾语义才闭合。

**③ `handle_cancel` 等后台线程收尾**

`_can_start()` 有 `_runner.is_alive()` 的串行保护。中止是异步的，
线程可能还卡在 `goto_pose` 里。取消时同步 `join(1.0)`，退干净就清 `_runner`，
这样用户点完中止立刻点开始也不会被"已有任务在执行"拒掉。

**硬证据**：端到端第【7】项 —— 中止后 `running=false`、日志出现"取消"、
之后还能再开新单并真跑完（目标数 14 → 16）。

### 7. 日志中文化

见上一轮 `log_translate.py`；本轮补充：`/api/tasks` 补 `list` 分支，
掐掉"读取任务失败"刷屏的根源。

---

## 三、测试

### 后端（338 项）

```
agv_common     98 passed
agv_waypoint   53 passed   ← 新增 9 项（外部改动自动重载 / 不丢数据）
agv_task       66 passed   ← 新增 7 项（中止语义）
agv_web       121 passed   ← 新增 2 项（GET/POST 双方法）
```

### 前端（141 项）

新增：地图点选（单击/拖动/长按三态）、小车正方形与车头车尾标注、
路线定时刷新与失败不清空。

### 端到端（13 项，`tools/webtest/task_multi.sh`）

真起 `fake_nav` + `waypoint_node` + `cargo_node` + `task_manager` + 网关：

```
✅ 3 条任务依次加入
✅ route 字段落盘正确（1号,1号,-）
✅ 执行结束 running=false
✅ 3 次「搬运完成」
✅ 无「失败或超时」
✅ 导航目标 14 个（两单走路线各 6 + 一单直连 2）
✅ 中止后立刻 running=false
✅ 中止指令确实送达 task_manager
✅ 中止后还能再开新任务并跑完
✅ 新任务真的发出导航目标（14 → 16）
```

### 测试有效性验证

把修复回退成旧逻辑，确认测试**真的会红**：

- 回退 `task_flow` 中止收尾 → 4 项失败
- 回退 `_wait_task_outcome` 判定 → 5 项失败

一个永远绿的测试比没有测试更危险。

---

## 四、过程中踩的坑

| 坑 | 教训 |
|---|---|
| `ROS_DOMAIN_ID=235` | 上限 232，233 起 FastDDS 直接退出。改 211 |
| 假 Nav2 用默认 QoS 发 `/amcl_pose` | `BasicNavigator` 订阅用 `TRANSIENT_LOCAL`，QoS 不兼容收不到，卡在 "Waiting for amcl_pose" 无限循环 |
| 测试只起 task_manager | 缺 `waypoint_node` → "取货点不存在"；缺 `cargo_node` → "顶升失败"。**整条链缺一个都跑不通** |
| 手写 YAML 造"外部改动" | 少了顶层 `routes:` 键，测出假失败。改用第二个 `RouteStore` 实例扮演终端，更贴近真实 |
| 给 save/delete 回填 `count` | 破坏了原有契约（路线 save 的 `count` 是途经点数，不是全表规模），原测试立刻红。**接口契约不要为了"看起来完整"去改** |
| `/tmp` 不持久 | 备份文件会消失，别把唯一副本放 `/tmp` |

---

## 五、复现命令

```bash
cd /home/yuki/agv/newros_car7
source /opt/ros/humble/setup.bash && source install/setup.bash

# 全量后端（338 项）
/usr/bin/python3 -m pytest src/agv_web/test/ src/agv_common/test/ \
                      src/agv_waypoint/test/ src/agv_task/test/ -q

# 前端（141 项）
NODE_PATH=/home/yuki/.workbuddy/binaries/node/workspace/node_modules \
  node tools/webtest/ui_test.js

# 多任务 + 中止恢复端到端（约 2 分钟，真起 5 个节点）
bash tools/webtest/task_multi.sh 3

# 渲染目视
/usr/bin/python3 tools/webtest/render_preview.py
```

---

## 六、仍需真车确认

沙箱里 `/dev/shm` 只有 10MB，Gazebo 起不来，以下项目只能用假 Nav2 间接验证：

- Gazebo 与 Nav2 真实激活耗时（影响中止时机窗口）
- `/cmd_vel_raw` 实际下发与车体响应
- 顶升/下降的真实机械动作与到位判断
- AMCL 初始位姿收敛
- `map_server` 真实地图加载告警
- 手机端触感（点选/长按的 500ms / 650ms 阈值是否合适）
- 中止时实际停车距离（软件已经第一时间发零速，但物理惯性要实测）
