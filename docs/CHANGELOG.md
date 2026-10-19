# 变更记录

> 只记"现象 → 根因 → 修复"的关键结论。接口细节见 `INTERFACES.md`，
> 历史原文（含逐项证据表）归档在 `docs/archive/`。

---

## 2026-10-19 重构（本次）

**目标**：文档从 15 份 4773 行压到 7 份；删掉全部测试与手工验收脚本；把 3236 行的网关拆开。

| 项 | 变化 |
|---|---|
| 文档 | 9 份根文档 + 6 份包 README → `README` + `docs/` 6 篇；三份修复报告与 git 说明归档 |
| 测试 | 删除 `src/*/test`（31 文件 541 项）与 `tools/webtest`（整目录）。回归代码在 `86706ea`，可 `git checkout` 取回 |
| 后端 | `web_gateway.py` 3236 行 → `gateway/` 8 个模块（256~691 行）+ 150 行入口 `main()` |
| 前端 | `app.js` 2685 行 → `core.js` 567 / `map.js` 583 / `console.js` 805 / `ui.js` 780 |
| 文件 | 删零引用 launch（`web_mapping`、`cargo`、`waypoint`）、`.bak/`、缓存、1.1G 编辑器索引 |
| 死码 | 删 `agv_task/init_robot_pose.py`（只是 `goto_pose` 的薄封装，零引用） |
| 保留入口 | 网页 `web_console`、业务栈 `agv_stack`、导航/建图 `navigation`/`mapping`、仿真 `sim`、终端控制台、`gazebo_view`、`stop_all`、`make_world_map` |

**没做的事**：`tools/agv_console.py`（1474 行）**故意不拆**。它全是交互式
`input()` 流程，拆完无法自动回归，收益（可读性）抵不上"日常入口被改坏"的风险。
只保留了文件里原有的 7 个分节标题，要拆时按那 7 段切即可。

**为什么删测试**：用户明确要求（"检查测试的代码文件可以直接删除"）。
代价是失去自动化回归，补偿是每阶段手动冒烟（`OPERATIONS.md` §9）与 git 可回滚。

---

## 第三轮：换栈残影 / 位姿作废 / 重定位（2026-10-19）

| 现象 | 根因 | 修复 |
|---|---|---|
| 导航切建图后地图黑屏、节点"不启动" | 换栈只杀了 launch 父进程；`gzserver` 兄弟进程仍占 11345，bind 失败**静默**；Ctrl-C 的节点死在 `destroy_node()`，FastDDS SHM 段未 unlink | `StackRunner.stop()` 五步清理；`start()` 前补清理；`_op_lock` 串行 |
| 换栈后位姿停在旧位置 | tf2 Buffer 缓存 10 秒历史，发布者死后 `lookup_transform` 不报错 | 判失效看 **stamp 年龄**；换栈 `tf_buffer.clear()` + 全部缓存作废 |
| 建图时前端反复重载地图 PNG | `map_epoch` 与 `map_version` 混用 | 拆成两个字段；epoch 一个动作只 +1 |
| 重新定位点了没反应 | 无订阅者时假装成功；前端把当前位姿原样发回 | 订阅数为 0 如实报失败；前端不发当前位置 |
| 中文地图名导航全黑 | `RewrittenYaml` 的 `yaml.dump` 未传 `allow_unicode`，中文变 `u5730u56FE…`，map_server `bad file` | `MapManager.launch_map_yaml()` 生成 ASCII 别名目录 |

## 第二轮：七类真机问题（2026-10-18）

| 现象 | 根因 | 修复 |
|---|---|---|
| 放多个任务只跑第一个 | 网关只认"`task_id` 变了"，第二次下发时上一单还没回 IDLE | 判据改为"id 变了 **且** 状态进活跃态"；并发拒绝要可见 |
| 任务选不了路线 | 前端只把路线名发给 `start_task`，后端没落到 `route` 字段 | 传 `route` 并透传到 `/agv/command` |
| 路线列表不显示 | `api()` 固定 POST，后端只注册 GET → 405 | 双方法注册 + 容忍空 body |
| 地图上点不中位置 | 单击/长按判定靠 `longPress === null` 推断 | 改用「按下时长 + 位移」三态 |
| 中止后卡"执行中"，之后既不能停也不能开新任务 | ① `POST /api/tasks` 缺 `list` 分支 → 刷新 400 → `taskRunning` 不更新；② 中止只发裸 `cancel_task`，`_gw_task_stop` 未置位，线程空转 300 秒；③ `handle_cancel` 改状态机却没 `publish_state()` | 三处全补；中止走 `stop_tasks()` |
| 中止落了 ERROR，之后无法启新任务 | `ERROR` 是 `_can_start()` 硬拒绝态 | 中止走 `_stop_quietly()`（回 IDLE + 停车 + 上报） |
| 日志满屏英文 | ROS/Nav2 原始输出直接透传 | `log_translate.py` 过滤噪音 + 翻译已知句式，翻不出的保留原文 |

## 第一轮：从零到能跑（2026-10-17）

| 现象 | 根因 | 修复 |
|---|---|---|
| 位姿被判"陈旧 57 年" | 网关用系统时钟、Gazebo 用仿真时钟 | `use_sim_time` 与 TF 同源 |
| handler 里 `NameError` | 闭包误用 `self`（闭包里没有 `self`，只有 `node`） | 统一 `node.*`；拆分后 handler 只认 `self.node` |
| 网页需要 8 个 `prompt()` | 原生弹窗在手机上体验极差 | 全部换成模态框 + 步进器 |

---

## 更早的接口演进

- 8 个同构 CRUD 服务合并为 `/agv/store`（`resource` + `action` 分发）
- 5 个平行数组改成 `NamedPose[]`（消灭下标错位导致的静默 bug）
- 删除精定位/docking 整套方案，改为"到点即顶升"
- 接口文件从 12 个降到 6 个
