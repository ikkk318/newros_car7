# 网页控制台 · 第三轮换栈与点位问题修复报告

> 承接《网页控制台修复报告-第二轮.md》。那份修的是**任务系统**；
> 这份修的是**模式切换（导航↔建图）留下的残影**、**位姿显示失准**、
> **点位缺朝向**、**重定位点不动**。
>
> 日期：2026-10-18　范围：`/home/yuki/agv/newros_car7`

---

## 一、用户报的问题与根因

| # | 现象 | 根因 | 严重度 |
|---|---|---|---|
| 1 | 导航↔建图互相切换后地图**有重影** | ① 网关的地图/位姿/激光缓存**只被新数据覆盖，从不作废**<br>② 前端 `loadMap` 无请求序号，乱序响应会让旧图盖掉新图 | 高（看错地图会撞） |
| 2 | 切换后小车**停在切换前的位置** | `refresh_pose` 里「查不到位姿就什么都不做」，旧位姿无限期沿用（**底下还压着 tf2 的坑，见 §2.2**） | 高（位置显示是假的） |
| 3 | 地图选点保存**没有 yaw** | 前端 `saveWaypoint` 里硬编码 `yaw: 0.0`，界面也从不显示朝向 | 中 |
| 4 | 只能点地图加点位，**没有输入方式** | 只有 `setPicked` 一条路径 | 中 |
| 5 | **重新定位用不了** | ① 前端把**当前位姿原样发回**（等于"我在原地告诉 AMCL 我在原地"）<br>② 没有位姿时直接 `return`（而那正是最需要它的时候）<br>③ 后端**哑发**：publish 完就回 `{'ok': True}`，没订阅者也说成功 | 高（功能形同虚设） |

---

## 二、修复要点

### 2.1 缓存必须「主动作废」，不能只靠「被覆盖」

旧实现里 `_gw_map_png / _gw_map_meta / _gw_pose / _gw_scan / _gw_plan`
只有**写入路径**，没有**作废路径**。换栈之后，旧栈的数据会一直活着，
直到新栈产出一份新数据把它盖掉 —— 而建图模式下**地图永远不会再来第二份**，
导航模式下 AMCL 也要几秒才收敛。这段窗口里画出来的就是旧栈的东西 = 重影。

新增 `reset_runtime_cache(reason)`，一处集中作废：

```python
def reset_runtime_cache(self, reason=''):
    with self._gw_lock:
        had_map = self._gw_map_png is not None
        self._gw_map_png = None
        self._gw_map_meta = None
        self._gw_map_epoch += 1
        self._gw_pose = None
        self._gw_scan = None
        self._gw_plan = None
        self._gw_task_state = None
        epoch = self._gw_map_epoch
    self._gw_last_pose_ok = 0.0
    self._gw_last_map_encode = 0.0
    try:
        self.tf_buffer.clear()      # ★ 不清就会"复活"旧栈位姿（见 §2.2）
    except Exception as exc:
        self.log('warn', f'tf 缓存清理失败（{type(exc).__name__}）：{exc}；'
                         '换栈后位姿可能短暂沿用旧值')
    self.log('info', f'{reason}：已清理上一套栈的地图/位姿/激光/路径缓存'
                     f'（地图代次 {epoch}{"" if had_map else "，原本就没有地图"}）')
    return epoch
```

### 2.2 最深的一层：tf2 Buffer 缓存 10 秒历史

这一条单独说，因为它**推翻了上一版的判断**。

第一版修复给位姿加了「保鲜期」：`_gw_pose` 超过 `pose_hold_sec` 就作废。
逻辑上没问题，但端到端测出来**位姿一直在变、yaw 每帧都不同**，
保鲜期根本不触发。

原因：`tf2.Buffer` 默认缓存 **10 秒**历史。发布者死了以后，
`lookup_transform('map', 'base_link', Time())` **不报错**，它返回
「最后一个已知变换」—— 查得到、transform 有效、只是 `header.stamp` 很旧。
所以「查不到才计时」这个判据永远为假，`reset_runtime_cache` 清掉的
`_gw_pose` 在 0.2 秒后又被这个旧变换填满。**小车就又停回旧位置了。**

修法：判失效**必须看 stamp 年龄**，不能看"查没查到"。

```python
age = self._tf_age(tf)
if age is not None and age > self.pose_hold_sec:
    stale = True
    pose = None

def _tf_age(self, tf):
    """该变换距今多少秒（算不出返回 None）。"""
    try:
        stamp = tf.header.stamp
        now = self.get_clock().now().to_msg()
        return ((now.sec + now.nanosec * 1e-9)
                - (stamp.sec + stamp.nanosec * 1e-9))
    except Exception:
        return None
```

加上 `reset_runtime_cache` 里的 `tf_buffer.clear()`，双保险。

**教训**：`lookup_transform` 成功 ≠ 数据新鲜。tf2 是"有历史"的，
它把「发布者死了」这件事藏得很好。

### 2.3 `map_epoch` 与 `map_version` 语义必须分开

- `map_version`：建图时**每 0.8 秒就变一次**（新扫描进来）
- `map_epoch`：**只有换栈才变**（新增，从 1 起）

混用会让前端每次建图更新都重载 PNG（无谓开销 + 闪烁）。
前端拿到新的 `map_epoch` 时的动作比"重载图片"重得多：
清空几何、复位自动适配、重新跟随、丢弃选中点。

`ws_handler` 里**先发 epoch 再发版本**，且代次变了要把 `last_ver` 复位成 -1：

```python
if snap['map_epoch'] != last_epoch:
    last_epoch = snap['map_epoch']
    last_ver = -1               # 代次变了，版本比较要重来
    await ws.send_json({'type': 'map_epoch', 'epoch': last_epoch})
if snap['map_version'] != last_ver:
    ...
```

顺序反了的话，前端会先用旧几何重载一次图，再被 epoch 清掉 —— 白闪一下。

### 2.4 epoch 一个动作只能 +1

第一版实现里 `set_mode` 内部清一次、`api_stack_start` 又补清一次，
端到端看到 epoch 从 1 直跳 3。

原则：**谁判断出「模式真的变了」，谁负责清。**

- `set_mode`：`changed = self.mode != m`，只有 `changed` 才清
- `_do_switch`：`prev_mode = node.mode; node.set_mode(mode);`
  `if prev_mode == mode: node.reset_runtime_cache(...)`
  （只处理「模式没变但原地重启栈」这一种情况，不重复清）
- `api_stack_start` 的降级分支：`if not changed: node.reset_runtime_cache(...)`

### 2.5 重新定位为什么「用不了」—— 三个原因叠在一起

**① 前端把当前位姿原样发回**

旧代码 `relocalize()` 拿 `S.snap.pose` 直接 POST。位姿是网关刚从 TF 查到的，
再发回去给 AMCL —— 相当于「我在原地告诉 AMCL 我在原地」。什么都不会变。

修法：弹框让用户**真的改**。新增 `uiPose(init, opts)` —— 可拖拽的方向盘 +
x/y/角度输入 + 0/90/180/270 快捷按钮，拖盘实时预览朝向箭头。

**② 没有位姿时直接 return**

旧代码 `if (!p) { pushLog('warn', '还没有位姿'); return; }`。
可"定位丢了"恰恰就是最需要重定位的时候 —— 没位姿的时候它反而是唯一有用的动作。

修法：无位姿也能点，初值回落到「地图中心 / 上一次已知位置 / 原点」。

**③ 后端哑发**

旧代码 `publish(msg); return {'ok': True}`。建图模式下根本没有 AMCL 订阅
`/initialpose`，消息发到空气里，界面却报成功。

修法：查订阅者数量，如实反馈：

```python
try:
    n_sub = self.initial_pub.get_subscription_count()
except Exception:
    n_sub = -1          # 查不了就别乱报失败

self.initial_pub.publish(msg)
with self._gw_lock:
    self._gw_pose = (float(x), float(y), float(yaw), time.time())
self._gw_last_pose_ok = time.monotonic()      # 本地缓存立刻跟上，不等收敛

if n_sub == 0:
    return False, ('已发布 /initialpose，但当前没有任何节点订阅它'
                   '（建图模式没有 AMCL，或导航栈还没激活）——这条重定位不会生效')
return True, f'已发布重定位 (x {x:.2f}, y {y:.2f}, θ {math.degrees(float(yaw)):.0f}°)'
```

### 2.6 点位 yaw

- `saveWaypoint` 走 `uiPose`，提交真实朝向（不再硬编码 0.0）
- 地图上画朝向箭头，点位列表显示 `θ 90°`
- 建图模式下**不画点位**：坐标系是 map 但地图还没冻结，画上去会飘

### 2.7 新增点位输入方式

`addWaypointManual()`：手输名字 + x + y + 朝向，默认值依次回落
`S.picked` → 当前位姿 → 原点。配合新增的 `editWaypoint()`，
已有点位也能改坐标/朝向/类型/名字。

---

## 三、附带优化（用户要求"看看还有什么能改进的"）

| 优化 | 说明 |
|---|---|
| 点位列表 **N+1 → 1 次** | `refreshWaypoints` 优先用 list 返回的 `items`。旧代码 10 个点位要发 11 次 ROS 服务调用，每次几百毫秒。保留老后端逐个 `get` 的回落分支 |
| 点位显示朝向 + 编辑按钮 | 原来只看得到名字和坐标，朝向是存的但看不见，也没法改 |
| 建图模式不画点位/清选中点 | 建图时点位图会随地图漂移，误导 |
| `manage_stack=false` 可切模式 | 旧代码 `/api/stack_start` 直接 503。但手工起栈的用户按"启动导航"按钮应该至少把**遥控通道**切过去（网关的职责），只是不代管进程。现在走降级分支并提示"ROS 栈请你在终端自行启动" |
| 位姿缺失时明确提示 | 读数区显示「定位未就绪（等 AMCL 或点「重新定位」）」+ `stale` 灰化，而不是留一个看不出真假的旧数字 |
| 地图 PNG 防手机缓存 | `cache:'no-store'` + 时间戳。只加时间戳防不住移动端浏览器的 PNG 启发式缓存 |

---

## 四、测试

### 后端（365 项，+27）

```
agv_common     98 passed
agv_waypoint   53 passed
agv_task       66 passed
agv_web       148 passed   ← 新增 27 项（test_stack_switch_reset.py）
```

新增 `src/agv_web/test/test_stack_switch_reset.py`（595 行 / 27 项）覆盖：
`reset_runtime_cache` 全清 + epoch 递增 + 幂等 + tf clear 失败不崩；
`set_mode` 只在变化时清；`refresh_pose` 保鲜期 + **过期变换判定**；
`set_initial_pose` 订阅者数量语义；WS 帧顺序；`manage_stack=false` 降级。

> **注意**：跑后端测试**必须先 source 环境**。不 source 会看到 15 个
> `ModuleNotFoundError: No module named 'agv_common.xxx'` 的 collection error，
> 看着像代码坏了，其实只是 `PYTHONPATH` 里没有 `install/`。

### 前端（191 项，+50）

```
【22】点位 yaw 必须带上（真派发 pointerdown 到方向盘左侧）
【23】手输方式添加点位
【24】重定位的三个根因
【25】换栈不留残影
【26】点位列表一次拿全
```

### 端到端（30 项，新增 `tools/webtest/stack_reset.sh`）

真起 injector + 真起网关 + 真连 WebSocket（aiohttp，`ws://…/ws?token=…`）：

```
【3】切模式：旧地图 / 旧位姿必须作废
  ✓ 数据源消失后位姿按保鲜期自然失效（证明过期判定在工作，
    而不是靠"查不到"这个不可靠信号）
  ✓ manage_stack=false 时切模式走降级路径而不是 503
  ✓ map_epoch 变了（1 → 2）
  ✓ 旧地图几何已清空（否则新图按旧 origin 画 = 重影）
  ✓ 旧位姿已清空（否则小车停在切换前的位置）
  ✓ 旧激光点已清空（属于另一套坐标系）

【4】WS 是不是真把换栈信号推给浏览器了
  ✓ 收到 map_epoch 事件（2 次）
  ✓ map_epoch 早于随后的 map_changed
  ✓ 最后一条 epoch 事件与快照一致（2 == 2）
  ✓ epoch 之后**每一帧** state 都没有旧位姿
  ✓ 位姿没有"清掉之后又被填回来"（tf2 缓存复活旧栈位姿的症状）

【5】重新定位：如实反馈
  /api/relocalize → {'ok': False, 'message': '已发布 /initialpose，但当前没有
  任何节点订阅它（建图模式没有 AMCL，或导航栈还没激活）——这条重定位不会生效'}
  ✓ 没有 AMCL 订阅者时如实报失败
  ✓ 发布后本地缓存位姿立刻可见（不等 AMCL 收敛）

──────────────────────────────────────────────
  通过 30 项，失败 0 项
```

已接入 `tools/webtest/run_all.sh` 的 `reset` 分组。

### 测试有效性验证（逐项故意退回旧逻辑，确认真的会红）

| 退回的改动 | 变红的测试 |
|---|---|
| 删掉 `_tf_age` 判定（`if False`） | `test_drops_stale_tf_even_when_lookup_succeeds`，并打印出被复活的 `(7.0, 8.0, 0.0, …)` |
| 去掉 `tf_buffer.clear()` | `test_clears_tf_buffer` + `test_tf_clear_failure_does_not_crash` |
| 退回「查不到才计时」 | `test_drops_pose_after_hold_window` |
| 重定位退回哑发（`if False`） | `test_fails_when_no_subscriber` |
| 前端写回 `yaw: 0.0` | 「提交的 yaw 是调过的朝向（实际 0）」 |

一个永远绿的测试比没有测试更危险。

---

## 五、过程中踩的坑

| 坑 | 教训 |
|---|---|
| **tf2 Buffer 缓存 10 秒** | `lookup_transform` 成功 ≠ 数据新鲜。判失效看 `header.stamp` 年龄，不看"查没查到" |
| 端到端首跑全红 | `install/` 里是旧编译产物。改完 Python 必须 `colcon build --symlink-install --packages-select agv_web`，并确认 `build/agv_web/agv_web/web_gateway.py` 里有新代码 |
| WS 握手 403 | `guard_middleware` 也拦 `/ws`，必须带 token：`ws://…/ws?token=xxx` |
| 稳态也收到 1 次 `map_epoch` | 新连接握手时后端必然补发当前 epoch（`last_epoch` 初始 -1）。epoch 从 1 起，测试改成「最多 1 次且必须是首帧」 |
| 换栈后位姿立刻被填回 | injector 是独立进程，不知道栈换了，还在发 TF。测试里**先停 injector** 才等价真实换栈（真机是 SIGINT 整个进程组） |
| 「事件 epoch 与快照一致」取第一条 | 采集窗口内有两次清理（先停栈再起栈），取第一条会误报。改取**最后**一条 |
| 「换栈前那一帧还有位姿」 | 与「特意停 injector 验证保鲜期」自相矛盾。改成更本质的性质：**位姿不允许清掉之后又被填回** |
| epoch 连跳两格（1→3） | 两处都在清。改「谁判断出模式真变，谁清」 |
| 前端 canvas 打桩 | jsdom 的 canvas 2d 缺方法，补 `rect() {}` / `ellipse() {}` / `strokeText() {}` |
| yaw 显示 3.1° 而不是 180° | `fmt(normYaw(RAD(d)), 1)` 里 `d` 已经是度，再 `RAD()` 一次变 3.14 度 |
| `/tmp` 不持久 | 备份别放那（这轮又踩了一次） |

---

## 六、复现命令

```bash
cd /home/yuki/agv/newros_car7
source /opt/ros/humble/setup.bash && source install/setup.bash

# 后端（365 项）—— 不 source 会报 collection error
/usr/bin/python3 -m pytest src/agv_web/test/ src/agv_common/test/ \
                      src/agv_waypoint/test/ src/agv_task/test/ -q

# 前端（191 项）
NODE_PATH=/home/yuki/.workbuddy/binaries/node/workspace/node_modules \
  node tools/webtest/ui_test.js

# 换栈端到端（30 项，约 1 分钟，真起 injector + 网关 + WS 客户端）
bash tools/webtest/stack_reset.sh

# 只跑本轮新增的后端用例
/usr/bin/python3 -m pytest src/agv_web/test/test_stack_switch_reset.py -q

# 一键全跑
bash tools/webtest/run_all.sh
```

---

## 七、仍需真车确认

沙箱里 `/dev/shm` 只有 10MB，Gazebo 起不来，以下只能真车看：

- **`pose_hold_sec=2.0` 是否合适**：真实 Nav2 激活要 3~10 秒，
  这段时间里位姿会显示"未就绪"。如果嫌提示时间太长，可以调到 3.0；
  调太小会被 AMCL 偶发丢帧骗到（车原地"闪没"）
- **方向盘拖拽手感**：`uiPose` 的方向盘在手机小屏上是否够大、拖拽是否跟手
- **AMCL 真实收敛**：重定位后 1~3 秒内小车图标应该移到目标点并稳定
- **真实 SIGINT 换栈下的 tf2 缓存表现**：进程组整体退出时
  `tf_buffer.clear()` 是否还有必要（保守起见保留）
- **建图模式下点「重新定位」的提示**是否够清楚（现在会明确说没有 AMCL 订阅）
