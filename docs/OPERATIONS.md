# 操作手册（构建 / 启动 / 建图 / 任务 / 排障）

> 本文是**启动与操作命令的唯一真源**。README 只留下最常用的三条命令。

---

## 1 环境

| 项 | 要求 |
|---|---|
| 系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| 仿真 | Gazebo Classic 11 |
| Python | 3.10（系统 python，勿用 conda/自编译版本） |
| 额外依赖 | `python3-aiohttp`（网页网关）、`python3-yaml`、`python3-numpy`、`python3-pil`（`make_world_map.py`） |

```bash
sudo apt install python3-aiohttp python3-yaml python3-numpy python3-pil
```

★ **不要让非系统 Python 排在 PATH 前面**：受管 Python（3.12/3.13）会让
`spawn_entity.py` 找不到 ROS 的 numpy（cp310），表现为**机器人根本没生成**、
`/odom` 有话题但 Publisher count 为 0。启动前确认：

```bash
which python3           # 应为 /usr/bin/python3
echo $PYTHONPATH        # 应为空或不含自定义 site-packages
```

---

## 2 构建

```bash
cd /home/yuki/agv/newros_car7
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只改了一个包时：

```bash
colcon build --packages-select agv_web && source install/setup.bash
```

★ **`install/` 是普通拷贝不是软链**（除非用了 `--symlink-install`）。改了 Python、launch 或
`static/*.js|css` 之后**必须重新 build**，否则 `ros2 launch` 跑的还是旧文件——
症状是"改了没反应"，很容易误判成代码没生效。

### 2.1 参数覆盖层

参数中心在 `src/agv_bringup/config/`：

```
config/default/*.yaml     打底（task / safety / cargo / waypoint / hardware）
config/real/*.yaml        实车覆盖（当前只有 README 占位）
```

`param_profile` 默认空 = **只用 default**；实车差异写好后传 `param_profile:=real`。
★ 参数名就叫 `param_profile`，**别改成 `profile`**——会被 gzserver 当成 `--profile` 吃掉。

---

## 3 启动矩阵

| 场景 | 命令 |
|---|---|
| **网页控制台（推荐）** | `ros2 launch agv_web web_console.launch.py token:=agv2024` |
| 网页只做代理，接已起好的栈 | `ros2 launch agv_web web_console.launch.py token:=agv2024 manage_stack:=false` |
| 终端控制台 | `python3 tools/agv_console.py` |
| 终端控制台：一键自检 | `python3 tools/agv_console.py check` |
| 终端控制台：列出地图库 | `python3 tools/agv_console.py maps` |
| 仿真 + 导航栈（不经网页） | `ros2 launch agv_navigation navigation.launch.py` |
| 仿真 + 建图栈（不经网页） | `ros2 launch agv_navigation mapping.launch.py` |
| 只起业务节点（不仿真） | `ros2 launch agv_bringup agv_stack.launch.py` |
| 业务节点 + 仿真 + 导航 | `ros2 launch agv_bringup agv_stack.launch.py with_sim:=true` |
| 看 Gazebo 里的车（网页无头栈时） | `python3 tools/gazebo_view.py` |

网页入口只起网关，**建图/导航由网页按钮拉起**。浏览器打开 `http://<车机IP>:8080`。

★ `token:=` 就是访问口令。不设口令则局域网内任何人都能控车，**正式使用务必设置并改掉示例口令**。

### 3.1 网页上三个按钮

| 按钮 | 做什么 |
|---|---|
| 启动导航 | 拉起 `agv_stack`（仿真 + AMCL + Nav2 + 任务层 + 仲裁）；**一定先弹选图框**（只有一张也弹，防止误点没有撤销机会） |
| 开始建图 | 拉起建图栈（仿真 + slam_toolbox），走一圈后点「保存地图」 |
| 停止 | 停整套栈（SIGINT → 等 → SIGKILL → 清 SHM → 等端口） |

切模式**先停旧的再起新的**——两套栈并存会互相抢话题/服务，表现是原地转、无响应。
已在某模式下再点同一按钮不会重启（Nav2 激活要 15~40 秒，白等）。

### 3.2 launch 之间的包含关系

```
web_console.launch.py          只起网关
  └─(网页按钮) agv_stack.launch.py with_sim:=true
        ├─ waypoint_node / cargo_node / task_manager_node / velocity_arbiter
        └─ navigation.launch.py
              ├─ sim.launch.py（Gazebo + spawn + EKF + RViz）
              └─ nav2_bringup
  └─(网页按钮) mapping.launch.py
        ├─ sim.launch.py
        ├─ slam_toolbox
        └─ waypoint_node
```

---

## 4 建图

### 4.1 网页建图（推荐）

1. 打开网页 → 点「开始建图」（等 Gazebo 起来，约 10~20 秒）
2. 用十字键遥控走一圈覆盖场地 —— 地图**实时长出来**（右上角显示格数与米数）
3. 走完点「保存地图」，输入名字 → 存进 `maps/<名字>/`
4. 切「启动导航」，在弹框里选这张图

保存时会自动做三件事：① 把建图草稿点位（`maps/_mapping/`）并入新地图；
② 补齐该地图专属的空 `waypoints.yaml` / `routes.yaml`；③ 给中文地图名生成 ASCII 别名目录
（`maps/.launch/<sha1>/`，否则 map_server 会因 `\uXXXX` 转义打不开文件、导航全黑）。

### 4.2 终端建图

```bash
ros2 launch agv_navigation mapping.launch.py
# 另开终端遥控（建图模式直接驱底盘）
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_raw
```

用终端控制台更省事：建图模式按 `k` 遥控、`m` 命名保存（自动落 `maps/<名字>/`）。

★ 建图时遥控必须走 `/cmd_vel_raw`（URDF 里 `cmd_vel:=cmd_vel_raw` 写死）。
**照导航模式发 `/agv/web_teleop` 车轮一动不动**——建图栈里没有 `task_manager`。

★ slam_toolbox 每 0.5 秒发一次全量 `/map`，网关按 `map_min_interval`（0.8 s）限流，
否则吃掉大量 CPU 并阻塞 ROS 回调。

---

## 5 点位 / 路线 / 任务

字段与格式见 `INTERFACES.md` §3、§8。这里只讲操作。

### 5.1 存点位

三种入口等价：网页点击地图、终端控制台、`ros2 service call`。

- **搬货点**（取货/放货用，`kind: cargo`）：要停在**能顶到货架**的位置
- **导航点**（只用来组路线，`kind: nav`）：只要可达即可

`world666map` 预置 **15 个点位**：5 个货架 + 5 个槽内 + 4 个导航点 + 起始点。
槽内点是两段式进槽的终点，不要手动改。

### 5.2 存路线

按点名从点位表**复制位姿快照**——之后改名/删点不影响已存路线；有点名不存在则整条拒绝。
`world666map` 预置两条：`1号`（4 点）、`路线1`（2 点）。

### 5.3 任务缓冲

任务与地图绑定（`maps/<地图名>/tasks.yaml`），由控制台「任务系统」读写，**节点不读它**。
每条 `{pick, drop, loops, route}`，执行时按 `loops` 次发 `start_task`，等回 `IDLE`/`DONE` 才发下一条。

### 5.4 自动驾驶一次搬运

```bash
ros2 topic pub --once /agv/command agv_interfaces/msg/AgvCommand \
  "{source: 'web', command: 'start_task', args: ['1号货架', '2号货架'], route: '1号', request_id: 't1'}"
```

---

## 6 控制台速查

### 6.1 终端控制台

```bash
python3 tools/agv_console.py             # 交互式，先选模式
python3 tools/agv_console.py check       # 一键自检（自己拉起业务节点）
python3 tools/agv_console.py maps        # 列出地图库
python3 tools/agv_console.py --mode nav  # 跳过模式选择
```

**模式选择**：`1` 建图　`2` 导航　`3` 只自检　`q` 退出

**建图模式**

| 键 | 作用 | 键 | 作用 |
|---|---|---|---|
| `k` | 键盘遥控（新终端，驱 `/cmd_vel_raw`） | `l` | 列出地图 |
| `m` | 保存地图（命名） | `d` | 删除地图（含其点位/路线/任务） |
| `n` | 切导航模式 | `q` | 退出 |

**导航模式 · 先加载地图**：`1` 加载地图　`2` 列出　`3` 删除　`m` 切回建图　`q` 退出

**导航模式 · 业务菜单**

| 键 | 作用 | 键 | 作用 |
|---|---|---|---|
| `1` | 存点位 | `s` | 状态快照 |
| `2` | 列出点位 | `n` | Nav2 lifecycle 状态 |
| `3` | 删除点位 | `l` | 任务栈日志尾 |
| `4` | 点名导航 | `c` | 一键自检 |
| `5` | 坐标导航 | `0` | 换地图（会重起整套栈） |
| `6` | 手动开小车 | `q` | 退出终端（只有此界面能退） |
| `7` | 路线管理 | `t` | 完整搬运 |
| `8` | 查询状态 | `u` / `d` | 顶升 / 下降 |
| `9` | 任务系统 | `x` / `r` | 急停 / 恢复 |

**层级规则**：进入子功能后按 `q` **只回主菜单**，不关终端；只有主菜单按 `q` 才退出
（有任务或缓冲非空会先确认）。

### 6.2 网页控制台

| 区域 | 操作 |
|---|---|
| 地图 | 单指拖动平移、双指缩放、单击取位置（存点位）、**长按 = 设导航目标** |
| 十字键 | 按住持续走，松手即停；速度档位可调 |
| 工作模式 | 启动导航 / 开始建图 / 停止 |
| 任务 | 添加、删、改路线、开始执行、中止 |
| 地图库 | 换图、删图、存当前地图 |
| 急停 | 顶栏红色按钮，任何状态可用 |

控制权：同一时刻只有一台设备能遥控（120 秒无操作释放），另一台显示"被占用"。

---

## 7 生命周期与清理

### 7.1 启动时自动清理残留

网关启动时会扫一遍上次没退干净的 ROS 进程（`gzserver`/`nav2_*`/`task_manager_node` 等），
先 SIGINT 给 6 秒、超时 SIGKILL，再清无主 SHM 段、等 11345 端口释放。
自己（及其祖先链）会被排除，不会自杀。

### 7.2 一键停所有进程

```bash
bash tools/stop_all.sh
```

顺序：SIGINT → 等 12 秒 → SIGKILL 兜底 → 按端口兜底，最后打印 8080~8100 是否释放。

★ **不要 `pkill -9 -f python3`**——会连编辑器/其它 python 一起杀。脚本里每个模式都限定到
具体脚本名或节点名。

★ **为什么要先 SIGINT**：launch / Gazebo / Nav2 靠 SIGINT 释放 DDS 端口、注销 `/dev/shm`、
优雅销毁 lifecycle 节点。直接 `-9` 会留僵尸节点和共享内存段，下次启动出现"同名节点互抢话题"，
看着像导航坏了。

### 7.3 端口还被占但进程已不在

典型场景：直接关掉终端窗口没按 Ctrl+C。

```bash
sudo ss -K 'sport = :8095'      # 强拆 socket
# 或注销重登桌面
```

### 7.4 换栈的五步清理（`StackRunner.stop()`）

① SIGINT 整组 → ② 等整组死透 → ③ 不死再 SIGKILL → ④ 清 `/dev/shm` 里**无主** `fastrtps_*` → ⑤ 等 11345 空出。

漏掉任何一步的后果：`gzserver` bind 失败是**静默**的（日志照打、世界里没数据）；
SHM 段残留会让新老节点互相发现不到（"节点全在、话题全在，彼此收不到"）。

---

## 8 故障排查

| 现象 | 原因 | 处理 |
|---|---|---|
| 服务一直 `waiting for service` | 节点没起，或没 source | `ros2 node list`；重开终端记得 `source install/setup.bash` |
| 节点"没起/无响应/查询超时" | 上次没退干净，多套栈互抢 | `bash tools/stop_all.sh`，再用网页/控制台启动（会自动清理） |
| `/scan` 全是 `inf` | 世界没加载，车在空世界 | 用带 `sim.launch.py` 的 launch；换世界用 `world_file:=/abs/xxx.world`（别改参数名） |
| 车不停 / 一直往前跑 | 上层停发了但底盘还在跑 | 仲裁 `cmd_timeout=0.5s` 会补零速；仍不停多半是残留遥控进程 `pkill -9 -f teleop_twist_keyboard` |
| 急停后导航全被拒 | 停在 `PAUSED` 未恢复 | 发一次 `resume`（控制台 `r`），不必重启 |
| Nav2 显示"未激活" | 仿真刚起，lifecycle 未激活完 | 等 15~40 s；仍不行看日志尾 |
| 换地图后点位/路线"不见了" | 点线跟地图绑定，换图就换文件 | 设计行为；用控制台 `0` 换图会自动切换 |
| 手动模式下导航被拒 | 互锁生效 | 手动结束会自动置回自动；手工验证发 `/agv/manual_mode false` |
| 点位贴墙/图外被拒 | `GoalValidator` 拦截 | 换可达点；阈值 `robot_clearance`（默认 0.30 m） |
| 导航加载地图后**全黑只剩起始点** | 中文地图名进了 map_server（`\uXXXX` 转义） | 已修：`MapManager.launch_map_yaml()` 自动生成 ASCII 别名。若仍黑，查 `maps/.launch/<sha1>/` 与自检里的「地图已加载」提示 |
| 导航模式按方向键**车一动不动** | 速度发到了 `/agv/web_teleop` 但没人订阅 | 已修：`task_manager_node` 订阅并转发。核对 `ros2 topic info /agv/web_teleop` 订阅数应为 1 |
| 建图模式遥控没反应 | 建图没有 task_manager，通道是 `/cmd_vel_raw` | 网关按 `stack_manager.teleop_topic_for()` 分流；手工发 `/cmd_vel_raw` |
| 发任务后日志"任务没能启动" | 受理与建单不同步 / 服务未就绪 | 已修（受理即建单）。若复现，看 `[指令] … 拒绝：` 原文，别猜 |
| 中止后既不能停也不能开新任务 | 中止落了 `ERROR`（硬拒绝态） | 已修：中止走 `_stop_quietly()` 回 IDLE |
| 日志满屏英文 | ROS/Nav2 原始输出 | `log_translate.py` 会过滤噪音并翻译已知句式，翻不出的保留原文 |
| 日志一直刷"连接断开" | WebSocket 短抖 | 断开满 5 秒才提示；重连间隔 2s→10s |

> ⚠️ **不要同时起两套仿真/导航栈**。同名节点会让话题、服务、action 互相抢，
> 看起来就像"导航坏了"。网页按模式隔离，一次只跑一套。

---

## 9 验收清单（改完代码后手过一遍）

无自动化测试，只能靠这几步。**每一步都要看输出，不要只看"没报错"**。

### 9.1 静态

```bash
python3 -m compileall -q src tools
for f in src/agv_web/agv_web/static/*.js; do node --check "$f" || echo "FAIL $f"; done
```

### 9.2 网页网关（不起栈）

```bash
ros2 launch agv_web web_console.launch.py token:=smoke manage_stack:=false port:=8099
```

另开终端逐条检查：

```bash
B=http://127.0.0.1:8099
curl -s -c /tmp/agv.cookie -X POST $B/api/login -H 'Content-Type: application/json' -d '{"token":"smoke"}'
curl -s -b /tmp/agv.cookie $B/api/state | head -c 300
curl -s -b /tmp/agv.cookie -o /dev/null -w '%{http_code}\n' $B/api/map.png     # 无栈应 404
curl -s -b /tmp/agv.cookie -X POST $B/api/store -H 'Content-Type: application/json' -d '{"resource":"waypoint","action":"list"}'   # 无栈应 503
curl -s -b /tmp/agv.cookie -o /dev/null -w '%{http_code}\n' $B/static/app.css  # 200
```

浏览器打开 `http://127.0.0.1:8099`：登录 → 地图区有网格 → 按方向键无 JS 报错 → 任务卡片能刷新。

### 9.3 完整栈

1. 网页点「启动导航」，选 `world666map`，等自检 8 项变绿（30~60 秒）
2. 发一个任务 `1号货架 → 2号货架`，看：接近取货点 → 顶升 → 运输 → 下降 → 完成
3. 点「中止」，确认按钮**立刻**变回「开始执行」，日志有停车
4. 点「开始建图」，等 20 秒，确认地图区在长、遥控能走
5. 切回「启动导航」，确认**没有黑屏**、没有 `Failed init_port`、没有 SHM 报错

任何一步不通过，先看网页日志面板的中文提示与 `[指令] … 拒绝：` 原文。
