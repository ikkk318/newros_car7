# agv_navigation —— 建图与导航（ROS 2 Humble + Nav2）

AGV 500×500 顶升搬运车的**建图 / 导航**配置包。纯配置文件（无源码），
配合 `agv_description`（仿真 + 机器人模型）使用。

> **平台**：ROS 2 Humble + Gazebo Classic 11（与 `agv_description` 一致）。
> Nav2 版本 1.1.x，参数已按 Humble 键名校正（见 `params/nav2_params.yaml` 头部说明）。

---

## 一、目录结构

```
agv_navigation/
├── launch/
│   ├── mapping.launch.py          ← 建图总启动：仿真 + slam_toolbox + RViz
│   └── navigation.launch.py       ← 导航总启动：仿真 + AMCL/Nav2 + RViz
├── params/
│   ├── slam_toolbox.yaml          ← slam_toolbox 建图参数（360° 全向雷达适配）
│   ├── nav2_params.yaml           ← Nav2 全套参数（AMCL/代价地图/DWB/行为）
│   └── collision_monitor.yaml     ← 碰撞监控参数
├── maps/
│   └── world666map/
│       ├── world666map.pgm        ← ★ 实测栅格地图（594×762 px @0.025）
│       ├── world666map.yaml       ← 地图描述（分辨率 0.025 / 原点 -10.7,-9.83）
│       ├── world666map_ref.pgm    ← 参考图（870×813）：make_world_map.py 生成，非导航用
│       ├── world666map_ref.yaml
│       ├── waypoints.yaml         ← 点位（5 个搬货点 + 起始点，kind: cargo）
│       └── routes.yaml            ← 路线（与本地图绑定）
├── rviz/
│   └── mapping.rviz               ← 建图 RViz（Fixed Frame=map）
└── package.xml / CMakeLists.txt
```

> 导航用的 RViz 配置直接用 nav2 自带的 `nav2_default_view.rviz`（含 Nav2 面板、
> 目标工具、代价地图），不再单独维护一份。

> **两份地图别搞混**：导航用的是 `world666map.pgm/.yaml`（控制台建图实测，594×762，
> 原点 −10.7/−9.83，用户已逐点验证，覆盖 x ∈ [−10.70, 4.125]、y ∈ [−9.83, 9.195]）；
> `world666map_ref.pgm/.yaml` 是脚本生成的参考图（870×813，原点 −11.304/−10.423），
> **尺寸与原点都不同，不能互相替换**。

---

## 二、编译

```bash
cd /home/yuki/new_ros_car
colcon build --packages-select agv_navigation
source install/setup.bash
```

---

## 三、建图（SLAM）

一条命令起「Gazebo 世界 + 本车 + slam_toolbox + RViz」：

```bash
ros2 launch agv_navigation mapping.launch.py
ros2 launch agv_navigation mapping.launch.py gui:=false rviz:=false   # 无头建图
ros2 launch agv_navigation mapping.launch.py world:=/abs/other.world  # 换世界
```

默认世界 = `agv_description/worlds/world666.world`（与 `navigation.launch.py` 的默认世界一致，
两份 launch 的默认值必须同源，否则地图与场景对不上）。

另开一终端遥控走一圈：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

雷达装在车体几何中心、固定 **360° 全向**（361 点，1°/点），建图与导航共用同一份扫描数据。
实测只给前向 180°/±90° 会让开阔处扫描退化、SLAM 航向被顶到搜索窗边缘，地图后期整体错位，
所以不要再限制视场角。

建完保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f src/agv_navigation/maps/world666map/world666map
# 保存后把 world666map.pgm/.yaml 放进 maps/world666map/，再 colcon build 装进 share
# （也可以直接用控制台建图模式的 [m] 命名保存，它会自动放好位置）
```

> 脚本 `tools/make_world_map.py` **不会覆盖**这份实测地图：它只写 `world666map_ref.pgm/.yaml`，
> 并刷新同目录的 `waypoints.yaml` / `routes.yaml`。

---

## 四、导航（AMCL + Nav2）

在已建好的地图上跑自主导航：

```bash
ros2 launch agv_navigation navigation.launch.py
ros2 launch agv_navigation navigation.launch.py map:=/abs/other.yaml
ros2 launch agv_navigation navigation.launch.py gui:=false rviz:=false   # 无头
```

默认组合：世界 `agv_description/worlds/world666.world` + 地图
`src/agv_navigation/maps/world666map/world666map.yaml`。

发导航目标（RViz 里用 **Nav2 面板**或 **2D Goal Pose** 工具点地图；或命令行）：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

> AMCL 标称初始位姿固定在 `(0, 0, 0)`（`nav2_params.yaml` 的 `initial_pose`），
> 与仿真出生点一致。**改出生点必须两处同步**，否则定位一开始就是偏的。

> 点位/路线不是本包提供的：它们由 `agv_waypoint` 的 `waypoint_node` 管理，文件就是
> 地图同目录下的 `waypoints.yaml` / `routes.yaml`（`agv_stack.launch.py` 会按所选地图注入路径）。
> 地图换了，点位与路线的坐标就失去意义，必须一起换。

---

## 五、和上一版（TurtleBot3 参数直搬）的关键差异

原 `nav2_params.yaml` / `slam_toolbox.yaml` 是从旧线 `~/agv` 的 TurtleBot3 配置
直接拷来的，含若干在 Humble 下**会报错或失效**的旧键。本次已修正并针对本车调参：

| # | 问题 | 修正 |
|---|---|---|
| 1 | 节点名 `recoveries_server`（Humble 已改名） | → `behavior_server`（旧名会让行为节点起不来） |
| 2 | 插件 `nav2_recoveries/Spin` 等 | → `nav2_behaviors/Spin` 等；`recovery_plugins` → `behavior_plugins` |
| 3 | `bt_navigator.default_bt_xml_filename`（已废弃） | 删除，改用内置默认行为树 |
| 4 | `plugin_lib_names` 缺 Humble 新节点 | 对齐官方全集（含 cancel / drive_on_heading / battery 等） |
| 5 | 缺 `velocity_smoother` / `smoother_server` 段 | 补齐（Humble bringup 会拉起这两个节点） |
| 6 | 机器人半径 0.22（TB3） | 改**方形足迹** 0.52×0.52（本车 500×500 + 余量），更准 |
| 7 | 速度上限 0.22（TB3 硬件限） | → 0.5 m/s（本车仿真限幅），加速度同步调为 1.0 / 2.0 |
| 8 | 雷达量程按 TB3 写（raytrace 3.0） | → 按本车 8 m 雷达重设（raytrace 7.5 / obstacle 7.0） |
| 9 | `amcl` 缺 `scan_topic` | 补 `scan` |

---

## 六、注意事项

- **雷达无盲区 + 后方保护**：雷达装在车体几何中心、360° 全向，前后左右都有数据
  （见 `agv_description`）。`velocity_arbiter` 倒车时改看车尾窄锥
  （`rear_half_angle_deg = 20°`），车尾太近会刹停/减速；前进时后方数据完全不参与。
  ★ 锥角不能开大：槽内两侧壁与车身平行、车心离壁仅 0.361 m，锥一放宽就会把侧壁
  读成"身后有障碍"，倒车被自己压住 → "能进不能出"复发（实测 ±20° 干净、±25° 就擦到）。
- **接管闸门另用窄锥**：择向接管单独用 `takeover_half_angle_deg = 20°` 判定，
  而近障减速仍用 45°。原因同上——前锥 45° 的边界会扫到平行槽侧壁（车心偏 0.111 m 时
  近壁仅 0.250 m，45° 射线命中 0.354 ≤ take 0.45），"前方很近"的假象骗开闸门，
  左右壁距一比又超过偏置阈值 → 车在槽内被反复夺向（实测进槽 36%~47% 采样中招）。
  收到 20° 后实测 0。**注意只收窄闸门**：槽内该减速慢行仍要减速，只是转向归导航。
- **进槽作业与出槽靠倒车**：槽内净宽 0.80 m（实测地图 0.72 m），车体半对角线 0.354 m，
  槽内掉头余量仅约 7 mm，会被膨胀层与栅格化吃掉 —— 所以**出槽只能倒车**，
  控制器必须允许倒车（`nav2_params.yaml` 的 DWB `min_vel_x = -0.20`）。
  这条不变量由 `src/agv_common/test/test_reverse_capability.py` 守着，
  它同时检查 `velocity_smoother` 下限、`behavior_server` 的 backup、
  以及局部代价地图必须把未知区当自由区（否则未探明区域会直接堵死倒车）。
- **到点对准不能被避障夺向**：到点后 Nav2 只发角速度修航向，而车在槽内前方恒为槽底挡板、
  恒满足避障接管条件。若此时仍让 `reactive_avoid` 接管角速度，对准动作会被换成固定
  0.5 rad/s 的左右摇摆，航向永远进不了 `yaw_goal_tolerance` → `goToPose` 不返回 →
  任务层死等到 `nav_timeout_s`（180 s）报错，现象是**"明明到了却以为没到、原地左右摇摆不停"**。
  因此 `reactive_avoid` 规定：`v_nav < 0`（倒车）与 `v_nav ≈ 0`（原地对准/停车）都不接管角速度，
  只有真正在往前挪（`v_nav > 0`）才择向。注意**调容差治不了这个病**：
  `SimpleGoalChecker` 的 `stateful: True` 一旦位置达标就改为只看航向。
  由 `test_reactive_avoid.py` 的三个回归用例守着。
- **地图与仿真世界**：`maps/world666map/world666map.pgm` 与
  `agv_description/worlds/world666.world` 是同一场地的两张表示（一张是建的图，一个是被仿真的世界）。
  换 world 记得重新建图。
- **world 出生点 = AMCL 初始位姿 = (0,0,0)**。
- 起仿真前先清进程，否则报 `Entity [agv_500] already exists`：
  `pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f spawn_entity`
