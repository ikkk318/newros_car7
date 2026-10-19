# AGV 500×500 差速底盘 —— 仿真模型规格书

> **交付对象**：负责创建仿真模型的 AI / 工程师
> **本包内容**：模型参数表 + 可直接使用的 URDF/xacro（模块化） + Gazebo 插件 + launch + 测试世界 + RViz 配置
> **版本**：v3.3 ｜ 2026-10-11

---

## 一、模型概述

| 项目 | 内容 |
|---|---|
| 包名 | `agv_description` |
| 机器人名 | `agv_500` |
| 车型 | 四轮差速底盘（2 驱动轮 + 2 万向轮） |
| 车体 | 500 × 500 mm 正方形 |
| 建模方式 | URDF + xacro（**全参数化 + 模块化**，改尺寸只改变量） |
| 仿真平台 | **Gazebo Classic 11 + ROS 2 Humble**（本机环境） |
| 模型数据 | 与交付版一致（仅结构/环境适配，几何未变） |

> v3.0 参考 chapt8 `fishbot_description` 的模块化组织重构：urdf 按
> base / actuator / sensor / plugins 拆分，插件做成宏。

---

## 二、坐标系与 TF 树

### 2.1 坐标系（ROS REP-103 / REP-105）

```
X 轴 → 车体正前方
Y 轴 → 车体正左方
Z 轴 → 竖直向上
```

**原点定义**

| 坐标系 | 定义 |
|---|---|
| `base_footprint` | **两驱动轮轴中点在地面的投影**，z = 0 |
| `base_link` | 与 base_footprint 重合（fixed joint，identity） |
| `odom` | 由差速插件发布，`odom → base_footprint` |

> **为什么原点放这里**：差速车的旋转中心（ICR）就是驱动轴中点。把原点放这儿，
> 原地旋转时 `odom` 的 (0,0,0) 就是旋转中心，TF 最干净。

**旋转方向**：逆时针为正（绕 +Z）。

### 2.2 TF 树

```
odom
└── base_footprint            (差速插件发布)
    └── base_link             (fixed)
        ├── left_wheel        (continuous, axis = 0 1 0)
        ├── right_wheel       (continuous, axis = 0 1 0)
        ├── caster_front      (fixed)
        ├── caster_rear       (fixed)
        ├── laser             (fixed)
        └── imu_link          (fixed)
```

---

## 三、参数表

### 3.1 车体

| 参数 | xacro 变量 | 值 | 说明 |
|---|---|---|---|
| 车体长（X） | `body_L` | **0.500 m** | |
| 车体宽（Y） | `body_W` | **0.500 m** | |
| 底盘厚 | `body_H` | 0.105 m | 占位值 |
| 离地间隙 | `clearance` | 0.025 m | 主流差速底盘：电机在底盘内，轮子嵌入属正常 |
| 整备质量 | `body_mass` | 20.0 kg | 占位值 |
| 对角线 | — | **0.707 m** | 原地旋转扫掠直径 |

### 3.2 驱动轮

| 参数 | xacro 变量 | 值 | 说明 |
|---|---|---|---|
| 轮径 | `wheel_D` | 0.100 m | 占位值，可改 |
| 轮半径 | `wheel_R` | 0.050 m | |
| 轮宽 | `wheel_W` | 0.040 m | |
| **轮距（左右轮心距，沿 Y）** | `track` | **0.440 m** | |
| **驱动轴纵向位置（沿 X）** | `drive_x` | **0.000 m** | 居中 |
| 单轮质量 | `wheel_mass` | 0.5 kg | 占位值 |

> 两个驱动轮分别位于 **y = +0.220（左）** 和 **y = −0.220（右）**，**x 均为 0**。
> 也就是说两轮在同一根横向轴上 —— 这是差速驱动的前提。
> 轮子圆柱的轴沿 Y，所以 visual/collision 的 `rpy = 1.5708 0 0`。

### 3.3 万向轮

| 参数 | xacro 变量 | 值 | 说明 |
|---|---|---|---|
| 万向轮直径 | `caster_D` | 0.050 m | 球形，占位值 |
| 前万向轮位置 | `caster_fx` | +0.200 m | |
| 后万向轮位置 | `caster_rx` | −0.200 m | |
| 单轮质量 | `caster_mass` | 0.2 kg | |

> 两个万向轮都在**车体中线上（y = 0）**，一前一后。
> 加上左右两个驱动轮，四个接地点构成一个**菱形支撑多边形**：
> 纵向 0.40 m（前后万向轮）、横向 0.44 m（左右驱动轮）。
>
> **建模方式**：用**球体 + 极低摩擦（μ = 0.001）**模拟，关节为 fixed。
> 球体低摩擦后等效于「自由滑动」，不需要真的建模旋转。

### 3.4 激光雷达

| 参数 | xacro 变量 | 值 | 说明 |
|---|---|---|---|
| 外壳半径 | `laser_R` | 0.035 m | 占位值（按常见 2D 雷达） |
| 外壳高 | `laser_H` | 0.040 m | 占位值 |
| **纵向位置（沿 X）** | `laser_x` | **0.000 m** | 车体**几何中心**（y = 0，与 IMU 同轴） |
| **扫描面高度** | `laser_z` | **0.170 m** | 高于底盘顶面 0.130 m |
| 安装倾角 | `rpy` | **0 0 0** | 严格水平 |
| 安装偏航 | `yaw` | 0 | 零位角 |
| 质量 | `laser_mass` | 0.2 kg | |

> ⚠️ **关键**：`laser_joint` 的 `origin` 的 **z = 0.170 是扫描面（光学中心）高度**，
> **不是外壳底面**。真机选型时务必查数据手册的「扫描平面高度」，差 10 mm 仿真与真机就对不上。
>
> ⚠️ **水平位置必须齐平或悬出车体前缘**，**绝不能凹进车体**。

### 3.5 IMU

| 参数 | xacro 变量 | 值 | 说明 |
|---|---|---|---|
| 安装高度 | `imu_z` | 0.100 m | 车体几何中心 |
| 质量 | `imu_mass` | 0.05 kg | |

### 3.6 传感器仿真参数

| 参数 | 值 |
|---|---|
| 雷达扫描点数 | 361（1°/点，±π 首尾重合） |
| 雷达角度范围 | −π ~ +π（360° 全向） |
| 雷达量程 | 0.05 ~ 8.0 m |
| 测距分辨率 | 0.01 m |
| 雷达更新率 | 15 Hz |
| 雷达噪声 | 高斯，σ = 0.005 m |
| **雷达可视化** | **打开 `visualize`（可直接看到射线覆盖）** |
| IMU 更新率 | 100 Hz |
| 里程计发布率 | 50 Hz |

### 3.7 环境与摩擦

| 参数 | 值 |
|---|---|
| 地面摩擦 | μ = 0.6（干水泥地） |
| 驱动轮摩擦 | μ = 0.6 |
| 万向轮摩擦 | μ = 0.001（等效自由滑动） |
| 车体摩擦 | μ = 0.6（离地 25 mm，正常不接触） |

### 3.8 控制限幅

| 参数 | 值 |
|---|---|
| 最大线速度 | 1.0 m/s |
| 最大角速度 | 1.0 rad/s |
| 最大线加速度 | 1.0 m/s² |
| 最大角加速度 | 2.0 rad/s² |

### 3.9 展开后的数值（xacro 变量 → 实际坐标）

以 `base_footprint` 为原点，单位 m。**下游建模可直接抄这张表**。

| 连杆 | x | y | z | 备注 |
|---|---|---|---|---|
| `base_footprint` | 0 | 0 | 0 | 驱动轴中点在地面的投影 |
| `base_link` | 0 | 0 | 0 | 与 base_footprint 重合 |
| 车体包络（box 中心） | 0 | 0 | 0.0775 | 体积 0.500 × 0.500 × 0.105 |
| `left_wheel` | **0** | **+0.220** | 0.050 | 圆柱轴沿 Y |
| `right_wheel` | **0** | **−0.220** | 0.050 | 圆柱轴沿 Y |
| `caster_front` | **+0.200** | **0** | 0.025 | 球，r = 0.025 |
| `caster_rear` | **−0.200** | **0** | 0.025 | 球，r = 0.025 |
| `laser` | **0** | **0** | **0.170** | 车体几何中心、360° 全向 |
| `imu_link` | 0 | 0 | 0.100 | 车体几何中心 |

**由上式推出的关键尺寸**

| 量 | 值 | 算法 |
|---|---|---|
| 车体对角线（原地旋转扫掠直径） | **0.7071 m** | √(0.5² + 0.5²) |
| 原地旋转扫掠半径 | 0.3536 m | 对角线 / 2 |
| 支撑多边形（菱形）纵向半轴 | 0.200 m | 前/后万向轮 |
| 支撑多边形（菱形）横向半轴 | 0.220 m | 左/右驱动轮 |
| 轮子外缘到车侧的距离 | 10 mm 内缩 | 0.250 − (0.220 + 0.020) |
| 车体底面高度 | 0.025 m | = clearance |
| 车体顶面高度 | 0.130 m | = clearance + body_H |
| 雷达外壳 z 范围 | 0.150 ~ 0.190 | 0.170 ± 0.020 |
| 雷达外壳 x 范围 | −0.035 ~ +0.035 | 0.000 ± 0.035 → 完全落在车体内 |
| 雷达底面高出车体顶面 | 20 mm | 0.150 − 0.130 |

### 3.10 几何重叠说明（建模时须知）

车体是**包络盒**，不是实体结构，所以下列重叠是刻意的，Gazebo 里不会报错：

| 现象 | 数值 | 为什么仿真无碍 |
|---|---|---|
| 驱动轮与车体包络重叠 | 轮顶 z = 0.100 > 车体底 z = 0.025 | Gazebo 默认关闭**父子连杆**自碰撞（`self_collide = false`） |
| 万向轮与车体包络重叠 | 球顶 z = 0.050 > 车体底 z = 0.025 | 同上 |
| 雷达与车体包含关系 | 圆心在 x = 0.000（车体正中），外壳完全在车体内 | 雷达是 fixed 关节子连杆，不与 base_link 自碰撞 |

> 如果以后把车体换成真实结构（带轮罩/开槽），记得同步调整 `clearance` 和 `body_H`，
> 并**连带检查 `laser_z`**，否则雷达可能低于承载面而被自身结构挡住。

---

## 四、话题接口

> 话题接口（Gazebo Classic 11）：插件直接发 ROS 话题，不需要话题桥。

| 话题 | 类型 | 方向 | 频率 | 来源 |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gazebo | — | 差速插件订阅 |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS | 50 Hz | 差速插件发布 |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo → ROS | 50 Hz | 差速插件（`odom → base_footprint`） |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → ROS | 50 Hz | 关节状态插件（左右轮角度，供 rsp 补全轮子 TF） |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo → ROS | 15 Hz | 雷达（360° 全向、361 点） |
| `/imu` | `sensor_msgs/msg/Imu` | Gazebo → ROS | 100 Hz | IMU |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS | — | Gazebo（`use_sim_time` 必需） |

---

## 五、文件结构

```
agv_description/                               ← 已是一个完整的 ROS 2 ament_cmake 包
├── README.md                          ← 本文件（规格书）
├── package.xml                        ← 依赖声明
├── CMakeLists.txt                     ← 只做 install(DIRECTORY ...)
├── urdf/
│   ├── agv_500.urdf.xacro             ← 主文件：可调参数区 + 模块拼装
│   ├── common_inertia.xacro           ← 惯性宏库（长方体/圆柱/球体）
│   ├── base.urdf.xacro                ← 车体（base_footprint + base_link）
│   ├── actuator/
│   │   ├── wheel.urdf.xacro           ← 驱动轮宏（左/右各实例化一次）
│   │   └── caster.urdf.xacro          ← 万向轮宏（前/后各实例化一次）
│   ├── sensor/
│   │   ├── laser.urdf.xacro           ← 激光雷达
│   │   └── imu.urdf.xacro             ← IMU
│   └── plugins/
│       └── gazebo_classic.xacro       ← Gazebo Classic 11 插件
├── launch/
│   ├── display.launch.py              ← 只看模型（RViz，无物理）
│   └── sim.launch.py                  ← Gazebo Classic 11 仿真
├── rviz/
│   ├── agv_500.rviz                   ← RViz 预设：RobotModel + TF + LaserScan（仿真用，Fixed Frame=odom）
│   └── agv_500_display.rviz           ← RViz 预设：只看模型（Fixed Frame=base_footprint，无 odom）
├── worlds/
│   └── agv_test.sdf                   ← 测试世界（SDF 1.7）
└── config/
    ├── odom_calib_sim.yaml            ← 里程计参数（仿真）
    └── odom_calib_real.yaml           ← 里程计参数（实测占位）
```

**无需 `ros2 pkg create`** —— `package.xml` 与 `CMakeLists.txt` 已随包提供，直接编译即可。

---

## 六、安装与运行

### 6.1 环境要求

| 项 | 要求 |
|---|---|
| ROS 2 | **Humble**（配 Gazebo Classic 11，本机已验证） |
| Gazebo | **Classic 11**（`ros-humble-gazebo-ros-pkgs`） |
| 系统 | Ubuntu 22.04 |
| GPU | 有独显最好；纯软件渲染也能跑，但雷达刷新率会掉 |

```bash
# Humble + Gazebo Classic 11
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-xacro \
     ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui
```

### 6.2 编译

```bash
# 本工作区：/home/yuki/new_ros_car（标准 src/ 布局），包在 src/agv_description
cd /home/yuki/new_ros_car
colcon build --packages-select agv_description
source install/setup.bash
```

### 6.3 运行

```bash
# ① 只看模型（不起物理，最先跑这个确认几何对不对）
ros2 launch agv_description display.launch.py

# ② 起 Gazebo 仿真（Classic 11）
ros2 launch agv_description sim.launch.py
ros2 launch agv_description sim.launch.py use_rviz:=false       # 不要 RViz
ros2 launch agv_description sim.launch.py gui:=false            # 无头（不起 Gazebo 窗口）
ros2 launch agv_description sim.launch.py world:=/abs/xxx.sdf   # 换世界

# ③ 只想看展开后的 URDF（不启动仿真）
xacro urdf/agv_500.urdf.xacro
```

### 6.4 发速度测试

```bash
# 走直线 0.3 m/s
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}, angular: {z: 0.0}}"

# 原地旋转 0.5 rad/s
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.5}}"

# 查看话题
ros2 topic list
ros2 topic echo /odom --once
ros2 topic hz /scan
```

---

## 七、模型自检

### 7.1 几何自检（跑 `display.launch.py` 时逐条看）

按这个顺序核对，30 秒就能发现 90% 的建模错误：

| # | 看什么 | 正确表现 |
|---|---|---|
| 1 | 两个驱动轮 | 在**左右两侧**，同一根横向轴上（沿 Y 分开），不是一前一后 |
| 2 | 两个万向轮 | 在**中线上**一前一后（沿 X 分开），球体，贴地 |
| 3 | 雷达 | 在车体**几何中心正上方**（与 IMU 同轴），360° 全向 |
| 4 | 雷达高度 | 高于车体顶面（0.170 > 0.130），不被车体遮挡 |
| 5 | 原点 | 在**两个驱动轮的连线上**，地面高度（z = 0） |
| 6 | 地面接触 | 4 个轮子的最低点都在 z = 0 平面上，没有悬空或陷地 |
| 7 | TF 树 | `odom → base_footprint → base_link → {left_wheel, right_wheel, caster_front, caster_rear, laser, imu_link}` |

### 7.2 运动学自检

| # | 测试项 | 方法 | 通过标准 |
|---|---|---|---|
| 1 | 走直线 | 发 `/cmd_vel` 线速度 0.3 m/s，跑 10 s | 前进 3 m，横向偏差 < 50 mm |
| 2 | 原地旋转 | 发角速度 0.5 rad/s，转满 2π | 角度误差 < 5° |
| 3 | 转弯半径 | 线速度 0.3 m/s + 角速度 0.3 rad/s | 半径 1 m，轨迹圆度误差 < 100 mm |
| 4 | 里程计精度 | 走 5 m 直线后回原点 | 累计位置误差 < 2% |

### 7.3 雷达扫描面自检

- 车从原点沿 +X 直行，观察雷达 `visualize`
- 预期：全向 360° 无盲区，前后左右都能看到墙
- 注意 2D 雷达的固有特性 —— **低于扫描面 170 mm 的障碍物在雷达里不存在**

---

## 八、建模注意事项（坑）

| # | 坑 | 说明 |
|---|---|---|
| 1 | **X / Y 轴写反** | 本车**纵向（前后）一律沿 X，横向（左右）沿 Y**。最容易犯的错：把 `track` 塞进 X 槽位，结果两个驱动轮变成前后排列、车转不动。本包已按正确轴向写好，改参数时不要动槽位 |
| 2 | **雷达 origin 的 z** | 必须是扫描面高度 0.170，不是外壳底面。这是最常见的错误 |
| 3 | **万向轮摩擦** | 摩擦设大（比如 0.6）会让车转不动或画圈。必须设 μ ≈ 0.001 |
| 4 | **圆柱轮子的 rpy** | URDF 圆柱默认轴是 Z，轮子要 `rpy="1.5708 0 0"` 把轴转到 Y |
| 5 | **轮子 joint 的 axis** | 必须是 `xyz="0 1 0"`，否则差速插件推不动 |
| 6 | **差速插件的轮距/轮径单位** | 是**米**，不是毫米。0.44 / 0.05（Classic 用 `wheel_diameter` = 0.100），不是 440 / 100 |
| 7 | **`frame_id` / `child_frame_id`** | 必须是 `odom` / `base_footprint`，和 TF 树一致 |
| 8 | **`Command` 参数类型** | launch 里 `ParameterValue(Command([...]), value_type=str)`，不加 `value_type=str` 会被当 float 解析而报错 |
| 9 | **仿真时间** | launch 里 rsp / rviz 都设了 `use_sim_time: True`。时钟由 gazebo_ros_init 插件发布（gzserver 自动加载），缺了 TF 就会跳 |
| 10 | **XML 注释里不能有 `--`** | 连续两个短横线在 XML 注释里非法，会让 xacro 直接解析失败。分隔线请用 `====` |
| 11 | **轮子 TF 缺一段** | Classic 的差速插件不发布轮子关节状态，必须另挂 `libgazebo_ros_joint_state_publisher.so`（本包已配），否则 `base_link -> left/right_wheel` 不出现在 TF 里 |
| 12 | **重起仿真前先杀干净** | 上次的 gzserver 没退干净时，新实例会报 `Entity [agv_500] already exists`。先 `pkill -9 -f gzserver` 再起 |
| 13 | **world 的 SDF 版本** | Classic 11（sdformat9）最高只认 SDF **1.7**。本包 worlds/agv_test.sdf 用 1.7 |

### 8.1 硬性要求

1. **全部参数用 xacro 变量** —— 改尺寸只改变量，不改实体。
2. **里程计标定参数独立成配置文件**（轮径、每圈脉冲数），见 `config/`。
3. **不要改已定项**：车体 500×500、轮距 440、驱动轴居中、雷达位姿 **(x=0.000, y=0, z=0.170)**。
4. **雷达全向 360°**：`laser_scan_min/max` = ±π、361 点（1°/点），装在车体几何中心，
   四周无自身结构遮挡，整圈射线都有效。不要改回前向 ±90°：实测那样会让开阔处
   可见点退化、SLAM 航向被顶到角度搜索窗边缘，地图后期整体错位/变形。

---

## 九、修订记录

| 版本 | 日期 | 变更 |
|---|---|---|
| v1.0 | 2026-10-10 | 首版：参数表 + URDF + Gazebo 插件 + launch |
| v1.1 | 2026-10-10 | 修正 URDF 中 X/Y 轴错位；纵向参数改名 `drive_x` / `caster_fx` / `caster_rx` / `laser_x`；修正 XML 注释中的非法 `--`；补齐 `package.xml`、`CMakeLists.txt`、`rviz/`、`worlds/`、`config/` |
| v2.0 | 2026-10-10 | 重新整理为**纯仿真模型规格书**，移除方案性内容；新增 §2.2 TF 树、§3.5 IMU 参数、§3.10 几何重叠说明、§四 话题接口 |
| v3.0 | 2026-10-10 | **模块化重构 + 切 Gazebo Classic 11 路径**（参考 chapt8 fishbot_description）：urdf 拆为 base / actuator / sensor / plugins；两个后端的插件改宏 + `sim:=` 参数切换；`sim.launch.py` 改用 gazebo_ros（spawn_entity，无 bridge）；新增 `sim_gz.launch.py` 备选；world 转 SDF 1.7（新增 gz 版原文件）；补 joint_state_publisher 插件使 TF 树完整；补 `agv_500_display.rviz`（无 odom 的只看模型配置）。**模型数值零改动（SDF 语义级 diff 已验证）** |
| v3.1 | 2026-10-10 | **按真机调整几何**：`clearance` 25→**80 mm**（车轮大部分外露，不再深埋车体）；激光雷达改为**低位前挑安装**（扫描面 170→**60 mm**，装在底盘底面之下、车体前缘外挑 85 mm，符合规格书 §6.1「雷达低于货架底面、距地 5~10cm」）；万向轮补**竖直支腿**（把轮球连到抬高后的底盘）；测试世界障碍物按新扫描面重新标定为 15/40/55/90 mm。自遮挡实测：正前方 ±15° 干净，自遮挡集中在后方（驱动轮 ±145°、前万向轮支腿 180°），符合真机低位雷达特性 |
| v3.2 | 2026-10-10 | **按用户意见回退几何**：确认「电机在底盘内、车轮嵌入、底盘低」是主流差速底盘常态，故 `clearance` 80→**25 mm**、撤掉万向轮支腿、撤掉雷达前挑支架、世界障碍物 15/40/55/90→**15/80/140/300 mm** 全部复原为交付版。雷达确定为**车体前立面正中安装**（x=0.250, z=0.170）。**几何与交付版完全一致** |
| v3.3 | 2026-10-11 | **删除 Jazzy / Gazebo Sim Harmonic 备选路径**（本机跑不了，仅保留 Humble + Classic 唯一路径）：删除 `urdf/plugins/gazebo_sim.xacro`、`launch/sim_gz.launch.py`、`worlds/agv_test_gz.sdf`；移除 xacro 的 `sim:=classic|gz` 切换（只留 classic 插件宏）；package.xml 移除注释掉的 `ros_gz_sim`/`ros_gz_bridge`；README/launch README/urdf README 同步去掉备选路径说明 |
| v3.4 | 2026-10-11 | **雷达改车体几何中心 + 全向 360°**：`laser_x` 0.250→**0.000**（原先装前立面，车体自身挡掉后方）；扫描角固定 **−π ~ +π、361 点**（1°/点），移除可配的 `laser_fov_deg` 及其在 `sim.launch.py` / `mapping.launch.py` 的透传；删除 `config/laser_filters_front.yaml`（原用于 ROS 侧屏蔽后方，已无意义）；同步修正 README 传感器参数表（15 Hz / σ=0.005） |

---

*本规格书由 WorkBuddy AI 生成，位于工作区，不属于项目目录（`D:/123pan/esp32ros_car`）。*
