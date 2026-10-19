urdf/
├── agv_500.urdf.xacro      ← 【总装说明书】唯一入口
├── common_inertia.xacro    ← 工具库（算惯量用的宏）
├── base.urdf.xacro         ← 积木块：车体
├── actuator/               ← 积木块：会动的部分
│   ├── wheel.urdf.xacro        驱动轮（左/右各用一次）
│   └── caster.urdf.xacro       万向轮（前/后各用一次）
├── sensor/                 ← 积木块：传感器
│   ├── laser.urdf.xacro        激光雷达
│   └── imu.urdf.xacro          IMU
└── plugins/                ← 仿真器专用（不属于真机）
    └── gazebo_classic.xacro    Gazebo Classic 11 插件
