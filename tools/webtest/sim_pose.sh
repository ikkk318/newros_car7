#!/usr/bin/env bash
# ────────────────────────────────────────────────────────────────────────────
# 仿真时基位姿专项验证：上游 sim time + 网关 wall time 时，小车还显不显示
#
# 为什么单独一个脚本：
#   这个故障的根因是**两个进程的时钟基准不同**（Gazebo 从 0 计时，
#   网关用系统时间 1.79e9），算出来的"数据年龄"是 57 年。任何单测都抓不到，
#   因为测试桩里 clock 和 stamp 都来自 time.time()，时基天然一致 ——
#   必须在真进程里发真的 /clock 才能复现。
#
# 用法：bash tools/webtest/sim_pose.sh
# ────────────────────────────────────────────────────────────────────────────
WS=/home/yuki/agv/newros_car7
cd "$WS" || { echo "找不到工作目录 $WS"; exit 1; }

# ★ 必须 /usr/bin 的 python3.10：3.13 没装 rclpy
export PATH=/usr/bin:/bin:/usr/local/bin:$PATH
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || echo "（install/ 未编译，先 colcon build）"

export ROS_DOMAIN_ID=213
export no_proxy='*'
export NO_PROXY='*'
export PYTHONUNBUFFERED=1

pkill -f "tools/webtest/injector.py" 2>/dev/null
pkill -f "port:=8791" 2>/dev/null
sleep 1

/usr/bin/python3 tools/webtest/sim_pose.py
rc=$?
exit $rc
