#!/usr/bin/env bash
# ────────────────────────────────────────────────────────────────────────────
# 换栈复位专项验证：切导航/建图之后，网页上不能留重影、不能留旧小车位置
#
# 为什么单独一个脚本：
#   这两个 bug 只在"真的换了一套栈"时才出现，任何静态检查、单测都抓不到 ——
#   它们本质是跨进程缓存残留。所以必须真起网关 + 真连 WebSocket。
#
# 与 mapping_teleop.sh 的区别：
#   那个测"建图模式下遥控话题对不对"，这个测"换栈后显示状态有没有清干净"。
#
# 用法：bash tools/webtest/stack_reset.sh
# ────────────────────────────────────────────────────────────────────────────
WS=/home/yuki/agv/newros_car7
cd "$WS" || { echo "找不到工作目录 $WS"; exit 1; }

# ★ PATH 里必须是 /usr/bin 的 python3.10：3.13 没装 ROS 依赖，
#   而且 injector 要 import rclpy。
export PATH=/usr/bin:/bin:/usr/local/bin:$PATH
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || echo "（install/ 未编译，先 colcon build）"

# ★ 端口别用 232 以上的 domain：FastDDS 超过上限会直接退出（踩过）
export ROS_DOMAIN_ID=211
export no_proxy='*'
export NO_PROXY='*'
export PYTHONUNBUFFERED=1

# 先清掉可能残留的同名进程，避免两套网关抢同一个端口
pkill -f "tools/webtest/injector.py" 2>/dev/null
pkill -f "web_gateway" 2>/dev/null
sleep 1

/usr/bin/python3 tools/webtest/stack_reset.py
rc=$?
exit $rc
