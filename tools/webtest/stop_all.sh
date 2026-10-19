#!/usr/bin/env bash
# ============================================================================
#  一键关闭所有 AGV 后台进程
#
#  用法（你自己的终端里跑，不需要 source）：
#      cd /home/yuki/agv/newros_car7
#      bash tools/webtest/stop_all.sh
#
#  为什么要先 SIGINT 再 SIGKILL：
#      ros2 launch / Gazebo / Nav2 都需要 SIGINT 才能干净退出（释放 DDS 端口、
#      注销 /dev/shm、删掉 lifecycle 节点）。直接 -9 会留下僵尸节点和共享内存
#      段，下次启动就会出现"同名节点互抢话题"的怪现象——看着像导航坏了。
#      所以这里给 12 秒优雅退出窗口，超时才强杀。
#
#  ★ 不要用 pkill -f python3 这类宽泛模式：会把 WorkBuddy 本体和其它 python
#    进程一起杀掉。下面每个模式都限定到具体脚本名或节点名。
# ============================================================================

WS=/home/yuki/agv/newros_car7

echo "════════════════════════════════════════════════════════════════"
echo " 关闭 AGV 后台进程"
echo "════════════════════════════════════════════════════════════════"

# ── 第 0 步：先看看现在有什么 ──────────────────────────────────────────────
echo
echo "【关闭前】相关进程："
FOUND=0
for pat in web_gateway web_console injector.py waypoint_node task_manager_node \
           velocity_arbiter robot_state_publisher slam_toolbox map_server amcl \
           controller_server planner_server bt_navigator gzserver gzclient \
           teleop_twist_keyboard; do
  pids=$(pgrep -f "$pat" 2>/dev/null | tr '\n' ' ')
  if [ -n "$pids" ]; then
    echo "   $pat -> $pids"
    FOUND=$((FOUND+1))
  fi
done
[ "$FOUND" -eq 0 ] && echo "   （没有匹配到进程）"

echo
echo "【关闭前】占用的端口（8080~8100）："
PORTS_BEFORE=$(ss -lnt 2>/dev/null | awk '{print $4}' | grep -oE ':(80[89][0-9]|8100)$' | tr -d ':' | sort -u | tr '\n' ' ')
echo "   ${PORTS_BEFORE:-（无）}"

# ── 第 1 步：SIGINT 优雅退出 ───────────────────────────────────────────────
echo
echo "【1/4】发送 SIGINT（优雅退出，给 12 秒）…"
for pat in "web_console.launch.py" "agv_stack.launch.py" "navigation.launch.py" \
           "slam.launch.py" "web_gateway" "injector.py"; do
  pids=$(pgrep -f "$pat" 2>/dev/null)
  if [ -n "$pids" ]; then
    echo "   SIGINT $pat ($(echo $pids | tr '\n' ' '))"
    kill -INT $pids 2>/dev/null
  fi
done

for i in $(seq 1 12); do
  sleep 1
  left=$(pgrep -f "web_console.launch.py|agv_stack.launch.py|web_gateway|injector.py" 2>/dev/null | wc -l)
  [ "$left" -eq 0 ] && { echo "   ✓ ${i} 秒内全部退出"; break; }
  [ "$i" -eq 12 ] && echo "   ⚠ 12 秒后仍有 $left 个，进入强杀"
done

# ── 第 2 步：SIGKILL 兜底 ──────────────────────────────────────────────────
echo
echo "【2/4】强制清理残留"
for pat in web_gateway web_console injector.py waypoint_node task_manager_node \
           velocity_arbiter robot_state_publisher slam_toolbox map_server amcl \
           controller_server planner_server bt_navigator gzserver gzclient \
           teleop_twist_keyboard; do
  pids=$(pgrep -f "$pat" 2>/dev/null)
  if [ -n "$pids" ]; then
    echo "   SIGKILL $pat"
    kill -9 $pids 2>/dev/null
  fi
done
sleep 1

# ── 第 3 步：按端口兜底（进程名对不上时用这个） ────────────────────────────
echo
echo "【3/4】按端口兜底（8095-8099 是历史测试遗留）"
for port in $(seq 8080 8100); do
  # ss 能给出 pid 就杀；给不出说明进程在别的 PID 命名空间，只能提示
  pid=$(ss -lntp 2>/dev/null | grep ":$port " | grep -oE 'pid=[0-9]+' | head -1 | cut -d= -f2)
  if [ -n "$pid" ]; then
    echo "   端口 $port 被 PID $pid 占用 → kill -9"
    kill -9 "$pid" 2>/dev/null
  fi
done
sleep 1

# ── 第 4 步：结果校验 ──────────────────────────────────────────────────────
echo
echo "【4/4】结果"
STILL=$(ss -lnt 2>/dev/null | awk '{print $4}' | grep -oE ':(80[89][0-9]|8100)$' | tr -d ':' | sort -u | tr '\n' ' ')
if [ -z "$STILL" ]; then
  echo "   ✅ 8080~8100 端口已全部释放"
else
  echo "   ⚠ 以下端口仍在监听：$STILL"
  echo
  echo "   这些端口的进程在别的 PID 命名空间（我这边的 shell 看不到它），"
  echo "   常见于上个终端会话直接关窗口、没走 Ctrl+C。请在你的终端里手动执行："
  echo
  echo "       sudo ss -K 'sport = :8095'"
  echo "       sudo ss -K 'sport = :8096'"
  echo "       （8097 8098 8099 同理）"
  echo
  echo "   或者直接注销重新登录桌面，这些 socket 会随会话一起消失。"
  echo "   注意：换端口启动也可以，不影响功能。"
fi

LEFT=$(pgrep -f "web_gateway|injector.py|web_console.launch.py|gzserver" 2>/dev/null | wc -l)
[ "$LEFT" -eq 0 ] && echo "   ✅ 没有残留 ROS / 网关进程" || echo "   ⚠ 还有 $LEFT 个残留进程"

echo
echo "════════════════════════════════════════════════════════════════"
echo " 现在可以重启了："
echo
echo "   cd $WS"
echo "   source install/setup.bash"
echo "   ros2 launch agv_web web_console.launch.py token:=agv2024 port:=8080"
echo
echo "════════════════════════════════════════════════════════════════"
