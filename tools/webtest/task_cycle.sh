#!/usr/bin/env bash
# 任务系统闭环验证：开始执行 → 取消 → 再启动
#
# ★ 为什么要单独一个脚本：这个 bug 的本质是"跨进程时序 + 状态事件"，
#   单测只能验证代码结构（有没有调用 stop_tasks、有没有 publish_state），
#   验证不了"点了中止之后界面真的能再开新任务"。必须真起 task_manager。
#
# 用法：bash tools/webtest/task_cycle.sh
set -o pipefail
cd "$(dirname "$0")/../.." || exit 1

export PATH=/usr/bin:/bin:/usr/local/bin:$PATH
export no_proxy='*' NO_PROXY='*'
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-231}
export AGV_NO_CLEANUP=1
PORT=${PORT:-8091}
TOKEN=agv2024

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

LOG=tools/webtest/task_cycle.log
: > "$LOG"

cleanup() {
  for p in $GW $TM; do
    [ -n "$p" ] && kill -INT "$p" 2>/dev/null
  done
  sleep 2
  for p in $GW $TM; do
    [ -n "$p" ] && kill -9 "$p" 2>/dev/null
  done
}
trap cleanup EXIT

echo "── 启动 task_manager（真正的状态机）──"
ros2 run agv_task task_manager_node >> "$LOG" 2>&1 &
TM=$!
sleep 3

echo "── 启动网页网关 ──"
ros2 launch agv_web web_console.launch.py token:=$TOKEN port:=$PORT \
  manage_stack:=false >> "$LOG" 2>&1 &
GW=$!

for _ in $(seq 1 30); do
  code=$(curl -s --noproxy '*' -o /dev/null -w '%{http_code}' -m 2 \
    "http://127.0.0.1:$PORT/?token=$TOKEN" 2>/dev/null)
  [ "$code" = "200" ] && break
  sleep 1
done

B="http://127.0.0.1:$PORT"
q() { curl -s --noproxy '*' -m 5 "$B$1&token=$TOKEN"; }
p() { curl -s --noproxy '*' -m 8 -X POST -H 'Content-Type: application/json' \
        -d "$2" "$B$1?token=$TOKEN"; }

pass=0; fail=0
chk() {  # chk 描述 期望 实际
  if [ "$2" = "$3" ]; then
    echo "  ✅ $1"; pass=$((pass+1))
  else
    echo "  ❌ $1 —— 期望[$2] 实际[$3]"; fail=$((fail+1))
  fi
}

echo
echo "【1】任务列表（以前 POST 到这里必 400 → 界面永远'执行中'）"
r=$(p '/api/tasks' '{"action":"list"}')
echo "   $r" | head -c 200; echo
echo "$r" | grep -q '"ok": *true' && chk "POST list 返回 ok" "y" "y" \
  || chk "POST list 返回 ok" "y" "n"
run0=$(echo "$r" | /usr/bin/python3 -c 'import sys,json;print(json.load(sys.stdin).get("running"))')
chk "初始 running=false" "False" "$run0"

echo
echo "【2】启动任务"
r=$(p '/api/tasks' '{"action":"start"}')
echo "   $r" | head -c 200; echo
run1=$(p '/api/tasks' '{"action":"list"}' | /usr/bin/python3 -c 'import sys,json;print(json.load(sys.stdin).get("running"))')
chk "启动后 running=true" "True" "$run1"

echo
echo "【3】中止任务（关键：以前只发裸命令，事件不置位 → 卡 300 秒）"
T0=$(date +%s)
r=$(p '/api/tasks' '{"action":"stop"}')
echo "   $r" | head -c 200; echo
# 立刻查，不等定时器 —— 用户点了按钮要立即看到变化
run2=$(p '/api/tasks' '{"action":"list"}' | /usr/bin/python3 -c 'import sys,json;print(json.load(sys.stdin).get("running"))')
chk "中止后立刻 running=false" "False" "$run2"
echo "   耗时 $(( $(date +%s) - T0 )) 秒"

echo
echo "【4】中止后能再启动（用户报的'不能启动新任务'）"
r=$(p '/api/tasks' '{"action":"start"}')
echo "   $r" | head -c 200; echo
echo "$r" | grep -q '"ok": *true' && chk "能再次启动" "y" "y" || chk "能再次启动" "y" "n"
run3=$(p '/api/tasks' '{"action":"list"}' | /usr/bin/python3 -c 'import sys,json;print(json.load(sys.stdin).get("running"))')
chk "再次 running=true" "True" "$run3"

echo
echo "【5】经 /api/command 取消（急停按钮那条路径）"
r=$(p '/api/command' '{"command":"cancel_task"}')
echo "   $r" | head -c 200; echo
run4=$(p '/api/tasks' '{"action":"list"}' | /usr/bin/python3 -c 'import sys,json;print(json.load(sys.stdin).get("running"))')
chk "cancel_task 也清 running" "False" "$run4"

echo
echo "【6】task_manager 是否真的发布了 IDLE（取消后状态要回落）"
st=$(q '/api/state?' | /usr/bin/python3 -c '
import sys,json
d=json.load(sys.stdin); t=d.get("task") or {}
print(t.get("state","None"))')
chk "取消后状态回 IDLE" "IDLE" "$st"

echo
echo "【7】日志里不该有英文噪音"
noise=$(q '/api/logs?' | /usr/bin/python3 -c '
import sys,json,re
d=json.load(sys.stdin)
bad=[l for l in d.get("logs",[])
     if re.search(r"Default logging verbosity|Creating bond timer|process started with pid",
                  l.get("text",""))]
print(len(bad))')
chk "噪音行已过滤" "0" "$noise"

echo
echo "────────────────────────────────────────"
echo "  通过 $pass 项，失败 $fail 项"
[ "$fail" = "0" ] && echo "  ✅ 任务闭环正常" || echo "  ❌ 有问题，看 $LOG"
