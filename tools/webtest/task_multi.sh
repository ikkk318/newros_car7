#!/usr/bin/env bash
# 多任务连续执行端到端验证（真起 task_manager + 网关 + 假 Nav2）
#
# ★ 为什么必须单独一个脚本：
#   用户报的 bug 是"放多个任务，只执行第一个，后面的得不到执行"。
#   它的根因是**跨进程时序**（1 Hz 状态发布 + 上一单 DONE 残帧 +
#   task_manager 收尾期间会拒绝新指令），单测只能验证代码结构对不对，
#   验证不了"真跑三单，车真的动三次"。
#
#   这里用 tools/webtest/fake_nav.py 顶替 Nav2（/navigate_to_pose +
#   /follow_path 的最小子集），让 task_manager 能真的走完整条流程，
#   并且不需要 10MB /dev/shm 起 Gazebo（沙箱里起不来）。
#
# 用法：bash tools/webtest/task_multi.sh [任务数]
set -o pipefail
cd "$(dirname "$0")/../.." || exit 1

export PATH=/usr/bin:/bin:/usr/local/bin:$PATH
export no_proxy='*' NO_PROXY='*'
# ★ 域号上限是 232（233 起 FastDDS 会报 "Calculated port number is too
#   high" 然后直接退出）。选 211 是为了避开 run_all.sh 常用的 189/191 区间。
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-211}
export AGV_NO_CLEANUP=1          # 别让网关的启动清理把本脚本起的进程杀了
PORT=${PORT:-8097}
TOKEN=agv2024
N=${1:-3}
LOG=tools/webtest/task_multi.log
: > "$LOG"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

cleanup() {
  for p in $GW $TM $WP $LF $FN; do
    [ -n "$p" ] && kill -INT "$p" 2>/dev/null
  done
  sleep 1
  for p in $GW $TM $WP $LF $FN; do
    [ -n "$p" ] && kill -9 "$p" 2>/dev/null
  done
}
trap cleanup EXIT

echo "── 1/6 假 Nav2（顶替真正的 navigate_to_pose action）──"
/usr/bin/python3 tools/webtest/fake_nav.py >> "$LOG" 2>&1 &
FN=$!
sleep 2

echo "── 2/6 waypoint_node（取货点/放货点要能被查到）──"
# ★ 少这一步，task_manager 会把"取货点不存在"当成任务失败，整条链跑不起来。
#   用真实地图目录里的 points/routes，和真机用同一份数据。
ros2 run agv_waypoint waypoint_node --ros-args \
  -p waypoint_file:="$(pwd)/maps/world666map/waypoints.yaml" \
  -p routes_file:="$(pwd)/maps/world666map/routes.yaml" >> "$LOG" 2>&1 &
WP=$!
sleep 3

echo "── 3/6 搬货服务（顶升/下降；现阶段只维护 busy 标志，不驱动硬件）──"
ros2 run agv_lift cargo_node >> "$LOG" 2>&1 &
LF=$!
sleep 2

echo "── 4/6 task_manager（真状态机 + 真流程）──"
ros2 run agv_task task_manager_node >> "$LOG" 2>&1 &
TM=$!
sleep 3

echo "── 5/6 网页网关（manage_stack=false，只管代理）──"
ros2 launch agv_web web_console.launch.py token:=$TOKEN port:=$PORT \
  manage_stack:=false >> "$LOG" 2>&1 &
GW=$!

ok=0
for _ in $(seq 1 40); do
  code=$(curl -s --noproxy '*' -o /dev/null -w '%{http_code}' -m 2 \
    "http://127.0.0.1:$PORT/?token=$TOKEN" 2>/dev/null)
  [ "$code" = "200" ] && { ok=1; break; }
  sleep 1
done
if [ "$ok" != "1" ]; then
  echo "❌ 网关没起来，看 $LOG"
  tail -30 "$LOG"
  exit 1
fi

B="http://127.0.0.1:$PORT"
q() { curl -s --noproxy '*' -m 6 "$B$1&token=$TOKEN"; }
p() { curl -s --noproxy '*' -m 10 -X POST -H 'Content-Type: application/json' \
        -d "$2" "$B$1?token=$TOKEN"; }
jget() { /usr/bin/python3 -c "import sys,json;d=json.load(sys.stdin);print($1)"; }

pass=0; fail=0
chk() {
  if [ "$2" = "$3" ]; then echo "  ✅ $1"; pass=$((pass+1));
  else echo "  ❌ $1 —— 期望[$2] 实际[$3]"; fail=$((fail+1)); fi
}

echo
echo "── 6/6 验证 ──"

echo
echo "【1】准备 $N 条任务（前两条带路线）"
p '/api/tasks' '{"action":"clear"}' > /dev/null
# 路线要真实存在：从真实地图库的 routes.yaml 里取一条
ROUTE=$(/usr/bin/python3 - <<'PY'
import sys
sys.path.insert(0, 'src/agv_waypoint')
from agv_waypoint.route_store import RouteStore
r = RouteStore('maps/world666map/routes.yaml')
names = r.list_names()
print(names[0] if names else '')
PY
)
echo "   使用路线：${ROUTE:-(无)}"
for i in $(seq 1 "$N"); do
  rt=""
  [ "$i" -le 2 ] && rt="$ROUTE"
  r=$(p '/api/tasks' "{\"action\":\"add\",\"pick\":\"1号槽内\",\"drop\":\"1号货架\",\"loops\":1,\"route\":\"$rt\"}")
  cnt=$(echo "$r" | jget 'len(d.get("tasks",[]))')
  chk "第 $i 条已加入（累计 $cnt）" "$i" "$cnt"
done

echo
echo "【2】核对落盘的 route 字段（网页能存、能读回来）"
tasks=$(p '/api/tasks' '{"action":"list"}' | jget 'json.dumps(d.get("tasks"),ensure_ascii=False)')
echo "   $tasks"
rr=$(p '/api/tasks' '{"action":"list"}' | /usr/bin/python3 -c '
import sys,json
ts=json.load(sys.stdin).get("tasks",[])
print(",".join(t.get("route","") or "-" for t in ts))')
echo "   各单路线：$rr"
first=$(echo "$rr" | cut -d, -f1)
chk "第 1 条带路线" "${ROUTE:--}" "$first"

echo
echo "【3】开始执行（关键：以前只有第 1 单真跑）"
r=$(p '/api/tasks' '{"action":"start"}')
echo "   $r" | head -c 200; echo
chk "启动受理" "True" "$(echo "$r" | jget 'd.get("ok")')"

echo "   等任务跑完（最多 $((N * 40)) 秒）…"
done_n=0
for _ in $(seq 1 $((N * 40))); do
  run=$(p '/api/tasks' '{"action":"list"}' | jget 'd.get("running")')
  [ "$run" = "False" ] && break
  sleep 1
done
chk "执行结束 running=false" "False" "$run"

echo
echo "【4】日志里必须有 $N 次「搬运完成」"
ok_n=$(q '/api/logs?' | /usr/bin/python3 -c '
import sys,json
d=json.load(sys.stdin)
print(sum(1 for l in d.get("logs",[]) if "搬运完成" in l.get("text","")))')
chk "完成次数 = $N" "$N" "$ok_n"

echo
echo "【5】日志里不许出现「任务失败或超时」"
bad=$(q '/api/logs?' | /usr/bin/python3 -c '
import sys,json
d=json.load(sys.stdin)
print(sum(1 for l in d.get("logs",[]) if "失败或超时" in l.get("text","")))')
chk "无失败" "0" "$bad"

echo
echo "【6】假 Nav2 实际收到的导航目标次数（硬证据：车真的动了 $N 次）"
cnt=$(cat tools/webtest/fake_nav.count 2>/dev/null || echo 0)
echo "   fake_nav 收到 $cnt 个目标"
if [ "$cnt" -ge "$N" ]; then
  chk "导航目标数 ≥ $N（每单至少一个目标）" "y" "y"
else
  chk "导航目标数 ≥ $N" "y" "n($cnt)"
fi

echo
echo "【7】执行中途中止：必须立刻停在 false，且之后还能再开新任务"
# ★ 用户报的第二个 bug：\"手动点中断后依旧显示执行中，之后既不能终止
#   也不能启动新任务\"。这里真的跑到一半就中止，看三个指标：
#   ① running 立刻变 false（界面不再卡"执行中"）
#   ② 中止确实发到了 task_manager（日志里出现\"任务被取消\"）
#   ③ 还能再开一单并且真的跑完（状态机没有卡死）
p '/api/tasks' '{"action":"clear"}' > /dev/null
for i in 1 2 3; do
  p '/api/tasks' '{"action":"add","pick":"1号槽内","drop":"1号货架","loops":1,"route":""}' > /dev/null
done
p '/api/tasks' '{"action":"start"}' > /dev/null
sleep 4                                  # 让它真的跑进第一单
before=$(cat tools/webtest/fake_nav.count 2>/dev/null || echo 0)
p '/api/tasks' '{"action":"stop"}' > /dev/null
run2=""
for _ in $(seq 1 15); do
  run2=$(p '/api/tasks' '{"action":"list"}' | jget 'd.get("running")')
  [ "$run2" = "False" ] && break
  sleep 1
done
chk "中止后立刻 running=false（不再卡执行中）" "False" "$run2"
cancelled=$(q '/api/logs?' | /usr/bin/python3 -c '
import sys,json
d=json.load(sys.stdin)
print(sum(1 for l in d.get("logs",[]) if "取消" in l.get("text","")))')
if [ "$cancelled" -ge 1 ]; then
  chk "中止指令确实送到了 task_manager" "y" "y"
else
  chk "中止指令确实送到了 task_manager" "y" "n"
fi
# 中止后再跑一单：验证状态机没卡死
after_stop=$(cat tools/webtest/fake_nav.count 2>/dev/null || echo 0)
p '/api/tasks' '{"action":"clear"}' > /dev/null
p '/api/tasks' '{"action":"add","pick":"1号槽内","drop":"1号货架","loops":1,"route":""}' > /dev/null
p '/api/tasks' '{"action":"start"}' > /dev/null
run3=true
for _ in $(seq 1 40); do
  run3=$(p '/api/tasks' '{"action":"list"}' | jget 'd.get("running")')
  [ "$run3" = "False" ] && break
  sleep 1
done
chk "中止后还能再开新任务并跑完" "False" "$run3"
after_new=$(cat tools/webtest/fake_nav.count 2>/dev/null || echo 0)
if [ "$after_new" -gt "$after_stop" ]; then
  chk "新任务真的发出了导航目标（$after_stop → $after_new）" "y" "y"
else
  chk "新任务真的发出了导航目标" "y" "n($after_stop → $after_new)"
fi

echo
echo "────────────────────────────────────────"
echo "  通过 $pass 项，失败 $fail 项"
if [ "$fail" = "0" ]; then
  echo "  ✅ 多任务连续执行 + 中止恢复均正常"
else
  echo "  ❌ 有问题，看 $LOG"
  echo "  ── 日志尾部 ──"
  tail -40 "$LOG"
fi
[ "$fail" = "0" ]
