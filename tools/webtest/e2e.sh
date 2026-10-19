#!/usr/bin/env bash
# 端到端测试：网关 + 数据注入器在同一进程树内（本沙箱跨命令 DDS 发现被隔断）
# 用法：bash tools/webtest/e2e.sh
# ★ 不能开 set -u：ROS 的 setup.bash 会读未绑定变量，一开就报错退出

WS=/home/yuki/agv/newros_car7
cd "$WS" || exit 1
export PATH=/usr/bin:/bin:/usr/local/bin:$PATH
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=123
export GAZEBO_MASTER_URI=http://127.0.0.1:11399
export no_proxy='*' NO_PROXY='*'
PORT=8090
T="token=agv2024"
B="http://127.0.0.1:$PORT"

cleanup() {
  [ -n "${GW_PID:-}" ] && kill -INT "$GW_PID" 2>/dev/null
  [ -n "${INJ_PID:-}" ] && kill -INT "$INJ_PID" 2>/dev/null
  sleep 2
  [ -n "${GW_PID:-}" ] && kill -9 "$GW_PID" 2>/dev/null
  [ -n "${INJ_PID:-}" ] && kill -9 "$INJ_PID" 2>/dev/null
}
trap cleanup EXIT

echo "############ 1. 启动数据注入器 ############"
/usr/bin/python3 tools/webtest/injector.py > tools/webtest/inj.log 2>&1 &
INJ_PID=$!
sleep 5
echo "注入器 PID=$INJ_PID"
tail -3 tools/webtest/inj.log

echo
echo "############ 2. 启动网关（独立域 123）############"
ros2 launch agv_web web_console.launch.py token:=agv2024 port:=$PORT > tools/webtest/gw.log 2>&1 &
GW_PID=$!
echo "网关 PID=$GW_PID"
for i in $(seq 1 30); do
  code=$(curl -s --noproxy '*' -o /dev/null -w '%{http_code}' -m 2 "$B/?$T")
  [ "$code" = "200" ] && break
  sleep 1
done
echo "网关 HTTP=$code（等待 ${i}s）"

echo
echo "############ A. 启动与登录 ############"
curl -s --noproxy '*' -o /dev/null -w "  GET /              -> %{http_code} %{size_download}B\n" "$B/?$T"
curl -s --noproxy '*' -o /dev/null -w "  GET /static/app.js -> %{http_code} %{size_download}B\n" "$B/static/app.js"
curl -s --noproxy '*' -o /dev/null -w "  GET /static/app.css-> %{http_code} %{size_download}B\n" "$B/static/app.css"
printf "  错误口令           -> "; curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' -d '{"token":"bad"}' -w "%{http_code} " "$B/api/login?$T"; echo
printf "  正确口令           -> "; curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' -d '{"token":"agv2024"}' -w "%{http_code} " "$B/api/login?$T"; echo
printf "  无口令访问 API     -> "; curl -s --noproxy '*' -o /dev/null -w "%{http_code}\n" "$B/api/state"

echo
echo "############ C. RViz 等价显示（注入真实数据）############"
sleep 3
curl -s --noproxy '*' "$B/api/state?$T" > tools/webtest/state.json
/usr/bin/python3 - <<'PY'
import json
d = json.load(open('/home/yuki/agv/newros_car7/tools/webtest/state.json'))
print('  map_version :', d['map_version'])
m = d['map_meta'] or {}
print('  map_meta    :', {k: m.get(k) for k in ('width', 'height', 'resolution')})
p = d['pose']
print('  pose        :', None if not p else {k: round(v, 3) for k, v in p.items()})
print('  scan 点数    :', len(d['scan']))
print('  scan 首个点  :', d['scan'][0] if d['scan'] else None)
print('  plan 点数    :', len(d['plan']))
print('  task        :', d['task'])
print('  lease 初始   :', d['lease'])
PY

echo
echo "  --- 地图 PNG ---"
curl -s --noproxy '*' -o tools/webtest/map.png -w "  GET /api/map.png -> %{http_code} %{content_type} %{size_download}B\n" "$B/api/map.png?$T"
/usr/bin/python3 -c "
import struct
b=open('/home/yuki/agv/newros_car7/tools/webtest/map.png','rb').read()
assert b[:8]==b'\x89PNG\r\n\x1a\n', '不是 PNG'
w,h=struct.unpack('>II', b[16:24])
print(f'  PNG 尺寸     : {w} x {h}  字节 {len(b)}')
"

echo
echo "############ D. 遥控链路（网页→ROS 方向）############"
echo "  拿控制权："
curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' -d '{"action":"acquire","who":"devA"}' "$B/api/lease?$T"; echo
echo "  发前进指令 v=0.2："
curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' -d '{"who":"devA","v":0.2,"w":0.0}' "$B/api/teleop?$T"; echo
echo "  超速请求 v=99（应被限幅到 0.35）："
curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' -d '{"who":"devA","v":99,"w":99}' "$B/api/teleop?$T"; echo
echo "  另一台设备抢控制权（应被拒）："
curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' -d '{"action":"acquire","who":"devB"}' -w " [%{http_code}]\n" "$B/api/lease?$T"
echo "  用别人的身份遥控（应被拒）："
curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' -d '{"who":"devB","v":0.1,"w":0}' -w " [%{http_code}]\n" "$B/api/teleop?$T"
sleep 2
echo "  注入器实际收到 /cmd_vel_raw 的帧："
grep -o 'RAW' tools/webtest/inj.log | wc -l 2>/dev/null || true

echo
echo "############ 业务接口 ############"
printf "  /api/maps    : "; curl -s --noproxy '*' "$B/api/maps?$T" | head -c 200; echo
printf "  /api/stack   : "; curl -s --noproxy '*' "$B/api/stack?$T" | head -c 250; echo
printf "  /api/check   : "; curl -s --noproxy '*' "$B/api/check?$T" | head -c 400; echo
printf "  /api/tasks   : "; curl -s --noproxy '*' "$B/api/tasks?$T" | head -c 200; echo
printf "  /api/routes  : "; curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' -d '{"action":"list"}' "$B/api/routes?$T" | head -c 250; echo

echo
echo "############ E. 日志推送 ############"
curl -s --noproxy '*' "$B/api/logs?$T" | /usr/bin/python3 -c "
import json,sys
logs=json.load(sys.stdin)['logs']
print(f'  日志条数: {len(logs)}')
for l in logs[-8:]: print(f\"  [{l['level']}] {l['text'][:110]}\")
"

echo
echo "############ 收尾 ############"
echo "  网关日志尾部："; tail -5 tools/webtest/gw.log
