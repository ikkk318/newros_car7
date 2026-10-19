cd /home/yuki/agv/newros_car7 || exit 1
export PATH=/usr/bin:/bin:/usr/local/bin:$PATH
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=141
export no_proxy='*'
PORT=8089; T="token=agv2024"; B="http://127.0.0.1:$PORT"

/usr/bin/python3 tools/webtest/injector.py > tools/webtest/inj4.log 2>&1 &
INJ=$!
sleep 4
ros2 launch agv_web web_console.launch.py token:=agv2024 port:=$PORT > tools/webtest/gw4.log 2>&1 &
GW=$!
for i in $(seq 1 30); do
  [ "$(curl -s --noproxy '*' -o /dev/null -w '%{http_code}' -m 2 "$B/?$T")" = "200" ] && break
  sleep 1
done
echo "网关就绪"

echo "=== 拿控制权 + 连发 3 帧遥控 ==="
curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' -d '{"action":"acquire","who":"devA"}' "$B/api/lease?$T" >/dev/null
for v in 0.2 0.3 0.1; do
  printf "  发 v=%s -> " "$v"
  curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' -d "{\"who\":\"devA\",\"v\":$v,\"w\":0.2}" "$B/api/teleop?$T"
  echo
  sleep 0.3
done
sleep 3
echo
echo "=== 注入器收到什么（这是 ROS 侧的铁证）==="
grep -E '^\[(RAW|WEB)\]' tools/webtest/inj4.log | head -10 || echo "  (没收到任何帧)"
echo "  RAW 帧数: $(grep -c '^\[RAW\]' tools/webtest/inj4.log)"
echo "  WEB 帧数: $(grep -c '^\[WEB\]' tools/webtest/inj4.log)"

echo
echo "=== 查 /cmd_vel_raw 的发布者数量 ==="
timeout 10 ros2 topic info /cmd_vel_raw 2>&1 | head -3

kill -INT $GW 2>/dev/null; sleep 2; kill -9 $GW $INJ 2>/dev/null
