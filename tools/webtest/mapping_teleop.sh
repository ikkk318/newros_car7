cd /home/yuki/agv/newros_car7 || exit 1
export PATH=/usr/bin:/bin:/usr/local/bin:$PATH
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=143
export no_proxy='*'
PORT=8088; T="token=agv2024"; B="http://127.0.0.1:$PORT"

/usr/bin/python3 tools/webtest/injector.py > tools/webtest/inj5.log 2>&1 &
INJ=$!
sleep 4

# ★ 关键：以 mode:=mapping + manage_stack:=false 起网关
#   这样不动 Gazebo，直接验证"建图模式下遥控话题是否是 /cmd_vel_raw"
echo "=== 以建图模式启动网关（不管理栈）==="
ros2 run agv_web web_gateway --ros-args \
  -p port:=$PORT -p token:=agv2024 \
  -p mode:=mapping -p manage_stack:=false > tools/webtest/gw5.log 2>&1 &
GW=$!
for i in $(seq 1 30); do
  [ "$(curl -s --noproxy '*' -o /dev/null -w '%{http_code}' -m 2 "$B/?$T")" = "200" ] && break
  sleep 1
done
echo "网关就绪（用了 ${i}s）"

echo
echo "=== 启动日志中的模式与遥控话题 ==="
grep -E '当前模式|遥控通道' tools/webtest/gw5.log | head -3

echo
echo "=== /api/stack 报告的 teleop_topic ==="
curl -s --noproxy '*' "$B/api/stack?$T" | /usr/bin/python3 -c "
import json,sys; d=json.load(sys.stdin)['stack']
print('  mode        :', d['mode'])
print('  manage      :', d['manage'])
print('  teleop_topic:', d['teleop_topic'])"

echo
echo "=== 发遥控，看落到哪个话题 ==="
curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' -d '{"action":"acquire","who":"devA"}' "$B/api/lease?$T" >/dev/null
for v in 0.15 0.25; do
  curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' -d "{\"who\":\"devA\",\"v\":$v,\"w\":0}" "$B/api/teleop?$T" >/dev/null
  sleep 0.3
done
sleep 2
echo "  注入器日志："
grep -E '^\[(RAW|WEB)\]' tools/webtest/inj5.log | head -6
echo "  RAW 帧数: $(grep -c '^\[RAW\]' tools/webtest/inj5.log)  (建图模式应为非0)"
echo "  WEB 帧数: $(grep -c '^\[WEB\]' tools/webtest/inj5.log)  (建图模式应发不到这里，只有看门狗零速)"

kill -INT $GW 2>/dev/null; sleep 2; kill -9 $GW $INJ 2>/dev/null
