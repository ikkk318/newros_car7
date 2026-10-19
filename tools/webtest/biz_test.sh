cd /home/yuki/agv/newros_car7 || exit 1
export PATH=/usr/bin:/bin:/usr/local/bin:$PATH
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=151
export no_proxy='*'
PORT=8085; T="token=agv2024"; B="http://127.0.0.1:$PORT"

echo "=== 启动业务节点（不需要 Gazebo）==="
ros2 run agv_waypoint waypoint_node > tools/webtest/wp.log 2>&1 &
WP=$!
sleep 5
echo "  waypoint_node: $(head -2 tools/webtest/wp.log | tr '\n' ' ')"

ros2 run agv_web web_gateway --ros-args -p port:=$PORT -p token:=agv2024 \
  -p manage_stack:=false > tools/webtest/gw8.log 2>&1 &
GW=$!
for i in $(seq 1 30); do
  [ "$(curl -s --noproxy '*' -o /dev/null -w '%{http_code}' -m 2 "$B/?$T")" = "200" ] && break
  sleep 1
done
sleep 2

echo
echo "=== /agv/store 服务是否就绪 ==="
curl -s --noproxy '*' "$B/api/check?$T" | /usr/bin/python3 -c "
import json,sys
d=json.load(sys.stdin)
for it in d['items']:
    if '服务' in it['item'] or '地图库' in it['item']:
        print(f\"  {'✓' if it['ok'] else '✗'} {it['item']}: {it['message']}\")"

echo
echo "=== 存点位（只输名字，坐标来自地图点选）==="
printf "  "; curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' \
  -d '{"resource":"waypoint","action":"save","name":"货架A","x":1.5,"y":0.8,"yaw":0.0,"kind":"cargo"}' "$B/api/store?$T" | head -c 250; echo
printf "  "; curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' \
  -d '{"resource":"waypoint","action":"save","name":"充电桩","x":-1.2,"y":-0.5,"yaw":1.57,"kind":"nav"}' "$B/api/store?$T" | head -c 250; echo

echo
echo "=== 列点位 ==="
curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' \
  -d '{"resource":"waypoint","action":"list"}' "$B/api/store?$T" | /usr/bin/python3 -c "
import json,sys; d=json.load(sys.stdin)
print('  ok:', d['ok'], '| 数量:', d['count'])
for i in d['items']: print(f\"    {i['name']:<8} ({i['x']:.2f},{i['y']:.2f}) yaw={i['yaw']:.2f} kind={i['kind']}\")"

echo
echo "=== 新建路线（point_names 直接传数组，不用输入逗号分隔串）==="
printf "  "; curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' \
  -d '{"action":"save","name":"A到B","point_names":["货架A","充电桩"]}' "$B/api/routes?$T" | head -c 200; echo
echo "=== 列路线 ==="
printf "  "; curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' \
  -d '{"action":"list"}' "$B/api/routes?$T" | head -c 250; echo

echo
echo "=== 任务缓冲 ==="
printf "  增: "; curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' \
  -d '{"action":"add","pick":"货架A","drop":"充电桩","loops":2}' "$B/api/tasks?$T" | head -c 200; echo
printf "  查: "; curl -s --noproxy '*' "$B/api/tasks?$T" | head -c 250; echo
printf "  清: "; curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' -d '{"action":"clear"}' "$B/api/tasks?$T" | head -c 120; echo

kill -INT $GW 2>/dev/null; kill -INT $WP 2>/dev/null; sleep 2; kill -9 $GW $WP 2>/dev/null
