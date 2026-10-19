cd /home/yuki/agv/newros_car7 || exit 1
export PATH=/usr/bin:/bin:/usr/local/bin:$PATH
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=171
export no_proxy='*'

/usr/bin/python3 tools/webtest/injector.py > tools/webtest/injF.log 2>&1 &
INJ=$!
sleep 3
ros2 run agv_waypoint waypoint_node > tools/webtest/wpF.log 2>&1 &
WP=$!
sleep 4
ros2 launch agv_web web_console.launch.py token:=agv2024 port:=8083 > tools/webtest/gwF.log 2>&1 &
GW=$!
PORT=8083; T="token=agv2024"; B="http://127.0.0.1:$PORT"
for i in $(seq 1 40); do
  [ "$(curl -s --noproxy '*' -o /dev/null -w '%{http_code}' -m 2 "$B/?$T")" = "200" ] && break
  sleep 1
done
sleep 3

echo "═══ 最终回归 ═══"
echo "1) 静态资源"
for f in "/" "/static/app.js" "/static/app.css"; do
  printf "   %-18s -> %s\n" "$f" "$(curl -s --noproxy '*' -o /dev/null -w '%{http_code} %{size_download}B' "$B$f?$T")"
done
echo "   新界面元素 modal: $(curl -s --noproxy '*' "$B/?$T" | grep -c 'id="modal"')"

echo "2) 状态链路"
curl -s --noproxy '*' "$B/api/state?$T" | /usr/bin/python3 -c "
import json,sys; d=json.load(sys.stdin)
print('   pose:', d['pose'], '| scan:', len(d['scan']), '| plan:', len(d['plan']),
      '| map_ver:', d['map_version'])"

echo "3) 地图 PNG"
curl -s --noproxy '*' -o tools/webtest/final_map.png -w "   %{http_code} %{content_type} %{size_download}B\n" "$B/api/map.png?$T"

echo "4) 自检接口（曾 NameError 500）"
# ★ 这里要等一会儿再查：/api/check 是"全项 AND"，而 TF 和 /scan 需要几个
#   发布周期才攒得出来。查太早会看到"地图已加载 ❌ / 定位可用 ❌ / 激光可用 ❌"，
#   这不是故障，是它还没来得及收数据。下面 sleep 5 + 逐项打印，就是为了
#   让读的人一眼分清"环境缺件"和"代码坏了"。
sleep 5
curl -s --noproxy '*' "$B/api/check?$T" -w "\n   HTTP %{http_code}\n" | /usr/bin/python3 -c "
import json,sys
raw=sys.stdin.read()
try:
    d=json.loads(raw.split('\n   HTTP')[0])
    print('   ok:', d['ok'], '| 检查项:', len(d['items']))
    for r in d['items']:
        print('     ', 'OK ' if r['ok'] else 'NG ', r['item'], '—', r.get('message',''))
except Exception as e:
    print('   解析失败:', raw[:200])"

echo "5) 业务接口（点位）"
curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' \
  -d '{"resource":"waypoint","action":"save","name":"终验点","x":0.5,"y":-0.3,"yaw":0,"kind":"nav"}' \
  "$B/api/store?$T" | head -c 150; echo
curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' \
  -d '{"resource":"waypoint","action":"list"}' "$B/api/store?$T" | /usr/bin/python3 -c "
import json,sys; d=json.load(sys.stdin)
print('   点位数量:', d['count'], [i['name'] for i in d['items']])"

echo "6) 遥控 + 控制权"
curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' -d '{"action":"acquire","who":"devA"}' "$B/api/lease?$T" >/dev/null
curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' -d '{"who":"devA","v":0.2,"w":0.1}' "$B/api/teleop?$T"
echo
sleep 1
echo "   ROS 侧收到: $(grep -c '^\[WEB\]' tools/webtest/injF.log) 帧"

echo "7) 日志与 WebSocket"
curl -s --noproxy '*' "$B/api/logs?$T" | /usr/bin/python3 -c "
import json,sys; d=json.load(sys.stdin); print('   日志条数:', len(d['logs']))"

echo "8) 网关有无异常"
if grep -qiE 'traceback|nameerror|typeerror|attributeerror' tools/webtest/gwF.log; then
  echo "   ⚠ 发现异常："; grep -iE 'traceback|error:' tools/webtest/gwF.log | head -5
else
  echo "   ✓ 无 traceback / NameError / TypeError"
fi

kill -INT $GW 2>/dev/null; kill -INT $INJ 2>/dev/null; kill -INT $WP 2>/dev/null
sleep 2; kill -9 $GW $INJ $WP 2>/dev/null
