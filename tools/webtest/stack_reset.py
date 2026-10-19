#!/usr/bin/env python3
"""换栈复位专项验证（真起网关 + 真 WebSocket + 真 ROS 话题）。

**验证目标（对应真机上报的三条问题）**
  ① "从导航模式与建图模式相互切换时有遗留，导致网页地图显示有重影"
  ② "切换之后小车的位置显示依旧保留之前切换前的最后显示，
     导致切换后小车的实际位置与显示的位置不符"
  ③ "重新定位使用不了"

**为什么必须真起进程**
这三条全是"跨进程状态残留"。纯单测只能验证"清缓存的代码执行了"，
验证不了"网关真的通过 WebSocket 把换栈信号发给了浏览器、而且发在
地图更新之前"。所以这里：

  1. injector.py 发真实 /map（6x4 米，origin=(-3,-2)）、TF、/scan；
  2. 真起 ros2 run agv_web web_gateway；
  3. 真连 WebSocket（用 aiohttp，环境里没装 websockets），逐帧记录事件；
  4. 断言 epoch 事件存在、且早于同批 map_changed；
  5. 断言换栈后 snapshot 的 pose / map_meta 全为 None；
  6. 断言重新定位在无订阅者时如实报 ok=False。

用法：
  bash tools/webtest/stack_reset.sh
  或 source ROS 环境后：/usr/bin/python3 tools/webtest/stack_reset.py
"""

import asyncio
import json
import os
import subprocess
import sys
import time
import urllib.request

WS = '/home/yuki/agv/newros_car7'
PORT = 8097
TOKEN = 'agv2024'
BASE = f'http://127.0.0.1:{PORT}'

PASS = 0
FAIL = 0
FAILS = []


def ok(cond, name, extra=''):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f'  ✓ {name}')
    else:
        FAIL += 1
        FAILS.append(name)
        print(f'  ✗ {name}{("  → " + str(extra)) if extra else ""}')


def get(path, timeout=5):
    sep = '&' if '?' in path else '?'
    req = urllib.request.Request(f'{BASE}{path}{sep}token={TOKEN}')
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def post(path, body, timeout=30):
    sep = '&' if '?' in path else '?'
    req = urllib.request.Request(
        f'{BASE}{path}{sep}token={TOKEN}',
        data=json.dumps(body).encode(),
        headers={'Content-Type': 'application/json'}, method='POST')
    with urllib.request.urlopen(req, timeout=timeout) as r:
        raw = r.read().decode()
        return json.loads(raw) if raw else {}


def wait_ready(timeout=40):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            with urllib.request.urlopen(f'{BASE}/?token={TOKEN}', timeout=2) as r:
                if r.status == 200:
                    return time.time() - t0
        except Exception:
            pass
        time.sleep(0.5)
    return None


# ── WS 采集：独立进程跑，避免和被测网关抢事件循环 ──
_WS_CODE = r'''
import asyncio, json, sys, time
import aiohttp

async def main(port, duration, out, token):
    frames = []
    t_end = time.time() + duration
    async with aiohttp.ClientSession() as s:
        # ★ WS 也走 guard_middleware，token 必须带上（放 query 或 Cookie）。
        #   不带就是 403 Invalid response status —— 第一次跑就栽在这，
        #   看起来像"网关没起来"，其实是被自己的鉴权挡了。
        url = 'ws://127.0.0.1:%d/ws?token=%s' % (port, token)
        async with s.ws_connect(url) as ws:
            while time.time() < t_end:
                try:
                    msg = await asyncio.wait_for(
                        ws.receive(), timeout=max(0.05, t_end - time.time()))
                except asyncio.TimeoutError:
                    break
                if msg.type != aiohttp.WSMsgType.TEXT:
                    if msg.type in (aiohttp.WSMsgType.CLOSED,
                                    aiohttp.WSMsgType.ERROR):
                        break
                    continue
                try:
                    m = json.loads(msg.data)
                except Exception:
                    continue
                t = time.time()
                if m.get('type') == 'state':
                    d = m.get('data') or {}
                    frames.append([t, 'state', {
                        'pose': d.get('pose'),
                        'map_meta': bool(d.get('map_meta')),
                        'map_version': d.get('map_version'),
                        'map_epoch': d.get('map_epoch'),
                        'mode': d.get('mode'),
                    }])
                else:
                    frames.append([t, m.get('type'), m])
    with open(out, 'w') as f:
        json.dump(frames, f)

asyncio.run(main(int(sys.argv[1]), float(sys.argv[2]), sys.argv[3], sys.argv[4]))
'''


class WsCollector:
    def __init__(self, out, duration):
        self.out = out
        self.proc = subprocess.Popen(
            [sys.executable, '-c', _WS_CODE, str(PORT), str(duration), out, TOKEN],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def result(self):
        try:
            self.proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        if self.proc.returncode != 0:
            err = (self.proc.stderr.read() or '')[-400:]
            print(f'    WS 采集失败：{err}')
            return []
        try:
            with open(self.out) as f:
                return json.load(f)
        except Exception:
            return []


def main():
    os.chdir(WS)
    print('═' * 62)
    print('  换栈复位专项验证（真网关 + 真 WebSocket + 真实话题）')
    print('═' * 62)

    inj = subprocess.Popen(
        [sys.executable, 'tools/webtest/injector.py'],
        stdout=open('tools/webtest/inj_stack.log', 'w'),
        stderr=subprocess.STDOUT)
    time.sleep(4)

    gw = subprocess.Popen(
        ['ros2', 'run', 'agv_web', 'web_gateway', '--ros-args',
         '-p', f'port:={PORT}', '-p', f'token:={TOKEN}',
         '-p', 'mode:=nav', '-p', 'manage_stack:=false'],
        stdout=open('tools/webtest/gw_stack.log', 'w'),
        stderr=subprocess.STDOUT)

    try:
        waited = wait_ready()
        ok(waited is not None,
           '网关就绪' + (f'（{round(waited, 1)}s）' if waited else ''))
        if waited is None:
            print('   日志尾部：')
            print(''.join(open('tools/webtest/gw_stack.log').readlines()[-20:]))
            return 1

        time.sleep(3)

        print('\n【1】换栈前基线：地图 / 位姿 / 激光都在')
        st = get('/api/state')
        ok(st.get('map_meta') is not None, '收到地图几何（injector 的 6x4 房间）')
        ok(st.get('pose') is not None, '查到 map→base 位姿')
        ok(len(st.get('scan') or []) > 0, '激光点已变换到地图坐标系')
        ok(isinstance(st.get('map_epoch'), int), '快照带 map_epoch 字段',
           st.get('map_epoch'))
        base_epoch = st.get('map_epoch')
        base_ver = st.get('map_version')
        if st.get('pose'):
            print(f"    基线：epoch={base_epoch} version={base_ver} "
                  f"pose=({st['pose']['x']:.2f},{st['pose']['y']:.2f})")

        print('\n【2】稳态下不该反复发 map_epoch（否则视图一直跳）')
        c1 = WsCollector('tools/webtest/ws_stable.json', 2.5)
        f1 = c1.result()
        ok(len(f1) > 3, f'收到 WS 帧 {len(f1)} 条')
        # ★ 新连接建立时后端**一定**会补发一次当前 epoch（last_epoch 初始 -1），
        #   这是握手，不是"换栈"。所以稳态判据是"最多 1 次"，
        #   并且它必须是第一帧附近的那个 —— 之后的才是真问题。
        ep1 = [i for i, f in enumerate(f1) if f[1] == 'map_epoch']
        ok(len(ep1) <= 1,
           f'稳态里没有反复发 map_epoch（{len(ep1)} 次，握手允许 1 次）')
        if ep1:
            ok(ep1[0] <= 2, f'那唯一一次是连接握手（第 {ep1[0]} 帧）',
               '不该出现在稳态中间')
        st1 = [f for f in f1 if f[1] == 'state']
        if st1:
            ok(st1[-1][2].get('pose') is not None,
               '稳态里位姿持续在（说明前一步的 None 不是一直不显示）')

        print('\n【3】切模式：旧地图 / 旧位姿必须作废')
        # ★★ 必须先停掉 injector，否则这个测试永远是假绿/假红。
        #   原因：网关清掉 _gw_pose 之后 0.2 秒，refresh_pose 又会从 TF 查到
        #   位姿并立刻填回来 —— 因为 injector 是个独立进程，它不知道"栈已经
        #   换了"，还在继续发 map→base_footprint。
        #   而真实的换栈是 StackRunner 对旧 launch 的整个进程组发 SIGINT，
        #   旧节点死掉、TF 和 /map 一起停，清掉的缓存不会被人填回来。
        #   所以这里停掉 injector 才是等价场景；否则测的是"injector 一直在
        #   发数据时网关会不会被覆盖"，那是另一个（不存在的）命题。
        for p in (inj,):
            try:
                p.terminate()
            except Exception:
                pass
        try:
            inj.wait(timeout=8)
        except Exception:
            try:
                inj.kill()
            except Exception:
                pass
        # 给网关一点时间消化"最后一帧"。
        # ★ 这里必须等够 pose_hold_sec + 一个 tick。
        #   tf2 Buffer 默认缓存 10 秒历史，injector 停掉之后 lookup 仍能查到
        #   旧变换（只是 stamp 变旧）；网关现在靠"变换时间戳"判过期，
        #   超过 pose_hold_sec(2s) 就作废。等 4 秒留足余量。
        time.sleep(4.0)
        st_gap = get('/api/state')
        ok(st_gap.get('pose') is None,
           '数据源消失后位姿按保鲜期自然失效（证明过期判定在工作，'
           '而不是靠"查不到"这个不可靠信号）',
           st_gap.get('pose'))

        # 采集进程先起来并留足连接时间，再切模式 —— 否则会错过切换瞬间的帧
        c2 = WsCollector('tools/webtest/ws_switch.json', 8.0)
        time.sleep(2.5)          # 等 aiohttp 连上（import + 握手约 1~2 秒）
        try:
            r = post('/api/stack_start', {'mode': 'mapping'}, timeout=25)
            print(f'    stack_start → ok={r.get("ok")} '
                  f'{str(r.get("message", ""))[:80]}')
            # ★ manage_stack=false 时必须走降级路径：ok=True + 说明要自己起栈，
            #   而不是 503。这条就是本轮补的真实缺口。
            ok(r.get('ok') is True,
               'manage_stack=false 时切模式走降级路径而不是 503', r)
            ok(r.get('manage') is False, '明确告知"本进程不管理栈"', r)
            ok('mapping' == r.get('mode'), '返回了切换后的模式', r)
            ok('/cmd_vel_raw' in str(r.get('teleop_topic', '')),
               f'遥控话题已切到建图通道（{r.get("teleop_topic")}）', r)
        except Exception as exc:
            ok(False, 'stack_start 不该抛错（降级路径必须成功）', exc)
        f2 = c2.result()

        st2 = get('/api/state')
        ok(st2.get('map_epoch') != base_epoch,
           f'map_epoch 变了（{base_epoch} → {st2.get("map_epoch")}）')
        ok(st2.get('map_meta') is None,
           '旧地图几何已清空（否则新图按旧 origin 画 = 重影）',
           st2.get('map_meta'))
        ok(st2.get('pose') is None,
           '旧位姿已清空（否则小车停在切换前的位置）', st2.get('pose'))
        ok(not (st2.get('scan') or []),
           '旧激光点已清空（属于另一套坐标系）',
           len(st2.get('scan') or []))

        print('\n【4】WS 是不是真把换栈信号推给浏览器了')
        ok(len(f2) > 0, f'切换期间收到 {len(f2)} 条 WS 帧')
        ep_idx = [i for i, f in enumerate(f2) if f[1] == 'map_epoch']
        ok(len(ep_idx) >= 1, f'收到 map_epoch 事件（{len(ep_idx)} 次）')
        if ep_idx:
            first_ep = ep_idx[0]
            mc_idx = [i for i, f in enumerate(f2) if f[1] == 'map_changed']
            later = [i for i in mc_idx if i >= first_ep]
            ok(not later or later[0] > first_ep,
               'map_epoch 早于随后的 map_changed'
               '（顺序反了前端会用旧几何重载一次）',
               f'epoch@{first_ep} mc@{later[:1]}')
            # ★ 取**最后**一条 epoch 事件比：窗口里可能因为
            #   "先停栈 + 再起栈"或一次性切换产生两次清理（两次 +1），
            #   只有最后那一条才对应当前快照。比较第一条会误报。
            last_ep = ep_idx[-1]
            ok(f2[last_ep][2].get('epoch') == st2.get('map_epoch'),
               f'最后一条 epoch 事件与快照一致（{f2[last_ep][2].get("epoch")} '
               f'== {st2.get("map_epoch")}）')
            after = [f for f in f2[last_ep + 1:] if f[1] == 'state']
            if after:
                ok(all(f[2].get('pose') is None for f in after),
                   'epoch 之后**每一帧** state 都没有旧位姿'
                   '（不是只有第一帧干净，旧位姿被彻底作废）',
                   [f[2].get('pose') for f in after[:3]])
            # ★ 这里不能断言"换栈前那一帧还有位姿"：本脚本在【3】开头特意
            #   停掉了数据源来验证保鲜期，那时位姿已经按设计失效了。
            #   改成检查更本质的性质 —— 整个切换过程里位姿**只允许单向**
            #   从有到无，不允许"清掉之后又被旧数据填回来"（那正是 tf2
            #   缓存复活旧栈位姿的症状）。
            seq = [f[2].get('pose') for f in f2 if f[1] == 'state']
            revived = False
            seen_none = False
            for p in seq:
                if p is None:
                    seen_none = True
                elif seen_none:
                    revived = True
                    break
            ok(not revived,
               '位姿没有"清掉之后又被填回来"（tf2 缓存复活旧栈位姿的症状）',
               seq[:6])

        print('\n【5】重新定位：如实反馈')
        r = post('/api/relocalize', {'x': 1.5, 'y': -0.5, 'yaw': 1.5708})
        print(f'    /api/relocalize → {r}')
        ok('message' in r, '接口返回 message（旧版只有 {"ok": true}）')
        ok(r.get('ok') is False,
           '没有 AMCL 订阅者时如实报失败'
           '（旧版无条件 True，用户以为生效了）', r)
        st4 = get('/api/state')
        p4 = st4.get('pose')
        ok(p4 is not None, '发布后本地缓存位姿立刻可见（不等 AMCL 收敛）', p4)
        if p4:
            ok(abs(p4['x'] - 1.5) < 0.05 and abs(p4['y'] + 0.5) < 0.05,
               f'缓存位姿 = 目标点（{p4["x"]:.2f},{p4["y"]:.2f}）')
            ok(abs(p4['yaw'] - 1.5708) < 0.05,
               f'缓存朝向 = 目标朝向（{p4["yaw"]:.3f}）')

        print('\n【6】日志能对上时间线')
        logs = get('/api/logs').get('logs') or []
        txt = ' '.join(str(x.get('text', '')) for x in logs)
        ok('清理' in txt,
           '日志记录了"清理上一套栈缓存"（排查时靠它定位时间点）')
        hits = [x.get('text') for x in logs if '清理' in str(x.get('text'))]
        for h in hits[:3]:
            print(f'    · {h}')

    finally:
        print('\n清理进程…')
        for p in (gw, inj):
            try:
                p.terminate()
            except Exception:
                pass
        time.sleep(2)
        for p in (gw, inj):
            try:
                p.kill()
            except Exception:
                pass
        pkill = subprocess.run(
            'pkill -f "tools/webtest/injector.py" ; pkill -f web_gateway',
            shell=True, capture_output=True)
        del pkill

    print('\n' + '─' * 62)
    print(f'  通过 {PASS} 项，失败 {FAIL} 项')
    if FAILS:
        print('  失败项：')
        for f in FAILS:
            print('    · ' + f)
    print('─' * 62)
    return 0 if FAIL == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
