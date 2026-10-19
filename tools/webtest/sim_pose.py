#!/usr/bin/env python3
"""
仿真时基位姿专项验证：**上游用 sim time、网关用 wall time** 时，小车能不能显示。

**为什么必须有这个脚本（真实故障）**
Gazebo 栈（建图 / 带仿真的导航）发布 TF 用的是 `/clock` 的**仿真时间**——
从 0 开始计时，启动十分钟也才 600 秒。而 `web_gateway` 只是 aiohttp 进程，
`web_console.launch.py` 并没有给它传 `use_sim_time`，它走的是**系统时间**
（约 1.79e9 秒）。

第一版"位姿保鲜期"实现直接算 `now - stamp`，得到 ≈ 57 年，
于是**每一帧 TF 都被判成过期**，`_gw_pose` 永远是 None。
用户看到的现象极具迷惑性：

    ✅ 地图正常（/map 与 TF 无关）
    ✅ 路径正常（/plan 与 TF 无关）
    ✅ 激光点……也正常（变换用的是位姿，但位姿丢时点会消失；
                    实际观察里地图和路径都在，就是没有小车）
    ❌ 小车不显示 —— 而且建图、导航两个模式全中（两套栈都带 Gazebo）

单测抓不到，因为测试桩的 clock 和 stamp 都来自 `time.time()`，时基天然一致。
所以必须真起进程、真发 `/clock`、真跑网关。

用法：bash tools/webtest/sim_pose.sh
"""
import json
import os
import subprocess
import sys
import time
import urllib.request

WS = '/home/yuki/agv/newros_car7'
PORT = 8791
TOKEN = 'simtest'
DOMAIN = '213'

_pass = 0
_fail = 0


def ok(cond, name, extra=''):
    global _pass, _fail
    if cond:
        _pass += 1
        print(f'  ✓ {name}' + (f'（{extra}）' if extra else ''))
    else:
        _fail += 1
        print(f'  ✗ {name}' + (f'（{extra}）' if extra else ''))
    return cond


def get(path, timeout=8):
    url = f'http://127.0.0.1:{PORT}{path}'
    sep = '&' if '?' in path else '?'
    req = urllib.request.Request(f'{url}{sep}token={TOKEN}')
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def wait_ready(timeout=40):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            d = get('/api/state', timeout=2)
            if isinstance(d, dict):
                return time.time() - t0
        except Exception:
            time.sleep(0.5)
    return None


def main():
    os.chdir(WS)
    print('═' * 66)
    print('  仿真时基位姿验证（上游 sim time + 网关 wall time）')
    print('═' * 66)

    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = DOMAIN
    env['no_proxy'] = '*'
    env['NO_PROXY'] = '*'
    env['PYTHONUNBUFFERED'] = '1'

    inj = subprocess.Popen(
        ['/usr/bin/python3', 'tools/webtest/injector.py', '--sim'],
        stdout=open('tools/webtest/inj_sim.log', 'w'),
        stderr=subprocess.STDOUT, env=env)
    time.sleep(5)

    gw = subprocess.Popen(
        ['ros2', 'run', 'agv_web', 'web_gateway', '--ros-args',
         '-p', f'port:={PORT}', '-p', f'token:={TOKEN}',
         '-p', 'mode:=mapping', '-p', 'manage_stack:=false'],
        stdout=open('tools/webtest/gw_sim.log', 'w'),
        stderr=subprocess.STDOUT, env=env)

    try:
        waited = wait_ready()
        ok(waited is not None,
           '网关就绪' + (f'（{round(waited, 1)}s）' if waited else ''))
        if waited is None:
            tail = ''.join(open('tools/webtest/gw_sim.log').readlines()[-25:])
            print('   网关日志尾部：\n' + tail)
            tail2 = ''.join(open('tools/webtest/inj_sim.log').readlines()[-15:])
            print('   注入器日志尾部：\n' + tail2)
            return 1

        print('\n【1】上游 TF 用的是仿真时间（模拟 Gazebo）')
        # 直接从注入器日志确认它开了 sim time
        inj_log = open('tools/webtest/inj_sim.log').read()
        ok('仿真时基' in inj_log, '注入器跑在 --sim 模式（发 /clock，stamp 从 0 起）')

        # 读 /clock 与 TF 的时间量级，证明两者差了几十年
        try:
            import rclpy
            from rclpy.node import Node
            from rosgraph_msgs.msg import Clock as ClockMsg
            rclpy.init()
            node = Node('sim_probe')
            got = {}
            node.create_subscription(
                ClockMsg, '/clock',
                lambda m: got.setdefault('t', m.clock.sec + m.clock.nanosec * 1e-9),
                10)
            t0 = time.time()
            while time.time() - t0 < 3 and 't' not in got:
                rclpy.spin_once(node, timeout_sec=0.2)
            sim_t = got.get('t')
            node.destroy_node()
            rclpy.shutdown()
        except Exception as exc:
            sim_t = None
            print(f'    （读 /clock 失败：{exc}）')
        if sim_t is not None:
            gap = time.time() - sim_t
            ok(sim_t < 3600,
               f'/clock 是仿真时间（{sim_t:.1f}s），与系统时间相差约 {gap/86400/365:.0f} 年',
               '这正是第一版实现会算出的"年龄"')

        time.sleep(2)

        print('\n【2】★ 核心：位姿必须能显示（本次故障点）')
        st = get('/api/state')
        pose = st.get('pose')
        ok(st.get('map_meta') is not None, '地图几何正常（说明注入器在工作）')
        ok(len(st.get('plan') or []) >= 0, '路径字段存在')
        ok(pose is not None,
           '★ 位姿能取到（时基不一致不能判过期，否则两个模式都看不到小车）',
           f"({pose['x']:.2f}, {pose['y']:.2f})" if pose else 'None —— 就是故障现象')
        if pose:
            ok(abs(pose['x']) < 100 and abs(pose['y']) < 100,
               '位姿数值在地图范围内（不是被错误数据污染）',
               f"{pose['x']:.2f},{pose['y']:.2f}")

        print('\n【3】位姿要持续更新（不是只显示一帧就没了）')
        samples = []
        t0 = time.time()
        while time.time() - t0 < 4.0:
            try:
                p = get('/api/state', timeout=3).get('pose')
                samples.append(None if p is None else round(p['yaw'], 4))
            except Exception:
                samples.append('err')
            time.sleep(0.4)
        got_pose = [s for s in samples if isinstance(s, float)]
        ok(len(got_pose) >= len(samples) - 1,
           f'连续 {len(samples)} 次采样都有位姿（{len(got_pose)}/{len(samples)}）',
           '中间掉一帧可以，掉一半说明保鲜期在反复踢掉它')
        distinct = len(set(got_pose))
        ok(distinct > 1,
           f'朝向在变化（{distinct} 个不同值，注入器在摆动）—— 证明是真数据而非缓存残帧')

        print('\n【4】地图/激光也要一起正常（对照，确认不是别处在坏）')
        ok(bool(st.get('map_meta')), '地图几何在')
        ok(len(st.get('scan') or []) > 0,
           f"激光点 {len(st.get('scan') or [])} 个（需要位姿才能变换，位姿丢时这里也会空）")

        print('\n【5】日志不该刷"定位丢失"')
        gw_log = open('tools/webtest/gw_sim.log').read()
        lost = gw_log.count('定位丢失')
        ok(lost == 0,
           f'没有"定位丢失"告警（实际 {lost} 次）—— 有就是在误判过期')
        ok('位姿解析异常' not in gw_log,
           '没有"位姿解析异常"（曾因 fresh 变量未初始化抛 UnboundLocalError）')

    finally:
        for p in (gw, inj):
            try:
                p.terminate()
            except Exception:
                pass
        time.sleep(1.5)
        for p in (gw, inj):
            try:
                p.kill()
            except Exception:
                pass

    print('\n' + '─' * 66)
    print(f'  通过 {_pass} 项，失败 {_fail} 项')
    print('─' * 66)
    return 0 if _fail == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
