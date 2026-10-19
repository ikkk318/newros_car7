#!/usr/bin/env python3
"""
board_e2e —— 真起网关节点 + 真跑 aiohttp/WebSocket，验证日志分流。

**为什么必须端到端（单测不够）**
`status_board` 的单测只能证明"分类函数判得对"。但真实链路还有三段
单测看不到的东西：
  1. `_pump_output` → `log_translate` → `log()` 这条真实链路上，
     翻译后的中文文本是否**仍然**能被分流规则认出来；
  2. `snapshot()` 是否真的把 `board` 发到 WebSocket 上（前端拿不到就是白的）；
  3. 200 条 1 Hz 避障日志涌进来时，**事件日志是否还留在 _gw_logs 里**
     —— 这正是用户报的"日志刷太快，顶升完成一闪而过"。

所以这里不 mock 任何东西：
  · 真 `rclpy.init()` + 真 `WebGateway()` 节点；
  · 真 `make_app()` + `web.AppRunner` 监听本地端口；
  · 真 `aiohttp` 客户端连 WebSocket 取值；
  · 日志走**真实的** log_translate 翻译链（等价于栈子进程 stdout 的样子）。

用法：
    python3 tools/webtest/board_e2e.py
"""

import asyncio
import json
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WS_ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
PORT = 8931
TOKEN = 'boardtest'

pass_n = 0
fail_n = 0
fails = []


def ok(cond, name, extra=''):
    global pass_n, fail_n
    if cond:
        pass_n += 1
        print(f'  \u2713 {name}')
    else:
        fail_n += 1
        fails.append(name)
        print(f'  \u2717 {name}' + (f'  \u2192 {extra}' if extra else ''))


def avoid_line(i):
    """与 velocity_arbiter.py 那条 get_logger().info(...) **完全同格式**。

    ★ 不能自己编一个"[避障] 前方 xx"了事：真实日志带闸门/左右后/速度
      变化/择向接管，翻译层会动它。必须用真格式才能证明
      "翻译后仍能被分流规则认出来"。
    """
    return (f'[INFO] [1792257198.790424458] [velocity_arbiter-3]: '
            f'[避障] 前方 {0.30 + (i % 40) * 0.01:.2f}m（闸门 0.55m） '
            f'左 0.90 右 1.20 后 3.10 \u2192 v 0.30\u2192'
            f'{0.30 - (i % 10) * 0.02:.2f}, w 0.00\u21920.20，择向接管')


async def main():
    import rclpy
    from aiohttp import web, ClientSession

    from agv_web.web_gateway import WebGateway, make_app
    from agv_web import log_translate as LT
    # 跨节点日志转发测试要用真的消息类型（和网关订阅的是同一个）
    try:
        from rcl_interfaces.msg import Log as RosoutLog
    except Exception:
        RosoutLog = None

    print('\u2500' * 62)
    print('  实时数据面板端到端（真节点 + 真 aiohttp + 真 WebSocket）')
    print('\u2500' * 62)

    old = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = '221'      # 干净域，避免残留节点抢
    rclpy.init()
    node = WebGateway()
    node.port = PORT
    node.auth.token = TOKEN
    node.auth.allow_ips = []

    # 真 spin 线程（和 main() 一样：aiohttp 主循环 + rclpy 子线程）
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    runner = web.AppRunner(make_app(node))
    await runner.setup()
    site = web.TCPSite(runner, '127.0.0.1', PORT)
    await site.start()
    ok(True, f'网关节点 + aiohttp 已启动（127.0.0.1:{PORT}）')

    base = f'http://127.0.0.1:{PORT}'
    try:
        async with ClientSession() as sess:
            # ── 先确认 HTTP 正常 ──
            async with sess.get(f'{base}/api/state?token={TOKEN}') as r:
                st = await r.json()
            ok('board' in st, 'snapshot 里带 board 字段')
            ok(isinstance(st.get('board'), list), 'board 是列表（前端直接遍历）')

            # ── 连 WebSocket ──
            async with sess.ws_connect(
                    f'ws://127.0.0.1:{PORT}/ws?token={TOKEN}', timeout=10) as ws:
                first = await ws.receive_json(timeout=10)
                ok(first.get('type') == 'state', 'WebSocket 能收到状态帧')

                # ══ 灌 200 条避障日志（等价 200 秒真实运行）══
                #  ★ 走真实链路：原始行 → log_translate.translate() → log()
                #    这正是栈子进程 stdout 被 _pump_output 读取时的路径。
                #
                #  ★ 期望值变了（第二轮）：以前断言"200 条一条都不进日志"，
                #    因为第一版把避障全塞进面板格子。现在面板是固定仪表
                #    （避障格的值来自激光，不来自日志），日志不再承担
                #    "显示当前状态"的职责，所以避障改为**限流留存**：
                #    5 秒一条。用户原话是"增加打印的时间间隔"，不是"别打了"——
                #    留几条痕迹，出事后能翻到"当时前方 0.4m"。
                print('\n  \u2500\u2500 灌 200 条 1 Hz 避障日志（真实格式，过翻译层）')
                n_kept = 0
                for i in range(200):
                    raw = avoid_line(i)
                    out = LT.translate(raw)      # 真翻译（不是直接喂中文）
                    node.log(LT.guess_level(raw), out)
                    if i == 60:
                        node.log('info', '[顶升] 开始（货架1）')
                    elif i == 120:
                        node.log('info', '[顶升] 结束（货架1）')
                    elif i == 180:
                        node.log('ok', '[任务完成] 货架1 \u2192 卸货区')
                    elif i == 190:
                        node.log('warn', '[下降] 开始（卸货区）')
                    elif i == 195:
                        node.log('info', '[下降] 结束（卸货区）')
                await asyncio.sleep(0.3)

                with node._gw_lock:
                    texts = [it['text'] for it in node._gw_logs]
                n_avoid = sum(1 for t in texts if '[避障]' in t)
                # 200 条按 5 秒窗口 → 约 40 条；给足余量 1~60
                ok(0 < n_avoid <= 60,
                   f'200 条避障被限流到 {n_avoid} 条（5 秒一条，不是全丢也不是全留）')
                ok(any('[顶升] 开始' in t for t in texts), '顶升开始留在日志里')
                ok(any('[顶升] 结束' in t for t in texts),
                   '顶升结束留在日志里（用户明确要求）')
                ok(any('[下降] 开始' in t for t in texts), '下降开始留在日志里')
                ok(any('[下降] 结束' in t for t in texts),
                   '下降结束留在日志里（用户明确要求）')
                ok(any('[任务完成]' in t for t in texts), '任务完成留在日志里')
                ok(len(texts) <= 120,
                   f'日志总量没被灌爆（{len(texts)} 条；旧实现会顶满 300）')

                # ── 跨节点转发：/rosout → 网页日志（成功绿 / 失败红）──
                #   ★ 这一步是本次改动最容易"看着对、其实没通"的地方：
                #     cargo_client 在**另一个进程**里打印，网关必须订阅 /rosout
                #     才收得到。所以这里真发一条 RosoutLog 消息，然后查
                #     node._gw_logs 里有没有、级别是不是 ok/error。
                if RosoutLog is not None:
                    def fake_rosout(name, text, level=20):
                        m = RosoutLog()
                        m.name = name
                        m.msg = text
                        m.level = level
                        return m

                    node._gw_logs.clear()
                    node.on_rosout(fake_rosout(
                        'task_manager_node', '[顶升] 成功（货架1）：已顶升，开始搬货', 20))
                    node.on_rosout(fake_rosout(
                        'task_manager_node', '[下降] 失败：搬货服务无应答（超时 5s）', 40))
                    node.on_rosout(fake_rosout(
                        'task_manager_node', '[导航中] 剩余 3.20 米 · 约 12 秒', 20))
                    node.on_rosout(fake_rosout(
                        'agv_web_gateway', '[顶升] 成功（自己发的，不该重复）', 20))
                    with node._gw_lock:
                        rl = list(node._gw_logs)
                    rtexts = [(it['level'], it['text']) for it in rl]
                    ok(any(lv == 'ok' and '顶升] 成功' in t for lv, t in rtexts),
                       f'★ 顶升成功被转发为绿字（ok 级），实际 {rtexts}')
                    ok(any(lv == 'error' and '下降] 失败' in t for lv, t in rtexts),
                       f'★ 下降失败被转发为红字（error 级），实际 {rtexts}')
                    ok(not any('[导航中]' in t for _lv, t in rtexts),
                       '★ Nav2 唠叨没被转发（否则降噪白做）')
                    ok(sum(1 for _lv, t in rtexts if '不该重复' in t) == 0,
                       '★ 网关自己的日志没被重复转发（否则每行出现两遍）')

                    # ── 跨通道去重：stdout 与 rosout 送同一条事件 ──
                    # ★ 真车日志里每件事都出现两遍（详情见 _norm_log 注释）。
                    #   这里模拟"stdout 先到（带节点前缀、级别只能判成 info）、
                    #   rosout 后到（正文、能判出 ok）"，要求最终只剩一条，
                    #   而且是绿字。
                    node._gw_logs.clear()
                    node._gw_log_recent.clear()
                    node.log('info', '[task_manager_node] [顶升] 成功（1号货架）：已顶升，开始搬货')
                    node.on_rosout(fake_rosout(
                        'task_manager_node', '[顶升] 成功（1号货架）：已顶升，开始搬货', 20))
                    with node._gw_lock:
                        rl = list(node._gw_logs)
                    hit = [it for it in rl if '顶升] 成功（1号货架）' in it['text']]
                    ok(len(hit) == 1,
                       f'★ 同一条事件只留一条（两条通道去重），实际 {len(hit)} 条：'
                       + str([it['text'] for it in hit]))
                    ok(hit and hit[0]['level'] == 'ok',
                       f'★ 去重后级别按"更醒目"合并，绿字不能丢（实际 '
                       f'{hit[0]["level"] if hit else "无"}）')

                # ── 仪表：固定 8 槽位 ──
                snap = node.snapshot()
                board = snap.get('board') or []
                keys = [b['key'] for b in board]
                expect = ['task', 'act', 'speed', 'avoid', 'pose', 'map', 'mode', 'link']
                ok(keys == expect,
                   f'★ 仪表固定 8 槽位且顺序恒定（实际 {keys}）')
                ok(all(b.get('text') for b in board),
                   '每个槽位都有文本（无数据时给占位"——"，不留空）')
                ok(all(len(b['text']) <= 14 for b in board),
                   '★ 每个值都够短（这是"不再看不全"的硬约束）：'
                   + str([(b['label'], len(b['text'])) for b in board]))
                # 动作格：没有任务时必须是"无动作"
                act = next((b for b in board if b['key'] == 'act'), {})
                ok(act.get('text') == '无动作',
                   f'空闲时动作格显示"无动作"（实际 {act.get("text")}）')

                # ── WS 上的 state 帧必须带 board ──
                got = False
                for _ in range(12):
                    m = await ws.receive_json(timeout=8)
                    if m.get('type') == 'state' and m['data'].get('board'):
                        got = True
                        keys2 = [b['key'] for b in m['data']['board']]
                        break
                ok(got, 'WebSocket 的 state 帧带 board（前端靠它渲染）')
                if got:
                    ok(keys2 == expect, 'WS 帧里也是固定 8 槽位')

                # ══ 反向验证：反复触发**同一条**事件日志，不能被折叠掉 ══
                #  "任务 1 完成 / 任务 2 完成" 归一化后是同一身份，
                #  但它们是不同事件，必须都留在日志里。
                print('\n  \u2500\u2500 反向验证：同类事件不能被去重吃掉')
                with node._gw_lock:
                    node._gw_logs.clear()
                for k in range(1, 6):
                    node.log('ok', f'[{k}/5] 搬运完成')
                with node._gw_lock:
                    ev = [it['text'] for it in node._gw_logs]
                ok(len(ev) == 5,
                   f'5 条"搬运完成"全部留在日志里（实际 {len(ev)} 条，'
                   f'被折叠就意味着历史里再也翻不到）')

                # ══ 换栈：仪表里"来自旧栈"的值必须清掉 ══
                #  ★ 不能笼统断言"7 行都空"：模式（nav/mapping）和连接数
                #    跟栈无关，换栈后照样成立，它们**应该**保持原值。
                #    要清的是"由旧栈数据算出来的那几格"：避障、速度、定位、地图。
                node.reset_runtime_cache('测试换栈')
                sn2 = {b['key']: b for b in (node.snapshot()['board'] or [])}
                ok(sn2.get('avoid', {}).get('text') == '无激光',
                   f'换栈后避障格回到"无激光"（实际 "{sn2.get("avoid", {}).get("text")}"）')
                ok(sn2.get('speed', {}).get('text') == '——',
                   f'换栈后速度格清空（实际 "{sn2.get("speed", {}).get("text")}"）')
                ok(sn2.get('map', {}).get('text') == '——',
                   f'换栈后地图格清空（实际 "{sn2.get("map", {}).get("text")}"）')
                ok(sn2.get('pose', {}).get('text') in ('未就绪', '——')
                   or sn2.get('pose', {}).get('text', '').startswith('丢失'),
                   f'换栈后定位格不再是"正常"（实际 "{sn2.get("pose", {}).get("text")}"）')
                with node._gw_lock:
                    ev2 = [it['text'] for it in node._gw_logs]
                ok(any('已清理上一套栈' in t for t in ev2),
                   '换栈事件本身留在日志里')

        return True
    finally:
        await runner.cleanup()
        try:
            ex.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        if old is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = old


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except Exception as exc:
        import traceback
        traceback.print_exc()
        fail_n += 1
        fails.append(f'测试过程异常：{type(exc).__name__}: {exc}')
    finally:
        print('\n' + '\u2500' * 62)
        print(f'  通过 {pass_n} 项，失败 {fail_n} 项')
        if fails:
            print('  失败项：')
            for f in fails:
                print(f'    \u00b7 {f}')
        print('\u2500' * 62)
    sys.exit(1 if fail_n else 0)
