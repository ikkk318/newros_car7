"""
cargo_client —— 搬货服务客户端（顶升 / 下降）。

★ 全项目唯一与"搬货机器"打交道的入口。
现阶段服务内部只维护"搬货中"标志并立即返回成功；
接真实硬件时改 agv_lift 的服务实现即可，这里不用动。

从原 task_manager_node 里抽出来的理由：原节点里 `_cargo_step` 与导航、状态上报
混在一起，而它的失败语义（服务不在 / 超时 / ok=false）与导航完全不同，
单独放这里便于把"搬货失败怎么办"讲清楚。

等待策略与 store_client 一致：短轮询 future，超时就判失败，
不使用阻塞式 spin_until_future_complete（会把回调组卡住）。
"""

import time                              # use 超时轮询


class CargoClient:                       # out 搬货客户端
    # 初始化
    # 参数：node - 宿主节点；srv_type - CargoCmd 服务类型；name - 服务名
    #       timeout_s - 单次等待上限(s)；abort_event - 中止标志
    # 返回：无
    def __init__(self, node, srv_type, name='/agv/cargo',
                 timeout_s=5.0, abort_event=None):   # out 对外构造函数
        self.node = node                 # def 宿主节点
        self.timeout_s = float(timeout_s or 5.0)     # def 等待上限
        self.abort = abort_event         # def 中止标志
        self._cli = node.create_client(srv_type, name)   # io 建客户端
        self._srv_type = srv_type        # def 服务类型（建请求用）

    # 是否被请求中止
    # 参数：无
    # 返回：bool
    def aborted(self):                   # in 内部方法
        return self.abort is not None and self.abort.is_set()

    # 调一次搬货服务
    # 参数：cmd - 'lift' / 'lower'；tag - 日志标记（点名）
    # 返回：True=受理成功
    #
    # ★ 日志措辞约定（2026-10-18 用户要求"打印请求结果，成功绿字失败红字"）：
    #     成功 → '[顶升] 成功（取货点）：已顶升，开始搬货'   ← 网页绿色
    #     失败 → '[顶升] 失败：/agv/cargo 搬货服务不可用'    ← 网页红色
    #   网关的跨节点转发（on_rosout）靠**文本**判绿/红，所以：
    #     · 成功必须含"成功/结束/完成"，且**不能**含"失败/超时/拒绝"；
    #     · 失败必须含"失败"，且用 log.error（ERROR 级 → 网页红字）。
    #   改这里的中文措辞会直接影响网页颜色，别随手改。
    def step(self, cmd, tag=''):         # out 对外接口
        log = self.node.get_logger()
        zone = _CMD_ZH.get(cmd, cmd)     # def 日志里的中文动作名
        where = f'（{tag}）' if tag else ''   # def 点位后缀
        if self.aborted():               # # 已取消/急停：不再动作
            # 不算失败（用户主动中止），但也要留痕，否则"顶升怎么没动静"无从查
            log.warn(f'[{zone}] 已取消：任务中止，未请求搬货服务{where}')
            return False
        if not self._cli.wait_for_service(timeout_sec=3.0):
            log.error(f'[{zone}] 失败：/agv/cargo 搬货服务不可用'
                      f'（agv_lift 没起？）{where}')
            return False
        req = self._srv_type.Request()   # def 请求
        req.command = str(cmd)
        req.request_id = f'{cmd}-{int(time.time() * 1000)}'   # use 便于对日志追溯
        future = self._cli.call_async(req)     # io 异步调用
        deadline = time.monotonic() + self.timeout_s
        while not future.done() and time.monotonic() < deadline:
            if self.aborted():           # # 等待期间被取消：放弃
                log.warn(f'[{zone}] 已取消：等待应答时任务中止{where}')
                return False
            time.sleep(0.02)
        res = future.result() if future.done() else None
        if res is None:                  # # 超时
            log.error(f'[{zone}] 失败：搬货服务无应答（超时 {self.timeout_s:.0f}s）'
                      f'{where}')
            return False
        if not res.ok:                   # # 服务明确拒绝
            log.error(f'[{zone}] 失败：{getattr(res, "message", "") or "服务拒绝"}'
                      f'{where}')
            return False
        # 成功：一条说清"哪个动作、哪个点位、服务回了什么"
        detail = getattr(res, 'message', '') or ''
        log.info(f'[{zone}] 成功{where}' + (f'：{detail}' if detail else ''))
        return True


# 动作英文 → 中文。只用于日志措辞，**不要**拿它做业务判断
# （业务判断一律用 'lift' / 'lower'，避免被本地化改动影响）。
_CMD_ZH = {'lift': '顶升', 'lower': '下降'}
