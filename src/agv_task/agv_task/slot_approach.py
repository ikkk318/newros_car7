"""
slot_approach —— 槽内点位的进槽策略（槽口对位 + 软到点判定）。

★ 为什么需要这个模块（实车/仿真都复现过的故障）：
    货架是「凵型槽」：槽内净宽 0.80 m，车体 0.50×0.50，半对角线 0.354 m。
    车开进槽里以后，四周只剩 ~0.19 m 余量，**原地纠偏角度是做不到的**
    （代价地图膨胀后更做不动）。而 Nav2 的 SimpleGoalChecker 要求
    「位置 ≤ 0.15 m 且 航向 ≤ 0.20 rad」才返回成功。

    于是只要进槽时航向偏了 0.2 rad 以上，就会进入死循环：
        到点 → 差一点点角度 → DWB 原地转 → 转不动/转过了
        → 进度检查器 10 秒判「没有进展」→ 恢复行为 spin/backup
        → 槽内 spin/backup 全被障碍挡住 → 角度越转越偏（实测偏到 138°）
        → 140 秒后 Goal failed，顶升/下降根本没机会执行。

    工程上的正解不是"让控制器更努力地转"，而是两条：
      ① **把纠偏动作挪到槽外**：先到槽口对位点（X号货架，周围有空间），
         再从槽口直着开进去 —— 最后 1.37 m 是直线，本来就不需要转身。
      ② **槽内的到点判据以位置为主**：位置到了（≤0.12 m）、角度差在
         可接受范围（默认 ≤0.35 rad ≈ 20°）并稳定 0.6 秒，就判到达，
         让搬货服务立刻执行，不再跟 Nav2 的 0.20 rad 死磕。

    本模块只做纯逻辑（命名约定 + 到点判据），不碰 ROS，
    这样判据可以被单测钉死（见 test_slot_approach.py）。

★ 为什么槽内点还能容忍 20°：
    顶升是一块平板往上顶，不是销孔插合；20° 对应的车体横向多占
    0.5*sin20° ≈ 0.17 m，槽内净宽 0.80 m 仍然放得下。
    再大就会顶偏货物，所以留 0.35 rad 上限，超出宁可报错也不硬顶。
"""

# 点位命名约定（waypoints.yaml 就是这么存的）：
#   「1号槽内」= 钻进槽、停在货物正下方的顶升点
#   「1号货架」= 槽口外的对位点，车头正对槽口
# 这两个名字只差后缀，所以槽内点的对位点可以直接推出来。
SLOT_SUFFIX = '槽内'
MOUTH_SUFFIX = '货架'

# 到点判定的默认值（可由 ROS 参数覆盖）
DEFAULT_ARRIVE_XY = 0.12      # 位置容差(m)
DEFAULT_ARRIVE_YAW = 0.35     # 角度容差(rad) ≈ 20°
DEFAULT_ARRIVE_HOLD = 0.6     # 连续满足多久才算真到（s）：防"路过即到点"
DEFAULT_STUCK_YAW = 0.75      # 角度差超过 43° 且一直不变 → 判"槽内转不动"
DEFAULT_STUCK_SEC = 8.0       # 上面这个判据要连续观察多久（s）


def is_slot_point(name):
    """点名是不是槽内点（只看后缀，不看 kind：kind 只区分搬货/导航）。"""
    return str(name or '').strip().endswith(SLOT_SUFFIX)


def mouth_name(name):
    """由槽内点推出槽口对位点名；不是槽内点返回 None。

    「1号槽内」→「1号货架」。推导失败（用户把点位改名了）时返回 None，
    调用方退回"直接进槽"的老行为，不会因为找不到点而卡死。
    """
    text = str(name or '').strip()
    if not text.endswith(SLOT_SUFFIX) or len(text) <= len(SLOT_SUFFIX):
        return None
    return text[:-len(SLOT_SUFFIX)] + MOUTH_SUFFIX


def wrap_angle(angle):
    """把角度归一化到 (-pi, pi]，比较航向差时必须用它。"""
    import math
    return math.atan2(math.sin(angle), math.cos(angle))


class SlotArrive:
    """槽内到点判定器：喂当前位置，返回 wait / arrive / stuck。

    ★ 为什么要有 hold（连续稳定时长）：
      0.5 m/s 的车穿过 0.12 m 的圆只需要 0.24 s。若一进圈就判到达，
      会在地图选点/路线经过时误判。要求连续 0.6 s 都在圈内，
      穿过的情况必然来不及满足；真到点的车会停住，轻松满足。

    ★ 为什么要有 stuck（转不动）分支：
      万一还是以很大的角度进了槽（换点位、地图变动、人工拖车），
      控制器会一直原地磨。与其磨 140 秒再报"Goal failed"，
      不如 8 秒就明确告诉用户"槽内纠不了角度"——这是可行动的报错，
      而不是让人对着日志猜。
    """

    def __init__(self, xy=DEFAULT_ARRIVE_XY, yaw=DEFAULT_ARRIVE_YAW,
                 hold=DEFAULT_ARRIVE_HOLD, stuck_yaw=DEFAULT_STUCK_YAW,
                 stuck_sec=DEFAULT_STUCK_SEC):
        self.xy = float(xy)
        self.yaw = float(yaw)
        self.hold = float(hold)
        self.stuck_yaw = float(stuck_yaw)
        self.stuck_sec = float(stuck_sec)
        self.reset()

    def reset(self):
        """换目标点时清空计时状态（同一个实例不要跨点复用）。"""
        self._ok_since = None      # 连续满足容差的起始时刻
        self._in_since = None      # 连续处在位置圈内的起始时刻
        self._best = None          # 进圈以来最小角度差
        self._best_at = None       # 最小角度差被刷新的时刻

    def feed(self, x, y, yaw, target, now):
        """喂一帧位姿。

        参数：x, y, yaw - 当前位姿（map 系）；target - (tx, ty, tyaw)；now - monotonic 秒
        返回：(verdict, dist, abs_dyaw)
              verdict = 'wait' | 'arrive' | 'stuck'
        """
        import math
        tx, ty, tyaw = target
        dist = math.hypot(float(tx) - float(x), float(ty) - float(y))
        dyaw = abs(wrap_angle(float(tyaw) - float(yaw)))

        if dist > self.xy:
            # 位置没到：所有"在圈内"的计时全部作废
            self._ok_since = None
            self._in_since = None
            self._best = None
            self._best_at = None
            return 'wait', dist, dyaw

        # 位置在圈内：开始/继续观察
        if self._in_since is None:
            self._in_since = float(now)
            self._best = dyaw
            self._best_at = float(now)
        elif dyaw < self._best - 0.05:
            # 角度真的在收敛（至少改善 0.05 rad 才算），刷新"还在努力"的时间戳
            self._best = dyaw
            self._best_at = float(now)

        if dyaw <= self.yaw:
            if self._ok_since is None:
                self._ok_since = float(now)
            elif (float(now) - self._ok_since) >= self.hold:
                return 'arrive', dist, dyaw
        else:
            self._ok_since = None

        # 位置到了、角度差很大、且连续 stuck_sec 毫无改善 → 判"转不动"
        if (dyaw > self.stuck_yaw and self._best_at is not None
                and (float(now) - self._best_at) >= self.stuck_sec):
            return 'stuck', dist, dyaw

        return 'wait', dist, dyaw
