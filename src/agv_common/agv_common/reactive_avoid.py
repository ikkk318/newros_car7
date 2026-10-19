"""
reactive_avoid —— 反应式避障仲裁（纯 Python，不依赖 ROS，可单独测试）。

用途（对应需求"靠近障碍减速 + 自主择向 + 远离后交还导航"）：
    给一条"导航想走的速度" (v_nav, w_nav) 和一圈激光距离，返回一条"实际该发"的
    速度 (v_out, w_out)。规则全部确定性，无随机——同一输入必得同一输出，
    便于复现、便于统计成功率、便于答辩解释。

三层逻辑（按优先级）：
    ① 距离缩放：看"前方扇区"最近障碍距离，越近线速度越低（近到临界直接为 0）。
    ② 择向接管：距离低于接管阈值且车速已很低时，比较左/右两侧扇区哪个更空，
       把角速度"偏"向空的那一侧（不是随机，是比大小）。
    ③ 缓回交还：障碍在远离（最近距离回升）时，角速度按固定步长平滑回到导航值，
       避免"一离开障碍就猛打方向"造成蛇形。

★ 两个"窄锥"是本模块的关键设计，都来自同一条实测教训：
  **扇区开得越宽，锥边界越斜，越容易扫到平行贴着车身的槽侧壁。**
  槽内净半宽 0.361 m、车半宽 0.25 m → 车心最多偏 0.111 m，近侧壁距车心 0.250 m。

  ① 接管闸门（`takeover_half_angle`，默认 20°）：只管"要不要夺向"。
     前扇区 45° 的锥边界命中侧壁 = 0.250/sin45° = 0.354 m ≤ take(0.45)，
     于是"前方很近"的假象骗开闸门；而左右侧壁距一比就超过偏置阈值 → 夺向。
     可两侧壁平行，往哪偏都是蹭墙 → 车在槽内反复摆、进槽姿态被搅乱。
     解析临界 h > asin(0.250/0.55) = 27.0°，实测（侧壁有尽头）临界 30°~35°，
     取 20° 留余量。实测 ±45° 时进槽 36%~47% 采样被夺向，收到 ±20° 后为 0。
     ★ 只收窄【闸门】，① 的距离缩放仍用 front_half_angle(45°)——槽内该减速慢行
       还是要减速（合理），只是不该被夺走转向。

  ② 后方窄锥（`rear_half_angle`，默认 20°）：倒车时另见下方倒车段说明。

  左带/右带本身没有问题：它们量的是横向净空（实测吻合 0.361 ∓ 偏离量），
  方向逻辑也对（偏离哪边远就往哪边回正）。它们只是"选边器"，**永远不会自己触发
  任何动作**——闸门不开，它们连参与的机会都没有。所以修闸门而不动它们。

★ 倒车（v_nav < 0）时看的是【车尾】：前方扇区描述的是"车要去的方向"，倒车时车在
  往后走，前方数据与去路无关，拿它减速/择向等于蒙眼开车——所以 ①② 里的前向判定
  对倒车豁免，改由后方窄锥（`rear_half_angle`，默认 20°）做同样的"近障减速 + 刹停"。
  真实后果（为什么必须豁免前向）：车在凵型槽内直退时前方恒为槽底挡板（永远"太近"），
  若不禁用择向，车身横向偏移符号一变（±0.08 m 足够）左右自由距离大小关系就翻转
  → 转向命令整个反号 → 车原地左右摇摆、退不出槽。
  真实后果（为什么后方锥必须窄）：槽内净宽 0.722 m，车心离两侧槽壁仅 0.361 m，
  锥一旦张开到能看见两侧壁，倒车就会被自己的减速逻辑压住甚至刹停 → 又变回"能进不能出"。
  实测（5 个槽位 × 整条倒车路径 × ±0.10 m 横向偏移 × ±3° 航向偏差）后方锥最小距离：
  ±20° → 1.20 m（干净）、±25° → 0.54 m（擦到侧壁）、±45° → 0.34 m（几乎刹停）。
  故取 20°，距临界留 5° 余量。

★ 原地对准（v_nav ≈ 0）时②也豁免，角速度原样交给上层。到点后 Nav2 只发角速度修航向，
  车不出溜，"择向绕障"无从谈起（绕障是挪动时的事）。若不豁免：槽内前方恒为槽底挡板、
  恒满足接管条件，Nav2 的对准角速度被换成固定 0.5 rad/s 的左右摇摆，航向永远进不了
  yaw_goal_tolerance → goToPose 不返回 → 任务层死等到 nav_timeout 报错。
  这就是"明明到了却以为没到、原地左右摇摆不停"的成因，且调容差无用——
  SimpleGoalChecker 的 stateful 一旦位置达标就只看航向。

设计取舍（为什么不做随机）：
    随机导致不可复现、无法统计门禁成功率、答辩讲不清。用"左右侧自由距离比较"
    得到方向，同样是"自主抉择"，但是确定性的，效果更稳。

坐标约定（REP-103）：x 向前、y 向左、z 向上；yaw 逆时针为正，单位弧度。
激光角度：0 = 正前方，正 = 逆时针（向左），负 = 顺时针（向右）。

★ 视场角无关：本模块只用 angle_min + i * angle_increment 推算每点角度，不假设点数，
  也不假设扫描范围。雷达从「前方 ±90°（181 点）」改成「360° 全向（361 点）」后，
  前/左/右三个扇区的判定完全不变；正后方（±90°~180°）本来就不参与任何扇区，
  所以倒车仍然"看不见去路"——这点由"倒车速度压得低"来兜，不是靠视场角。
"""

import math                                       # use 角度换算与三角函数


# 把角度归一化到 (-pi, pi]
# 参数：angle - 任意弧度
# 返回：归一化后的弧度
def _wrap(a):                                    # in 内部工具
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ReactiveAvoider:                           # out 反应式避障仲裁器
    # 初始化：读入全部可调参数（数值全部可从参数中心覆盖）
    # 参数：
    #   stop_distance      距障 ≤ 此值(m)：前方直接刹停（v=0）
    #   slow_distance      距障 ≤ 此值(m)：开始按比例减速（stop~slow 之间线性过渡）
    #   take_over_distance 距障 ≤ 此值(m)：进入"择向接管"，角速度交给本模块决定
    #   front_half_angle   前方扇区半张角(rad)：只统计该扇区内的最近距离
    #   side_max_angle     侧方最大角(rad)：左右两侧统计到该角度为止（默认 90°）
    #   slow_min_speed     减速过渡段的最小线速度(m/s)：不会减到 0 之外
    #   takeover_max_speed 进入择向接管所需的"车速上限"(m/s)：太快不接管，先减速
    #   yaw_handback_rate  远离障碍后角速度交还导航的步长(rad/s per 调用)
    #   side_clearance_margin 左右择向时的偏置(m)：空得更多的一侧再额外偏一点
    #   takeover_half_angle 接管闸门锥半张角(rad)：比 front_half_angle 更窄（默认 20°）
    #   rear_half_angle    后方扇区半张角(rad)：倒车时只看这个窄锥（默认 20°）
    # 返回：无
    def __init__(self,                                   # out 对外构造函数
                 stop_distance=0.30,
                 slow_distance=0.90,
                 take_over_distance=0.60,
                 front_half_angle=math.radians(45.0),
                 side_max_angle=math.radians(90.0),
                 takeover_half_angle=math.radians(20.0),
                 slow_min_speed=0.10,
                 takeover_max_speed=0.25,
                 yaw_handback_rate=0.30,
                 side_clearance_margin=0.10,
                 rear_half_angle=math.radians(20.0)):
        self.stop_distance = float(stop_distance)                # def 刹停距离
        self.slow_distance = float(slow_distance)                # def 减速起点距离
        self.take_over_distance = float(take_over_distance)      # def 接管距离
        self.front_half_angle = abs(float(front_half_angle))     # def 前方扇区半角
        self.side_max_angle = abs(float(side_max_angle))         # def 侧方最大角
        # ★ takeover_half_angle 必须比 front_half_angle 窄（实测结论，别随手开大）：
        #   前扇区 45° 的锥边界会打到【平行于车身的槽侧壁】。槽内净半宽 0.361 m、
        #   车半宽 0.25 m → 车心最多偏 0.111 m，此时近侧壁距车心 0.250 m；
        #   45° 射线命中 = 0.250/sin45° = 0.354 m ≤ take(0.45) → 闸门被侧壁骗开；
        #   而左侧壁距、右侧壁距一差就超过 0.10 的偏置阈值 → 接管夺向。
        #   可两侧壁是平行的，往哪偏都是蹭墙 → 车在槽内反复摆、进槽姿态被搅乱。
        #   解析临界：要看得见侧壁须 h > asin(0.250/0.55) = 27.0°；
        #   实测（真实地图、侧壁有尽头）临界在 30°~35° 之间。取 20° 留足余量。
        #   实测：±45° 时进槽 36%~47% 采样被夺向；收到 ±20° 后为 0。
        #   注意只把【闸门】收窄，① 的距离缩放仍用 front_half_angle(45°)——
        #   槽内该减速慢行还是要减速（那是合理的），只是不该被夺走转向。
        self.takeover_half_angle = abs(float(takeover_half_angle))  # def 接管闸门锥半角
        # 防呆：闸门锥宽于前锥就失去意义（等于没改），直接夹到前锥以内
        if self.takeover_half_angle > self.front_half_angle:
            self.takeover_half_angle = self.front_half_angle
        self.slow_min_speed = float(slow_min_speed)              # def 减速下限
        self.takeover_max_speed = float(takeover_max_speed)      # def 接管车速上限
        self.yaw_handback_rate = float(yaw_handback_rate)        # def 交还步长
        self.side_clearance_margin = float(side_clearance_margin)  # def 择向偏置
        self.rear_half_angle = abs(float(rear_half_angle))       # def 后方扇区半角（窄锥）
        # ★ rear_half_angle 为什么必须"窄"（实测结论，别随手开大）：
        #   槽内净宽 0.722 m，车心离两侧槽壁仅 0.361 m。车在槽内倒车时，两侧槽壁
        #   与车身平行、离得很近；后方锥一旦张开到能"看"到它们，倒车就会被自己的
        #   减速逻辑一直压着（甚至刹停）→ 又变回"能进不能出"。
        #   在实测地图上对 5 个槽位、整条倒车路径（含 ±0.10 m 横向偏移、±3° 航向
        #   偏差）做射线投射，量得后方锥的最小距离：
        #       ±15° / ±20° / ±22° → 最小 1.20 m（完全干净，>slow 0.55）
        #       ±25° → 0.54 m（开始擦到槽侧壁，触发减速）
        #       ±45° → 0.34 m（几乎刹停）
        #   故取 20°（距 25° 的临界留 5° 余量）。
        #   这不会漏掉真正的正后方障碍：车宽 0.5 m，只要它落在车后正对的那条
        #   0.5 m 宽走廊里，20° 锥在 0.72 m 内就能看到（0.361/0.72 ≈ 26.6°）。

        # 内部状态（跨调用记忆，用于"缓回"）
        self._engaged = False     # def 当前是否处于"择向接管"模式
        self._yaw = 0.0           # def 接管模式下本模块给出的角速度（用于缓回）

    # 扇区内最近距离（忽略无效读数）
    # 参数：ranges - 激光距离数组；angle_min - 起始角(rad)；angle_increment - 角步长(rad)
    #       center - 扇区中心角(rad)；half - 扇区半张角(rad)；range_max - 视为"无限远"的上限(m)
    # 返回：最近有效距离(m)；扇区内无有效读数时返回 range_max
    def _min_in_sector(self, ranges, angle_min, angle_increment,   # in 内部方法
                       center, half, range_max):
        return self._min_in_band(ranges, angle_min, angle_increment,
                                 center - half, center + half, range_max)

    # 角度带内最近距离：统计角度落在 [lo, hi]（可跨 ±pi）内的最近有效读数
    # 参数：ranges/angle_min/angle_increment 同激光；lo/hi - 带边界(rad)；range_max - 上限(m)
    # 返回：最近有效距离(m)；带内无有效读数时返回 range_max
    def _min_in_band(self, ranges, angle_min, angle_increment,   # in 内部方法
                     lo, hi, range_max):
        n = len(ranges)
        best = float(range_max)                      # def 当前最近距离（初值=最远）
        found = False                                # def 是否见过有效读数
        span = hi - lo
        for i in range(n):                           # # 逐点扫描
            r = ranges[i]                            # use 该点距离
            if not math.isfinite(r) or r <= 0.0:     # # 无效读数（inf/nan/0）跳过
                continue
            if r >= range_max:                       # # 超量程：视为无限远
                continue
            ang = angle_min + i * angle_increment    # use 该点角度（未归一化）
            d = _wrap(ang - lo)                      # use 相对下边界的有符号偏移
            if 0.0 <= d <= span:                     # # 落在 [lo, hi] 带内
                found = True
                if r < best:
                    best = r
        return best if found else float(range_max)   # use 无读数按"很远"处理

    # 主入口：给导航速度 + 激光，算出实际该发的速度
    # 参数：
    #   v_nav, w_nav  - 导航想要的速度(m/s, rad/s)
    #   ranges        - 激光距离数组(list/array)
    #   angle_min     - 激光起始角(rad)
    #   angle_increment - 激光角步长(rad)
    #   range_max     - 激光量程(m)，用作"无限远"上限
    #   dt            - 距上次调用的时间(s)，用于缓回的步长推进
    # 返回：(v_out, w_out, info) —— info 是 dict，便于调试/日志（含是否接管、前方距离等）
    def arbitrate(self, v_nav, w_nav, ranges, angle_min,        # out 对外接口
                  angle_increment, range_max=8.0, dt=0.1):
        v_nav = float(v_nav)                         # def 导航线速度
        w_nav = float(w_nav)                         # def 导航角速度
        range_max = float(range_max)                 # def 量程

        # ① 前方最近距离与左右两侧自由距离（左右侧带不含前方锥，避免重叠）
        front = self._min_in_sector(ranges, angle_min, angle_increment,   # use 前方扇区
                                    0.0, self.front_half_angle, range_max)
        # 接管【闸门】：比前扇区更窄，专用于"要不要夺向"这一个判断。
        #   为什么必须单独量而不是复用 front：前扇区 45° 的锥边界会扫到平行于车身的
        #   槽侧壁（距车心仅 0.36 m，锥边界 45° 时命中 0.51 m，车一偏就 ≤0.45），
        #   于是"前方很近"的假象骗开了闸门 → 车在槽内被反复夺向。
        #   窄锥只关心"正前方真有没有东西"，侧壁进不来（20° 时命中 ≥0.73 m）。
        front_gate = self._min_in_sector(ranges, angle_min, angle_increment,  # use 接管闸门
                                         0.0, self.takeover_half_angle, range_max)
        left = self._min_in_band(ranges, angle_min, angle_increment,      # use 左侧带（+）
                                 self.front_half_angle, self.side_max_angle, range_max)
        right = self._min_in_band(ranges, angle_min, angle_increment,     # use 右侧带（-）
                                  -self.side_max_angle, -self.front_half_angle, range_max)
        # 后方窄锥：只在倒车时参与判定（见 ②'）。360° 雷达下才可能真有数据；
        # 若雷达仍是半圈（无后方点），_min_in_band 会返回 range_max，等价于"通畅"。
        rear = self._min_in_band(ranges, angle_min, angle_increment,      # use 后方窄锥（跨 ±π）
                                 math.pi - self.rear_half_angle, math.pi, range_max)
        rear2 = self._min_in_band(ranges, angle_min, angle_increment,
                                  -math.pi, -math.pi + self.rear_half_angle, range_max)
        rear = min(rear, rear2)                                           # use 取两半中更近的

        # ② 线速度按"前方距离"缩放
        if front <= self.stop_distance:              # # 太近：直接刹停
            v_out = 0.0
        elif front >= self.slow_distance:            # # 够远：全速
            v_out = v_nav
        else:                                        # # 中间：线性过渡
            ratio = (front - self.stop_distance) / max(self.slow_distance - self.stop_distance, 1e-6)
            # ★ 外层 min(v_nav, ...)：slow_min_speed 只是"别爬得太慢"的下限，绝不能把
            #   上层要停的速度抬上去。否则 Nav2 到点原地对准时（v_nav=0）被硬塞
            #   0.10 m/s 前进 → 进度检查器判"没进展" → 反复恢复行为 → 到点前原地转。
            v_out = min(v_nav, max(self.slow_min_speed, v_nav * ratio))
        # 方向保护：导航若在后退（v_nav<0），前方减速不适用（后面看不见），交给下层限制
        #   ★ 注意这里只豁免了线速度；角速度的豁免在下面 ③ 的 want_takeover 里，
        #     两处必须成对，否则"倒车不减速但仍被抢转向"。
        if v_nav < 0.0:
            v_out = v_nav

        # ②' 后方保护（只在倒车时生效）：车在往后走，车尾才是"去路"。
        #   ★ 与 ② 严格对应：退车就把"看哪里"从车头换成车尾，而不是两边都看。
        #     若前进时也拿后方数据刹停，等于车往前跑却被身后的墙拽住，是错的。
        #   判据用 v_nav（非 v_out）：v_out 可能被前方减速下限抬动，不代表真实意图。
        #   rear 锥很窄（默认 20°），槽内两侧壁不在锥内（实测最小 1.20 m），
        #   所以槽内倒车不会被自己压住——这是"能进不能出"不复发的前提。
        if v_nav < 0.0:
            if rear <= self.stop_distance:           # # 车尾太近：刹停（防撞底线）
                v_out = 0.0
            elif rear < self.slow_distance:          # # 车尾接近：按比例压低倒车速度
                ratio = (rear - self.stop_distance) / max(
                    self.slow_distance - self.stop_distance, 1e-6)
                # 与前进段同构（那边是 min(v_nav, max(下限, v_nav*ratio))）：
                # 取"缩放值"与"下限"中幅度更大的，再夹住不超过上层给的幅度，
                # 最后取负号还原为倒车方向。
                mag = max(abs(v_nav) * ratio, self.slow_min_speed)   # use 别爬太慢
                v_out = -min(mag, abs(v_nav))                        # use 绝不放大

        # ③ 择向接管判定：够近 且 车速已降到接管阈值以下 → 接管角速度
        # ★ 倒车豁免（v_nav < 0）：前向扇区是"车要去的方向"，倒车时车在往后走，
        #   前方数据与去路无关，拿它择向等于蒙眼打方向。后果是：车在凵型槽内直退时，
        #   前方 0.38 m 恒为槽底挡板（永远"太近"），仲裁器持续抢转向；车身横向偏移
        #   符号一变（±0.08 m 足够），左右自由距离的大小关系就翻转，转向命令整个反号
        #   → 车在原地左右摇摆、退不出来。倒车时角速度原样交给上层（导航/恢复行为）。
        # ★ 原地对准豁免（v_nav ≈ 0）：到点后 Nav2 只发角速度做航向对准，此时车不前移、
        #   "择向绕障"没有意义（绕障是挪动时的事）。若不豁免，槽内前方 0.38 m 恒满足接管
        #   条件，Nav2 的对准角速度会被换成固定 0.5 rad/s 的左右摇摆，航向永远进不了
        #   yaw_goal_tolerance(0.20) → goToPose 不返回 → 任务层死等到 nav_timeout 报错，
        #   表现为"明明到了却以为没到、原地左右摇摆不停"。
        #   判据用 v_nav 而非 v_out：v_out 在近障过渡段会被减速下限抬成 0.10，
        #   那是"前向减速"的输出，不代表车真的在往前走。
        # ★ 闸门用 front_gate（窄锥）而非 front（45° 宽锥）：宽锥会被槽侧壁骗开，
        #   见 __init__ 里 takeover_half_angle 的说明与实测数据。
        creeping_forward = v_nav > 1e-3
        want_takeover = (creeping_forward
                         and front_gate <= self.take_over_distance
                         and abs(v_out) <= self.takeover_max_speed)

        if want_takeover:                            # # 进入/保持"择向"模式
            self._engaged = True
            # 左右哪边更空就往哪边偏（确定性：比大小，空得多的那侧）
            diff = left - right                      # def 左右自由距离差
            if abs(diff) < self.side_clearance_margin:    # # 差不多空：不额外偏向，保持原角速度方向
                target = w_nav if abs(w_nav) > 1e-3 else 0.0
            elif diff > 0.0:                         # # 左边更空：向左偏（+，逆时针）
                target = max(abs(w_nav), 0.5)        # use 至少给一个可观测的转向量
            else:                                    # # 右边更空：向右偏（-，顺时针）
                target = -max(abs(w_nav), 0.5)
            # 限幅到与原角速度同量级（由参数中心的角速度上限兜底在调用方）
            self._yaw = target
            w_out = target
        else:                                        # # 未接管：把角速度平滑交还导航
            if self._engaged:                        # # 刚从接管退出：按步长缓回
                step = self.yaw_handback_rate * max(float(dt), 0.0)
                if self._yaw > w_nav:
                    self._yaw = max(w_nav, self._yaw - step)
                else:
                    self._yaw = min(w_nav, self._yaw + step)
                if abs(self._yaw - w_nav) < 1e-3:    # # 已经回到导航值：彻底交还
                    self._engaged = False
                    self._yaw = w_nav
                w_out = self._yaw
            else:                                    # # 常态：原样跟随导航
                w_out = w_nav

        info = {                                     # def 调试信息
            'front': front, 'front_gate': front_gate,
            'left': left, 'right': right, 'rear': rear,
            'v_nav': v_nav, 'w_nav': w_nav,
            'v_out': v_out, 'w_out': w_out,
            'engaged': self._engaged,
        }
        return v_out, w_out, info

    # 是否处于择向接管状态（供外部判断/记录）
    # 参数：无
    # 返回：bool
    def engaged(self):                               # out 对外接口
        return self._engaged

    # 复位（如导航结束/急停后）：清空记忆状态
    # 参数：无
    # 返回：无
    def reset(self):                                 # out 对外接口
        self._engaged = False
        self._yaw = 0.0
