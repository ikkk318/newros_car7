"""
goto_pose —— 给目标点前往目标点的函数库（无 main，供其他代码 import 使用）。

用法（在某个节点里）：
    from nav2_simple_commander.robot_navigator import BasicNavigator
    from agv_task.goto_pose import goto_pose
    navigator = BasicNavigator()
    goto_pose(navigator, 3.0, 1.0, 0.0)     # 开往 (3.0, 1.0)，朝向 0

原理：走 Nav2 的动作接口 /navigate_to_pose（由 BasicNavigator 的 goToPose 封装）。
过程持续读取动作反馈（feedback），周期性打印关键数据：
    当前位置、已用时间、预计剩余时间、剩余距离、恢复次数。
结束后返回动作结果（TaskResult 枚举）。

坐标系约定（REP-103）：x 向前、y 向左、z 向上；yaw 逆时针为正，单位弧度。
"""
import threading                                    # use 把阻塞等待放到旁路线程
import time                                         # use 节流打印计时

from geometry_msgs.msg import PoseStamped           # def 位姿消息类型
from nav2_simple_commander.robot_navigator import TaskResult   # use 导航结果枚举
from rclpy.duration import Duration                 # def 时长类型（解析反馈时长）
from tf_transformations import (                    # use 欧拉角/四元数互转
    quaternion_from_euler)


# 结果枚举 → 中文说明（打印用）
RESULT_TEXT = {                                     # def 结果码中文对照表
    TaskResult.SUCCEEDED: '成功',
    TaskResult.CANCELED: '被取消',
    TaskResult.FAILED: '失败',
}


# 位姿合成：x, y, yaw → 指定坐标系下的 PoseStamped
# 参数：x, y - 位置(m)；yaw - 朝向(rad)；frame_id - 坐标系名（默认 map）
# 返回：PoseStamped
def make_goal_pose(x, y, yaw, frame_id='map'):      # out 对外接口
    pose = PoseStamped()                            # def 位姿消息
    pose.header.frame_id = frame_id                 # def 位姿所属坐标系
    pose.pose.position.x = float(x)                 # def 位置 x
    pose.pose.position.y = float(y)                 # def 位置 y
    pose.pose.position.z = 0.0                      # def 平面车，z 恒为 0
    q = quaternion_from_euler(0.0, 0.0, float(yaw))   # use 绕 z 的偏航角转四元数
    pose.pose.orientation.x = q[0]                  # def 四元数 x
    pose.pose.orientation.y = q[1]                  # def 四元数 y
    pose.pose.orientation.z = q[2]                  # def 四元数 z
    pose.pose.orientation.w = q[3]                  # def 四元数 w
    return pose                                     # use 返回合成位姿


# 前往目标点：等 Nav2 就绪 → 发目标 → 循环读反馈打印关键数据 → 返回结果
# 参数：navigator - BasicNavigator 实例（已 rclpy.init，自带后台 executer 转圈）
#       x, y    - 目标位置(m)；yaw - 目标朝向(rad)
#       frame_id - 坐标系（默认 map）
#       print_period - 反馈打印间隔(s)，默认 1.0，避免刷屏
#       logger  - 可选 rclpy logger；不给则用 print
#       timeout_s - 总超时(s)：超过则取消目标并按失败返回；<=0 表示不限时
#       abort_event - 可选 threading.Event；一旦置位就立刻放弃并返回 CANCELED
#       arrive_check - 可选回调：返回 None 表示"还没到"，返回 dict 表示"上层判定到达"
#                      dict = {'kind': 'arrive'|'fail', 'text': '要打印的中文说明'}
#                      ★ 为什么允许上层自己判到达（槽内点位）：Nav2 的到点判据是
#                        「位置 0.15 m 且航向 0.20 rad」，而凵型槽里车根本转不动，
#                        0.2 rad 的航向差会让它永远"差一点"，原地磨到任务超时。
#                        把槽内的判据交给任务层（位置为主、角度放宽），
#                        搬货服务才等得到执行。见 slot_approach.py。
# 返回：TaskResult 结果枚举（SUCCEEDED/CANCELED/FAILED）
#
# ★ 为什么要 abort_event（真机 bug：点了中止，车还在往前冲）
#   原来只有 NavRunner.step() 在下发**前**检查一次中止标志，goto_pose 内部
#   一概不管。而 goto_pose 的第一句 waitUntilNav2Active() 是阻塞的：
#   第一次导航要等 amcl/bt_navigator 全部激活，真机上常要 3~10 秒。
#   如果用户正好在这段里点了「中止」：
#       handle_cancel → _abort_task.set() → nav.cancel()
#   但那时 Nav2 还没有目标可取消，cancel 是空操作；等 waitUntilNav2Active()
#   返回后，goto_pose **照样** goToPose —— 于是"已取消任务并停车"的日志
#   刚打完，20 毫秒后车又开出去了。
#   修法：把这个等待也做成可中止的，并在每个关键节点复查标志。
def goto_pose(navigator, x, y, yaw, frame_id='map', print_period=1.0, logger=None,
              timeout_s=0.0, abort_event=None, arrive_check=None):   # out 对外接口
    # 说明：不能把 logger.info 缓存成方法再调用——rclpy 会在切级别时报
    # "Logger severity cannot be changed between calls"。这里用一个闭包按需输出。
    def _log(text):                                 # in 内部输出函数
        if logger is not None:                      # # 有 logger 打 ROS 日志
            logger.info(text)
        else:                                       # # 没有则退回 print
            print(text)

    def _aborted():                                 # in 内部：是否已被要求中止
        return abort_event is not None and abort_event.is_set()

    # 等 Nav2 就绪：不能直接死等（见上面的注释），拆成"边等边看标志"。
    # waitUntilNav2Active 自身没有超时/中断参数，所以放在线程里跑，
    # 主循环只负责轮询——这样点到中止能立刻返回，不必等它自然结束。
    if abort_event is not None:
        waiter = threading.Thread(
            target=navigator.waitUntilNav2Active, daemon=True)
        waiter.start()
        while waiter.is_alive():
            if _aborted():                          # # 等待期间被取消
                _log('[导航] 等待 Nav2 就绪时收到中止，放弃本次导航')
                return TaskResult.CANCELED
            time.sleep(0.05)
        if _aborted():                              # # 刚好卡在就绪那一刻被取消
            return TaskResult.CANCELED
    else:
        navigator.waitUntilNav2Active()             # io 阻塞等 Nav2 全部节点激活

    if _aborted():                                  # # 就绪后、发目标前再复查一次
        _log('[导航] 下发前已被取消，不发送导航目标')
        return TaskResult.CANCELED

    goal = make_goal_pose(x, y, yaw, frame_id)      # use 合成目标位姿
    _log(f'发送导航目标：x={x:.2f} 米，y={y:.2f} 米，yaw={yaw:.2f} 弧度（坐标系 {frame_id}）')
    navigator.goToPose(goal)                        # io 通过动作接口发送导航目标

    start_time = time.monotonic()                   # def 开始计时（算实际耗时）
    last_print = 0.0                                # def 上次打印时刻（节流用）
    last_left = None                                # def 上次打印时的剩余距离(m)
    last_rec = None                                 # def 上次打印时的恢复次数

    # 动作未结束前持续读反馈（feedback）并周期性打印关键数据
    #
    # ★★ 为什么不再"每 print_period 秒打一条"（用户："日志打印速度太快
    #    根本看不过来"）：
    #    这条 [导航中] 是整份日志里最多的（实测 221 行日志里 32 条），
    #    每条约 90 字。一趟 10 分钟的导航就是 600 条 × 90 字，
    #    把环形缓冲冲得只剩"当前位置 x=1.23"。
    #    但完全不打也不行：车卡住时用户需要知道"它到底在往哪挪"。
    #    所以改成**事件触发**：满足下面任一条才打一条 —
    #      ① 距上次打印满 _FB_GAP_MIN 秒，**且**剩余距离变化 ≥ _FB_MOVE_M
    #         （原地不动时不打：这正是刷屏的来源）；
    #      ② 恢复次数变了（卡住→自恢复，是必须知道的事件）；
    #      ③ 距上次打印超过 _FB_ALIVE_S 秒（哪怕没挪动，也要留个"还活着"的痕迹）。
    #    静止不动的导航（等障碍、等电梯）从此一条都不打。
    #
    # ★★ 为什么"距离变化 ≥0.3 米"这版降噪在真车上完全失效（实测踩到）：
    #    车正常跑的速度就是 0.3 m/s 上下，而 print_period 默认 1.0 秒 ——
    #    每秒走 0.30~0.32 米，**每一帧都刚好越过 0.3 米阈值**，
    #    于是 moved 恒真、quiet 恒真，退化成 1 Hz 原样刷屏。
    #    截取的实车日志：13.54→13.20→12.88→12.58→12.28→11.98→11.66→11.35，
    #    8 条全在 11 秒内，一条不少。
    #    结论：判据不能只看"变了多少"，必须**同时**给足最小间隔；
    #    阈值也要留出采样抖动余量（0.8 米 = 约 2.5 秒的行程）。
    while not navigator.isTaskComplete():           # # 动作未完成则继续循环
        now = time.monotonic()                      # use 当前时刻
        # ★ 上层到点判定（槽内点位）：放在读反馈之前，早一帧判到就早一帧收工。
        #   判到以后必须 cancelTask：否则 Nav2 还在原地纠偏，车会一边"已到点"
        #   一边继续转，顶升动作与转动叠在一起。
        if arrive_check is not None:
            note = None
            try:
                note = arrive_check()
            except Exception:
                note = None          # 判据自身出错不能拖垮导航，退回 Nav2 判据
            if note:
                text = str(note.get('text') or '')
                if text:
                    _log(text)
                try:
                    navigator.cancelTask()          # io 取消目标（顺带停车）
                except Exception:
                    pass
                if note.get('kind') == 'fail':
                    return TaskResult.FAILED
                # 取消到速度平滑器真正发零速还有一拍，等住再交回任务层，
                # 否则顶升会在车还在溜的瞬间执行。
                time.sleep(0.4)
                return TaskResult.SUCCEEDED
        feedback = navigator.getFeedback()          # use 读取动作反馈
        if feedback is not None:
            left = _feedback_left(feedback)         # def 剩余距离(m) 或 None
            rec = _feedback_recoveries(feedback)    # def 恢复次数 或 None
            if _should_report(now, last_print, left, last_left, rec, last_rec,
                              print_period):
                last_print = now                    # use 更新打印时刻
                last_left = left
                last_rec = rec
                _log(_format_feedback(feedback))    # use 解析并打印反馈关键数据
        if _aborted():                              # # 行驶途中被取消：主动取消目标
            _log('[导航] 收到中止，取消当前导航目标')
            try:
                navigator.cancelTask()              # io 取消当前目标
            except Exception:
                pass
            time.sleep(0.2)
            return TaskResult.CANCELED
        if timeout_s > 0.0 and (now - start_time) >= timeout_s:    # # 总超时：主动取消
            _log(f'[导航超时] 超过 {timeout_s:.0f} 秒仍未到达，取消目标并判失败')
            try:
                navigator.cancelTask()              # io 取消当前目标
            except Exception:
                pass
            time.sleep(0.3)
            return TaskResult.FAILED               # use 按失败返回，交由上层处理
        time.sleep(0.1)                             # use 让出 CPU，避免空转

    result = navigator.getResult()                  # use 读取动作最终结果
    elapsed = time.monotonic() - start_time         # use 实际总耗时（s）
    text = RESULT_TEXT.get(result, f'未知状态({result})')   # use 结果码转中文
    # ★ 只在"非成功"时打这一条（用户："日志打印速度太快"）。
    #   成功的情况下面 task_flow 紧接着会打 `[任务完成] A → B`，
    #   `[导航结束] 结果：成功 | 实际耗时 12.3 秒` 是纯重复 ——
    #   实测 221 行日志里 16 条全是这句。
    #   失败/取消则必须打：那是"为什么没到"的唯一线索。
    if result != TaskResult.SUCCEEDED or _aborted():
        _log(f'[导航结束] 结果：{text} | 实际耗时 {elapsed:.1f} 秒')
    # ★ 停稳那一刻被取消也算取消：交给上层按"中止"处理，别报成"成功"，
    #   否则任务线程会继续顶升/下降，用户以为没停下来。
    if _aborted():
        return TaskResult.CANCELED
    return result                                   # use 返回结果枚举


# ── [导航中] 打印节流参数 ──
# ★ 这三个是"实测调出来的"，不是凭感觉定的：
#   _FB_GAP_MIN 8 秒 —— 车 0.3 m/s 时 8 秒走 2.4 米，足够看出进度，
#                      又不会把 10 分钟的导航写成 600 行；
#   _FB_MOVE_M  0.8 米 —— 比 0.3 米留出采样抖动余量。原来定 0.3 恰好等于
#                      "每秒的行程"，导致每一帧都判定为"动了"，降噪完全失效；
#   _FB_ALIVE_S 20 秒 —— 车卡住（等障碍/等电梯）时也要留个"还活着"的痕迹，
#                      否则用户看到 20 秒没日志会以为程序死了。
_FB_GAP_MIN = 8.0
_FB_MOVE_M = 0.8
_FB_ALIVE_S = 20.0


def _should_report(now, last_print, left, last_left, rec, last_rec,
                   print_period):
    """这一帧要不要打一条 [导航中]？

    ★ 为什么抽成纯函数（而不是写在循环里）：
      这个判据已经改错两次 —— 第一版 1 Hz 原样刷屏，第二版"距离变化
      ≥0.3 米"在 0.3 m/s 的车速下每帧都成立，等于没改。判据本身必须能
      被单独测（见 test_goto_feedback.py），否则只能靠真车跑一趟 10 分钟
      导航去数日志条数。

    参数：
        now          当前 monotonic 时刻
        last_print   上次打印时刻
        left         本帧剩余距离(m)；None = 取不到
        last_left    上次打印时的剩余距离(m)
        rec          本帧恢复次数；None = 取不到
        last_rec     上次打印时的恢复次数
        print_period 配置的打印间隔(s)（会被 _FB_GAP_MIN 抬到下限）
    返回：bool
    """
    # 第一帧必打：连一次基线都没有时，用户最想知道"开始走了，还剩多少"。
    # ★ 不要靠 (now - last_print) 判断首帧 —— 那要求调用方传进来的 now 是个
    #   够大的 monotonic 值，测试里用 0 就永远打不出来（这类"只在该环境
    #   成立"的隐式前提最容易埋雷）。
    if last_left is None:
        return True
    # 取不到距离时按"有可能变了"处理：宁可多打一条，也不能一直憋着不吭声
    moved = left is None or abs(left - last_left) >= _FB_MOVE_M
    # ★ 最小间隔取 max(配置值, 硬下限)：配置成 1.0 也不能真的 1 Hz
    quiet = (now - last_print) >= max(float(print_period), _FB_GAP_MIN)
    alive = (now - last_print) >= _FB_ALIVE_S
    return bool(quiet and (moved or alive or rec != last_rec))



# 从反馈里取"剩余距离"，取不到返回 None
# 参数：feedback - NavigateToPose 的动作反馈
# 返回：float 或 None
#
# ★ 为什么要单独取而不是在 _format_feedback 里解析字符串：
#   节流判据要拿它做数值比较（>=0.3m 才算"真的在挪"），
#   从拼好的中文串里再正则抠数字太脆（格式一改就失效）。
def _feedback_left(feedback):                       # in 内部辅助
    try:
        return float(feedback.distance_remaining)
    except (AttributeError, TypeError, ValueError):
        return None


# 从反馈里取"恢复次数"，取不到返回 None
# 参数：feedback - NavigateToPose 的动作反馈
# 返回：int 或 None
def _feedback_recoveries(feedback):                 # in 内部辅助
    try:
        return int(feedback.number_of_recoveries)
    except (AttributeError, TypeError, ValueError):
        return None


# 解析动作反馈，拼成一行关键数据（字段可能为空，做保护，防打印时崩溃）
# 参数：feedback - NavigateToPose 的动作反馈
# 返回：可打印的字符串
#
# ★ 为什么从 90 字压到 40 字以内：
#   原来这行是
#     "[导航中] 当前位置 x=1.23 米 y=4.56 米 yaw=-0.78 (-44.7°) |
#      已用 12.3 秒，预计剩余 45.6 秒 | 剩余距离 3.21 米 | 恢复次数 0"
#   手机上要折 3 行，而真正回答"还要多久、还有多远"的只有两个数。
#   x/y/yaw 在网页地图上本来就有（小车图标 + 坐标显示），重复。
#   现在只留：剩余距离 / 预计剩余 / 恢复次数（有恢复才提）。
def _format_feedback(feedback):                     # in 内部辅助
    try:
        remain = Duration.from_msg(                 # use 预计剩余时间（s）
            feedback.estimated_time_remaining).nanoseconds / 1e9
        left = float(feedback.distance_remaining)   # def 剩余距离（m）
        rec = int(feedback.number_of_recoveries)    # def 自恢复次数
        text = f'[导航中] 剩余 {left:.2f} 米'
        # ★ 时间只在"算出来了"时才显示。
        #   刚发出目标时 Nav2 的 estimated_time_remaining 还是 0，真车日志
        #   里就会打出 "剩余 13.54 米 · 约 0 秒" —— 两个数字自相矛盾，
        #   用户第一反应是"导航算错了/程序坏了"。
        #   用 `1.0 <= remain < 86400` 一次挡掉三种脏值：
        #     0（还没算）、nan（比较恒为 False）、inf（超上限）。
        if 1.0 <= remain < 86400.0:
            text += f' · 约 {remain:.0f} 秒'
        # 恢复次数只在真发生过时才提：0 次的绝大多数行程不必占位置，
        # 一旦 >0 说明车卡过（自恢复），必须让人看见。
        if rec > 0:
            text += f' · 已恢复 {rec} 次'
        return text
    except Exception as e:                          # # 字段还没填充完整：降级打印
        return f'[导航中] 反馈数据暂不完整（{e}）'
