# AgvCommand.msg —— 统一指令总线（控制台 / 语音 / 调度共用）
# ------------------------------------------------------------------
# 各入口把操作都转成这一条消息发到 /agv/command，由 task_manager_node 统一消费。
#
# command 取值：
#   goto         args = [点名] 或 [x, y, yaw]
#   start_task   args = [取货点名, 放货点名]
#   cancel_task  取消当前任务（args 空）
#   stop         急停（args 空，任何状态都受理）
#   resume       急停/暂停后恢复（args 空）
#
# route：可选路线名。给了就做"多点导航"——
#   goto       → 路线各点 → 目标点
#   start_task → 取货点 → 路线各点 → 放货点
#   空串表示不用路线，仍是单点/两点导航。

string   source        # 指令来源："web" | "voice" | "scheduler"
string   command       # 动作名：goto | start_task | cancel_task | stop | resume
string[] args          # 参数：点名，或坐标 [x, y, yaw]（按字符串传入）
string   route         # 可选：本次要经过的路线名（空=不用路线）
float32  confidence    # 置信度（语音用 0~1；其他入口填 1.0）
string   request_id    # 请求唯一号：用于去重与追踪
