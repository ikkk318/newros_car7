# TaskState.msg —— 任务状态（控制台状态面板 / 语音播报订阅）
# ------------------------------------------------------------------
# task_manager_node 发布到 /agv/task_state。
#
# state 取值：
#   IDLE       空闲：可接新任务
#   GO_LOAD    前往取货点（带路线时依次经过路线各点）
#   LIFTING    顶升中（调 /agv/cargo 搬货服务）
#   TRANSPORT  运输中：已顶升，前往放货点（带路线时经过路线各点）
#   LOWERING   下降放货中
#   DONE       任务完成（随后自动回 IDLE）
#   ERROR      异常（导航失败 / 搬货服务失败），需人工复位
#   PAUSED     暂停：收到 stop（急停）后冻结，可 resume 恢复

string  task_id          # 任务唯一号（一次完整搬运一个）
string  state            # 状态串：见上方取值表
string  target_location  # 当前目标点名（取货点 / 路线点 / 放货点）
float32 progress         # 任务进度 0~1
string  message          # 中文提示，可直接显示
