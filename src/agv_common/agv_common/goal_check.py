"""
goal_check —— 目标点合法性的**唯一判据**（纯 Python，不依赖 ROS，可单独测试）。

要解决的问题（原设计里的一处真实矛盾）：
    同一个"这个目标点能不能去"，原先有两个判据各判各的：
      ① 指令门卫 CommandRouter：按 task.yaml 的 field_min/max 矩形范围判；
      ② 点位预检 MapChecker   ：按实测栅格地图判（图外 / 压障碍 / 净空不足）。
    两个范围不一样（field_* 比实测地图大一圈，x 正方向差得最多：
    field_* 放到 10.45，实测图只到 4.15）。于是同一个坐标会出现
    「门卫放行、预检拒绝」——同一件事两种结论，用户看到的是"有时能发、有时被拒"。

修法：把判据收敛成**一个对象**，谁要判都问它。
    · 有实测地图（map_yaml 给了且能读）→ 以地图为准（权威判据）：
      图外拒绝、压障碍拒绝、净空不足拒绝。
    · 没给地图（如纯逻辑测试、没走加载地图流程）→ 退化为矩形范围判据，
      此时 field_* 才生效。这样 field_* 不再是"第二个判据"，
      而只是"没有地图时的兜底"，二者不可能再互相矛盾。

判据与 Nav2 的 map_server 解码保持一致：灰度 ≥ occupied_gray（默认 65）
视为可通行；灰度 205 这类"未观测格"按可通行处理（与 trinary 解码一致，
不额外拦截导航器实际能走的点）。
"""

# 兜底矩形范围（没加载地图时用）：默认给一个很宽的范围，等于"不拦"
DEFAULT_FIELD = {                        # def 缺省场地范围
    'min_x': -1e9, 'max_x': 1e9, 'min_y': -1e9, 'max_y': 1e9,
}


class GoalValidator:                     # out 目标点合法性唯一判据
    # 初始化：优先用地图，读不到就退化为矩形范围
    # 参数：map_yaml - 地图 yaml 路径（空=不用地图）
    #       robot_clearance - 所需净空(m)
    #       field - 兜底矩形 {'min_x','max_x','min_y','max_y'}，可缺省
    # 返回：无
    def __init__(self, map_yaml='', robot_clearance=0.30, field=None):   # out 对外构造函数
        self.map_yaml = str(map_yaml or '').strip()
        self.robot_clearance = float(robot_clearance)
        self.field = dict(DEFAULT_FIELD)
        self.field.update({k: float(v) for k, v in (field or {}).items()
                           if k in DEFAULT_FIELD})
        self.checker = None              # def 地图检查器（有图才有）
        self.error = ''                  # def 地图加载失败原因
        if self.map_yaml:                # # 给了地图：尝试加载
            try:
                from agv_common.map_check import MapChecker
                self.checker = MapChecker(self.map_yaml,
                                          robot_clearance=self.robot_clearance)
            except Exception as exc:     # # 读不到图：记下来，退化为矩形判据
                self.error = str(exc)
                self.checker = None

    # 是否以地图为判据（否则为矩形兜底）
    # 参数：无
    # 返回：bool
    def map_based(self):                 # out 对外接口
        return self.checker is not None

    # 一句话描述当前判据（启动日志用）
    # 参数：无
    # 返回：str
    def describe(self):                  # out 对外接口
        if self.checker is not None:
            c = self.checker
            return (f'判据=实测地图 {self.map_yaml}'
                    f'（x∈[{c.ox:.2f}, {c.ox + c.W * c.res:.2f}]，'
                    f'y∈[{c.oy:.2f}, {c.oy + c.H * c.res:.2f}]，'
                    f'所需净空 {self.robot_clearance:.2f} m）')
        if self.error:
            return f'判据=矩形兜底（地图加载失败：{self.error}）'
        return '判据=矩形兜底（未指定地图）'

    # 判断一个目标点能不能去
    # 参数：x, y - 目标坐标(m)
    # 返回：(ok: bool, reason: str)
    def check(self, x, y):               # out 对外接口
        try:
            x = float(x)
            y = float(y)
        except (TypeError, ValueError):
            return False, '坐标格式非法'

        if self.checker is not None:     # # 有图：以地图为唯一判据
            return self.checker.check_point(x, y)

        # 无图：矩形兜底
        f = self.field
        if not (f['min_x'] <= x <= f['max_x'] and f['min_y'] <= y <= f['max_y']):
            return False, (f'目标点 ({x:.2f}, {y:.2f}) 超出场地范围 '
                           f'x[{f["min_x"]:.0f}, {f["max_x"]:.0f}] '
                           f'y[{f["min_y"]:.0f}, {f["max_y"]:.0f}]，已拒绝')
        return True, f'目标点 ({x:.2f}, {y:.2f}) 在场地范围内'


# 便捷构造：只用一个矩形范围做判据（纯逻辑测试、无地图场景用）
# 参数：min_x/max_x/min_y/max_y - 矩形范围(m)
# 返回：GoalValidator
def rect_validator(min_x=-1e9, max_x=1e9, min_y=-1e9, max_y=1e9):   # out 对外接口
    return GoalValidator(map_yaml='', field={
        'min_x': min_x, 'max_x': max_x, 'min_y': min_y, 'max_y': max_y})
