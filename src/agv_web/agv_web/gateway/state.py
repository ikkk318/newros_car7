"""
state.py —— 节点的"状态一半"：日志、ROS 回调、位姿/TF、全量快照。

**为什么按"状态 / 动作"对半分**
原来的 `WebGateway` 一个类 1348 行，里面混着两类完全不同的东西：
  · **状态**：ROS 回调把数据写进 `_gw_*` 字段，`snapshot()` 读出来给前端；
  · **动作**：遥控、发指令、切模式、跑任务缓冲、存地图（见 `commands.py`）。
两者共享同一个 `_gw_lock`，但读写方向相反。分开之后，
"数据从哪来、被谁读走"可以一个文件读完，不用在 1300 行里来回跳。

**这里的锁纪律（一破就死锁）**
`_gw_lock` 是非重入的 `threading.Lock()`，而 `log()` 内部要取它。
所以**持有 `_gw_lock` 时绝不能再调 `log()`** —— 锁内只做字段赋值。
本文件所有 `with self._gw_lock:` 块都遵守这一条，改的时候别加日志。
"""



import time                                # use 时基/超时判定

import rclpy                                 # use 给 tf2 传零时刻
from tf2_ros import (                        # use 区分"查不到"与"还没来"
    LookupException, ConnectivityException, ExtrapolationException,
)

from agv_common.geometry import quaternion_to_yaw
from agv_interfaces.srv import AgvQuery

from ..grid_image import occupancy_to_png, grid_meta, scan_points_to_map
from ..status_board import build_gauges, front_distance
from .util import (
    _DUP_WINDOW, _LEVEL_RANK,
    _norm_log, _relay_is_self, _relay_is_success, _relay_wanted,
)



class StateMixin:
    """节点状态：ROS 回调往里写，`snapshot()` 一次取全给 HTTP。

    ★ 本类**不定义 `__init__`**：MRO 是
      `WebGateway → StateMixin → CommandMixin → Node`，
      谁在这里写 `__init__` 又不 super() 调用，`Node.__init__` 就永远不执行，
      rclpy 会在第一次 create_subscription 时炸得莫名其妙。
      字段初始化统一放在 `node.py` 的 `_init_shared_state()`。
    """



    # ── 日志：事件全留，高频状态限流 ──
    def log(self, level, text):                # out 记一条日志（自动降噪）
        """记一条日志 —— 但先问 `status_board.LogFilter` 这条该不该留。

        ★ 为什么要降噪（用户原话："日志打印速度太快根本看不过来"）：
          日志列表以前什么都收，避障/代价地图/重连这类 1 Hz 的运行时状态
          十几分钟就把 300 条环形缓冲冲干净，用户翻回去只看得到"前方
          0.42m"这种没用的历史，"顶升完成"一闪而过再也找不回来。

        ★ 降噪的处置分三种（判定逻辑在 status_board.py，纯逻辑、可单测）：
            · 事件（顶升、完成、换栈、保存地图…）→ 全留
            · 已知高频状态（避障 5s、代价地图 30s…）→ 限流留存，不是全丢
            · 没预料到的高频重复 → 超过阈值折叠，并补一条提示
          注意：**不再向面板写文本**。面板的值来自状态量（build_gauges），
          第一版把日志原文塞进面板格子，才有"格子固定死、长了看不全"的问题。

        ★ 无论如何都仍然转发给 ROS logger：终端里该有的还得有，
          网页日志只是"给人看的那一份"。
        """
        text = str(text)
        keep = True
        note = ''
        try:
            # 纯内存操作、O(规则数)，可以直接在回调线程里调
            verdict = self._gw_logf.feed(level, text)
            keep = verdict.get('claim', 'log') != 'drop'
            note = verdict.get('note') or ''
        except Exception:
            keep = True      # 降噪出任何问题都不能影响正常日志
        if keep:
            # ── 跨通道去重：同一条事件只留一条 ──
            # ★ 为什么必须做（真车日志实测）：同一条顶升/下降既走子进程
            #   stdout、又走 /rosout 转发，网页日志里每条都出现两遍。
            #   用户要的是"简化日志"，一进两遍等于白降噪一半。
            norm = _norm_log(text)
            now_m = time.monotonic()
            with self._gw_lock:
                dup_item = None
                for n, ts, it in self._gw_log_recent:
                    if n == norm and (now_m - ts) <= _DUP_WINDOW:
                        dup_item = it
                        break
                if dup_item is not None:
                    # ★ 重复了也不能直接丢：两条通道能判出的级别不一样。
                    #   stdout 那条只有 guess_level（info/warn/error），
                    #   /rosout 那条才知道"顶升成功 = 绿"（ok）。
                    #   谁先到都有可能，所以按"更醒目"的级别合并 ——
                    #   否则绿字时有时无，用户以为成功失败随机。
                    if _LEVEL_RANK.get(level, 0) > _LEVEL_RANK.get(
                            dup_item.get('level', 'info'), 0):
                        dup_item['level'] = level
                else:
                    item = {'t': round(time.time(), 3), 'level': level,
                            'text': text}
                    self._gw_logs.append(item)
                    self._gw_log_recent.append((norm, now_m, item))
                    if len(self._gw_log_recent) > 32:
                        del self._gw_log_recent[:-32]
                    if note:
                        # 折叠提示跟在被折叠的那条后面，用户才知道"不是我漏看"
                        self._gw_logs.append({'t': round(time.time(), 3),
                                              'level': 'warn', 'text': note})
                    if len(self._gw_logs) > self.log_lines:
                        del self._gw_logs[:len(self._gw_logs) - self.log_lines]
        if level == 'error':
            self.get_logger().error(text)
        elif level == 'warn':
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)


    # ── 回调 ──
    def on_task_state(self, msg):              # in 任务状态
        with self._gw_lock:
            self._gw_task_state = {'task_id': msg.task_id, 'state': msg.state,
                                'target': msg.target_location,
                                'progress': float(msg.progress),
                                'message': msg.message, 't': time.time()}

    def on_manual(self, msg):                  # in 手动模式
        with self._gw_lock:
            self._gw_manual = bool(msg.data)

    def on_map(self, msg):                     # in 地图（只在变化时重新编码）
        """收到 OccupancyGrid → 编码成 PNG。

        ★ 建图模式下 slam_toolbox 每 map_update_interval(0.5s) 就发一次全量 /map，
          而一次 594x762 编码约 0.11 秒。若每条消息都编，会占掉 ~23% 的 CPU 且
          在编码期间阻塞 ROS 回调线程，导致激光/位姿回调堆积。
          所以这里按 map_min_interval 限流：到点才编，中间的地图直接丢掉
          （建图本来就是渐进的，丢几帧中间的图完全没影响）。
        """
        now = time.monotonic()
        if self.mode == 'mapping' and self.map_min_interval > 0.0:
            if (now - self._gw_last_map_encode) < self.map_min_interval:
                return
        try:
            meta = grid_meta(msg)
            png = occupancy_to_png(meta['width'], meta['height'], msg.data)
        except Exception as exc:
            self.log('error', f'地图编码失败：{exc}')
            return
        self._gw_last_map_encode = now
        with self._gw_lock:
            # 建图时尺寸可能变（slam_toolbox 会扩张地图），一并记下来
            grew = (self._gw_map_meta is not None
                    and (meta['width'] != self._gw_map_meta['width']
                         or meta['height'] != self._gw_map_meta['height']))
            self._gw_map_meta = meta
            self._gw_map_png = png
            self._gw_map_version += 1
        if grew:
            self.log('info', f"建图中：地图扩张到 {meta['width']}x{meta['height']} 格")

    def on_scan(self, msg):                    # in 激光
        with self._gw_lock:
            self._gw_scan = {'ranges': list(msg.ranges),
                          'angle_min': float(msg.angle_min),
                          'inc': float(msg.angle_increment),
                          'range_min': float(msg.range_min),
                          'range_max': float(msg.range_max),
                          't': time.time()}

    def on_plan(self, msg):                    # in 全局路径
        pts = [(float(p.pose.position.x), float(p.pose.position.y))
               for p in msg.poses]
        with self._gw_lock:
            self._gw_plan = pts

    def on_vel(self, msg):                     # in 底盘实际速度（/cmd_vel_raw）
        """缓存底盘真正收到的速度，供仪表"速度"格显示。

        ★ 为什么还要记单调时钟：这个值用来判断"底盘链路还活不活"。
          发零速之后仲裁仍会以 publish_rate 继续发 0，所以只要话题还在通，
          值会一直刷新；一旦停更超过 1 秒，说明仲裁/底盘链路断了，
          仪表该显示"——"而不是挂着一个"停止"骗人。
        """
        try:
            v = float(msg.linear.x)
            w = float(msg.angular.z)
        except (AttributeError, TypeError, ValueError):
            return
        with self._gw_lock:
            self._gw_speed = (v, w, time.monotonic())

    def on_cargo_busy(self, msg):              # in 载货中标志（/agv/cargo/busy）
        """缓存"正在载货"，供仪表"动作"格判"车上有没有货"。

        ★ 为什么仪表不自己推、要听这个标志：
          TaskState 在任务结束/出错时说不清货在哪 —— 中止在运输途中时状态机
          回 IDLE，但货还压在车上。cargo_node 的 busy 是唯一权威事实。
          这里不判新鲜度：cargo_node 每秒补发一次，值本身就是持续状态而非
          瞬时事件；它停发只意味着节点没了，保持最后的已知值比假装"没货"
          安全（安全问题宁可多报警）。
        """
        try:
            self._gw_cargo_busy = bool(msg.data)
        except (AttributeError, TypeError):
            return

    def on_rosout(self, msg):                  # in 其它节点日志 → 网页日志
        """把任务节点/搬货节点的关键日志转发到网页运行日志。

        ★ 用户原话："到达搬货点请求服务后打印请求结果出来（成功绿字，失败
          红字），并显示在运行日志"。顶升/下降是 task_manager_node 在**另一个
          进程**里调的，网关的 self.log() 根本收不到 —— 必须走 /rosout。

        ★ 为什么只转发关键事件、不全转发：
          /rosout 汇总了全系统日志（Nav2、slam、AMCL 全在里面）。全转发会把
          刚做好的降噪一次性废掉，网页日志重新变成刷屏。所以这里只挑
          "顶升/下降/任务边界"这几类，其余交给终端看。

        ★ 为什么按 node_name 排除自己：
          本节点也会往 /rosout 写（self.get_logger().info(...)），
          而 self.log() 已经记过一遍。不排除会**每条日志重复两遍**。
        """
        try:
            name = str(getattr(msg, 'name', '') or '')
            text = str(getattr(msg, 'msg', '') or '')
            lv = int(getattr(msg, 'level', 20))
        except (AttributeError, TypeError, ValueError):
            return
        if not text:
            return
        # 排除自己：名字可能是 'agv_web_gateway'、'/agv_web_gateway'
        # 或带命名空间的 '/agv/agv_web_gateway'。判据见 _relay_is_self。
        if _relay_is_self(name, self.get_name()):
            return
        # ★ 记下"指令被拒"的原文，供 _start_one 失败时给出真实原因。
        #   放在过滤之前：即使这条日志因为限流没转到网页，也要能取到。
        if '拒绝' in text and '[指令]' in text:
            self._gw_last_reject = (time.monotonic(), text)
        # ★ 记下 map_server 加载失败的原文（同上，放在过滤之前）。
        #   命中几种典型文案：nav2 的中文包装、以及 map_io 的英文原文。
        if ('地图加载失败' in text
                or 'Failed processing YAML' in text
                or 'Failed to load map' in text
                or 'bad file' in text):
            self._gw_map_error = (time.monotonic(), text)
        # ★ 先判级别：ERROR 以上无条件转发。
        #   理由：严重错误不该因为"文本没命中关键词"被过滤掉 ——
        #   过滤器的目的是压噪音，不是藏故障。
        is_bad = lv >= 40
        if not is_bad and not _relay_wanted(text):
            return
        # ROS 级别数值 → 网页日志级别（前端靠这个上色：ok=绿 error=红）
        #   DEBUG/INFO=10/20 → info；WARN=30 → warn；ERROR=40/FATAL=50 → error
        level = 'info'
        if lv >= 40:
            level = 'error'
        elif lv >= 30:
            level = 'warn'
        # ★ 顶升/下降成功要让用户看到"绿字"，但它本身是 INFO 级。
        #   判据放在文本里而不是级别里：cargo_client 成功时打"结束"、失败打"失败"。
        if level == 'info' and _relay_is_success(text):
            level = 'ok'
        self.log(level, text)

    def on_tick(self):                         # 5 Hz：刷新位姿 + 遥控掉线保护
        self.refresh_pose()
        if self.watchdog.should_stop():        # # 前端停止发速度：立刻发零速
            self.publish_twist(0.0, 0.0)
            self.watchdog.idle()
            # ★ 只停车，**不收回控制权**（本轮修的手感 bug）。
            #   teleop_stale_sec 只有 0.35 秒，而控制权租约的闲置期限是
            #   120 秒 —— 原来在这里 force_release()，等于把 120 秒的锁
            #   降级成 0.35 秒：用户点「取得控制权」后只要手一松，
            #   0.35 秒锁就没了，再按方向键只会得到"请先取得控制权"。
            #   实测日志：15:58:29 取得 → 15:58:33 已被收回。
            #   车的安全由上面那帧零速保证，锁的回收交给 ControlLease
            #   自己的 120 秒闲置超时（前端每 20 秒还会续期一次）。
            self.log('warn', '遥控已停止（0.35 秒没有新指令）→ 已发零速；'
                             '控制权保留，闲置 120 秒自动释放')

    def on_query_tick(self):                   # 0.5 Hz：服务查询兜底
        self.refresh_pose(use_service=True)

    # ── 位姿 ──
    def refresh_pose(self, use_service=False):  # out 刷新位姿缓存
        # ★ 这三个必须先初始化：下面 try 里出异常时会跳过赋值，
        #   而后面 `if pose is not None and fresh:` 无条件读 fresh。
        #   曾经 fresh 只在 try 内定义，一旦 lookup 抛异常就
        #   UnboundLocalError —— 位姿永远刷新不了、小车不显示的另一种死法。
        pose = None
        stamp = 0.0
        fresh = False
        stale_reason = ''      # def 判定过期时的原因（只在日志里用）
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame,
                                                rclpy.time.Time())
            tr = tf.transform.translation
            rot = tf.transform.rotation
            # ★ quaternion_to_yaw 的签名是 (x, y, z, w) 四个标量，不是一整个
            #   四元数对象。以前这里写成 quaternion_to_yaw(rot) 会抛
            #   TypeError: missing 3 required positional arguments，而下面
            #   的 `except Exception: pose = None` 把它静默吞掉，于是位姿
            #   永远是 None —— 前端小车、朝向箭头、激光散点全部不显示，
            #   看起来像"TF 查不到"，实际是这一行传参错了。
            pose = (float(tr.x), float(tr.y),
                    quaternion_to_yaw(rot.x, rot.y, rot.z, rot.w),
                    time.time())
            stamp = self._tf_stamp(tf)                       # def 数据来源时间戳
            # ★★ 关键：判断"这个变换本身是不是过期的"。
            #   这是本轮最深的坑。tf2 Buffer 默认缓存 10 秒历史，而且
            #   lookup_transform(..., Time()) 取的是"时间戳最新的那个变换"。
            #   旧栈的节点被杀之后，它会**继续返回最后一个已知变换**，
            #   查得到、不报错、看起来完全正常 —— 于是：
            #     · 位姿保鲜期（只在"查不到"时才计时）永远不触发；
            #     · reset_runtime_cache 刚清掉的 _gw_pose 在 0.2 秒后
            #       又被这条旧变换填满；
            #     · 界面上的小车稳稳停在切换前的位置，还跟着旧数据动。
            #   所以不能只看"查没查到"，必须看"这帧数据有多新"。
            fresh = self._tf_is_fresh(tf, self.pose_hold_sec)
            if not fresh:
                stale_reason = 'TF 变换时间戳太旧（上游已停止发布）'
        except (LookupException, ConnectivityException, ExtrapolationException):
            pose = None
            stamp = 0.0
        except Exception as exc:
            # ★ 不能静默吞掉：这类错误会让整块显示功能失效却毫无提示。
            #   限流打印，避免 5 Hz 刷屏。
            now = time.monotonic()
            if now - self._gw_last_pose_err >= 5.0:
                self._gw_last_pose_err = now
                self.log('error', f'位姿解析异常（{type(exc).__name__}）：{exc}')
            pose = None
            stamp = 0.0

        if pose is not None and fresh:
            with self._gw_lock:
                self._gw_pose = pose
                self._gw_pose_stamp = stamp
                self._gw_pose_clock = self.get_clock()
                self._gw_pose_fresh = True
                self._gw_pose_stale = False      # 拿到新鲜位姿，清掉"旧了"的标记
            self._gw_last_pose_ok = time.monotonic()
            return

        if pose is None and use_service:
            # ★ 服务兜底（0.5 Hz）只在 TF 真拿不到时用。
            #   TF 拿得到但"太旧"时不能用它补：服务查的也是同一套栈的状态，
            #   旧栈残留一样会答上一个旧值，反而绕过新鲜度检查。
            sp = self.query_pose_by_service()
            if sp is not None:
                with self._gw_lock:
                    self._gw_pose = sp
                    self._gw_pose_stamp = 0.0
                    self._gw_pose_clock = None
                    self._gw_pose_fresh = True
                self._gw_last_pose_ok = time.monotonic()
                return

        # ★ 位姿"保鲜期"：查不到（或查到的太旧）时不代表车在原点，
        #   而是"这套栈还没定位好"。以前这里什么都不做，_gw_pose 就一直
        #   保留上一个模式（或上一张图）的最后一帧 —— 用户切模式后会看到
        #   小车稳稳停在旧位置，实际车可能在别处或者根本没定位，
        #   这是最危险的一种"看着正常"。
        with self._gw_lock:
            had_pose = self._gw_pose is not None
            if pose is not None and not fresh:
                # 数据本身就旧了，不必等保鲜期，立刻作废（旧栈已经停了）
                self._gw_pose = None
                # 记住"已知数据是旧的"，让前端区分"没定位"和"定位丢了"
                self._gw_pose_stale = True
                verdict = 'stale'
            elif self._gw_last_pose_ok and \
                    (time.monotonic() - self._gw_last_pose_ok) > self.pose_hold_sec:
                self._gw_pose = None
                self._gw_pose_stale = True
                verdict = 'hold-expired'
            else:
                verdict = 'keep'
            self._gw_pose_fresh = False
        if verdict != 'keep' and had_pose:
            # ★ 必须限流：refresh_pose 是 5 Hz，不限流会在日志里刷成一片，
            #   把真正有用的信息顶掉。3 秒一次足够看出"什么时候开始丢的"。
            now_m = time.monotonic()
            if now_m - self._gw_last_pose_lost_log >= 3.0:
                self._gw_last_pose_lost_log = now_m
                self.log('warn',
                         f'定位丢失：{stale_reason or "超过保鲜期没拿到新位姿"}'
                         f'（{self.pose_hold_sec:.1f}s）→ 界面改为显示"定位未就绪"')

    def _tf_stamp(self, tf):                   # out 该变换的时间戳（秒，float）
        """把 TF 的 header.stamp 摊成秒。取不到返回 0.0。"""
        try:
            s = tf.header.stamp
            return float(s.sec) + float(s.nanosec) * 1e-9
        except Exception:
            return 0.0

    def _tf_age(self, tf):                     # out 该变换距今多少秒（算不出返回 None）
        """算 tf 的时间戳有多旧。

        ★ 为什么要单独一个方法：仿真用 sim time、真机用 wall time，
          混用会得出"差 50 年"这种荒谬的年龄。两边都用节点自己的 clock，
          保证同一时间基准。算不出来就返回 None（调用方据此不做判断）。

        ★★ 但"用节点自己的 clock"这件事本身有一个致命前提：**网关自己的
          clock 必须和上游发布 TF 的时钟同源**。实际情况是：
            · Gazebo 栈（建图 / 带仿真的导航）上游用 /clock 的 **sim time**
              （启动后才几百秒）；
            · 网关只起了一个 aiohttp 进程，从 launch 里【没有】拿到
              use_sim_time，走的是 **wall time**（1.79e9 秒）。
          两者相减 ≈ 57 年，于是每一帧都被判成"太旧"，位姿永远是 None，
          界面上两个模式都看不到小车 —— 这就是本方法第一版引入的回归。
          所以下面用"同一个 clock 对象"做时基校验，不同源就不做减法。
        """
        stamp = self._tf_stamp(tf)
        if stamp <= 0.0:
            return None
        try:
            now = self._clock_seconds(self.get_clock())
        except Exception:
            return None
        if now is None:
            return None
        age = now - stamp
        # ★ 时基不同源的兜底：真实的时间差不会超过一天（86400s）。
        #   超过就说明"我这边的 now 和上游的 stamp 用的不是同一个时钟"，
        #   此时减法结果毫无意义，退回"是否在更新"的判断（见 _tf_is_fresh），
        #   绝不能拿这个荒谬的差值去判过期 —— 那会让所有位姿都被丢掉。
        if age > 86400.0 or age < -86400.0:
            return None
        # 轻微"未来"的时间戳（<2s）是允许的：TF 插值/广播抖动常见，
        # 别把它当成异常。超过 2 秒的未来数据仍然判为不可信。
        if age < -2.0:
            return None
        return age

    def _clock_seconds(self, clock):           # out clock 当前时刻（秒，float）
        """把 ROS clock 摊成秒。拿不到返回 None。"""
        try:
            msg = clock.now().to_msg()
            return float(msg.sec) + float(msg.nanosec) * 1e-9
        except Exception:
            return None

    def _tf_is_fresh(self, tf, hold):          # out 该 TF 是否可用于显示
        """综合判断一个 TF 变换能不能当作"现在的位置"。

        两条判据按"时基是否可比"分流，缺一不可：

        【A｜时基可比，正常路径】网关 clock 与 TF 的 stamp 同源（真机 wall
            time，或网关也开了 use_sim_time）。直接比年龄，`age <= hold` 即新鲜。
            这是最准的判据，也是首选的。

        【B｜时基不可比，仿真兜底】网关走 wall time、上游 TF 走 /clock 的
            sim time，相减是几十亿秒，纯属没意义的数字。此时改用**"时间戳
            是否还在前进"**：
              · stamp 变了 → 上游还在发 → 新鲜；
              · stamp 一直不变，且已经卡住超过 hold 秒 → 上游停了 → 过期。
            这一条必须保留，否则 Gazebo 场景下（网关没拿到 use_sim_time）
            每一帧都会被判过期，两个模式都看不到小车 —— 也就是这次报的 bug。

        两条路都不做"算不出来就当真"：tf2 缓存复活旧栈位姿这个坑，
        在 B 路里同样会被"stamp 卡住超时"抓住。
        """
        age = self._tf_age(tf)
        if age is not None:                    # A：时基可比，直接比年龄
            return age <= hold
        # B：时基不可比，看时间戳是否在前进
        stamp = self._tf_stamp(tf)
        if stamp <= 0.0:
            return False
        now_m = time.monotonic()
        with self._gw_lock:
            prev = self._gw_pose_stamp
            prev_at = self._gw_pose_stamp_at
            if prev_at <= 0.0 or stamp != prev:
                # 时间戳变了（或第一次见）= 上游还在发。记下时刻供下次比对。
                self._gw_pose_stamp = stamp
                self._gw_pose_stamp_at = now_m
                return True
            # stamp 没变。可能只是这一帧没更新（TF 30 Hz、我们 5 Hz 读），
            # 也可能是上游停了。用"卡住多久"区分，超 hold 才算过期。
            return (now_m - prev_at) <= hold


    def query_pose_by_service(self):           # out 用 /agv/query 兜底取位姿
        if not self.query_cli.service_is_ready():
            return None
        req = AgvQuery.Request()
        req.what = 'pose'
        try:
            fut = self.query_cli.call_async(req)
            # 不阻塞 spin 线程太久；拿不到就下次再说
            deadline = time.time() + 0.15
            while not fut.done() and time.time() < deadline:
                time.sleep(0.01)
            if not fut.done():
                return None
            res = fut.result()
            if res is None or not res.ok:
                return None
            return (float(res.x), float(res.y), float(res.yaw), time.time())
        except Exception:
            return None


    # ══════════════════ 快照：给 HTTP/WS 一次取全 ══════════════════
    #
    # ★ 锁内只做"拷贝字段"，锁外才算激光点、建仪表 —— 后面这几步
    #   要遍历几百个点，占着锁会让 /api/state 排队，网页看着卡。


    # ── 快照：给 HTTP 一次取全 ──
    def snapshot(self):                        # out 当前全量状态
        with self._gw_lock:
            pose = self._gw_pose
            ts = self._gw_task_state
            scan = self._gw_scan
            plan = list(self._gw_plan or [])
            estop = self._gw_estop
            manual = self._gw_manual
            version = self._gw_map_version
            epoch = self._gw_map_epoch
            meta = self._gw_map_meta
            speed = self._gw_speed
            pose_fresh = self._gw_pose_fresh
            pose_ok_at = self._gw_last_pose_ok
            t_idx = self._gw_task_idx
            t_total = self._gw_task_total
            ws_n = len(self._gw_ws_clients)
        # 激光点要在锁外算（数量大），位姿用快照
        scan_pts = []
        front = None
        if scan is not None:
            # ★ 正前方距离不依赖位姿：它是纯激光角度的计算，
            #   定位丢了也该能显示"前方 0.5m"（那正是最需要看的时候）。
            front = front_distance(scan)
        if scan is not None and pose is not None:
            scan_pts = scan_points_to_map(
                scan['ranges'], scan['angle_min'], scan['inc'],
                pose[0], pose[1], pose[2],
                range_min=scan['range_min'], range_max=scan['range_max'])
            # 抽稀：网页画几千个点没必要，隔几个取一个；保留最近的点更密
            if len(scan_pts) > 720:
                step = max(1, len(scan_pts) // 720)
                scan_pts = scan_pts[::step]

        # ── 仪表输入 ──
        # 速度超过 1.5 秒没更新就当没有：发零速后仲裁仍以 publish_rate
        # 持续发 0，所以"有值"本身就说明链路通；停更即说明链路断了。
        # 挂着上一次的 "0.30" 比显示"——"危险得多。
        sp_now = None
        if speed is not None and (time.monotonic() - speed[2]) <= 1.5:
            sp_now = (speed[0], speed[1])
        pose_age = None
        if pose_ok_at > 0.0:
            pose_age = round(max(0.0, time.monotonic() - pose_ok_at), 1)
        gstate = {
            'pose_ok': bool(pose is not None and pose_fresh),
            'pose_age': pose_age,
            'task_state': ts,
            'task_running': bool(self.task_running),
            'task_idx': t_idx,
            'task_total': t_total,
            'cargo_busy': bool(self._gw_cargo_busy),
            'speed': sp_now,
            'front': front,
            'avoid_on': self._gw_logf.avoid_engaged(),
            'ws_n': ws_n,
            'map_meta': meta,
            'mode': self.mode,
            'manual': manual,
            'estop': estop,
        }
        return {'pose': ({'x': pose[0], 'y': pose[1], 'yaw': pose[2]} if pose else None),
                'task': ts, 'scan': scan_pts, 'plan': plan,
                'estop': estop, 'manual': manual,
                'mode': self.mode,
                'map_version': version,
                'map_epoch': epoch,
                'map_meta': meta,
                'lease': self.lease.state(),
                # ★ 顶部仪表：**固定 8 行**，每行 {key,label,text,tone}。
                #   值来自上面这些状态量，不是日志文本 —— 所以文本天生很短
                #   （10~20 字），不会出现"格子放不下、后面看不到"。
                #   每帧发全量的好处：前端可以永久复用 8 个 DOM 节点，
                #   只改 textContent，不增删、不闪、不跳滚动位置。
                'board': build_gauges(gstate),
                't': time.time()}
