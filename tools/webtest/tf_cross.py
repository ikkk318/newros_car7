#!/usr/bin/env python3
"""在 e2e 场景里作为兄弟进程运行，验证网关同款写法能否查到注入器发的 TF。

★ 关键：完全复用网关的写法 —— MultiThreadedExecutor + Buffer + TransformListener，
   这样才能复现网关查不到 TF 的现象。
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import (Buffer, TransformListener, LookupException,
                     ConnectivityException, ExtrapolationException)


class Probe(Node):
    def __init__(self):
        super().__init__('tf_cross_probe')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.2, self.tick)
        self.n = 0
        self.ok = 0

    def tick(self):
        self.n += 1
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint',
                                                 rclpy.time.Time())
            tr = tf.transform.translation
            rot = tf.transform.rotation
            yaw = math.atan2(2 * (rot.w * rot.z), 1 - 2 * rot.z * rot.z)
            self.ok += 1
            if self.n % 5 == 1:
                print(f'  ✓ [{self.n}] 查到 TF x={tr.x:.2f} y={tr.y:.2f} '
                      f'yaw={yaw:.3f} frame={tf.header.frame_id}', flush=True)
        except (LookupException, ConnectivityException,
                ExtrapolationException) as e:
            if self.n % 10 == 1:
                print(f'  ✗ [{self.n}] {type(e).__name__}: {e}', flush=True)
        except Exception as e:
            print(f'  ✗ [{self.n}] 其他 {type(e).__name__}: {e}', flush=True)


def main():
    rclpy.init()
    node = Probe()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    # 跑 12 秒后报告
    time.sleep(12)
    print(f'\n  小结：{node.ok}/{node.n} 次查询成功', flush=True)
    ex.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
