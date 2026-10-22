#!/usr/bin/env python3
"""订阅 /lio/odom 并写 CSV，用于实验后计算轨迹指标。

用法: python3 odom_recorder.py --out /path/to/odom.csv
收到 SIGTERM/SIGINT 时 flush 并退出。
"""
import argparse
import csv
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry


class OdomRecorder(Node):
    def __init__(self, out_path: str, topic: str = "/lio/odom"):
        super().__init__("exp_odom_recorder")
        self._f = open(out_path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(
            ["t_ns", "x", "y", "z", "qx", "qy", "qz", "qw", "vx", "vy", "vz"]
        )
        self._n = 0
        qos = QoSProfile(
            depth=500,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, topic, self._cb, qos)
        self.get_logger().info(f"recording {topic}")

    def _cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        t = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._w.writerow(
            [t, p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z]
        )
        self._n += 1
        if self._n % 200 == 0:
            self._f.flush()

    def close(self) -> None:
        try:
            self._f.flush()
            self._f.close()
        except Exception:
            pass
        print(f"[odom_recorder] wrote {self._n} samples", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--topic", default="/lio/odom")
    args = ap.parse_args()

    rclpy.init()
    node = OdomRecorder(args.out, args.topic)

    def _stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
