#!/usr/bin/env python3
"""临时诊断：跨进程 pub/sub。用法: _ping.py pub|sub [seconds]"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

mode = sys.argv[1]
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

rclpy.init()
n = Node("ping_" + mode)
cnt = [0]


def cb(m):
    cnt[0] += 1


pub = n.create_publisher(String, "/xr", 10)
sub = n.create_subscription(String, "/xr", cb, 10)
t0 = time.time()
while time.time() - t0 < dur:
    if mode == "pub":
        pub.publish(String(data="x"))
    rclpy.spin_once(n, timeout_sec=0.05)
print("%s recv=%d" % (mode, cnt[0]))
n.destroy_node()
rclpy.shutdown()
