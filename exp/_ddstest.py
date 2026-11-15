#!/usr/bin/env python3
"""临时诊断：同进程 pub/sub 走 DDS 回环，验证当前 WSL 的 ROS2 中间件是否可用。"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class T(Node):
    def __init__(self):
        super().__init__("ddstest")
        self.n = 0
        self.pub = self.create_publisher(String, "/ddstest", 10)
        self.sub = self.create_subscription(String, "/ddstest", self.cb, 10)

    def cb(self, msg):
        self.n += 1


rclpy.init()
node = T()
t0 = time.time()
while time.time() - t0 < 10.0:
    node.pub.publish(String(data="x"))
    rclpy.spin_once(node, timeout_sec=0.1)
print("recv=%d" % node.n)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if node.n > 0 else 1)
