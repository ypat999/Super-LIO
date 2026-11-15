#!/usr/bin/env python3
"""临时诊断：统计 /livox/imu 与 /livox/lidar 的实际到达消息数（不经 ros2 CLI/daemon）。"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

try:
    from livox_ros_driver2.msg import CustomMsg
    HAVE_LIVOX = True
except Exception:
    HAVE_LIVOX = False


class C(Node):
    def __init__(self):
        super().__init__("cnt")
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.ni = 0
        self.nl = 0
        self.create_subscription(Imu, "/livox/imu", self.ci, qos)
        if HAVE_LIVOX:
            self.create_subscription(CustomMsg, "/livox/lidar", self.cl, qos)

    def ci(self, m):
        self.ni += 1

    def cl(self, m):
        self.nl += 1


rclpy.init()
n = C()
t0 = time.time()
while time.time() - t0 < 15.0:
    rclpy.spin_once(n, timeout_sec=0.1)
print("imu=%d lidar=%d (livox_type=%s)" % (n.ni, n.nl, HAVE_LIVOX))
