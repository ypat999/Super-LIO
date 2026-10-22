#!/usr/bin/env python3
"""从 bag 里读 /livox/imu，统计“安静窗口”的陀螺均值，判断零偏是否随时间漂移。

安静窗口判据：2 s 内陀螺模长标准差 < 0.05 rad/s（无转动）。窗口均值≈当时零偏。
用法：python3 imu_bias_drift.py <bag_dir> [win_s]
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

bag = sys.argv[1]
win = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("", ""))
r.set_filter(rosbag2_py.StorageFilter(topics=["/livox/imu"]))

ts, gyr, acc = [], [], []
while r.has_next():
    _topic, data, _t = r.read_next()
    m = deserialize_message(data, Imu)
    ts.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    gyr.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
    acc.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])

ts = np.array(ts) - ts[0]
gyr = np.array(gyr)
acc = np.array(acc)
deg = 180.0 / np.pi
print(f"{bag}: {len(ts)} IMU 样点, {ts[-1]:.1f} s, 平均 {len(ts)/ts[-1]:.1f} Hz")
print(f"  原始陀螺均值(全程) {np.degrees(gyr.mean(0))} deg/s")
print(f"  加速度模长均值 {np.linalg.norm(acc,axis=1).mean():.4f} (g? m/s^2?)")

step = int(round(win / np.median(np.diff(ts))))
print(f"\n安静窗口（陀螺 std<0.05 rad/s）每 {win:.0f} s 一格：")
print(f"{'t0':>7} {'n':>5} {'|g|std':>7} {'gx':>8} {'gy':>8} {'gz':>8}  {'|a|':>7}")
rows = []
for i in range(0, len(ts) - step, step):
    g = gyr[i:i + step]
    a = acc[i:i + step]
    s = np.linalg.norm(g, axis=1).std()
    if s > 0.05:
        continue
    mg = g.mean(0) * deg
    rows.append((ts[i], mg[0], mg[1], mg[2], np.linalg.norm(a, axis=1).mean()))
    print(f"{ts[i]:7.1f} {step:5d} {s:7.4f} {mg[0]:8.3f} {mg[1]:8.3f} {mg[2]:8.3f}  "
          f"{np.linalg.norm(a,axis=1).mean():7.3f}")

if len(rows) >= 3:
    rows = np.array(rows)
    print(f"\n安静窗口 {len(rows)} 个，覆盖 {rows[0,0]:.0f}..{rows[-1,0]:.0f} s")
    for k, ax in enumerate("xyz"):
        A = np.vstack([rows[:, 0], np.ones(len(rows))]).T
        sol, *_ = np.linalg.lstsq(A, rows[:, 1 + k], rcond=None)
        print(f"  bg_{ax}: 首 {rows[0,1+k]:+7.3f} 末 {rows[-1,1+k]:+7.3f} deg/s, "
              f"漂移速率 {sol[0]*60:+.4f} deg/s/min")
else:
    print("\n安静窗口不足 3 个，无法拟合漂移")
