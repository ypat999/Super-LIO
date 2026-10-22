#!/usr/bin/env python3
"""时序基准审计（对应排查项 P5）：只读 bag，不改任何算法。

要看的是四件事：
  1) 雷达帧时间基：header.stamp 与逐点 offset_time 的关系（min/max/是否单调）；
  2) 帧间隔是否恒为 100 ms，有无抖动/跳变/重复；
  3) IMU 采样率、丢样、时间戳重复、最大间隔；
  4) 每帧 [start, start+max_offset] 区间内 IMU 覆盖是否完整（去畸变输入是否够）。

lidar 只抽 1/STEP 帧做逐点解析（4.9 GB 全量反序列化太慢），IMU 全量保留。

用法: python3 timebase_audit.py <bag_dir> [step]
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg

bag = sys.argv[1]
step = int(sys.argv[2]) if len(sys.argv) > 2 else 25

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
       rosbag2_py.ConverterOptions("", ""))
r.set_filter(rosbag2_py.StorageFilter(topics=["/livox/lidar", "/livox/imu"]))

imu_t, imu_g = [], []
fr_stamp, fr_n, fr_off_min, fr_off_max, fr_mono, fr_recv = [], [], [], [], [], []
fr_look, fr_frac = [], []
k = 0

while r.has_next():
    topic, data, t_recv = r.read_next()
    if topic == "/livox/imu":
        m = deserialize_message(data, Imu)
        imu_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        imu_g.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
    else:
        k += 1
        if (k - 1) % step:
            continue
        m = deserialize_message(data, CustomMsg)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        ot = np.fromiter((p.offset_time for p in m.points), dtype=np.int64,
                         count=len(m.points))
        fr_stamp.append(st)
        fr_n.append(len(m.points))
        fr_off_min.append(int(ot.min()) if ot.size else -1)
        fr_off_max.append(int(ot.max()) if ot.size else -1)
        fr_mono.append(int(np.count_nonzero(np.diff(ot) < 0)) if ot.size > 1 else 0)
        if ot.size > 1:
            cs = np.maximum.accumulate(ot)
            fr_look.append(float((cs - ot).max()) * 1e-9)
            fr_frac.append(float(np.count_nonzero(np.diff(ot) < 0)) / ot.size)
        else:
            fr_look.append(0.0)
            fr_frac.append(0.0)
        fr_recv.append(t_recv * 1e-9)

imu_t = np.array(imu_t)
imu_g = np.array(imu_g)
fr_stamp = np.array(fr_stamp)
fr_n = np.array(fr_n)
fr_off_min = np.array(fr_off_min) * 1e-9
fr_off_max = np.array(fr_off_max) * 1e-9
fr_mono = np.array(fr_mono)
fr_recv = np.array(fr_recv)

print(f"==== {bag}")
print(f"lidar 收到 {k} 帧，抽样解析 {len(fr_stamp)} 帧（1/{step}）；IMU 样点 {len(imu_t)}")

d = np.diff(fr_stamp)
print("\n[1] 雷达帧 header.stamp 间隔 (s)  [抽样序列]")
print(f"  median {np.median(d)*1e3:.3f} ms   mean {d.mean()*1e3:.3f} ms   "
      f"min {d.min()*1e3:.3f}   max {d.max()*1e3:.3f}   std {d.std()*1e3:.3f}")
for thr in (1e-3, 10e-3):
    print(f"  |Δ - 100ms| > {thr*1e3:.0f} ms 的抽样帧: "
          f"{int(np.count_nonzero(np.abs(d - 0.1) > thr))}")
print(f"  严格单调: {bool(np.all(d > 0))}   重复帧戳: {int(np.count_nonzero(d == 0))}")

print("\n[2] 逐点 offset_time（相对 header.stamp）")
print(f"  min: median {np.median(fr_off_min)*1e3:.3f} ms  范围 "
      f"[{fr_off_min.min()*1e3:.3f}, {fr_off_min.max()*1e3:.3f}]")
print(f"  max: median {np.median(fr_off_max)*1e3:.3f} ms  范围 "
      f"[{fr_off_max.min()*1e3:.3f}, {fr_off_max.max()*1e3:.3f}]")
print(f"  逐帧点数 median {int(np.median(fr_n))}  范围 [{fr_n.min()}, {fr_n.max()}]")
print(f"  offset_time 非单调的帧: {int(np.count_nonzero(fr_mono > 0))} / {len(fr_mono)}")
fr_look = np.array(fr_look)
fr_frac = np.array(fr_frac)
print(f"  乱序点占比: median {np.median(fr_frac)*100:.2f}%  max {fr_frac.max()*100:.2f}%")
print(f"  最坏\"回看跨度\" max(t_before) - t_i: median {np.median(fr_look)*1e3:.3f} ms  "
      f"p95 {np.percentile(fr_look,95)*1e3:.3f} ms  max {fr_look.max()*1e3:.3f} ms")
if np.median(fr_off_min) > 1e-3:
    print("  注：首点 offset 明显 >0 ⇒ header.stamp 早于该帧第一个点")

print("\n[3] IMU")
dt = np.diff(imu_t)
print(f"  平均 {len(imu_t)/np.ptp(imu_t):.1f} Hz   dt median {np.median(dt)*1e3:.3f} ms  "
      f"p99 {np.percentile(dt,99)*1e3:.3f} ms  max {dt.max()*1e3:.3f} ms")
print(f"  dt<=0（重复/倒序）: {int(np.count_nonzero(dt <= 0))}   "
      f"dt>10ms 的缺口: {int(np.count_nonzero(dt > 0.01))}")
if np.any(dt > 0.01):
    idx = np.where(dt > 0.01)[0]
    print("  最大 5 个缺口: " + ", ".join(
        f"{imu_t[i]-imu_t[0]:.1f}s:{dt[i]*1e3:.1f}ms"
        for i in idx[np.argsort(dt[idx])[-5:][::-1]]))

print("\n[4] 每帧 IMU 覆盖（去畸变输入）")
i0 = np.searchsorted(imu_t, fr_stamp, side="left")
i1 = np.searchsorted(imu_t, fr_stamp + fr_off_max, side="right")
cov = i1 - i0
expect = np.maximum(1, np.round(fr_off_max / np.median(dt))).astype(int)
ratio = cov / expect
print(f"  帧内 IMU 个数: median {int(np.median(cov))}  最少 {int(cov.min())}")
print(f"  覆盖比 cov/expect: median {np.median(ratio):.3f}  "
      f"<0.8 的帧 {int(np.count_nonzero(ratio < 0.8))}  "
      f"<0.5 的帧 {int(np.count_nonzero(ratio < 0.5))}")
print(f"  帧戳早于首个 IMU 样点的帧: {int(np.count_nonzero((fr_stamp - imu_t[0]) < 0))}")

print("\n[5] 帧内转过的角度（IMU 扣静止段零偏后积分，frame 区间）")
deg = 180.0 / np.pi
static = (imu_t - imu_t[0]) < 20.0
bg = imu_g[static].mean(0) if static.sum() > 100 else np.zeros(3)
print(f"  静止段(前 20 s)陀螺均值 bg = {np.round(bg*deg, 3)} deg/s")
rot = []
for a, b in zip(i0, i1):
    if b - a < 2:
        rot.append(np.nan)
        continue
    w = (imu_g[a:b] - bg).mean(0)
    rot.append(np.linalg.norm(w) * (imu_t[b - 1] - imu_t[a]) * deg)
rot = np.array(rot)
print(f"  |Δθ|/帧: median {np.nanmedian(rot):.3f}°  p95 {np.nanpercentile(rot,95):.3f}°")
print(f"  ⇒ 1 ms 时间基准误差 ≈ {np.nanmedian(rot)*1e-3:.5f}°/帧（每 100 ms 帧）")

print("\n[6] 接收时刻 vs 帧戳（驱动/传输延迟）")
lag = fr_recv - fr_stamp
print(f"  recv - stamp: median {np.median(lag)*1e3:.1f} ms  "
      f"p5 {np.percentile(lag,5)*1e3:.1f}  p95 {np.percentile(lag,95)*1e3:.1f}")
