#!/usr/bin/env python3
"""比较多个 Super-LIO 运行的世界系 yaw 时间线（用于时移消融）。

假设：雷达点时刻统一多算 Δ ⇒ 每帧位姿偏 −ω·Δ ⇒ 累计 yaw 偏置 = −Δ·Δθ(t)。
于是两组运行之间 Δyaw(t) = −(Δ2−Δ1)·Δθ(t)，与"累计转角"成正比。

对 Δyaw(t) 关于参考运行的累计转角 Δθ(t) 做最小二乘，斜率即等效 Δ 差：
  slope(deg/deg) = −(Δ2−Δ1)[s]

用法: python3 timesweep_cmp.py <run_dir> [<run_dir> ...]
      第一个作为参考；网格 10 s。
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import load_odom

runs = sys.argv[1:]


def yaw_series(run):
    odom = load_odom(os.path.join(run, "odom.csv"))
    t = odom["t_ns"].astype(np.float64) * 1e-9
    qx, qy, qz, qw = odom["qx"], odom["qy"], odom["qz"], odom["qw"]
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy),
                     1.0 - 2.0 * (qy * qy + qz * qz))
    return t - t[0], np.degrees(np.unwrap(yaw)), odom


ref_t, ref_y, _ = yaw_series(runs[0])
tend = min(ref_t[-1], 430.0)
grid = np.arange(10.0, tend + 1e-9, 10.0)
ref_g = np.interp(grid, ref_t, ref_y)
rot = ref_g - ref_g[0]
print(f"ref = {os.path.basename(runs[0])}  末累计转角 {rot[-1]:+.1f}°  "
      f"最大 |Δθ| {np.abs(rot).max():.1f}°")

for run in runs:
    t, y, odom = yaw_series(run)
    g = np.interp(grid, t, y)
    d = g - ref_g
    name = os.path.basename(run)
    A = np.vstack([rot, np.ones_like(rot)]).T
    sol, *_ = np.linalg.lstsq(A, d, rcond=None)
    res = d - A @ sol
    ss = 1.0 - res.var() / max(d.var(), 1e-12)
    print(f"\n{name}  (rows={len(t)}, 末 z={odom['z'][-1]:+.2f})")
    print(f"  Δyaw RMS {np.sqrt((d**2).mean()):.3f}°  max|Δ| {np.abs(d).max():.3f}°")
    print(f"  拟合 Δyaw = {sol[0]:+.5f}·Δθ {sol[1]:+.3f}   R²={ss:.3f}")
    print(f"  ⇒ 等效 Δ 差 = {-sol[0]*1e3:+.3f} ms")
    idx = np.arange(0, len(grid), 5)
    print("  t(s)     " + "".join(f"{grid[i]:8.0f}" for i in idx))
    print("  Δθ(deg)  " + "".join(f"{rot[i]:+8.1f}" for i in idx))
    print("  Δyaw     " + "".join(f"{d[i]:+8.3f}" for i in idx))
