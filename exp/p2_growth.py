#!/usr/bin/env python3
"""E-1 / P-2：A、B 两线在同一 bag 上的差值随"螺旋周期数"的增长。

不做事后对齐：各 run 以自身首帧为原点（与 p2_windows.py div 同口径）。
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import load_odom

A = sys.argv[1] if len(sys.argv) > 1 else "results/m40_p2_full_a/odom.csv"
B = sys.argv[2] if len(sys.argv) > 2 else "results/m40_p2_full_b/odom.csv"


def load(p):
    o = load_odom(p)
    t = o["t_ns"].astype(np.float64) * 1e-9
    t -= t[0]
    xyz = np.stack([o["x"], o["y"], o["z"]], axis=1).astype(np.float64)
    xyz -= xyz[0]
    return t, xyz


tA, pA = load(A)
tB, pB = load(B)

# p2_windows.py profile 给出的每 360° 周期边界（bag 时间 → 相对首帧）
marks = [46.2, 70.0, 79.4, 89.7, 99.5, 109.7, 121.6, 132.4,
         236.2, 248.2, 259.1, 269.2, 280.1, 294.6, 312.2]

print("%6s %9s %9s %9s %9s" % ("periods", "t(s)", "|A-B|cm", "p95cm", "pathA_m"))
for k, tm in enumerate(marks, 1):
    if tm > min(tA[-1], tB[-1]):
        break
    ia = np.searchsorted(tA, tm) - 1
    ib = np.searchsorted(tB, tm) - 1
    d = np.linalg.norm(pA[ia] - pB[ib])
    tt = np.linspace(0, tm, 300)
    iaa = np.searchsorted(tA, tt) - 1
    ibb = np.searchsorted(tB, tt) - 1
    ok = (iaa >= 0) & (ibb >= 0)
    dd = np.linalg.norm(pA[iaa[ok]] - pB[ibb[ok]], axis=1)
    path = np.linalg.norm(np.diff(pA[:ia + 1], axis=0), axis=1).sum()
    print("%6d %9.1f %9.1f %9.1f %9.1f"
          % (k, tm, d * 100, np.percentile(dd, 95) * 100, path))

# 终点（整包）
ia, ib = len(tA) - 1, len(tB) - 1
d = np.linalg.norm(pA[ia] - pB[ib])
print("\n整包 end: t=%.1f/%.1f s  |A-B|=%.1f cm" % (tA[-1], tB[-1], d * 100))
print("A 端点 %s 里程 %.1f m" % (np.round(pA[-1], 3), np.linalg.norm(np.diff(pA, axis=0), axis=1).sum()))
print("B 端点 %s 里程 %.1f m" % (np.round(pB[-1], 3), np.linalg.norm(np.diff(pB, axis=0), axis=1).sum()))
