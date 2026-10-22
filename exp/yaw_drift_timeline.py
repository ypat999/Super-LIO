#!/usr/bin/env python3
"""按时间重建「墙面主方向 ε(t)」曲线，定位 yaw 漂移是随时间累积还是随转角累积。

依赖 SC-PGO 导出（lio.sc_pgo.enable=true）：<run_dir>/odom_poses.txt + Scans/*.pcd
做法：把相邻若干关键帧的 body 扫描按位姿变换到世界系、只留当前位姿附近 r 米，
      竖向性筛选 + xy RANSAC 直线 + 法向主方向(mod 90°)，再沿时间连续展开。
输出：t, z, ε(deg), 强度, 线数；以及 ε 对 t / 对累计转角 / 对灵敏度积分的拟合。

用法：python3 yaw_drift_timeline.py <run_dir> [bin_kf] [radius_m]
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import read_pcd_xyz
from wall_metrics import vertical_filter, ransac_lines, dominant_angle, _wrap_to

run = sys.argv[1]
bin_kf = int(sys.argv[2]) if len(sys.argv) > 2 else 10
radius = float(sys.argv[3]) if len(sys.argv) > 3 else 7.0
mode = sys.argv[4] if len(sys.argv) > 4 else "all"   # all | floor

poses = np.loadtxt(os.path.join(run, "odom_poses.txt"))
if poses.ndim == 1:
    poses = poses[None, :]
n = len(poses)
R = poses[:, [0, 1, 2, 4, 5, 6, 8, 9, 10]].reshape(-1, 3, 3)
T = poses[:, [3, 7, 11]]

files = sorted(f for f in os.listdir(os.path.join(run, "Scans")) if f.endswith(".pcd"))
print(f"{run}: poses={n} scans={len(files)} (成对按索引对齐)")

rows = []
for s in range(0, min(n, len(files)), bin_kf):
    e = min(s + bin_kf, n, len(files))
    Rs, Ts = R[s:e], T[s:e]
    if mode == "floor":
        # 只在“平飞”窗口测量：起降段/楼梯段的墙族混杂，模 90° 主方向会跳变
        dz = np.abs(np.diff(Ts[:, 2]))
        dxy = np.linalg.norm(np.diff(Ts[:, :2], axis=0), axis=1)
        if len(dz) == 0 or np.median(dz) > 0.05 or np.median(dxy) < 0.02:
            continue
    acc = []
    for i in range(s, e):
        p = read_pcd_xyz(os.path.join(run, "Scans", files[i]))
        if p is None or len(p) == 0:
            continue
        w = p @ Rs[i - s].T + Ts[i - s]
        d = np.linalg.norm(w[:, :2] - Ts[i - s, :2], axis=1)
        acc.append(w[d < radius])
    if not acc:
        continue
    w = np.concatenate(acc)
    if len(w) < 600:
        continue
    vm = vertical_filter(w, cell=0.2, min_span=0.6)
    wp = w[vm]
    if len(wp) < 400:
        continue
    lines = ransac_lines(wp[:, :2], min_inliers=max(80, int(0.02 * len(wp))),
                         max_lines=8, min_span=1.0, min_density=2.0, min_coverage=0.4)
    th, st = dominant_angle(lines, fold=4)
    if th is None or st < 0.5:
        continue
    rows.append({"t": (s + e) / 2 * 0.0, "k": (s + e) // 2, "z": float(np.median(T[s:e, 2])),
                 "th": th, "strength": st, "n": len(wp), "n_lines": len(lines)})

if len(rows) < 8:
    print("有效窗口太少，放弃")
    sys.exit(0)

k = np.array([r["k"] for r in rows], float)
z = np.array([r["z"] for r in rows])
th = np.array([r["th"] for r in rows])
st = np.array([r["strength"] for r in rows])
period = np.pi / 4
un = np.zeros_like(th)
un[0] = th[0]
for i in range(1, len(th)):
    un[i] = un[i - 1] + _wrap_to(th[i] - th[i - 1], period)
deg = np.degrees(un - un[0])

print(f"\n{'idx':>6} {'z':>7} {'yaw(deg)':>9} {'Δ前':>7} {'强度':>5} {'线':>3} {'点':>6}")
prev = 0.0
for i, r in enumerate(rows):
    print(f"{r['k']:6d} {r['z']:7.2f} {deg[i]:9.3f} {deg[i]-prev:+7.3f} "
          f"{r['strength']:5.2f} {r['n_lines']:3d} {r['n']:6d}")
    prev = deg[i]

print(f"\n首末差 {deg[-1]-deg[0]:+.2f} deg，跨 {len(deg)} 个窗口")
# 稳健剔除离群（外部建筑朝向不同）
A = np.vstack([k, np.ones_like(k)]).T
for _ in range(3):
    sol, *_ = np.linalg.lstsq(A, deg, rcond=None)
    res = deg - A @ sol
    keep = np.abs(res) < max(1.2, 3 * res.std())
    A, deg, k = A[keep], deg[keep], k[keep]
    z = z[keep]
print(f"稳健拟合（剔离群后 {len(deg)} 点）: ε = {sol[0]:+.5f} deg/关键帧, 残差 std={res[keep].std():.3f}°")
print(f"  z ~ {z.min():+.1f}..{z.max():+.1f} m")

# 线性 vs 分段：按时间前后半段比较
h = len(deg) // 2
a1, *_ = np.linalg.lstsq(np.vstack([k[:h], np.ones(h)]).T, deg[:h], rcond=None)
a2, *_ = np.linalg.lstsq(np.vstack([k[h:], np.ones(len(k) - h)]).T, deg[h:], rcond=None)
print(f"  前半段斜率 {a1[0]:+.5f} deg/关键帧，后半段 {a2[0]:+.5f}")
