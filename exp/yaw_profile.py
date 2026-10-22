#!/usr/bin/env python3
"""从 map.pcd 直接测“墙面主方向 vs 高度”曲线，检验扭曲是否逐层等量同向。

做法：沿 z 每 1 m 取一层点 -> 竖向性筛选 -> xy RANSAC 直线 -> 法向主方向(mod 90°)
      逐层做 mod-90° 连续展开，得到 yaw(z)；再对 z 线性拟合得 deg/m 与 deg/floor。
用法：python3 yaw_profile.py <run_dir> [zbin] [--align-gravity]

--align-gravity：FAST-LIO 输出的世界系 z 不一定与重力对齐（它把初始 IMU 姿态当成
世界系，本 bag 的传感器是 30° 俯仰安装）。此时 z 不是"高度"，deg/m 没有意义。
该选项用第一圈（绕 10F 走廊）轨迹平面的法向估计真实竖直方向，把 odom 与 map 一起
旋到重力对齐系（1F 在负 z 侧），再算 yaw(z)。Point-LIO / Super-LIO 已自带重力对齐，
不要加这个选项。
"""
import sys
import os

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import (load_odom, read_pcd_xyz, traj_arrays, estimate_up,
                    rot_from_to)
from wall_metrics import vertical_filter, ransac_lines, dominant_angle, _wrap_to

ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
FLAGS = {a for a in sys.argv[1:] if a.startswith("--")}
ALIGN = "--align-gravity" in FLAGS

run = ARGS[0]
zbin = float(ARGS[1]) if len(ARGS) > 1 else 1.0


odom = load_odom(os.path.join(run, "odom.csv"))
t, xyz = traj_arrays(odom)
pts = read_pcd_xyz(os.path.join(run, "map.pcd"), max_points=8_000_000)
print(f"{run}: odom={len(xyz)} pts={len(pts)}")

if ALIGN:
    up = estimate_up(xyz, t)
    R = rot_from_to(up)
    tilt = np.degrees(np.arccos(min(1.0, abs(float(up[2])))))
    lap = (t - t[0]) < 115.0
    z0 = (xyz[lap] @ R.T)[:, 2] if lap.sum() > 50 else (xyz @ R.T)[:, 2]
    print(f"[align] up={np.round(up, 4)} 与 +z 夹角 {tilt:.2f}°, "
          f"第一圈 z 范围 {z0.min():+.2f}..{z0.max():+.2f} m")
    xyz = xyz @ R.T
    pts = pts @ R.T.astype(pts.dtype)

# 井道中心：爬楼段轨迹中位数
vz = np.gradient(xyz[:, 2])
fly = np.abs(vz) > 0.08
center = np.median(xyz[fly, :2], axis=0) if fly.sum() > 20 else np.median(xyz[:, :2], axis=0)
r = 12.0
sel = ((pts[:, 0] - center[0]) ** 2 + (pts[:, 1] - center[1]) ** 2) < r * r
pts = pts[sel]
zlo, zhi = np.percentile(pts[:, 2], 0.5), np.percentile(pts[:, 2], 99.5)
print(f"center=({center[0]:.2f},{center[1]:.2f})  z {zlo:.1f}..{zhi:.1f}  pts={len(pts)}")

rows = []
zc = zlo + zbin / 2
while zc < zhi:
    band = pts[(pts[:, 2] > zc - zbin / 2) & (pts[:, 2] < zc + zbin / 2)]
    if len(band) < 400:
        zc += zbin
        continue
    vm = vertical_filter(band, cell=0.2, min_span=0.6)
    wp = band[vm]
    if len(wp) < 300:
        zc += zbin
        continue
    lines = ransac_lines(wp[:, :2], min_inliers=max(80, int(0.02 * len(wp))),
                         max_lines=8, min_span=1.0, min_density=2.0, min_coverage=0.4)
    th, st = dominant_angle(lines, fold=4)
    if th is None or st < 0.3:
        zc += zbin
        continue
    rows.append({"z": zc, "th": th, "strength": st, "n": len(wp),
                 "n_lines": len(lines),
                 "span": max([L["span"] for L in lines]) if lines else 0.0})
    zc += zbin

if len(rows) < 4:
    print("有效层太少，放弃")
    sys.exit(0)

z = np.array([r["z"] for r in rows])
th = np.array([r["th"] for r in rows])
period = np.pi / 4
# mod-90° 连续展开
un = np.zeros_like(th)
un[0] = th[0]
for i in range(1, len(th)):
    un[i] = un[i - 1] + _wrap_to(th[i] - th[i - 1], period)
un_deg = np.degrees(un)

print(f"\n{'z':>8} {'yaw(deg)':>9} {'Δ相邻':>8} {'强度':>6} {'线数':>4} {'最长线':>6} {'点数':>7}")
prev = None
for i, r in enumerate(rows):
    d = "" if prev is None else f"{un_deg[i]-prev:+8.3f}"
    print(f"{r['z']:8.2f} {un_deg[i]:9.3f} {d:>8} {r['strength']:6.2f} {r['n_lines']:4d} {r['span']:6.1f} {r['n']:7d}")
    prev = un_deg[i]

A = np.vstack([z, np.ones_like(z)]).T
sol, *_ = np.linalg.lstsq(A, un_deg, rcond=None)
resid = un_deg - A @ sol
print(f"\n线性拟合: yaw = {sol[0]:+.4f} deg/m × z {sol[1]:+.2f}   "
      f"残差 std={resid.std():.3f}°  max|res|={np.abs(resid).max():.3f}°")
print(f"整段高度 {z[-1]-z[0]:.1f} m, yaw 变化 {un_deg[-1]-un_deg[0]:+.2f}°")
for h in (3.0, 3.7, 4.0, 4.5):
    print(f"  若层高 {h:.1f} m -> 每层 {sol[0]*h:+.3f}°  (占比 {sol[0]*h/360*100:+.3f}%)")
