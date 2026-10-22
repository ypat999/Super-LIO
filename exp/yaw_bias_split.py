#!/usr/bin/env python3
"""把每帧世界系 yaw 变化拆成「IMU 预测增量」与「配准观测修正量」，用于定位系统性 yaw 偏置。

用法：python3 yaw_bias_split.py <run_dir> [seg_s]

输入：<run_dir>/debug.csv（需 lio.debug.level>=1，含 rv/rv_pred/bg 列）
输出：
  1) 全程分解：Σ预测、Σ观测、Σ后验，观测修正的符号平衡与量级
  2) 按 seg_s 分段的同口径分解（看偏置是否集中在某段/随高度累积）
  3) 灵敏度积分：单位残差零偏 (rad/s) 在 x/y/z 上各能造成多少 deg 世界 yaw
  4) 尺度型假设所需的陀螺比例误差（用总转角路径换算）
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import load_odom, traj_arrays

run = sys.argv[1]
seg_s = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0


def so3_exp(w):
    th = np.linalg.norm(w, axis=-1)
    K = np.zeros(w.shape[:-1] + (3, 3))
    K[..., 0, 1] = -w[..., 2]
    K[..., 0, 2] = w[..., 1]
    K[..., 1, 0] = w[..., 2]
    K[..., 1, 2] = -w[..., 0]
    K[..., 2, 0] = -w[..., 1]
    K[..., 2, 1] = w[..., 0]
    small = th < 1e-8
    a = np.where(small, 1.0, np.sin(th) / np.where(small, 1.0, th))
    b = np.where(small, 0.5, (1 - np.cos(th)) / np.where(small, 1.0, th ** 2))
    return np.eye(3) + a[..., None, None] * K + b[..., None, None] * (K @ K)


def so3_log(R):
    """返回旋转向量（N,3）；只在接近单位阵时使用，走小角度分支。"""
    v = np.stack([R[:, 2, 1] - R[:, 1, 2],
                  R[:, 0, 2] - R[:, 2, 0],
                  R[:, 1, 0] - R[:, 0, 1]], axis=1) * 0.5
    return v


d = np.genfromtxt(os.path.join(run, "debug.csv"), delimiter=",", names=True)
n = d.size
# debug.csv 的 t 列在旧版里被默认精度量化（1e4 s），不能直接求差；
# 这里按行号线性铺到 odom 的实际时间跨度上（帧率抖动 <1%，对 60 s 分段无影响）。
odom = load_odom(os.path.join(run, "odom.csv"))
if odom is not None:
    todom, xyz = traj_arrays(odom)
    total_s = float(todom[-1] - todom[0])
else:
    xyz = None
    total_s = float(d["t"][-1] - d["t"][0]) or 1278.8
if total_s <= 0 or not np.isfinite(total_s):
    total_s = 1278.8
t = np.linspace(0.0, total_s, n)
Rp = so3_exp(np.stack([d["rvx"], d["rvy"], d["rvz"]], axis=1))
Rq = so3_exp(np.stack([d["rvx_pred"], d["rvy_pred"], d["rvz_pred"]], axis=1))

# dR_obs = R_post * R_pred^T  （世界系，观测在当帧施加的增量）
Robs = Rp @ np.transpose(Rq, (0, 2, 1))
yaw_obs = so3_log(Robs)[:, 2]
# dR_imu = R_pred,k * R_post,k-1^T （世界系，预测在当帧施加的增量）
Rimu = Rq[1:] @ np.transpose(Rp[:-1], (0, 2, 1))
yaw_imu = np.concatenate([[0.0], so3_log(Rimu)[:, 2]])
# 后验增量（真值口径，用来校验分解闭合）
Rpost = Rp[1:] @ np.transpose(Rp[:-1], (0, 2, 1))
yaw_post = np.concatenate([[0.0], so3_log(Rpost)[:, 2]])

deg = 180.0 / np.pi
yaw_imu_d, yaw_obs_d, yaw_post_d = yaw_imu * deg, yaw_obs * deg, yaw_post * deg

print(f"{run}: {n} 帧, {t[-1]:.0f} s")
print("\n=== 1) 全程分解（世界系 yaw，单位 deg） ===")
print(f"  Σ IMU 预测增量        : {yaw_imu_d.sum():+9.2f}")
print(f"  Σ 观测修正量          : {yaw_obs_d.sum():+9.2f}")
print(f"  Σ 后验增量            : {yaw_post_d.sum():+9.2f}   (闭合误差 {yaw_post_d.sum()-yaw_imu_d.sum()-yaw_obs_d.sum():+.3e})")
print(f"  Σ|IMU 预测增量|       : {np.abs(yaw_imu_d).sum():9.2f}   (总转角路径)")
print(f"  Σ|观测修正量|         : {np.abs(yaw_obs_d).sum():9.2f}")
print(f"  每帧观测修正 RMS      : {np.sqrt((yaw_obs_d**2).mean()):9.4f}   |max|={np.abs(yaw_obs_d).max():.4f}")
print(f"  观测修正符号 正/负    : {100.0*(yaw_obs_d>0).mean():.1f}% / {100.0*(yaw_obs_d<0).mean():.1f}%")
print(f"  观测/预测 的比值      : {yaw_obs_d.sum()/yaw_imu_d.sum()*100:+.2f} %")

# 修正量 vs 预测增量的回归（常值偏置 = 截距；比例型 = 斜率）
if yaw_imu_d.std() > 1e-9:
    A = np.vstack([yaw_imu_d, np.ones_like(yaw_imu_d)]).T
    sol, *_ = np.linalg.lstsq(A, yaw_obs_d, rcond=None)
    resid = yaw_obs_d - A @ sol
    print(f"  corr = {sol[0]:+.5f}·pred {sol[1]:+.5f}   (R²={1-resid.var()/yaw_obs_d.var():.3f})")
    print(f"    → 斜率≈比例型误差 {sol[0]*100:+.2f}% ；截距≈常值偏置 {sol[1]:+.5f} deg/帧"
          f" = {sol[1]/(np.median(np.diff(t))):+.4f} deg/s")

# 单位残差零偏的灵敏度：eps(t) = -∫(R·e_bias)_z dt
Rpost_avg = Rp
Rt = np.transpose(Rp, (0, 2, 1))
dt = np.gradient(t)
sens = {}
for i, ax in enumerate("xyz"):
    e = np.zeros(3)
    e[i] = 1.0
    w = Rpost_avg @ e            # 世界系
    sens[ax] = np.sum(w[:, 2] * dt) * deg   # rad/s * s -> deg
print("\n=== 2) 残差零偏灵敏度（1e-3 rad/s 常值偏置能造成多少 deg 世界 yaw） ===")
for ax in "xyz":
    print(f"  bg_{ax}: {sens[ax]*1e-3:+8.2f} deg / 全程；折合 {sens[ax]*1e-3/t[-1]*60:+7.3f} deg/min")

print(f"\n=== 3) 分段（{seg_s:.0f} s）分解 ===")
print(f"{'t0':>7} {'z(m)':>7} {'Σ|pred|':>9} {'Σpred':>8} {'Σobs':>8} {'obs+%':>6} {'|obs|RMS':>9}")
edges = np.arange(0, t[-1] + seg_s, seg_s)
for a, b in zip(edges[:-1], edges[1:]):
    m = (t >= a) & (t < b)
    if m.sum() < 5:
        continue
    print(f"{a:7.0f} {d['pz'][m][-1]:7.2f} {np.abs(yaw_imu_d[m]).sum():9.2f} "
          f"{yaw_imu_d[m].sum():+8.2f} {yaw_obs_d[m].sum():+8.2f} "
          f"{100.0*(yaw_obs_d[m]>0).mean():6.1f} {np.sqrt((yaw_obs_d[m]**2).mean()):9.4f}")

print("\n=== 4) bg 首尾 ===")
for ax in "xyz":
    v = d["bg" + ax]
    print(f"  bg_{ax}: {v[:50].mean()*deg:+8.4f} -> {v[-50:].mean()*deg:+8.4f} deg/s "
          f"(Δ={(v[-50:].mean()-v[:50].mean())*deg:+.4f})")

odom2 = load_odom(os.path.join(run, "odom.csv"))
if odom2 is not None:
    _, xyz2 = traj_arrays(odom2)
    print(f"\n  odom: {len(xyz2)} 行, z {xyz2[:,2].min():+.1f}..{xyz2[:,2].max():+.1f} m")
