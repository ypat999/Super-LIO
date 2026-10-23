#!/usr/bin/env python3
"""实验 B 复算：每帧观测修正（后验 R+ vs 预测 R−）在竖直轴上的投影 vs 角速度。

动机
----
去畸变后的单帧结构（实验 A）与转向无关，说明帧内补偿不是扭转漂移的来源。
剩下两种可能：
  a. 雷达点时刻比 IMU 晚 dt  => 观测把姿态"往回拉"，修正量 ∝ ω·dt；
  b. 陀螺整体偏小 k           => 同上，等效 dt。
两者都给 cy ∝ ω，只能靠改参数各跑一次来判别。
本脚本给出判据：把 corr = rotvec(exp(rv)·exp(rv_pred)ᵀ) 投影到世界竖直轴，
按 |ω| 分档看 cy/帧，哪一组参数把 60-120 档压到 0 就是主因。

口径
----
- rv/rv_pred 来自 debug.csv，是世界系姿态的 so3 log，修正量必须用
  exp(rv)·exp(rv_pred)ᵀ 合成，不能直接差分量（会丢二阶项）。
- ω 用 odom.csv 逐帧四元数：rotvec(R[i+1]·R[i]ᵀ)·up/dt。
- up_world 取 [0,0,-1]：Super-LIO 首帧做重力对齐，世界 z 即竖直（实测
  R(q0)·up_imu = [+0.005,+0.003,-1.000]，用精确 -z 的投影差 <1e-4）。

用法
----
  python3 obs_corr.py results_drone/sl_dbg2 results_drone/sl_toff_m3 ...
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from loop_error import quat_to_rot, rotvec

EDGES = [0.0, 10.0, 30.0, 60.0, 120.0, 1e9]


def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def exp_so3(v):
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    K = skew(v / th)
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * (K @ K)


def load(run):
    """返回逐帧 (t, w_up_deg_s, cy_deg, n_effect)。"""
    dbg = np.genfromtxt(os.path.join(run, "debug.csv"), delimiter=",", names=True)
    odo = np.genfromtxt(os.path.join(run, "odom.csv"), delimiter=",", names=True)
    if dbg.size < 50 or odo.size < 50:
        raise SystemExit(f"{run}: 行数不足（debug={dbg.size} odom={odo.size}）")

    t = odo["t_ns"].astype(np.float64) * 1e-9
    q = np.stack([odo["qx"], odo["qy"], odo["qz"], odo["qw"]], axis=1)
    R = np.stack([quat_to_rot(x) for x in q])
    up = np.array([0.0, 0.0, -1.0])
    dt = np.maximum(np.diff(t), 1e-6)
    w = np.array([rotvec(R[i + 1] @ R[i].T) for i in range(len(R) - 1)]) @ up / dt
    tm = 0.5 * (t[:-1] + t[1:])

    td = dbg["t"].astype(np.float64)
    i = np.clip(np.searchsorted(tm, td), 0, len(tm) - 1)
    wq = w[i]

    rv = np.stack([dbg["rvx"], dbg["rvy"], dbg["rvz"]], axis=1)
    rp = np.stack([dbg["rvx_pred"], dbg["rvy_pred"], dbg["rvz_pred"]], axis=1)
    cy = np.array([rotvec(exp_so3(a) @ exp_so3(b).T) for a, b in zip(rv, rp)]) @ up
    return td, np.degrees(wq), np.degrees(cy), dbg["n_effect"].astype(float)


def bins(w, cy, wmin):
    out = []
    for lo, hi in zip(EDGES[:-1], EDGES[1:]):
        for sgn, name in ((1, "+"), (-1, "-")):
            if sgn > 0:
                m = (w >= lo) & (w < hi) & (w > 0)
            else:
                m = (-w >= lo) & (-w < hi) & (w < 0)
            if m.sum() == 0:
                continue
            out.append((lo, hi, name, int(m.sum()), cy[m].sum(), cy[m].mean(),
                        np.median(np.abs(cy[m]))))
    return out


def fit(w, cy, wmin):
    m = np.abs(w) > wmin
    if m.sum() < 50:
        return None
    A = np.vstack([np.ones(m.sum()), w[m]]).T
    (a, b), *_ = np.linalg.lstsq(A, cy[m], rcond=None)
    pred = a + b * w[m]
    ss = ((cy[m] - pred) ** 2).sum()
    st = ((cy[m] - cy[m].mean()) ** 2).sum()
    return a, b, 1.0 - ss / max(st, 1e-12), int(m.sum())


def load_axes(run):
    """逐帧 (t, h, w_world(3,N) deg/s, c_world(3,N) deg, n_effect)。

    c_world = rotvec(exp(rv)·exp(rv_pred)ᵀ)：rv/rv_pred 都是世界系姿态的 so3 log，
    所以修正量直接就是世界系三分量，c_x/c_y 是绕世界水平轴的修正（俯仰/横滚），
    c_z 是绕竖直轴的修正（偏航）。h 用 debug.csv 自己的 pz（Super-LIO 世界 z 朝下）。
    """
    dbg = np.genfromtxt(os.path.join(run, "debug.csv"), delimiter=",", names=True)
    odo = np.genfromtxt(os.path.join(run, "odom.csv"), delimiter=",", names=True)
    if dbg.size < 50 or odo.size < 50:
        raise SystemExit(f"{run}: 行数不足（debug={dbg.size} odom={odo.size}）")

    t = odo["t_ns"].astype(np.float64) * 1e-9
    q = np.stack([odo["qx"], odo["qy"], odo["qz"], odo["qw"]], axis=1)
    R = np.stack([quat_to_rot(x) for x in q])
    dt = np.maximum(np.diff(t), 1e-6)
    wv = np.array([rotvec(R[i + 1] @ R[i].T) for i in range(len(R) - 1)]) / dt[:, None]
    tm = 0.5 * (t[:-1] + t[1:])

    td = dbg["t"].astype(np.float64)
    i = np.clip(np.searchsorted(tm, td), 0, len(tm) - 1)

    rv = np.stack([dbg["rvx"], dbg["rvy"], dbg["rvz"]], axis=1)
    rp = np.stack([dbg["rvx_pred"], dbg["rvy_pred"], dbg["rvz_pred"]], axis=1)
    c = np.array([rotvec(exp_so3(a) @ exp_so3(b).T) for a, b in zip(rv, rp)])
    return (td - td[0], -dbg["pz"].astype(np.float64), np.degrees(wv[i]),
            np.degrees(c), dbg["n_effect"].astype(float))


def axes_report(run, td, h, wv, cv):
    """分轴汇总：修正量累计、逐帧均值、对同轴角速度的回归斜率。"""
    from floor_yaw import find_legs, find_plateaus
    ph = [(f"plat{k}", a, b) for k, (a, b) in enumerate(find_plateaus(td, h))]
    ph += [(("up" if s > 0 else "dn"), a, b) for a, b, s in find_legs(td, h)]
    ph = sorted(ph, key=lambda x: x[1])
    print(f"\n=== {run}")
    print(f"  {'段':<7}{'t0':>7}{'t1':>7}{'Σcx°':>9}{'Σcy°':>9}{'Σcz°':>9}"
          f"{'cx/帧':>8}{'cy/帧':>8}{'cz/帧':>8}")
    for nm, i0, i1 in ph:
        if i1 - i0 < 5:
            continue
        s = cv[i0:i1].sum(0)
        n = i1 - i0
        print(f"  {nm:<7}{td[i0]:>7.1f}{td[i1]:>7.1f}{s[0]:>+9.2f}{s[1]:>+9.2f}"
              f"{s[2]:>+9.2f}{s[0]/n:>+8.4f}{s[1]/n:>+8.4f}{s[2]/n:>+8.4f}")
    tot = cv.sum(0)
    print(f"  {'全包':<7}{td[0]:>7.1f}{td[-1]:>7.1f}{tot[0]:>+9.2f}{tot[1]:>+9.2f}"
          f"{tot[2]:>+9.2f}   （累计修正）")
    print(f"  {'轴':<4}{'回归 c = a + b·ω':>26}{'R²':>8}{'|ω|中位':>9}{'|c|中位':>9}")
    for k, ax in enumerate("xyz"):
        m = np.abs(wv[:, k]) > 10.0
        if m.sum() < 50:
            continue
        A = np.vstack([np.ones(m.sum()), wv[m, k]]).T
        (a, b), *_ = np.linalg.lstsq(A, cv[m, k], rcond=None)
        pred = a + b * wv[m, k]
        r2 = 1.0 - ((cv[m, k] - pred) ** 2).sum() / max(
            ((cv[m, k] - cv[m, k].mean()) ** 2).sum(), 1e-12)
        print(f"  {ax:<4}{a:>+12.4f} {b:>+12.5f} {r2:>8.3f}"
              f"{np.median(np.abs(wv[:, k])):>9.1f}{np.median(np.abs(cv[:, k])):>9.4f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wmin", type=float, default=10.0)
    ap.add_argument("--axes", action="store_true",
                    help="分轴：绕世界 x/y（俯仰/横滚）vs z（偏航）的观测修正")
    ap.add_argument("--detail", action="store_true", help="打印每个 run 的完整分档表")
    ap.add_argument("runs", nargs="+")
    a = ap.parse_args()

    if a.axes:
        for run in a.runs:
            axes_report(os.path.basename(run.rstrip("/")), *load_axes(run)[:4])
        return

    summary = []
    for run in a.runs:
        name = os.path.basename(run.rstrip("/"))
        td, w, cy, neff = load(run)
        print(f"\n=== {name}  帧 {len(w)}  |ω|>10 的 {int((np.abs(w) > 10).sum())} 帧"
              f"  n_eff 中位 {np.median(neff):.0f}")
        f = fit(w, cy, a.wmin)
        if f:
            print(f"回归 cy = {f[0]:+.4f} + {f[1]:+.6f}·ω[deg/s]   "
                  f"R²={f[2]:.3f}  n={f[3]}   => 等效 {f[1]*1e3:+.3f} ms")
        b = bins(w, cy, a.wmin)
        if a.detail:
            print(f"{'|ω|档':>12}{'符号':>4}{'帧':>6}{'Σcy°':>9}{'cy/帧°':>9}{'中位|cy|°':>10}")
            for lo, hi, s, n, scy, mcy, mabs in b:
                tag = f"{lo:.0f}-{hi:.0f}" if hi < 1e8 else f"{lo:.0f}+"
                print(f"{tag:>12}{s:>4}{n:>6}{scy:>+9.2f}{mcy:>+9.4f}{mabs:>10.3f}")
        hi_bin = [x for x in b if x[0] == 60.0]
        summary.append((name, f, hi_bin, float(np.median(neff))))

    print(f"\n{'run':<22}{'斜率(ms)':>10}{'R²':>7}{'60-120+':>10}{'60-120-':>10}{'n_eff中位':>10}")
    for name, f, hb, neff in summary:
        pos = next((x[5] for x in hb if x[2] == "+"), float("nan"))
        neg = next((x[5] for x in hb if x[2] == "-"), float("nan"))
        if f:
            print(f"{name:<22}{f[1]*1e3:>+10.3f}{f[2]:>7.3f}"
                  f"{pos:>+10.4f}{neg:>+10.4f}{neff:>10.0f}")
        else:
            print(f"{name:<22}{'n/a':>10}")


if __name__ == "__main__":
    main()
