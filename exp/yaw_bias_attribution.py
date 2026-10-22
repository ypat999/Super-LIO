#!/usr/bin/env python3
"""定位「逐层同向同角度 yaw 偏转」的来源：常值零偏残差(∝时间) 还是 陀螺 scale(∝转角)。

思路
----
把每层墙面相对参考层的 yaw 偏差 e_i（wall_metrics 已给出）当成"该时刻的 yaw 误差"，
再对两个候选自变量做回归：

    e(t) = b0 + b_t * t + b_rot * Θ(t)

  b_t   [deg/s]      → 常值角速度残差（零偏没跟住 / 观测注入的固定 yaw 速率）
  b_rot [deg/deg]    → 陀螺 scale 误差（转得越多偏得越多，与时间无关）

两种机制在"每层转 4x90°、爬 3.7 m"这种重复节拍下都会表现成"每层同向同角度"，
但 1F 大厅/10F 起点长时间停留（时间多、转角少）可以把二者区分开，所以先看
t 与 Θ 的相关性，再决定结论可信度。

另外输出每层 e_i 与层序的差分量，检验是否真的"每层一样大"。

用法
----
  python3 yaw_bias_attribution.py results_xxx/expy1_m2_dbg [ref_floor_name]
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import load_odom, traj_arrays
from wall_metrics import analyze_run

DEG = 180.0 / np.pi


def yaw_of(odom):
    qx, qy, qz, qw = (odom[k].astype(np.float64) for k in ("qx", "qy", "qz", "qw"))
    return np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def main():
    run = sys.argv[1]
    ref_name = sys.argv[2] if len(sys.argv) > 2 else "10"

    res = analyze_run(run, ref_floor_name=ref_name)
    if not res or not res.get("rows"):
        print("无可用的分层墙面结果")
        return
    rows = [r for r in res["rows"] if np.isfinite(r.get("dyaw_deg", np.nan))]
    if len(rows) < 4:
        print("可靠层数不足")
        return

    odom = load_odom(os.path.join(run, "odom.csv"))
    t_abs, xyz = traj_arrays(odom)
    t = t_abs - t_abs[0]
    yaw = yaw_of(odom)
    dyaw = wrap(np.diff(yaw))
    dt = np.diff(t)
    # 累计"绝对转角"：每帧转角取模长，与转向正负无关（scale 误差只看走过的角度）
    cum_rot = np.concatenate([[0.0], np.cumsum(np.abs(dyaw))]) * DEG

    print(f"{run}")
    print(f"{'floor':>6} {'rel':>4} {'z':>7} {'t_mid':>7} {'dwell':>6} "
          f"{'e_yaw':>7} {'Θ':>9} {'reli':>4}")
    T, TH, E = [], [], []
    prev_e = None
    for r in rows:
        tm = float(r["t_first"]) + float(r.get("dwell", 0.0)) * 0.5
        th = float(np.interp(tm, t, cum_rot))
        e = float(r["dyaw_deg"])
        print(f"{r['floor']:>6} {r['rel']:>4} {r['z']:7.2f} {tm:7.1f} "
              f"{r.get('dwell', 0.0):6.1f} {e:7.3f} {th:9.1f} {r.get('reliable', 0):4d}"
              + (f"   Δ{e - prev_e:+.3f}" if prev_e is not None else ""))
        prev_e = e
        if r.get("reliable", 1):
            T.append(tm)
            TH.append(th)
            E.append(e)

    T = np.asarray(T)
    TH = np.asarray(TH)
    E = np.asarray(E)
    if len(T) < 4:
        print("\n可靠样本不足，无法回归")
        return

    print(f"\n可靠样本 {len(T)} 个；t 范围 {T.min():.0f}..{T.max():.0f} s，"
          f"Θ 范围 {TH.min():.0f}..{TH.max():.0f} deg")
    c = np.corrcoef(T, TH)[0, 1]
    print(f"corr(t, Θ) = {c:+.4f}"
          + ("   ← 强共线，两个机制无法分离" if abs(c) > 0.98 else "   ← 可分离"))

    # 单变量拟合
    for name, X in (("t [s]", T), ("Θ [deg]", TH)):
        A = np.vstack([X, np.ones_like(X)]).T
        sol, *_ = np.linalg.lstsq(A, E, rcond=None)
        r2 = 1.0 - ((E - A @ sol) ** 2).sum() / max(((E - E.mean()) ** 2).sum(), 1e-12)
        print(f"  仅 {name:9s}: 斜率 {sol[0]:+.3e} deg/{name.split()[0]}  R²={r2:.3f}")

    A = np.vstack([T, TH, np.ones_like(T)]).T
    sol, *_ = np.linalg.lstsq(A, E, rcond=None)
    r2 = 1.0 - ((E - A @ sol) ** 2).sum() / max(((E - E.mean()) ** 2).sum(), 1e-12)
    print(f"\n二元回归 e = {sol[1]:+.3e}*Θ + {sol[0]:+.3e}*t + {sol[2]:+.3f}   R²={r2:.3f}")
    print(f"  等价陀螺 scale 误差 = {sol[1] * 1e6:+.1f} ppm")
    print(f"  等价零偏残差        = {sol[0] / DEG:+.2e} rad/s"
          f"  ({sol[0] * 60:+.3f} deg/min)")

    # 每层增量：检验"每层偏同样角度"
    print("\n逐层增量（相邻可靠层 e 的差 / 层间隔）：")
    for i in range(1, len(T)):
        dz = rows[i].get("z", np.nan)
        print(f"  #{i}: Δe={E[i] - E[i - 1]:+.3f} deg, Δt={T[i] - T[i - 1]:+.1f} s, "
              f"ΔΘ={TH[i] - TH[i - 1]:+.1f} deg")


if __name__ == "__main__":
    main()
