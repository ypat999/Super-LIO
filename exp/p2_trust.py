#!/usr/bin/env python3
"""M.40 E-2：Trust 阈值正向核验（16~25° 的真实修正帧有多少、贡献多大）。

输入一个 DEBUG_LEVEL=2 的 run 目录（含 debug.csv / node.log），输出：

  1. 逐帧姿态修正角 corr_r = |log(R_post * R_pred^T)| 的分位与分档计数
     （rv / rv_pred 两列即后验与预测的旋转向量）；
  2. 16~25° 档的具体帧：t、corr_r、deg_cond、n_effect —— 这些正是
     trust_jump_rot_max=15 时会被整帧否决、25 时会被采纳的修正；
  3. node.log 里 [Trust] 否决记录（阈值 15 跑的 run 才有）；
  4. 平移修正 |p_post - p_pred| 的同期分布（debug 无 p_pred，改用相邻帧
     位移与 rv 同步比对，仅作量级参考）。

用法:
  ./p2_trust.py results/m40_e2_1f_r15
"""
import argparse
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import load_odom  # noqa: F401  (保持与其它分析脚本同一依赖)


def read_debug_csv(path):
    with open(path) as f:
        head = f.readline().strip().split(",")
    arr = np.genfromtxt(path, delimiter=",", skip_header=1)
    return head, arr


def so3_exp(v):
    th = np.linalg.norm(v, axis=1)
    K = np.zeros((len(v), 3, 3))
    K[:, 0, 1], K[:, 0, 2] = -v[:, 2], v[:, 1]
    K[:, 1, 0], K[:, 1, 2] = v[:, 2], -v[:, 0]
    K[:, 2, 0], K[:, 2, 1] = -v[:, 1], v[:, 0]
    s = np.sinc(th / np.pi)[:, None, None]
    c = ((1 - np.cos(th)) / np.maximum(th * th, 1e-16))[:, None, None]
    return np.eye(3)[None] + s * K + c * (K @ K)


def corr_angle(rv, rv_pred):
    R = so3_exp(rv) @ np.transpose(so3_exp(rv_pred), (0, 2, 1))
    c = np.clip((np.trace(R, axis1=1, axis2=2) - 1.0) / 2.0, -1.0, 1.0)
    return np.degrees(np.arccos(c))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    a = ap.parse_args()

    dbg = os.path.join(a.run, "debug.csv")
    head, arr = read_debug_csv(dbg)
    ix = {n: i for i, n in enumerate(head)}
    rv = arr[:, [ix["rvx"], ix["rvy"], ix["rvz"]]]
    rvp = arr[:, [ix["rvx_pred"], ix["rvy_pred"], ix["rvz_pred"]]]
    t = arr[:, ix["t"]]
    cond = arr[:, ix["deg_cond"]]
    neff = arr[:, ix["n_effect"]] if "n_effect" in ix else np.zeros(len(t))

    ang = corr_angle(rv, rvp)
    print(f"== {a.run} ==")
    print(f"帧数 {len(t)}  时长 {t[-1] - t[0]:.1f}s")
    print("corr_r 分位(deg): p50=%.2f p90=%.2f p95=%.2f p99=%.2f max=%.2f"
          % tuple(np.percentile(ang, [50, 90, 95, 99, 100])))
    bins = [(0, 1), (1, 5), (5, 15), (15, 25), (25, 1e9)]
    for lo, hi in bins:
        n = int(((ang >= lo) & (ang < hi)).sum())
        print(f"  {lo:>4.0f}~{hi if hi < 1e8 else 'inf':>4} deg : {n:5d} 帧 "
              f"({100.0 * n / len(t):5.2f}%)")

    m = (ang >= 15) & (ang < 25)
    if m.any():
        print("\n-- 15~25 deg 修正帧（rot_max=15 会否决、=25 会采纳）--")
        print(f"{'t(s)':>9}{'corr_r':>8}{'deg_cond':>10}{'n_effect':>9}")
        for i in np.argsort(-ang * m)[:25]:
            if not m[i]:
                break
            print(f"{t[i] - t[0]:>9.2f}{ang[i]:>8.2f}{cond[i]:>10.2f}{neff[i]:>9.0f}")
        print(f"  该档 deg_cond: p50={np.median(cond[m]):.1f}")
        rest = ~m & (ang < 15)
        print(f"  其余帧 deg_cond: p50={np.median(cond[rest]):.1f}")

    log = os.path.join(a.run, "node.log")
    if os.path.exists(log):
        pat = re.compile(r"\[Trust\].*?(?:rot|deg)[^0-9\-]*([0-9.]+)")
        hits = []
        with open(log, errors="ignore") as f:
            for line in f:
                if "[Trust]" in line:
                    hits.append(line.rstrip())
        print(f"\n-- node.log [Trust] 行: {len(hits)} 条 --")
        for h in hits[:15]:
            print("  " + h[:160])
        if len(hits) > 15:
            print(f"  ... 其余 {len(hits) - 15} 条略")


if __name__ == "__main__":
    main()
