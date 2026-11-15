#!/usr/bin/env python3
"""E-1 辅助：同一 run 内不同"周期数窗口"的 deg_cond 分布对比。

P-2 的判据之一是"cond 分布几乎不动"：如果重复周期数增加时 cond 分布基本不变，
而 A/B 轨迹差显著变大，就说明漂移来自"反复匹配同一结构的累积"，不是退化度变化。
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WIN = [(0.0, 46.2), (0.0, 79.4), (0.0, 132.4)]


def load(run):
    p = os.path.join(HERE, run, "debug.csv")
    with open(p) as f:
        head = f.readline().strip().split(",")
    arr = np.genfromtxt(p, delimiter=",", skip_header=1)
    ix = {n: i for i, n in enumerate(head)}
    t = arr[:, ix["t"]]
    t = t - t[0]
    return t, arr[:, ix["deg_cond"]]


for run in sys.argv[1:]:
    t, cond = load(run)
    print(f"== {run}  帧数={len(t)} 时长={t[-1]:.1f}s")
    for lo, hi in WIN:
        m = (t >= lo) & (t <= hi)
        c = cond[m]
        print("  窗 %.0f~%.0f s  帧 %4d  cond p50=%7.1f p95=%7.1f >150=%.1f%%"
              % (lo, hi, m.sum(), np.percentile(c, 50), np.percentile(c, 95),
                 100.0 * (c > 150).mean()))
    print("  全段         帧 %4d  cond p50=%7.1f p95=%7.1f >150=%.1f%%"
          % (len(t), np.percentile(cond, 50), np.percentile(cond, 95),
             100.0 * (cond > 150).mean()))
