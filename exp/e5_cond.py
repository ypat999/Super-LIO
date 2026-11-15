#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统计 1f 两条线的退化条件数 deg_cond 与 [Degeneracy] 触发帧占比。"""
import sys
import os
import csv


def pct(v, a):
    return v[min(len(v) - 1, int(a * len(v)))]


for p in sys.argv[1:]:
    f = os.path.join(p, "debug.csv")
    conds = []
    with open(f) as fh:
        for d in csv.DictReader(fh):
            if d.get("deg_cond") not in (None, ""):
                conds.append(float(d["deg_cond"]))
    conds.sort()
    n = len(conds)
    print("%-20s 帧 %d  deg_cond p50=%.1f p90=%.1f p95=%.1f max=%.1f  >150 占 %.1f%%"
          % (os.path.basename(p.rstrip("/")), n, pct(conds, .5), pct(conds, .9),
             pct(conds, .95), conds[-1],
             100.0 * sum(1 for c in conds if c > 150) / n))
