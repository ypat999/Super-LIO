#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M.40 E-4 判据：未优化 vs 优化（OctVoxMap 占用位图 + AABB 剪枝）的
Observe 耗时、deg_cond 分布、轨迹一致性与地图点数。

用法: python3 e4_cmp.py [基线run] [优化run]
"""
import os
import re
import sys

import numpy as np

EXP = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(EXP, "results")

PAT = re.compile(r"\[ \[(\w+)\] \] avg: ([0-9.eE+-]+) ms , max: ([0-9.eE+-]+) ms , calls: (\d+)")


def timings(run):
    acc = {}
    with open(os.path.join(RES, run, "node.log"), errors="ignore") as f:
        for line in f:
            m = PAT.search(line)
            if not m:
                continue
            k, avg, mx, n = m.group(1), float(m.group(2)), float(m.group(3)), int(m.group(4))
            a = acc.setdefault(k, [0.0, 0.0, 0, 0.0])
            a[0] += avg * n
            a[1] = max(a[1], mx)
            a[2] += n
            a[3] += avg
    return acc


def cond(run):
    p = os.path.join(RES, run, "debug.csv")
    if not os.path.exists(p):
        return None
    with open(p) as f:
        hdr = f.readline().strip().split(",")
    d = np.genfromtxt(p, delimiter=",", names=True)
    c = d["deg_cond"]
    c = c[np.isfinite(c) & (c > 0)]
    if not len(c):
        return None
    return (np.median(c), np.percentile(c, 95), float((c > 150).mean() * 100), len(c))


def odom(run):
    d = np.genfromtxt(os.path.join(RES, run, "odom.csv"), delimiter=",", names=True)
    return np.stack([d["x"], d["y"], d["z"]], 1)


def main():
    base = sys.argv[1] if len(sys.argv) > 1 else "m40_e4_base"
    opt = sys.argv[2] if len(sys.argv) > 2 else "m40_e4_opt"
    for run in (base, opt):
        acc = timings(run)
        print("== %s" % run)
        for k in ("Undistort", "Observe"):
            if k in acc:
                s, mx, n, _ = acc[k]
                print("   %-10s 加权均值 %.2f ms/帧  最大 %.2f ms  calls=%d"
                      % (k, s / max(n, 1), mx, n))
        c = cond(run)
        if c:
            print("   deg_cond   p50=%.1f  p95=%.1f  >150 占比=%.1f%%  (n=%d)" % c)
        x = odom(run)
        print("   odom %d 行  末位姿 (%.3f %.3f %.3f)  路径 %.2f m"
              % (len(x), *x[-1], np.linalg.norm(np.diff(x, axis=0), axis=1).sum()))
    xa, xb = odom(base), odom(opt)
    n = min(len(xa), len(xb))
    d = np.linalg.norm(xa[:n] - xb[:n], axis=1)
    print("\n轨迹逐帧差（前 %d 行）: 中位=%.4f m  p95=%.4f m  max=%.4f m 末帧=%.4f m"
          % (n, np.median(d), np.percentile(d, 95), d.max(), d[-1]))


if __name__ == "__main__":
    main()
