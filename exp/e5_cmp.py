#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按时间最近邻配对两条 odom，给出位置差的分位数与逐段演化（E-5 用）。

用法: python3 e5_cmp.py <ref_dir> <cmp_dir> [dt_max_ms]
输出: 配对帧数、位置差 p50/p95/max、每隔 60 s 的平均差、末端差。
"""
import sys
import os
import math
import csv
import bisect


def load(p):
    ts, ps = [], []
    with open(os.path.join(p, "odom.csv")) as f:
        for d in csv.DictReader(f):
            ts.append(int(d["t_ns"]))
            ps.append((float(d["x"]), float(d["y"]), float(d["z"])))
    return ts, ps


def main():
    ref, cmp_ = sys.argv[1], sys.argv[2]
    dt_max = int(sys.argv[3]) * 1e6 if len(sys.argv) > 3 else 20e6
    t1, p1 = load(ref)
    t2, p2 = load(cmp_)
    diffs = []
    for t, p in zip(t2, p2):
        i = bisect.bisect_left(t1, t)
        cand = [j for j in (i - 1, i) if 0 <= j < len(t1)]
        if not cand:
            continue
        j = min(cand, key=lambda k: abs(t1[k] - t))
        if abs(t1[j] - t) > dt_max:
            continue
        diffs.append((t, math.dist(p, p1[j]), math.hypot(p[0] - p1[j][0], p[1] - p1[j][1]),
                      p[2] - p1[j][2]))
    if not diffs:
        sys.exit("没有可配对的帧")
    vals = sorted(d for _, d, _, _ in diffs)
    n = len(vals)
    q = lambda a: vals[min(n - 1, int(a * n))]
    hs = sorted(h for _, _, h, _ in diffs)
    zs = sorted(abs(z) for _, _, _, z in diffs)
    print("配对 %d 帧  p50=%.3f  p90=%.3f  p95=%.3f  max=%.3f  末端=%.3f m"
          % (n, q(0.5), q(0.9), q(0.95), vals[-1], diffs[-1][1]))
    print("  水平分量 p50=%.3f p95=%.3f | 竖直|dz| p50=%.3f p95=%.3f  末端 dz=%+.3f"
          % (hs[n // 2], hs[min(n - 1, int(0.95 * n))], zs[n // 2],
             zs[min(n - 1, int(0.95 * n))], diffs[-1][3]))
    t0 = diffs[0][0]
    print("时间窗内的平均差（每 60 s，单位 m，格式 总/水平/竖直）：")
    bucket = {}
    for t, d, h, z in diffs:
        k = int((t - t0) / 1e9 // 60)
        bucket.setdefault(k, []).append((d, h, z))
    line = []
    for k in sorted(bucket):
        b = bucket[k]
        line.append("%3ds %.2f/%.2f/%+.2f"
                    % (k * 60, sum(x[0] for x in b) / len(b),
                       sum(x[1] for x in b) / len(b), sum(x[2] for x in b) / len(b)))
    for i in range(0, len(line), 4):
        print("  " + " | ".join(line[i:i + 4]))


if __name__ == "__main__":
    main()
