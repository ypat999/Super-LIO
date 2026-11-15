#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""1f 双圈：定位回到起点的时刻，切分两圈并给出各圈的里程/端点偏移（E-5 用）。"""
import sys
import os
import math
import csv


def load(p):
    rows = []
    with open(os.path.join(p, "odom.csv")) as f:
        for d in csv.DictReader(f):
            rows.append((int(d["t_ns"]), float(d["x"]), float(d["y"]), float(d["z"])))
    return rows


def main():
    for p in sys.argv[1:]:
        r = load(p)
        t0 = r[0][0]
        s = r[0]
        # 先离开起点 >10 m，再找第一次回到 1 m 内的帧（双圈分界）
        left = False
        best = None
        for i, x in enumerate(r):
            d = math.hypot(x[1] - s[1], x[2] - s[2])
            if not left and d > 10.0:
                left = True
            elif left and d < 1.0 and i < len(r) - 100:
                best = i
                break
        if best is None:
            print("%s: 未找到回到起点的圈分界" % os.path.basename(p.rstrip("/")))
            continue
        b = r[best]
        seg1 = sum(math.dist(r[i][1:4], r[i + 1][1:4]) for i in range(best))
        seg2 = sum(math.dist(r[i][1:4], r[i + 1][1:4]) for i in range(best, len(r) - 1))
        print("%s" % os.path.basename(p.rstrip("/")))
        print("  圈分界 t=%.1f s  到起点水平距离 %.3f m  dz %.3f m"
              % ((b[0] - t0) / 1e9, math.hypot(b[1] - s[1], b[2] - s[2]), b[3] - s[3]))
        print("  第1圈 %.1f m / %.1f s    第2圈 %.1f m / %.1f s"
              % (seg1, (b[0] - t0) / 1e9, seg2, (r[-1][0] - b[0]) / 1e9))
        e = r[-1]
        print("  终点 3D %.3f m（水平 %.3f / 竖直 %.3f）"
              % (math.dist(s[1:4], e[1:4]), math.hypot(e[1] - s[1], e[2] - s[2]), e[3] - s[3]))


if __name__ == "__main__":
    main()
