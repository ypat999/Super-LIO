#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""打印两条 odom 的 z / 水平半径随时间演化（每 30 s），用于判断漂移是斜坡还是台阶。"""
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
    runs = [(os.path.basename(p.rstrip("/")), load(p)) for p in sys.argv[1:]]
    t0 = min(r[0][0] for _, r in runs)
    print("%6s" % "t(s)" + "".join("%22s" % n for n, _ in runs))
    print("%6s" % "" + "".join("%22s" % "z / r_horiz" for _ in runs))
    for k in range(0, 720, 30):
        line = "%6d" % k
        for _, r in runs:
            cand = [x for x in r if 0 <= x[0] - t0 - k * 1e9 < 1e9]
            if not cand:
                line += "%22s" % "-"
            else:
                x = cand[len(cand) // 2]
                line += "%10.2f /%10.3f" % (x[3], math.hypot(x[1], x[2]))
        print(line)


if __name__ == "__main__":
    main()
