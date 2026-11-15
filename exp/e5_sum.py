#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""汇总一个 run 目录的 odom 轨迹统计（E-5 用）。"""
import sys
import os
import math
import csv

def load(p):
    rows = []
    with open(p) as f:
        r = csv.DictReader(f)
        for d in r:
            rows.append((float(d["t_ns"]), float(d["x"]), float(d["y"]), float(d["z"]),
                         float(d["qx"]), float(d["qy"]), float(d["qz"]), float(d["qw"])))
    return rows

def yaw_of(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

def main():
    for p in sys.argv[1:]:
        rows = load(os.path.join(p, "odom.csv"))
        seg = 0.0
        for a, b in zip(rows, rows[1:]):
            seg += math.dist(a[1:4], b[1:4])
        s, e = rows[0], rows[-1]
        d3 = math.dist(s[1:4], e[1:4])
        dh = math.hypot(e[1] - s[1], e[2] - s[2])
        dyaw = math.degrees(yaw_of(*e[4:8]) - yaw_of(*s[4:8]))
        dyaw = (dyaw + 180.0) % 360.0 - 180.0
        zs = [r[3] for r in rows]
        dur = (rows[-1][0] - rows[0][0]) / 1e9
        print("%-22s rows=%5d  %7.1f s  %7.1f m  end3D=%.3f  horiz=%.3f  dz=%.3f  dyaw=%.2f  z=[%.2f,%.2f]"
              % (os.path.basename(p.rstrip("/")), len(rows), dur, seg, d3, dh,
                 e[3] - s[3], dyaw, min(zs), max(zs)))

if __name__ == "__main__":
    main()
