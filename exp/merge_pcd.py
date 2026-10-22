#!/usr/bin/env python3
"""把多个 binary PCD（分块落盘的点云）合并成一个只含 xyz 的 binary PCD。

point_lio 的 pcd_save 只能按 interval 分块写盘，且落点固定在源码目录的 PCD/ 下，
这里把这些分块合成 wall_metrics / yaw_profile 需要的单个 map.pcd。

用法: python3 merge_pcd.py <out.pcd> <in1.pcd> [in2.pcd ...]
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import _pcd_header

out = sys.argv[1]
srcs = sys.argv[2:]

arrays = []
for p in srcs:
    dt, n, off = _pcd_header(p)
    if dt is None or n <= 0:
        print(f"[merge_pcd] skip {p} (unsupported header)")
        continue
    raw = np.memmap(p, dtype=dt, mode="r", offset=off, shape=(n,))
    xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float32)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    arrays.append(xyz)
    print(f"[merge_pcd] {p}: {len(xyz)}")

data = np.concatenate(arrays) if arrays else np.zeros((0, 3), np.float32)
header = (
    "# .PCD v0.7 - Point Cloud Data file format\n"
    "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
    f"WIDTH {len(data)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
    f"POINTS {len(data)}\nDATA binary\n"
)
with open(out, "wb") as f:
    f.write(header.encode("ascii"))
    f.write(np.ascontiguousarray(data).tobytes())
print(f"[merge_pcd] {len(srcs)} files -> {len(data)} pts -> {out}")
