#!/usr/bin/env python3
"""把 binary PCD 做体素降采样（保留每格第一个点）。

GenZ-LIO 的 pcd_save 输出的是"全部去畸变点直接投到世界系"，430 s 就有几千万点；
Super-LIO 的 map.pcd 是 0.25 m 体素降采样后的结果。为了让 yaw_profile.py 的
RANSAC 在两个框架上看到同样密度的墙面，这里把 GenZ 的地图降到同一体素尺寸。

用法: python3 downsample_pcd.py <in.pcd> <out.pcd> [voxel=0.25]
"""
import sys

import numpy as np

TYPEMAP = {("F", 4): "f4", ("F", 8): "f8", ("U", 1): "u1", ("U", 2): "u2",
           ("U", 4): "u4", ("U", 8): "u8", ("I", 1): "i1", ("I", 2): "i2",
           ("I", 4): "i4", ("I", 8): "i8"}


def read_header(path):
    fields, sizes, types, counts = [], [], [], []
    npts, data, offset = 0, "ascii", 0
    with open(path, "rb") as f:
        while True:
            line = f.readline()
            if not line:
                break
            offset += len(line)
            s = line.decode("ascii", errors="ignore").strip()
            if s.startswith("FIELDS"):
                fields = s.split()[1:]
            elif s.startswith("SIZE"):
                sizes = [int(v) for v in s.split()[1:]]
            elif s.startswith("TYPE"):
                types = s.split()[1:]
            elif s.startswith("COUNT"):
                counts = [int(v) for v in s.split()[1:]]
            elif s.startswith("POINTS"):
                npts = int(s.split()[1])
            elif s.startswith("DATA"):
                data = s.split()[1]
                break
    if not counts:
        counts = [1] * len(fields)
    return fields, sizes, types, counts, npts, data, offset


def main():
    src, dst = sys.argv[1], sys.argv[2]
    voxel = float(sys.argv[3]) if len(sys.argv) > 3 else 0.25

    fields, sizes, types, counts, npts, data, offset = read_header(src)
    if data != "binary":
        raise SystemExit(f"{src}: 只支持 DATA binary（这是 {data}）")
    if list(counts) != [1] * len(counts):
        raise SystemExit(f"{src}: 不支持 count>1 的字段 {counts}")

    dt = np.dtype([(f, TYPEMAP[(t, s)]) for f, t, s in zip(fields, types, sizes)])
    raw = np.memmap(src, dtype=dt, mode="r", offset=offset, shape=(npts,))
    xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float64)
    ok = np.isfinite(xyz).all(axis=1)
    raw, xyz = raw[ok], xyz[ok]

    # 三维体素键打包成 int64：每轴 21 位（±1e6 个体素，±250 km @0.25 m）
    key = np.floor(xyz / voxel).astype(np.int64) + (1 << 20)
    packed = (key[:, 0] << 42) | (key[:, 1] << 21) | key[:, 2]
    _, idx = np.unique(packed, return_index=True)
    idx.sort()  # 保持原始顺序，便于按扫描顺序查看
    sel = raw[idx]
    print(f"{src}: {npts} -> {len(sel)} 点（voxel {voxel} m，压缩 {npts/len(sel):.1f}x）")

    if "intensity" not in sel.dtype.names:
        inten = np.zeros(len(sel), dtype=np.float32)
    else:
        inten = sel["intensity"].astype(np.float32)
    out = np.empty(len(sel), dtype=np.dtype([("x", "f4"), ("y", "f4"),
                                            ("z", "f4"), ("intensity", "f4")]))
    out["x"], out["y"], out["z"], out["intensity"] = sel["x"], sel["y"], sel["z"], inten

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
        "TYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {len(out)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(out)}\nDATA binary\n"
    )
    with open(dst, "wb") as f:
        f.write(header.encode("ascii"))
        out.tofile(f)
    print(f"-> {dst}")


if __name__ == "__main__":
    main()
