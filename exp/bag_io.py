#!/usr/bin/env python3
"""实验产物读取的公共函数：odom.csv 与 binary PCD。"""
import csv
import os

import numpy as np


def load_odom(path):
    if not os.path.exists(path):
        return None
    try:
        data = np.genfromtxt(path, delimiter=",", names=True)
    except Exception:
        return None
    if data is None or data.size == 0:
        return None
    if data.shape == ():
        data = data.reshape(1)
    return data


def traj_arrays(odom):
    xyz = np.stack([odom["x"], odom["y"], odom["z"]], axis=1).astype(np.float64)
    t_abs = odom["t_ns"].astype(np.float64) * 1e-9
    return t_abs, xyz


def _pcd_header(path):
    """解析 binary PCD 头，返回 (dtype, npts, data_offset)；不支持时 (None, 0, 0)。"""
    fields, sizes, types, counts = [], [], [], []
    npts = 0
    data_kind = "binary"
    with open(path, "rb") as f:
        while True:
            line = f.readline()
            if not line:
                return None, 0, 0
            s = line.decode("ascii", errors="ignore").strip()
            if s.startswith("FIELDS"):
                fields = s.split()[1:]
            elif s.startswith("SIZE"):
                sizes = [int(x) for x in s.split()[1:]]
            elif s.startswith("TYPE"):
                types = s.split()[1:]
            elif s.startswith("COUNT"):
                counts = [int(x) for x in s.split()[1:]]
            elif s.startswith("POINTS"):
                npts = int(s.split()[1])
            elif s.startswith("DATA"):
                data_kind = s.split()[1]
                offset = f.tell()
                break
        if data_kind != "binary" or not fields:
            return None, 0, 0
        np_type = {"F": "f", "U": "u", "I": "i"}
        dtype = []
        for fn, sz, tp, ct in zip(fields, sizes, types, counts or [1] * len(fields)):
            base = np_type.get(tp, "f") + str(sz)
            dtype.append((fn, base, (ct,)) if ct > 1 else (fn, base))
        return np.dtype(dtype), npts, offset


def pcd_num_points(path):
    if not os.path.exists(path):
        return 0
    _, npts, _ = _pcd_header(path)
    return npts


def read_pcd_xyz(path, max_points=None):
    """读取 binary PCD，返回 (N,3) float64 数组；不支持的类型返回 None。

    max_points 给定时先按均匀随机抽样（固定种子）再转 float64，
    避免整栋楼 1e8 级地图把内存吃满。
    """
    if not os.path.exists(path):
        return None
    dt, npts, offset = _pcd_header(path)
    if dt is None or npts <= 0:
        return None
    raw = np.memmap(path, dtype=dt, mode="r", offset=offset, shape=(npts,))
    if max_points and npts > max_points:
        idx = np.random.default_rng(0).choice(npts, max_points, replace=False)
        raw = raw[np.sort(idx)]
    xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def pcd_info(path, max_points=2_000_000):
    xyz = read_pcd_xyz(path, max_points=max_points)
    if xyz is None:
        return None
    npts = pcd_num_points(path)
    if xyz.size == 0:
        return {"points": npts, "extent": None}
    lo, hi = xyz.min(axis=0), xyz.max(axis=0)
    return {"points": int(npts), "extent": hi - lo}
