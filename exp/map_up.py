#!/usr/bin/env python3
"""从 PCD 自身估计竖直方向（不依赖 odom / IMU），用于核对各框架世界系是否水平。

原理
----
楼梯井里面积最大的平面是楼板/天花（真实水平面），其次是墙面（真实竖直面）。
对降采样点做 kNN+PCA 求法向，按平面度加权，
  - 占优法向 = 楼板法向 = 竖直方向；
  - 同时给出“与 y/z 轴夹角”，判断该 PCD 是否被整体旋转（GenZ/FAST-LIO 不做重力对齐）。

用法
----
  python3 map_up.py results_drone/sl_nw05/map.pcd results_genz/results_drone/gz_pf6_hmoff/map_ds.pcd
"""
import os
import sys

import numpy as np
from scipy.spatial import cKDTree


def read_pcd(path):
    with open(path, "rb") as f:
        n, fields = None, None
        while True:
            line = f.readline()
            if not line:
                raise SystemExit(f"{path}: 头部没有 DATA")
            s = line.decode("ascii", "ignore").strip()
            if s.startswith("FIELDS"):
                fields = s.split()[1:]
            elif s.startswith("POINTS"):
                n = int(s.split()[1])
            elif s.startswith("DATA"):
                if "binary" not in s:
                    raise SystemExit(f"{path}: 只支持 binary PCD（{s}）")
                break
        arr = np.frombuffer(f.read(n * 4 * len(fields)), dtype=np.float32)
    arr = arr.reshape(n, len(fields))
    i = [fields.index(k) for k in ("x", "y", "z")]
    return arr[:, i].astype(np.float64)


def normals(pts, k=16, sample=60000, seed=0):
    rng = np.random.default_rng(seed)
    if len(pts) > sample:
        pts = pts[rng.choice(len(pts), sample, replace=False)]
    tr = cKDTree(pts)
    _, idx = tr.query(pts, k=k, workers=-1)
    nb = pts[idx]                       # (n,k,3)
    nb = nb - nb.mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", nb, nb) / k
    w, v = np.linalg.eigh(cov)
    nrm = v[:, :, 0]                    # 最小特征值对应法向
    planarity = (w[:, 1] - w[:, 0]) / np.maximum(w[:, 2], 1e-12)
    return nrm, planarity, pts


def report(path, k=16):
    pts = read_pcd(path)
    nrm, pl, sp = normals(pts, k)
    m = pl > np.percentile(pl, 60)
    w = pl[m] ** 2
    N = nrm[m]
    M = np.einsum("n,ni,nj->ij", w, N, N)
    ev, evec = np.linalg.eigh(M)
    u = evec[:, -1]
    if u[2] > 0:
        u = -u
    a_up = np.degrees(np.arccos(np.clip(u @ np.array([0, 0, -1.0]), -1, 1)))
    a_z = np.degrees(np.arccos(np.clip(abs(u[2]), -1, 1)))
    # 竖直面法向（水平法向）占比：用于确认估计出的 u 确实是“平的东西的法向”
    horiz = np.abs(N @ u) < 0.25
    print(f"{os.path.basename(os.path.dirname(path)):<18}"
          f"{os.path.basename(path):<12}{len(pts):>8}"
          f"   up_map=[{u[0]:+.3f} {u[1]:+.3f} {u[2]:+.3f}]"
          f"   与-z夹角={a_up:5.1f}°  与z轴夹角={a_z:5.1f}°"
          f"   竖直墙面占比={100*horiz.mean():4.1f}%")


def main():
    for p in sys.argv[1:]:
        if not os.path.exists(p):
            print(f"{p}: 不存在")
            continue
        report(p)


if __name__ == "__main__":
    main()
