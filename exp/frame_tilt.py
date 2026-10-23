#!/usr/bin/env python3
"""量测世界系相对"真实竖直"的姿态偏置（不解算轨迹，只用落盘地图）。

原理
----
真实建筑物的墙面是竖直的，其法向水平（nz = 0）；地面是水平的，其法向竖直
（|nz| = 1）。若估计出的世界系相对真实重力歪了 δ（roll/pitch 偏置），则整张
地图被刚性旋转 δ，于是：
    竖直墙面的 nz  ≈  δ·cos(φ − φ0)   （φ = 法向的方位角）
    水平地面的 nz  ≈  1 − O(δ²)
所以把墙上点的 nz 对方位角拟合一条正弦，**幅值就是 δ**，相位就是倾的方向。
这个判据与地形起伏无关（地形只影响地面，不影响墙），因此比拟合地面可靠。

用法:
  python3 -B exp/frame_tilt.py <run_dir> [<run_dir> ...]
  python3 -B exp/frame_tilt.py --block 25 <run_dir>   # 顺带按区块看是否全局一致
每个 run_dir 需要 map.pcd。
"""
import argparse
import os
import sys

import numpy as np

try:
    import open3d as o3d
except ImportError:
    raise SystemExit("需要 open3d")


def load_map(path, voxel):
    pc = o3d.io.read_point_cloud(path)
    if voxel > 0:
        pc = pc.voxel_down_sample(voxel)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.8, max_nn=30))
    pc.orient_normals_consistent_tangent_plane(10)
    return np.asarray(pc.points), np.asarray(pc.normals)


def robust_sine(phi, nz, iters=6):
    """拟合 nz = a·cos(phi) + b·sin(phi) + c，迭代剔除离群点。"""
    A = np.c_[np.cos(phi), np.sin(phi), np.ones(len(phi))]
    m = np.ones(len(phi), bool)
    for _ in range(iters):
        coef, *_ = np.linalg.lstsq(A[m], nz[m], rcond=None)
        r = nz - A @ coef
        s = 1.4826 * np.median(np.abs(r[m] - np.median(r[m])))
        m = np.abs(r) < max(3 * s, 0.02)
    amp = float(np.hypot(coef[0], coef[1]))
    phase = float(np.degrees(np.arctan2(coef[1], coef[0])))
    return amp, phase, float(coef[2]), m


def ground_tilt(xyz, cell=1.0, iters=12, zband=6.0, radius=0.0):
    """1 m 网格取最低点，IRLS 拟合 z = a·x + b·y + c。
    返回 (a_deg, b_deg, 总倾角_deg, 倾向方位_deg, 内点, 总格子, 残差std)。

    先按 zband 粗筛：园区单层场地的地面起伏不可能超过几米，超出即远场/树冠/
    屋顶的假"最低点"（实测不过滤时拟合会被拉到 10°、残差 13 m）。
    radius>0 时只取距云质心该半径内的格子，用于"原地静止"这类小范围重建。"""
    P0 = xyz
    if radius > 0:
        c = P0.mean(0)
        P0 = P0[np.hypot(P0[:, 0] - c[0], P0[:, 1] - c[1]) < radius]
    k = np.floor(P0[:, :2] / cell).astype(np.int64)
    key = k[:, 0] * 100000 + k[:, 1]
    order = np.lexsort((P0[:, 2], key))
    ks = key[order]
    first = np.ones(len(ks), bool)
    first[1:] = ks[1:] != ks[:-1]
    P = P0[order[first]]
    if len(P) < 50:
        return None
    med = np.median(P[:, 2])
    P = P[np.abs(P[:, 2] - med) < zband]
    if len(P) < 50:
        return None
    A = np.c_[P[:, 0], P[:, 1], np.ones(len(P))]
    m = np.ones(len(P), bool)
    for _ in range(iters):
        coef, *_ = np.linalg.lstsq(A[m], P[m, 2], rcond=None)
        r = P[:, 2] - A @ coef
        s = 1.4826 * np.median(np.abs(r[m] - np.median(r[m])))
        m = np.abs(r) < max(3.0 * s, 0.05)
    return (float(np.degrees(np.arctan(coef[0]))),
            float(np.degrees(np.arctan(coef[1]))),
            float(np.degrees(np.arctan(np.hypot(coef[0], coef[1])))),
            float(np.degrees(np.arctan2(coef[1], coef[0]))),
            int(m.sum()), len(P), float(np.std(r[m])))


def analyse(run, voxel, wall_thr, block, radius):
    # 既接受 run 目录（读 map.pcd），也接受直接的 .pcd 路径（按圈切出的 lap_*.pcd）
    if run.lower().endswith(".pcd"):
        p, name = run, os.path.basename(run)[:-4]
    else:
        p, name = os.path.join(run, "map.pcd"), os.path.basename(run.rstrip("/"))
    if not os.path.exists(p):
        print(f"{name}: 无 {p}")
        return
    xyz, nrm = load_map(p, voxel)
    print(f"\n== {name}  下采样后 {len(xyz)} 点 (voxel {voxel})")

    nz = nrm[:, 2]
    phi = np.arctan2(nrm[:, 1], nrm[:, 0])

    # 墙面：法向接近水平
    w = np.abs(nz) < wall_thr
    amp, ph, c, m = robust_sine(phi[w], nz[w])
    print(f"  墙面候选 |nz|<{wall_thr}: {w.sum()} 点，内点 {m.sum()}")
    print(f"  ⇒ 竖直度偏差 δ = {np.degrees(np.arcsin(np.clip(amp,-1,1))):.3f}°"
          f"  方向 φ0 = {ph:+.1f}°  (nz 偏置常数项 {c:+.4f})")

    # 地面：1 m 网格最低点做鲁棒平面拟合（比 mean nz 可靠，能直接给两个方向的分量）
    g = ground_tilt(xyz, radius=radius)
    if g:
        print(f"  地面平面(1m网格最低点, {g[4]}/{g[5]} 格入内): "
              f"倾角 x {g[0]:+.3f}° / y {g[1]:+.3f}° ⇒ 总 {g[2]:.3f}° "
              f"倾向 {g[3]:+.1f}°  残差std {g[6]:.2f} m")

    # 分区块：看偏置是全局刚性还是随位置变化
    if block > 0:
        print(f"  按 {block} m 区块（仅统计墙面点 >= 200 的块）:")
        ix = np.floor(xyz[:, 0] / block).astype(int)
        iy = np.floor(xyz[:, 1] / block).astype(int)
        key = ix * 10000 + iy
        print(f"  {'块中心x':>8}{'块中心y':>8}{'墙点':>7}{'δ(°)':>8}{'φ0(°)':>8}")
        for k in np.unique(key):
            s = (key == k) & w
            if s.sum() < 200:
                continue
            a2, p2, _, m2 = robust_sine(phi[s], nz[s])
            cx = (k // 10000 + 0.5) * block
            cy = (k % 10000 + 0.5) * block
            print(f"  {cx:>8.1f}{cy:>8.1f}{s.sum():>7}"
                  f"{np.degrees(np.arcsin(np.clip(a2,-1,1))):>8.3f}{p2:>8.1f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--voxel", type=float, default=0.3)
    ap.add_argument("--wall-thr", type=float, default=0.25)
    ap.add_argument("--block", type=float, default=0.0)
    ap.add_argument("--radius", type=float, default=0.0,
                    help="地面拟合只取距质心该半径内的格子（原地静止重建用）")
    ap.add_argument("runs", nargs="+")
    a = ap.parse_args()
    for r in a.runs:
        analyse(r, a.voxel, a.wall_thr, a.block, a.radius)


if __name__ == "__main__":
    main()
