#!/usr/bin/env python3
"""逐高度层「相对刚体配准」：把每个 run 的地图按 2 m 切层，逐层对参照 run 做
point-to-plane ICP，得到该层的 6-DoF 相对位姿，再看它随高度怎么变。

为什么需要这个口径
------------------
map_tilt.py 量的是「单个 run 自己的水平面法向随高度旋转」，是绝对量。
但 gz_pf6_hmoff 被用户判为最接近真值，它自己也有 +26/+51 mdeg/m 的斜率
（残差 6°、地图很脏），所以「绝对斜率」不能直接当「谁更弯」的判据。

本脚本给出的是**相对量**：同一高度层里，test 地图相对 ref 地图要多转多少、
多平移多少。它把两件事分开：
  - 每层变换的**常值项**（与 h 无关）= 整体刚性错位，可被一次 R,t 吸收，肉眼
    上看是「整块歪了一点」，不是弯腰；
  - 每层变换的**斜率项**（随 h 线性累积）= 一次刚性变换吸收不掉的形状差，
    这才是肉眼上的「弯腰」；
  - 拟合**残差**（逐层值偏离直线的 std）= 非线性弯曲，比线性斜率更严格的弯腰判据。

同时逐层给出 dx/dy/dz 与 roll/pitch/yaw，覆盖「全方向准确匹配」的所有自由度
（dz 的斜率就是两次建图的竖直尺度差 —— GenZ 的层高偏高 2% 会直接体现成
 dz/h ≈ +2%）。

用法
----
  source /opt/ros/humble/setup.bash
  python3 map_slab_cmp.py --bag /home/ywj/slam_data/livox_record_drone \
      --ref exp/results_drone/gz_pf6_hmoff --align-up \
      exp/results_drone/sl_nw035 exp/results_drone/sl_nw03
"""
import argparse
import os
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from loop_error import load_run, imu_up
from map_bend import read_pcd
from map_tilt import load_pts, rotation


def voxel_ds(P, leaf=0.15):
    if len(P) == 0:
        return P
    k = np.floor(P / leaf).astype(np.int64)
    _, idx = np.unique(k, axis=0, return_index=True)
    return P[idx]


def normals(P, k=12):
    """kNN+PCA 法向（只用于 point-to-plane 的雅可比，不做筛选）。"""
    tr = cKDTree(P)
    _, idx = tr.query(P, k=k, workers=-1)
    nb = P[idx] - P[idx].mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", nb, nb) / k
    _, v = np.linalg.eigh(cov)
    return v[:, :, 0]


def icp_point_to_plane(A, NA, B, iters=25, max_dist=0.5):
    """把 B 配到 A（A 带法向）。返回 (R, t, inlier_ratio, rmse, overlap)。

    初值取单位变换：两边已经各自被旋到「竖直=z、起点朝向=+x」，
    逐层剩余错位是厘米~分米级，不需要全局粗配准。
    """
    tr = cKDTree(A)
    R = np.eye(3)
    t = np.zeros(3)
    ratio, rmse = 0.0, np.inf
    for it in range(iters):
        P = B @ R.T + t
        d, idx = tr.query(P, workers=-1)
        m = d < max_dist
        if m.sum() < 200:
            return None
        p, q, n = P[m], A[idx[m]], NA[idx[m]]
        r = np.einsum("ij,ij->i", n, p - q)
        # Cauchy 权：楼梯间有扶手/管线等细结构，纯最小二乘会被拉走
        s = 1.4826 * np.median(np.abs(r - np.median(r))) + 1e-6
        w = 1.0 / np.sqrt(1.0 + (r / (2.0 * s)) ** 2)
        J = np.hstack([np.cross(p, n), n])
        Jw, rw = J * w[:, None], r * w
        H = Jw.T @ Jw
        g = -Jw.T @ rw
        H += (1e-6 * np.trace(H) / 6.0) * np.eye(6)
        try:
            x = np.linalg.solve(H, g)
        except np.linalg.LinAlgError:
            return None
        dR = Rotation.from_rotvec(x[:3]).as_matrix()
        R = dR @ R
        t = dR @ t + x[3:]
        if np.linalg.norm(x[:3]) < 1e-6 and np.linalg.norm(x[3:]) < 1e-5:
            break
    P = B @ R.T + t
    d, idx = tr.query(P, workers=-1)
    m = d < max_dist
    if m.sum() < 200:
        return None
    ratio = float(m.mean())
    rmse = float(np.sqrt(np.mean(d[m] ** 2)))
    # 重合度按参照侧统计：参照层的点有多大比例能找到源点对应。
    # 注意逐腿比较时必须两边都先按腿切开，否则参照层的另一侧井壁会
    # 把重合度直接砍到 0.5 以下（第一版就是这么把上行井筛没的）。
    tr2 = cKDTree(P)
    d2, _ = tr2.query(A, workers=-1)
    overlap = float(np.mean(d2 < max_dist))
    return R, t, ratio, rmse, overlap


def slab_cmp(ref_pts, tst_pts, hc, dz, leaf=0.15):
    A = voxel_ds(ref_pts[np.abs(ref_pts[:, 2] - hc) < 0.5 * dz], leaf)
    B = voxel_ds(tst_pts[np.abs(tst_pts[:, 2] - hc) < 0.5 * dz], leaf)
    if len(A) < 400 or len(B) < 400:
        return None
    NA = normals(A)
    r = icp_point_to_plane(A, NA, B)
    if r is None:
        return None
    R, t, ratio, rmse, ov = r
    if ov < 0.5:
        # 重合度太低，配准结果的旋转不可信
        return None
    w = Rotation.from_matrix(R).as_rotvec()
    # 全局刚体变换已经先扣掉了，逐层剩下的应该是度、分米量级；超出说明
    # 该层几何退化（只剩一面长直墙时，沿墙平移/绕墙法向旋转不可观测），
    # 这种层不能进拟合。z 方向单独放宽：两次建图的竖直尺度差在本 bag 上
    # 约 2%，到 -34 m 层会累积到 0.7 m，这是要测的信号而不是退化。
    if (np.degrees(np.linalg.norm(w)) > 3.0 or np.linalg.norm(t[:2]) > 0.6
            or abs(t[2]) > 1.6):
        return None
    return dict(h=hc, dx=t[0], dy=t[1], dz=t[2],
                rx=np.degrees(w[0]), ry=np.degrees(w[1]), rz=np.degrees(w[2]),
                ratio=ratio, rmse=rmse, ov=ov, nA=len(A), nB=len(B))


def global_align(ref_pts, tst_pts, leaf=0.3, max_dist=2.0, iters=40):
    """整图刚性配准：先把这个 run 相对参照的整体旋转+平移扣掉。

    逐层残差只有在「整体刚性」被拿走之后才是干净的弯腰量；否则一层里
    的常数项（两个 run 的首帧朝向差 ~7°）会把每层都顶到钳制边界。
    """
    A = voxel_ds(ref_pts, leaf)
    B = voxel_ds(tst_pts, leaf)
    r = icp_point_to_plane(A, normals(A), B, iters=iters, max_dist=max_dist)
    if r is None:
        return None
    return r


def fit(rows, key):
    a = rows
    if len(a) < 4:
        return None
    H = np.array([r["h"] for r in a], float)
    V = np.array([r[key] for r in a], float)
    # Huber 迭代重加权：个别层可能配准偏了（细结构/单面墙），
    # 普通最小二乘会让一层把整条斜率拉歪。
    k, b0 = np.polyfit(H, V, 1)
    w = np.ones(len(H))
    for _ in range(8):
        r = V - (k * H + b0)
        s = 1.4826 * np.median(np.abs(r - np.median(r))) + 1e-6
        w = 1.0 / np.maximum(1.0, np.abs(r) / (1.345 * s))
        k, b0 = np.polyfit(H, V, 1, w=np.sqrt(w))
    res = V - (k * H + b0)
    return k, b0, float(np.std(res[w > 0.5])), len(a)


def report(rows, label, dz):
    if len(rows) < 4:
        print(f"{label:<34} 有效层数不足 ({len(rows)})")
        return
    h = np.array([r["h"] for r in rows])
    print(f"{label:<34} h=[{h.min():+.1f},{h.max():+.1f}] n={len(rows)}")
    hdr = "    量     常值(h=0)      斜率/100m     残差   "
    print(hdr)
    for key, unit, scale, nm in (("rx", "°", 1e2, "绕x(roll)"),
                                 ("ry", "°", 1e2, "绕y(pitch)"),
                                 ("rz", "°", 1e2, "绕z(yaw)"),
                                 ("dx", "m", 1e2, "平移x"),
                                 ("dy", "m", 1e2, "平移y"),
                                 ("dz", "m", 1e2, "平移z")):
        f = fit(rows, key)
        if f is None:
            continue
        k, b0, res, n = f
        # 斜率按「每 100 m 高度差」给，直观：40 m 行程就是 ×0.4
        print(f"    {nm:<10} {b0:+9.3f}{unit}  {k*scale:+10.2f}{unit}/100m "
              f"{res:8.3f}{unit}")
    rms = np.array([r["rmse"] for r in rows])
    ov = np.array([r["ov"] for r in rows])
    print(f"    配准质量   rmse={rms.mean():.3f}m  重合度={ov.mean():.2f}  "
          f"最差={rms.max():.3f}m")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--bag", default="/home/ywj/slam_data/livox_record_drone")
    ap.add_argument("--ref", default="exp/results_drone/gz_pf6_hmoff")
    ap.add_argument("--align-up", action="store_true")
    ap.add_argument("--dz", type=float, default=2.0)
    ap.add_argument("--near", type=float, default=12.0)
    ap.add_argument("--by-leg", action="store_true")
    ap.add_argument("--detail", action="store_true")
    a = ap.parse_args()

    up = None
    if a.align_up:
        up = imu_up(a.bag)
        print(f"重力(IMU系) = {np.round(up, 4)}")

    def prep(run):
        pts = load_pts(run)
        A = rotation(run, up)
        pts = A.apply(pts)
        r = load_run(run)
        if r is None:
            return None  # 早期 run 的 odom 格式不兼容，批量跑时跳过
        t, xyz, _ = r
        t = t - t[0]
        xyz = A.apply(xyz)
        idx = None
        if a.near is not None:
            tr = cKDTree(xyz)
            d, idx = tr.query(pts, workers=-1)
            m = d < a.near
            pts, idx = pts[m], idx[m]
        # 每个点按「最近轨迹点的时刻」归到下行段 / 上行段
        tb = t[int(np.argmin(xyz[:, 2]))]
        dn = (t[idx] < tb) if idx is not None else None
        return pts, dn

    ref_prep = prep(a.ref)
    if ref_prep is None:
        raise SystemExit(f"{a.ref}: 轨迹不可读")
    ref_pts, ref_dn = ref_prep
    lo = np.percentile(ref_pts[:, 2], 1.0)
    hi = np.percentile(ref_pts[:, 2], 99.0)
    print(f"参照 {os.path.basename(a.ref.rstrip('/'))}: {len(ref_pts)} 点 "
          f"h=[{lo:+.1f},{hi:+.1f}]")

    for run in a.runs:
        p = prep(run)
        name = os.path.basename(run.rstrip("/"))
        if p is None:
            print(f"\n{name}: 轨迹不可读，跳过")
            continue
        pts, dn = p
        g = global_align(ref_pts, pts)
        if g is None:
            print(f"\n{name}: 整图配准失败")
            continue
        R0, t0, gr, grmse, gov = g
        pts = pts @ R0.T + t0
        w0 = Rotation.from_matrix(R0).as_rotvec()
        print(f"\n{name}")
        print(f"  整体刚性(已扣除):  yaw={np.degrees(w0[2]):+.2f}° "
              f"pitch={np.degrees(w0[1]):+.2f}° roll={np.degrees(w0[0]):+.2f}° "
              f"t=({t0[0]:+.2f},{t0[1]:+.2f},{t0[2]:+.2f})m "
              f"重合度={gov:.2f} rmse={grmse:.3f}m")
        l2 = max(lo, np.percentile(pts[:, 2], 1.0))
        h2 = min(hi, np.percentile(pts[:, 2], 99.0))
        rows = []
        for hc in np.arange(l2 + 0.5 * a.dz, h2, a.dz):
            r = slab_cmp(ref_pts, pts, hc, a.dz)
            if r is not None:
                rows.append(r)
        report(rows, name, a.dz)
        if a.by_leg and dn is not None and ref_dn is not None:
            # 两条腿的地图必须同时按腿切开：剪刀梯同一高度层有两侧井壁，
            # 只切源侧会让参照层的另一侧井壁把重合度砍半。
            for nm, m in (("下行井", dn), ("上行井", ~dn)):
                rm = ref_dn if nm == "下行井" else ~ref_dn
                sub = []
                for hc in np.arange(l2 + 0.5 * a.dz, h2, a.dz):
                    r = slab_cmp(ref_pts[rm], pts[m], hc, a.dz)
                    if r is not None:
                        sub.append(r)
                report(sub, f"  └ {nm}", a.dz)
        if a.detail:
            print("    h      dx     dy     dz     rx     ry     rz    ov  rmse")
            for r in rows:
                print(f"  {r['h']:+6.1f} {r['dx']:+6.3f} {r['dy']:+6.3f} "
                      f"{r['dz']:+6.3f} {r['rx']:+6.2f} {r['ry']:+6.2f} "
                      f"{r['rz']:+6.2f} {r['ov']:4.2f} {r['rmse']:5.3f}")


if __name__ == "__main__":
    main()
