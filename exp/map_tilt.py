#!/usr/bin/env python3
"""逐高度「水平面法向」倾角：直接量地图本身弯腰多少（不依赖任何参照 run）。

为什么换这个口径
----------------
前面用两图互相关（map_bend.py）量相对水平错位，在本 bag 上会饱和：
楼梯井是剪刀梯、同一高度层里有两侧井壁（x≈+6.7 与 +12.9 两个峰），
互相关峰没有唯一解；而且建图会经窗户扫到 79~91 m 外的楼外结构。

本脚本只用一个 run 自己的**水平面法向**，是绝对量：
- Super-LIO 在初始化时用静止段加速度对齐重力（kf_align_gravity=true,
  ref_gravity_axis=2），世界里 -z 就是重力方向；
- 楼梯间每层都有楼板 / 休息平台 / 踏步，全是真实水平面，任何高度都有；
- 按高度切层，层内取点做 kNN+PCA 求法向，只留 |n_z| 大的（水平面），
  按平面度加权合成该层主导法向 u。

θ(h) = u 相对世界 z 的偏角。θ 随 h 线性变化 ⇔ 地图在这一段被整体转了一个
角度，即「弯腰」。同时给出绕 x（pitch）和绕 y（roll）两个分量——
用户观察到的正是「回程产生固定的绕 x 轴旋转偏差」。

注意：若给了 --align-up，会先用该 run 自己的 up_imu 把地图旋到「竖直=z」，
这会**扣掉初始化重力的常值误差**，只剩随高度变化的部分（真弯腰）。
不加 --align-up 时 lz 轴就取该 run 世界系自己的 z。

用法
----
  python3 map_tilt.py --bag /home/ywj/slam_data/livox_record_drone --align-up \
      exp/results_drone/sl_nw035 exp/results_drone/sl_nw03
"""
import argparse
import os
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from loop_error import load_run
from attitude_drift import up_rotation
from map_bend import read_pcd


def load_pts(run):
    for nm in ("map.pcd", "map_ds.pcd"):
        p = os.path.join(run, nm)
        if os.path.exists(p):
            return read_pcd(p)
    raise SystemExit(f"{run}: 没有 map.pcd / map_ds.pcd")


def rotation(run, up_imu):
    """把该 run 世界系旋到「竖直=z、起点朝向=+x」（与 attitude_drift.align_to_up 同口径）。"""
    if up_imu is None:
        return Rotation.identity()
    q = None
    try:
        _, _, q = load_run(run)
    except Exception:
        pass
    if q is None:
        return up_rotation(up_imu)
    q0 = q[0] / np.linalg.norm(q[0])
    R0 = Rotation.from_quat(q0)
    # up_imu 在机体系，必须先转到世界系
    A0 = up_rotation(R0.apply(up_imu))
    hb = A0.apply(R0.apply([1.0, 0.0, 0.0]))
    hb[2] = 0.0
    n = np.linalg.norm(hb)
    if n <= 1e-6:
        return A0
    ang = np.arctan2(hb[1], hb[0])
    return Rotation.from_rotvec([0.0, 0.0, -ang]) * A0


def slab_tilt(pts, hc, dz, k=16, max_pts=120000, seed=0):
    """取 h≈hc 的一层，返回 (倾x, 倾y, 点数, 水平面占比)。"""
    m = np.abs(pts[:, 2] - hc) < 0.5 * dz
    P = pts[m]
    if len(P) < 300:
        return None
    rng = np.random.default_rng(seed)
    if len(P) > max_pts:
        P = P[rng.choice(len(P), max_pts, replace=False)]
    tr = cKDTree(P)
    _, idx = tr.query(P, k=k, workers=-1)
    nb = P[idx]
    nb = nb - nb.mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", nb, nb) / k
    w, v = np.linalg.eigh(cov)
    nrm = v[:, :, 0]
    planarity = (w[:, 1] - w[:, 0]) / np.maximum(w[:, 2], 1e-12)
    # 「水平面」判据：法向接近竖直。阈值必须收紧到 |n_z|>0.9（约 25°），
    # 否则楼梯段的大斜面板（梯段底面 / 斜梁，倾角 25~35°）会被算进来，
    # 把该层的倾角直接带偏几度。
    hz = np.abs(nrm[:, 2])
    keep = (hz > 0.9) & (planarity > np.percentile(planarity, 50))
    if keep.sum() < 100:
        return None
    N = nrm[keep]
    if N[:, 2].mean() > 0:
        N = -N
    ww = planarity[keep] ** 2 * hz[keep]
    # 稳健取中位：斜面板/梯级边缘的离群法向不会把整层拉走
    tx = np.degrees(np.arcsin(np.clip(N[:, 1], -1, 1)))
    ty = np.degrees(np.arcsin(np.clip(-N[:, 0], -1, 1)))
    order = np.argsort(tx)
    cw = np.cumsum(ww[order])
    tx = float(tx[order][np.searchsorted(cw, 0.5 * cw[-1])])
    order = np.argsort(ty)
    cw = np.cumsum(ww[order])
    ty = float(ty[order][np.searchsorted(cw, 0.5 * cw[-1])])
    return tx, ty, int(m.sum()), float(keep.mean())


def profile(pts, dz, label):
    lo, hi = np.percentile(pts[:, 2], [1.0, 99.0])
    rows = []
    for hc in np.arange(lo + 0.5 * dz, hi, dz):
        r = slab_tilt(pts, hc, dz)
        if r is None:
            continue
        rows.append((hc, r[0], r[1], r[2], r[3]))
    if len(rows) < 4:
        print(f"{label:<26} 层数不足")
        return
    a = np.array(rows)
    # 逐层倾角对高度线性拟合（两个轴分别做）：斜率 = 随高度累积的旋转（弯腰）
    kx = np.polyfit(a[:, 0], a[:, 1], 1)
    ky = np.polyfit(a[:, 0], a[:, 2], 1)
    rx = a[:, 1] - np.polyval(kx, a[:, 0])
    ry = a[:, 2] - np.polyval(ky, a[:, 0])
    print(f"{label:<26} n={len(a):>3} h=[{a[0,0]:+.1f},{a[-1,0]:+.1f}] "
          f"倾x: 常值={kx[1]:+5.2f}° 斜率={kx[0]*1000:+6.1f}mdeg/m 残差={rx.std():4.2f}° | "
          f"倾y: 常值={ky[1]:+5.2f}° 斜率={ky[0]*1000:+6.1f}mdeg/m 残差={ry.std():4.2f}°")
    if os.environ.get("DETAIL"):
        for hc, tx, ty, n, frac in rows:
            print(f"    h={hc:+7.2f}  倾x={tx:+6.2f}°  倾y={ty:+6.2f}°  n={n:>7}  水平面={frac*100:4.1f}%")


def analyze(run, up_imu, dz=2.0, near=12.0, by_leg=False):
    pts = load_pts(run)
    A = rotation(run, up_imu)
    pts = A.apply(pts)
    name = os.path.basename(run.rstrip("/"))

    try:
        t, xyz, _ = load_run(run)
        t = t - t[0]
        xyz = A.apply(xyz)
    except Exception:
        t = xyz = None

    # 只用轨迹周边，剔掉经窗户看到的远处楼外结构（那些面也水平，但会把
    # 层内点数分布拉偏，且与本次扫描的局部倾斜无关）
    if near is not None and xyz is not None:
        tr = cKDTree(xyz)
        d, idx = tr.query(pts, workers=-1)
        m = d < near
        pts, idx = pts[m], idx[m]
    else:
        idx = None

    if not by_leg or xyz is None:
        profile(pts, dz, name)
        return

    # 按「最近的轨迹点属于下行还是上行」把地图拆成两个楼梯井分别定量。
    # 这一点是判「弯腰」的关键：整体刚性倾斜（两次都同样斜）与真弯腰
    # （一段斜、另一段平）在合并统计里会互相抵消。
    tb = t[int(np.argmin(xyz[:, 2]))]
    for nm, m in (("下行井", t[idx] < tb), ("上行井", t[idx] >= tb)):
        profile(pts[m], dz, f"{name}[{nm}]")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--bag", default="")
    ap.add_argument("--align-up", action="store_true")
    ap.add_argument("--dz", type=float, default=2.0)
    ap.add_argument("--near", type=float, default=12.0)
    ap.add_argument("--by-leg", action="store_true",
                    help="按下行/上行两个楼梯井分别定量（判弯腰的关键）")
    args = ap.parse_args()

    up_imu = None
    if args.align_up:
        from loop_error import imu_up
        up_imu = imu_up(args.bag)
    for r in args.runs:
        analyze(r, up_imu, dz=args.dz, near=args.near, by_leg=args.by_leg)


if __name__ == "__main__":
    main()
