#!/usr/bin/env python3
"""以某个 run 为参照，量其余 run 的点云地图「逐高度的水平错位」（相对弯腰）。

为什么不用「墙面位置随高度」
----------------------------
楼梯是剪刀梯，人在一条腿上本来就左右摆动 ±1.5 m（真实几何），
而漂移在同一条腿上只有 1~2 m，直接拟合 x(h)/y(h) 会被摆动的锯齿污染。
两个 run 走的是**同一条真实路径**，锯齿在两者的地图里都有 ⇒ 逐高度做
二维互相关取相对错位，锯齿与场景结构自动抵消，剩下的就是两者的漂移差。

口径
----
- 每个 run 用自己的 up_world=R(q0)·up_imu 旋到「竖直=z、起点朝向=+x」
  （与 attitude_drift.py --align-up 一致）。
- 先用整图（0.25 m 栅格，±4 m 搜索）求一次全局 xy 平移，消掉起点零点差。
- 再按 0.5 m 高度分箱，每箱建 0.1 m 栅格，与参照做互相关，取峰值位置
  作为该高度层的相对水平错位 (sx, sy)。
- 对 sx(h)、sy(h) 线性拟合，斜率即两图的相对弯腰（deg）。

用法
----
  python3 map_bend.py --bag /home/ywj/slam_data/livox_record_drone --align-up \
      --ref results_genz/results_drone/gz_pf6_hmoff \
      results_drone/sl_nw05 results_drone/sl_dbg2 results_genz/results_drone/gz_pf1
"""
import argparse
import os
import sys

import numpy as np
from scipy.ndimage import gaussian_filter
from scipy.signal import fftconvolve
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from attitude_drift import up_rotation, H
from loop_error import load_run


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


def load_map(run, up_imu, near=None):
    """读地图并对齐到「竖直=z、起点朝向=+x」。

    near: 若给定，只保留距该 run 自身轨迹水平距离 < near 米的点。
    建图会通过窗户扫到几十米外的楼外结构，那些点在两图间既不对应、
    又体量巨大，会把逐高度互相关的峰拉走（实测逐层位移满量程跳变）。
    """
    pts = None
    for nm in ("map.pcd", "map_ds.pcd"):
        p = os.path.join(run, nm)
        if os.path.exists(p):
            pts = read_pcd(p)
            break
    if pts is None:
        raise SystemExit(f"{run}: 没有 map.pcd / map_ds.pcd")
    if up_imu is None:
        return pts
    from scipy.spatial import cKDTree
    t, xyz, quat = load_run(run)
    R0 = Rotation.from_quat(quat[0] / np.linalg.norm(quat[0]))
    A = up_rotation(R0.apply(up_imu))
    hb = A.apply(R0.apply([1.0, 0.0, 0.0]))
    hb[2] = 0.0
    if np.linalg.norm(hb) > 1e-6:
        th = float(np.arctan2(hb[1], hb[0]))
        A = Rotation.from_rotvec([0.0, 0.0, -th]) * A
    pts = A.apply(pts)
    if near is not None:
        tr = cKDTree((A.apply(xyz))[:, :2])
        d, _ = tr.query(pts[:, :2], workers=-1)
        pts = pts[d < near]
    return pts


def raster(pts, res, lo, n, blur=0.0):
    ij = np.floor((pts[:, :2] - lo) / res).astype(np.int64)
    m = (ij[:, 0] >= 0) & (ij[:, 0] < n) & (ij[:, 1] >= 0) & (ij[:, 1] < n)
    g = np.zeros((n, n), dtype=np.float32)
    np.add.at(g, (ij[m, 0], ij[m, 1]), 1.0)
    if blur > 0:
        g = gaussian_filter(g, blur, mode="constant")
    return g


def best_shift(a, b, res, max_shift):
    """返回 b 相对 a 的平移量 (Δx,Δy)（米）：b ≈ a 平移 Δ。

    互相关峰的位置本身就是 b 相对 a 的偏移，但 fftconvolve 的索引顺序
    与符号各差一次（已用合成数据核对：b=a+(0.5,0) → 返回 (+0.5,0)）。
    """
    a = a - a.mean()
    b = b - b.mean()
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-6 or nb < 1e-6:
        return None, 0.0
    c = fftconvolve(a, b[::-1, ::-1], mode="same")
    n = c.shape[0]
    k = int(round(max_shift / res))
    ctr = n // 2
    sub = c[ctr - k:ctr + k + 1, ctr - k:ctr + k + 1]
    jj, ii = np.unravel_index(np.argmax(sub), sub.shape)
    peak = float(sub[jj, ii])
    ctr_s = np.abs(c).max()
    return (np.array([-(jj - k), -(ii - k)]) * res,
            peak / ctr_s if ctr_s > 0 else 0.0)


def analyze(run, ref_pts, up_imu, res=0.15, win=4.0, step=2.0,
            max_shift=2.0, blur=1.5, near=10.0):
    pts = load_map(run, up_imu, near)
    h = pts @ H
    hr = ref_pts @ H

    # 全局零点对齐：粗栅格整图互相关（网格按参照自身范围自适应）
    lo = ref_pts[:, :2].min(0) - 1.0
    n = int(np.ceil((ref_pts[:, :2] - lo).max() / 0.25)) + 2
    ga = raster(ref_pts, 0.25, lo, n)
    gb = raster(pts, 0.25, lo, n)
    d0, q0 = best_shift(ga, gb, 0.25, 3.0)
    if d0 is None:
        return None
    pts = pts.copy()
    pts[:, :2] += d0

    n = int(np.ceil((ref_pts[:, :2] - lo).max() / res)) + 2
    h0 = max(h.min(), hr.min())
    h1 = min(h.max(), hr.max())
    edges = np.arange(h0, h1 + step, step)
    rows = []
    for c in edges:
        a, b = c - win / 2, c + win / 2
        ma = (hr >= a) & (hr < b)
        mb = (h >= a) & (h < b)
        if ma.sum() < 2000 or mb.sum() < 2000:
            continue
        d, q = best_shift(raster(ref_pts[ma], res, lo, n, blur),
                          raster(pts[mb], res, lo, n, blur), res, max_shift)
        if d is None or q < 0.5:
            continue
        rows.append((c, d[0], d[1], q, int(ma.sum()), int(mb.sum())))
    if len(rows) < 6:
        return None
    hh = np.array([r[0] for r in rows])
    sx = np.array([r[1] for r in rows])
    sy = np.array([r[2] for r in rows])
    kx, bx = np.polyfit(hh, sx, 1)
    ky, by = np.polyfit(hh, sy, 1)
    return dict(run=os.path.basename(run.rstrip("/")), d0=d0, q0=q0, rows=rows,
                kx=float(np.degrees(np.arctan(kx))),
                ky=float(np.degrees(np.arctan(ky))),
                ex=float(np.sqrt(np.mean((sx - (kx * hh + bx)) ** 2))),
                ey=float(np.sqrt(np.mean((sy - (ky * hh + by)) ** 2))),
                nrow=len(rows),
                span=[float(sx.min()), float(sx.max()),
                      float(sy.min()), float(sy.max())])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--ref", required=True)
    ap.add_argument("--align-up", action="store_true")
    ap.add_argument("--detail", action="store_true")
    ap.add_argument("runs", nargs="*")
    a = ap.parse_args()

    up_imu = None
    if a.align_up:
        from attitude_drift import load_imu
        print("[imu] 读整包加速度 ...", flush=True)
        imu = load_imu(a.bag)
        a0 = imu[1][:, imu[0] < 3.0].mean(axis=1)
        up_imu = -a0 / np.linalg.norm(a0)

    print(f"[ref] {a.ref}", flush=True)
    ref_pts = load_map(a.ref, up_imu, None if up_imu is None else 10.0)
    hr = ref_pts @ H
    print(f"[ref] {len(ref_pts)} 点（轨迹 10 m 邻域），h {hr.min():.1f}..{hr.max():.1f}")

    print(f"\n{'run':<20}{'全局dx':>8}{'全局dy':>8}{'q':>6}"
          f"{'相对倾x':>9}{'相对倾y':>9}{'残差x':>8}{'残差y':>8}{'层数':>5}"
          f"{'sx范围':>16}{'sy范围':>16}")
    res = []
    for run in a.runs:
        r = analyze(run, ref_pts, up_imu)
        if r is None:
            print(f"{os.path.basename(run.rstrip('/')):<20} 跳过")
            continue
        res.append(r)
        s = r["span"]
        print(f"{r['run']:<20}{r['d0'][0]:>+8.2f}{r['d0'][1]:>+8.2f}{r['q0']:>6.2f}"
              f"{r['kx']:>+9.2f}{r['ky']:>+9.2f}{r['ex']:>8.3f}{r['ey']:>8.3f}"
              f"{r['nrow']:>5}"
              f"{f'[{s[0]:+.2f},{s[1]:+.2f}]':>16}"
              f"{f'[{s[2]:+.2f},{s[3]:+.2f}]':>16}")

    if a.detail:
        for r in res:
            print(f"\n== {r['run']}（相对参照的逐高度错位，+ 表示该 run 的地图偏向 +x/+y）")
            print(f"  {'h':>7}{'sx':>8}{'sy':>8}{'q':>7}{'pts_ref':>9}{'pts':>9}")
            for h, sx, sy, q, na, nb in r["rows"]:
                print(f"  {h:>7.1f}{sx:>+8.2f}{sy:>+8.2f}{q:>7.2f}{na:>9}{nb:>9}")


if __name__ == "__main__":
    main()
