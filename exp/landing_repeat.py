#!/usr/bin/env python3
"""用楼梯本身的「几何周期性」当标尺，绝对值量漂移（不需要任何参照 run）。

思路
----
这段 bag 是「下 10 层再原路回 10 层」。楼梯每层结构完全相同，所以：
  - 每层之间的**高度差**必须恒定（楼层高）→ 竖直尺；
  - 每个折返平台的**水平位置**必须严格交替于两个固定点 → 水平尺。
轨迹在每个平台处都要 180° 折返，折返点就是平台。把折返点按顺序取出来，
它们的 (x, y, h) 序列在完美建图下应当是严格的「周期 2、无趋势」序列。
任何随楼层累积的趋势 = 漂移（这才是「全方向准确匹配」的直接度量）。

口径
----
- 把轨迹对齐到「竖直=z、起点朝向=+x」（--align-up，与 attitude_drift 一致）。
- 水平面内做 PCA 取主方向 u，把轨迹投影到 (u, v)。
- 对 u(t) 找局部极值（正负都找）= 折返点。
- 对每个折返点取平滑后的位置 → (x, y, h)。
- 分奇偶两组，各自对楼层序号线性拟合，斜率 = 每层漂移；残差 = 抖动。

用法
----
  python3 landing_repeat.py --bag /home/ywj/slam_data/livox_record_drone --align-up \
      exp/results_drone/sl_nw035 exp/results_genz/results_drone/gz_pf6_hmoff
"""
import argparse
import os
import sys

import numpy as np
from scipy.signal import find_peaks, savgol_filter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from loop_error import load_run


def rotation(run, up_imu):
    from map_tilt import rotation as r
    return r(run, up_imu)


def smooth(a, win):
    win = int(win) | 1
    if win < 3 or len(a) < win:
        return a
    return savgol_filter(a, win, 2, axis=0)


def extract(t, xyz, min_prom=0.6, min_dist_s=8.0):
    P = xyz[:, :2].copy()
    Pm = P - P.mean(0)
    w, v = np.linalg.eigh(Pm.T @ Pm)
    e = v[:, -1]
    if e[0] < 0:
        e = -e
    u = P @ e
    vv = P @ np.array([-e[1], e[0]])

    dt = float(np.median(np.diff(t)))
    us = smoothing = None
    us = smooth(u, round(2.0 / dt))
    md = max(1, int(round(min_dist_s / dt)))
    idx = []
    for s in (1.0, -1.0):
        pi, _ = find_peaks(s * us, prominence=min_prom, distance=md)
        idx.append(pi)
    idx = np.unique(np.concatenate(idx))
    idx = idx[(idx > 10) & (idx < len(t) - 10)]

    Ps = smooth(xyz, round(1.5 / dt))
    out = []
    for i in idx:
        out.append((t[i], float(Ps[i, 0]), float(Ps[i, 1]), float(Ps[i, 2]),
                    float(us[i]), float(vv[i])))
    return np.array(out), e


def analyze(run, up_imu, show=False):
    t, xyz, _ = load_run(run)
    t = t - t[0]
    A = rotation(run, up_imu)
    xyz = A.apply(xyz)
    L, e = extract(t, xyz)
    name = os.path.basename(run.rstrip("/"))
    if len(L) < 12:
        print(f"{name:<20} 折返点不足 ({len(L)})")
        return None
    h = L[:, 3]
    # 以最低点切分「下行 / 上行」
    ib = int(np.argmin(h))
    dh = np.abs(np.diff(h))
    # 有效半层：|Δh| 落在 [1.2, 3.2] m（剔掉底层挑高大空间处的伪折返）
    ok = np.zeros(len(h), bool)
    ok[0] = dh[0] >= 1.2
    for i in range(1, len(h)):
        ok[i] = dh[i - 1] >= 1.2
    pitch = np.median(dh[(dh > 1.2) & (dh < 3.2)])
    spread = np.std(dh[(dh > 1.2) & (dh < 3.2)])

    legs = []
    for nm, sl in (("dn", np.arange(0, ib + 1)), ("up", np.arange(ib, len(h)))):
        sl = sl[ok[sl]]
        if len(sl) >= 4:
            # 同一段楼梯的折返点应当都落在同一个楼梯井里；底层挑高大空间、
            # 回到 10F 的终点等「伪折返」离得很远，按到中位的水平距离剔掉
            c = np.median(L[sl][:, 1:3], axis=0)
            keep = np.linalg.norm(L[sl][:, 1:3] - c, axis=1) < 5.0
            sl = sl[keep]
        if len(sl) < 6:
            legs.append((nm, None))
            continue
        out = {}
        for par in (0, 1):
            s = sl[par::2]
            if len(s) < 3:
                continue
            k = np.arange(len(s), dtype=float)
            out[par] = (np.polyfit(k, L[s, 1], 1)[0], np.polyfit(k, L[s, 2], 1)[0],
                        len(s))
        legs.append((nm, out))

    def fmt(out):
        if not out:
            return "    -    "
        a = out.get(0, (np.nan, np.nan, 0))
        b = out.get(1, (np.nan, np.nan, 0))
        return (f"x{a[0]:+.3f}/{b[0]:+.3f} y{a[1]:+.3f}/{b[1]:+.3f}")

    print(f"{name:<20} n={len(L):>3} 半层Δh={pitch:4.2f}±{spread:4.2f}m(层高{2*pitch:4.2f}) "
          f"| 下行 {fmt(legs[0][1])} | 上行 {fmt(legs[1][1])}  [m/半层, 偶/奇]")
    if show:
        print(f"    {'k':>3}{'t':>8}{'x':>9}{'y':>9}{'h':>9}{'dh':>7}")
        for k, r in enumerate(L):
            print(f"    {k:>3}{r[0]:>8.1f}{r[1]:>9.3f}{r[2]:>9.3f}{r[3]:>9.3f}"
                  f"{'' if k == 0 else f'{r[3]-L[k-1][3]:>7.2f}'}")
    return dict(name=name, pitch=pitch, legs=legs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--bag", default="")
    ap.add_argument("--align-up", action="store_true")
    ap.add_argument("--show", action="store_true")
    a = ap.parse_args()
    up_imu = None
    if a.align_up:
        from loop_error import imu_up
        up_imu = imu_up(a.bag)
    for r in a.runs:
        analyze(r, up_imu, show=a.show)


if __name__ == "__main__":
    main()
