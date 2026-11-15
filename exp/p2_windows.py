#!/usr/bin/env python3
"""M.40 P-2（重复周期数 -> 漂移幅度）取窗与判读工具。

两个子命令：

  profile <odom.csv> [--step 10]
      打印 z / 累计绕竖直轴转角 / 里程 的时间剖面，并按每 360° 转角给出
      "螺旋周期"边界，用于从楼梯 bag 里挑 1 周期 / 3 周期窗口。

  div <odomA.csv> <odomB.csv> <t0> <t1>
      同一时间窗内两条轨迹（如 A 线与 B 线）的端点差与逐点差：
      两条 run 都以各自首帧为原点，比较时不做事后对齐 —— 这种"完全不校正"的
      口径才对应 P-2 要问的"漂移随重复周期数的增长"。

口径说明：
  - Super-LIO 世界系自带重力对齐，绕世界 z 的转角即竖直轴转角；
  - 时间一律相对 odom.csv 首帧（与 loop_error.py 一致）。
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import load_odom


def q2R(q):
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def rotvec(R):
    c = np.clip((np.trace(R, axis1=1, axis2=2) - 1.0) / 2.0, -1.0, 1.0)
    ang = np.arccos(c)
    ax = np.stack([R[:, 2, 1] - R[:, 1, 2],
                   R[:, 0, 2] - R[:, 2, 0],
                   R[:, 1, 0] - R[:, 0, 1]], axis=1)
    s = np.sin(ang)
    out = np.zeros_like(ax)
    ok = s > 1e-9
    out[ok] = ax[ok] / (2.0 * s[ok, None]) * ang[ok, None]
    return out


def load(path):
    odom = load_odom(path)
    if odom is None or len(odom) < 20:
        raise SystemExit("读不到 odom: %s" % path)
    t = odom["t_ns"].astype(np.float64) * 1e-9
    t = t - t[0]
    xyz = np.stack([odom["x"], odom["y"], odom["z"]], axis=1).astype(np.float64)
    quat = np.stack([odom["qx"], odom["qy"], odom["qz"], odom["qw"]], axis=1)
    return t, xyz, quat


def cum_yaw(t, quat):
    R = q2R(quat)
    rv = rotvec(np.einsum("nij,njk->nik", R[1:], np.transpose(R[:-1], (0, 2, 1))))
    yaw = np.concatenate([[0.0], np.cumsum(np.degrees(rv[:, 2]))])
    return yaw


def cmd_profile(a):
    t, xyz, quat = load(a.odom)
    yaw = cum_yaw(t, quat)
    step = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    path = np.concatenate([[0.0], np.cumsum(step)])
    print(f"dur={t[-1]:.1f}s  path={path[-1]:.1f}m  z={xyz[:,2].min():+.2f}..{xyz[:,2].max():+.2f}"
          f"  cum_yaw={yaw.min():+.0f}..{yaw.max():+.0f}deg")
    print(f"{'t(s)':>7}{'z':>8}{'cumyaw':>9}{'path':>9}")
    # 按时间等间隔打点
    tt = np.arange(0, t[-1], a.step)
    for s in tt:
        i = np.searchsorted(t, s, side="right") - 1
        print(f"{s:>7.1f}{xyz[i,2]:>8.2f}{yaw[i]:>9.1f}{path[i]:>9.1f}")
    # 每 360° 转角一个周期边界（按 |Δcum_yaw| 的过零点插值）
    marks = []
    for k in range(1, int(abs(yaw[-1]) // 360) + 1):
        target = np.sign(yaw[-1]) * 360.0 * k
        i = int(np.argmax(np.abs(yaw) >= abs(target)))
        marks.append((k, t[i], yaw[i]))
    print("\n-- 每 360° 转角（一个螺旋周期）的边界 --")
    for k, tk, yk in marks:
        print(f"  period {k}: t={tk:7.1f}s  cum_yaw={yk:+8.1f}deg  path="
              f"{path[int(np.searchsorted(t, tk))]:7.1f}m")


def cmd_div(a):
    tA, pA, _ = load(a.runA)
    tB, pB, _ = load(a.runB)
    # 各 run 首帧为原点
    pA = pA - pA[0]
    pB = pB - pB[0]
    m = min(tA[-1], tB[-1])
    t1 = min(a.t1, m)
    common = np.linspace(max(a.t0, 0.0), t1, 200)
    ia = np.searchsorted(tA, common) - 1
    ib = np.searchsorted(tB, common) - 1
    ok = (ia >= 0) & (ib >= 0)
    common, ia, ib = common[ok], ia[ok], ib[ok]
    d = np.linalg.norm(pA[ia] - pB[ib], axis=1)
    iA = np.searchsorted(tA, t1) - 1
    iB = np.searchsorted(tB, t1) - 1
    end = np.linalg.norm(pA[iA] - pB[iB])
    print(f"窗口 {common[0]:.1f}~{t1:.1f}s ({t1 - common[0]:.1f}s)")
    print(f"  A 端点 |p|=({pA[iA,0]:+.3f},{pA[iA,1]:+.3f},{pA[iA,2]:+.3f})  "
          f"里程={np.linalg.norm(np.diff(pA[:iA+1],axis=0),axis=1).sum():.1f}m")
    print(f"  B 端点 |p|=({pB[iB,0]:+.3f},{pB[iB,1]:+.3f},{pB[iB,2]:+.3f})  "
          f"里程={np.linalg.norm(np.diff(pB[:iB+1],axis=0),axis=1).sum():.1f}m")
    print(f"  端点差 |A-B| = {end*100:.1f} cm   逐点差 p50={np.percentile(d,50)*100:.1f} "
          f"p95={np.percentile(d,95)*100:.1f} max={d.max()*100:.1f} cm")


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("profile")
    p.add_argument("odom")
    p.add_argument("--step", type=float, default=10.0)
    p.set_defaults(fn=cmd_profile)
    p = sub.add_parser("div")
    p.add_argument("runA")
    p.add_argument("runB")
    p.add_argument("t0", type=float)
    p.add_argument("t1", type=float)
    p.set_defaults(fn=cmd_div)
    a = ap.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
