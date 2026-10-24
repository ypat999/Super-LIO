#!/usr/bin/env python3
"""历史结论污染审计（M.25 阶段 0.2）。

背景：M.22/M.24 已确认"处理停顿 ⇒ IMU 欠载 ⇒ 去畸变点云被扭坏 ⇒ 悬停崩塌"，
凡带停顿的 run 其误差被崩塌主导，不能进参数结论。但更早的 M.13/M.14/M.15/M.19
结论表是在不知道这条机制的情况下排的，需要用同一判据重新打标签。

判据（与 run 的 meta/odom 一致，不需要 bag、不需要 debug.csv）：
  · odom gap：相邻 odom 帧时间差 > --gap（默认 0.15 s）的次数
  · 悬停位移：速度中值滤波后 < --vth（默认 0.10 m/s）且持续 >= --mindur（默认 2 s）
    的静止段，其内部最大位移
  · 任一项超阈值 ⇒ 标记"污染"（gap > 0 或 悬停位移 > --colp，默认 1.0 m）

用法：
  python3 -B exp/pollution_audit.py exp/results_drone exp/results_drone_1f ...
  python3 -B exp/pollution_audit.py --csv /tmp/audit.csv exp/results_drone
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import load_odom
from loop_error import health_metrics


def audit(run, gap, vth, mindur, colp):
    odom = load_odom(os.path.join(run, "odom.csv"))
    if odom is None or len(odom) < 20:
        return None
    t = odom["t_ns"].astype(np.float64) * 1e-9
    xyz = np.stack([odom["x"], odom["y"], odom["z"]], 1).astype(np.float64)
    hm = health_metrics(t, xyz, v_thr=vth, min_s=mindur, gap_thr=gap)
    contaminated = hm["gaps"] > 0 or hm["hover"] > colp
    return {
        "run": os.path.basename(run.rstrip("/")),
        "dur": float(t[-1] - t[0]),
        "gaps": hm["gaps"],
        "gap_max": hm["gap_max"],
        "hover": hm["hover"],
        "n_stop": hm["n_stop"],
        "bad": contaminated,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gap", type=float, default=0.15, help="odom gap 阈值 s")
    ap.add_argument("--vth", type=float, default=0.10, help="静止速度阈值 m/s")
    ap.add_argument("--mindur", type=float, default=2.0, help="静止最短时长 s")
    ap.add_argument("--colp", type=float, default=1.0, help="悬停位移污染阈值 m")
    ap.add_argument("--csv", default=None, help="把审计表写成 CSV")
    ap.add_argument("roots", nargs="+")
    a = ap.parse_args()

    rows = []
    for root in a.roots:
        if not os.path.isdir(root):
            continue
        for name in sorted(os.listdir(root)):
            run = os.path.join(root, name)
            if not os.path.isdir(run):
                continue
            r = audit(run, a.gap, a.vth, a.mindur, a.colp)
            if r is None:
                continue
            r["root"] = os.path.basename(root.rstrip("/"))
            rows.append(r)

    print(f"{'root':<20}{'run':<28}{'dur(s)':>8}{'gaps':>6}{'gap_max':>9}"
          f"{'悬停位移':>9}{'静止段':>7}  标签")
    n_bad = 0
    for r in rows:
        flag = "污染" if r["bad"] else "可信"
        n_bad += int(r["bad"])
        print(f"{r['root']:<20}{r['run']:<28}{r['dur']:>8.1f}{r['gaps']:>6}"
              f"{r['gap_max']:>9.2f}{r['hover']:>9.3f}{r['n_stop']:>7}  {flag}")
    print(f"\n合计 {len(rows)} 个 run，污染 {n_bad} 个（{100.0 * n_bad / max(len(rows), 1):.1f}%）")

    if a.csv:
        with open(a.csv, "w", encoding="utf-8") as f:
            f.write("root,run,dur_s,gaps,gap_max_s,hover_disp_m,n_stop,label\n")
            for r in rows:
                f.write(f"{r['root']},{r['run']},{r['dur']:.1f},{r['gaps']},"
                        f"{r['gap_max']:.3f},{r['hover']:.3f},{r['n_stop']},"
                        f"{'污染' if r['bad'] else '可信'}\n")
        print(f"CSV -> {a.csv}")


if __name__ == "__main__":
    main()
