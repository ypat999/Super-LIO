#!/usr/bin/env python3
"""分楼层 / 分楼梯的 yaw 比较：把"一圈回原点"的单一闭环判据拆开。

动机
----
新 bag 是"两条楼梯一下一上、一圈回原点"。上行楼梯与下行楼梯对地图的扭转方向
相反、模长相近，回到原点时几乎相互抵消，于是
    |p_end − p_start| 与 闭环 yaw
只剩抵消后的残差，看不出任何一段楼梯自身的扭曲量，也无法判断哪一格更好。
必须按楼层停留 / 按单程楼梯分开量 yaw。

高度轴
------
Super-LIO 世界系 z 朝下（ref_gravity = (0,0,−g)），FAST-LIO 的世界系 z 又偏离重力
约 30°，都不能直接当高度。统一用 bag 起始静态段 IMU 均值得 up_imu，再乘各框架
自己的首帧姿态：
    up_world = R(q0)·up_imu,  h = x·up_world
并用 rot_from_to(up_world) 把 odom 与 map 一起旋到"z 就是高度"的系里。

测量
----
1. 楼层访问段：h(t) 上的停留平台（滑窗极差法，比 |dh/dt| 稳）；
2. 楼梯段：h(t) 单调上升/下降的段（阈值 + 滞回），每段一个 xy 中心；
3. 点云侧：在每个楼梯段自己的 xy 邻域内按 h 每 zbin 一层取点 -> 竖向性筛选 ->
   xy RANSAC -> 墙面法向主方向（fold=4，矩形井道 mod 90°）-> 连续展开 ->
   对 h 线性拟合，得到该楼梯的 deg/m 与整段 Δyaw；
4. 楼层侧：在每个楼层访问段自己的 xy 邻域内取 h±1 m 的点，同上得该层墙面 yaw，
   与起始楼层相减。

用法
----
  python3 floor_yaw.py --bag /home/ywj/slam_data/livox_record_drone <run_dir>...
  python3 floor_yaw.py --bag ... --detail sl_base_fr6 --zbin 2 ...   # 打印剖面
  python3 floor_yaw.py --bag ... --no-map ...                        # 只看停留段
"""
import argparse
import os
import sys

import numpy as np
from scipy.ndimage import maximum_filter1d, minimum_filter1d, median_filter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import read_pcd_xyz, rot_from_to
from loop_error import imu_up, load_run, quat_to_rot, measure
from wall_metrics import vertical_filter, ransac_lines, dominant_angle, _wrap_to


def find_plateaus(t, h, dwell=8.0, tol=0.8):
    """h(t) 停留平台：dwell 时长窗口内极差 < tol 的段。返回 [(i0,i1)]。"""
    if len(t) < 20:
        return []
    dt = float(np.median(np.diff(t)))
    W = max(3, int(round(dwell / dt)) | 1)
    hi = maximum_filter1d(h, W, mode="nearest")
    lo = minimum_filter1d(h, W, mode="nearest")
    still = (hi - lo) < tol
    runs = []
    i, n = 0, len(t)
    while i < n:
        if still[i]:
            j = i
            while j + 1 < n and still[j + 1]:
                j += 1
            a, b = min(i + W // 2, n - 1), max(j - W // 2, min(i + W // 2, n - 1))
            if t[b] - t[a] >= dwell * 0.8:
                runs.append((a, b))
            i = j + 1
        else:
            i += 1
    return runs


def find_legs(t, h, thr=0.10, min_dh=3.0, min_dur=5.0, gap_s=2.0):
    """h(t) 单调上升 / 下降段（滞回 + 断点容忍），返回 [(i0,i1,sign)]。"""
    if len(t) < 20:
        return []
    dt = float(np.median(np.diff(t)))
    win = max(3, int(round(1.0 / dt)) | 1)
    v = np.gradient(median_filter(h, size=win, mode="nearest"), t)
    gm = max(1, int(round(gap_s / dt)))
    lim = thr * 0.35
    runs = []
    i, n = 0, len(t)
    while i < n:
        s = 1 if v[i] > thr else (-1 if v[i] < -thr else 0)
        if s == 0:
            i += 1
            continue
        j, last = i + 1, i
        while j < n:
            if (v[j] > lim) if s > 0 else (v[j] < -lim):
                last = j
            elif j - last > gm:
                break
            j += 1
        j = last
        if (t[j] - t[i] >= min_dur) and abs(h[j] - h[i]) >= min_dh:
            if runs and runs[-1][2] == s and t[i] - t[runs[-1][1]] < 3.0:
                runs[-1] = (runs[-1][0], j, s)
            else:
                runs.append((i, j, s))
        i = j + 1
    return runs


def yaw_profile(pts, sel_xy, h_lo, h_hi, zbin, fold, min_band=300, min_wall=200):
    """在 xy 邻域内按高度分层测墙面主方向，返回带连续展开 un 的行。"""
    rows = []
    z = h_lo + zbin / 2.0
    while z < h_hi:
        m = sel_xy & (np.abs(pts[:, 2] - z) < zbin / 2.0)
        band = pts[m]
        row = {"h": float(z), "n": int(len(band)), "th": None, "un": np.nan,
               "strength": 0.0, "n_lines": 0, "span": 0.0}
        if len(band) >= min_band:
            wp = band[vertical_filter(band, cell=0.2, min_span=0.6)]
            if len(wp) >= min_wall:
                lines = ransac_lines(wp[:, :2],
                                     min_inliers=max(60, int(0.02 * len(wp))),
                                     max_lines=10, min_span=1.0,
                                     min_density=1.5, min_coverage=0.35)
                row["n_lines"] = len(lines)
                if lines:
                    row["span"] = max(L["span"] for L in lines)
                    th, st = dominant_angle(lines, fold=fold)
                    row["th"], row["strength"] = th, st
        rows.append(row)
        z += zbin

    period = np.pi / fold
    prev = None
    for r in rows:
        if r["th"] is None:
            continue
        r["un"] = r["th"] if prev is None else prev + _wrap_to(r["th"] - prev, period)
        prev = r["un"]
    return rows


def fit_line(x, y):
    if len(x) < 4 or np.ptp(x) < 1e-6:
        return float("nan"), float("nan")
    A = np.vstack([x, np.ones_like(x)]).T
    sol, *_ = np.linalg.lstsq(A, y, rcond=None)
    return float(sol[0]), float(np.sqrt(np.mean((y - A @ sol) ** 2)))


def analyze(run, up_imu, args):
    loaded = load_run(run)
    if loaded is None:
        return None
    t_abs, xyz, quat = loaded
    t = t_abs - t_abs[0]
    R0 = quat_to_rot(quat[0])
    up = R0 @ up_imu
    R_align = rot_from_to(up)
    a = xyz @ R_align.T
    h = a[:, 2]

    eps = []
    for k, (i, j) in enumerate(find_plateaus(t, h, args.dwell, args.tol)):
        eps.append({
            "label": f"S{k}",
            "t0": float(t[i]), "t1": float(t[j]),
            "h": float(np.median(h[i:j + 1])),
            "h_lo": float(np.percentile(h[i:j + 1], 5)),
            "h_hi": float(np.percentile(h[i:j + 1], 95)),
            "c": np.median(a[i:j + 1, :2], axis=0),
            "i0": i, "i1": j,
        })
    hmin = min(e["h"] for e in eps)
    hmax = max(e["h"] for e in eps)
    for k, e in enumerate(eps):
        if k == 0:
            e["note"] = "起始楼层"
        elif e["h"] >= hmax - 0.5:
            e["note"] = "顶层"
        elif e["h"] <= hmin + 0.5:
            e["note"] = "最低楼层"
        else:
            e["note"] = ""

    legs = []
    for (i, j, s) in find_legs(t, h, args.leg_thr):
        c = np.median(a[i:j + 1, :2], axis=0)
        legs.append({
            "name": ("上行" if s > 0 else "下行") + f"{sum(1 for L in legs if L['sign'] == s) + 1}",
            "sign": s, "t0": float(t[i]), "t1": float(t[j]),
            "h0": float(h[i]), "h1": float(h[j]), "c": c,
        })

    res = {"run": os.path.basename(run.rstrip("/")), "n": len(xyz), "up": up,
           "fold": args.fold,
           "h_range": (float(h.min()), float(h.max())), "eps": eps, "legs": legs}

    if args.no_map or not legs:
        return res

    pts = read_pcd_xyz(os.path.join(run, "map.pcd"), max_points=args.max_points)
    if pts is None or len(pts) == 0:
        res["legs"] = []
        return res
    pts = pts @ R_align.T

    for L in legs:
        lo = min(L["h0"], L["h1"])
        hi = max(L["h0"], L["h1"])
        sel = ((pts[:, 0] - L["c"][0]) ** 2 + (pts[:, 1] - L["c"][1]) ** 2) < args.radius ** 2
        rows = yaw_profile(pts, sel, lo + 1.0, hi - 1.0, args.zbin, args.fold)
        ok = [(r["h"], r["un"]) for r in rows if np.isfinite(r["un"])]
        L["rows"] = rows
        if len(ok) >= 4:
            hh = np.array([o[0] for o in ok])
            yy = np.degrees(np.array([o[1] for o in ok]))
            slope, rms = fit_line(hh, yy)
            L["slope"] = slope
            L["rms"] = rms
            L["dyaw"] = float(yy[-1] - yy[0])
            L["yaw_lo"] = float(yy[0])
            L["yaw_hi"] = float(yy[-1])
            L["h_span"] = float(hh[-1] - hh[0])
            L["n_bins"] = len(ok)
        else:
            L["slope"] = L["dyaw"] = L["yaw_lo"] = L["yaw_hi"] = float("nan")
            L["rms"] = L["n_bins"] = float("nan")
            L["h_span"] = float("nan")

    # 楼层访问段：各自 xy 邻域内 h±1 m 的墙面 yaw。
    # 单个裁剪半径下单条主方向会随半径抖 ±2°，这里取 0.75/1.0/1.25 倍半径的中位数，
    # 并把半极差作为不确定度一起输出。
    for e in eps:
        vals = []
        for k in (0.75, 1.0, 1.25):
            rr = args.floor_radius * k
            sel = ((pts[:, 0] - e["c"][0]) ** 2 + (pts[:, 1] - e["c"][1]) ** 2) < rr * rr
            rows = yaw_profile(pts, sel, e["h"] - 1.0, e["h"] + 1.0, 2.0, args.fold,
                               min_band=300, min_wall=200)
            best = max((r for r in rows if r["th"] is not None),
                       key=lambda r: r["n_lines"] * r["strength"], default=None)
            if best is not None and best["strength"] > 0.5:
                vals.append((float(np.degrees(best["un"])), best["strength"],
                             best["n_lines"], best["n"]))
        if vals:
            # 三个半径各自可能落在折叠分支的两侧（墙向恰在 45° 附近），先按 fold 折回再取中位
            y0 = vals[0][0]
            yv = np.array([y0 + wrap_fold(v[0] - y0, args.fold) for v in vals])
            e.update({"yaw": float(np.median(yv)),
                      "yaw_unc": float(yv.max() - yv.min()) / 2.0,
                      "strength": vals[0][1], "n_lines": vals[0][2],
                      "n_band": vals[0][3]})
        else:
            e.update({"yaw": np.nan, "yaw_unc": np.nan,
                      "strength": 0.0, "n_lines": 0, "n_band": 0})
    ref = next((e["yaw"] for e in eps if np.isfinite(e.get("yaw", np.nan))), np.nan)
    for e in eps:
        e["dyaw"] = wrap_fold(e.get("yaw", np.nan) - ref, args.fold)
    return res


def wrap_fold(x, fold=4):
    """fold 折叠后角度只能确定到 mod (180/fold)°，取绝对值最小的一支。"""
    if not np.isfinite(x):
        return x
    p = 180.0 / fold
    return (x + p / 2.0) % p - p / 2.0


def link_two_legs(legs, fold=4):
    """上下行各取第一条有效段，返回 (up, down) 与底部/顶部相对 yaw。"""
    up = next((L for L in legs if L["sign"] > 0 and np.isfinite(L.get("slope", np.nan))), None)
    dn = next((L for L in legs if L["sign"] < 0 and np.isfinite(L.get("slope", np.nan))), None)
    out = {"up": up, "dn": dn, "sign_opposite": np.nan,
           "base_diff": np.nan, "top_diff": np.nan, "loop_yaw": np.nan}
    if up is not None and dn is not None:
        out["sign_opposite"] = int(np.sign(up["slope"]) * np.sign(dn["slope"]) < 0)
        out["base_diff"] = wrap_fold(dn["yaw_lo"] - up["yaw_lo"], fold)
        out["top_diff"] = wrap_fold(dn["yaw_hi"] - up["yaw_hi"], fold)
        # 底部两梯的墙向差里含"建筑本身的两梯夹角 + 全程累积扭转"，
        # 顶部两梯的差几乎只含建筑夹角（两次到访相隔几分钟），相减即得地图隐含的闭环扭转。
        out["loop_yaw"] = out["base_diff"] - out["top_diff"]
    return out


def print_run(r, detail_zbin):
    print(f"\n=== {r['run']}  帧={r['n']}  up_world={np.round(r['up'], 3)}  "
          f"高度 {r['h_range'][0]:+.2f}..{r['h_range'][1]:+.2f} m")

    print("  楼层访问段")
    print(f"  {'id':<4}{'t(s)':>15}{'h(m)':>17}{'dwell':>8}"
          f"{'xy中心':>18}{'地图墙面yaw°':>20}{'Δ vs 起始°':>12}  备注")
    for e in r["eps"]:
        yaw = e.get("yaw", np.nan)
        dyaw = e.get("dyaw", np.nan)
        print(f"  {e['label']:<4}{e['t0']:7.1f}-{e['t1']:6.1f}"
              f"{e['h']:+8.2f}±{max(e['h_hi']-e['h'], e['h']-e['h_lo']):5.2f}"
              f"{e['t1']-e['t0']:8.1f}"
              f"{'(%+.1f,%+.1f)' % (e['c'][0], e['c'][1]):>18}"
              f"{yaw:11.2f}±{(e.get('yaw_unc') or 0.0):5.2f}{dyaw:+12.2f}  {e['note']}")

    for L in r["legs"]:
        rows = L.get("rows") or []
        if not rows:
            continue
        print(f"  楼梯段 {L['name']}  t {L['t0']:.1f}-{L['t1']:.1f}s  "
              f"h {L['h0']:+.2f}->{L['h1']:+.2f}  xy中心 "
              f"({L['c'][0]:+.1f},{L['c'][1]:+.1f})  拟合 {L.get('slope', np.nan):+.3f} deg/m"
              f"  Δyaw {L.get('dyaw', np.nan):+.2f}°  rms {L.get('rms', np.nan):.2f}°")
        if detail_zbin:
            print(f"    {'h(m)':>7}{'yaw°':>9}{'Δ相邻°':>9}{'强度':>6}{'线数':>5}{'点数':>7}")
            prev = None
            for b in rows:
                d = "" if prev is None or not np.isfinite(b["un"]) else \
                    f"{np.degrees(b['un'] - prev):+9.2f}"
                print(f"    {b['h']:+7.1f}{np.degrees(b['un']):9.2f}{d:>9}"
                      f"{b['strength']:6.2f}{b['n_lines']:5d}{b['n']:7d}")
                if np.isfinite(b["un"]):
                    prev = b["un"]


def summary_header():
    print(f"\n{'run':<22}{'闭环3D':>8}{'闭环yaw':>8}{'上行deg/m':>10}{'上行Δ°':>8}"
          f"{'下行deg/m':>10}{'下行Δ°':>8}{'反号':>5}{'底部两梯差°':>12}"
          f"{'顶部两梯差°':>12}{'地图闭环°':>10}")


def summary_line(r, loop=None):
    L = link_two_legs(r["legs"], r.get("fold", 4))
    return (f"{r['run']:<22}"
            f"{(loop['d3'] if loop else np.nan):>8.3f}"
            f"{(loop['yaw'] if loop else np.nan):+8.3f}"
            f"{(L['up']['slope'] if L['up'] else np.nan):+10.3f}"
            f"{(L['up']['dyaw'] if L['up'] else np.nan):+8.2f}"
            f"{(L['dn']['slope'] if L['dn'] else np.nan):+10.3f}"
            f"{(L['dn']['dyaw'] if L['dn'] else np.nan):+8.2f}"
            f"{int(L['sign_opposite']) if L['sign_opposite'] == L['sign_opposite'] else '-':>5}"
            f"{L['base_diff']:+12.2f}{L['top_diff']:+12.2f}{L['loop_yaw']:+10.2f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--static", type=float, default=3.0)
    ap.add_argument("--dwell", type=float, default=8.0, help="停留段最短时长 s")
    ap.add_argument("--tol", type=float, default=0.8, help="dwell 窗内高度极差阈值 m")
    ap.add_argument("--leg-thr", type=float, default=0.10, help="楼梯段 |dh/dt| m/s")
    ap.add_argument("--zbin", type=float, default=2.0, help="高度分层厚度 m")
    ap.add_argument("--fold", type=int, default=4, help="墙面方向折叠数（矩形井道 4）")
    ap.add_argument("--radius", type=float, default=5.0, help="楼梯段 xy 裁剪半径 m")
    ap.add_argument("--floor-radius", type=float, default=8.0, help="楼层 xy 裁剪半径 m")
    ap.add_argument("--max-points", type=int, default=6_000_000)
    ap.add_argument("--no-map", action="store_true")
    ap.add_argument("--detail", default="", help="逗号分隔，打印这些 run 的分层剖面")
    ap.add_argument("runs", nargs="+")
    args = ap.parse_args()

    up_imu = imu_up(args.bag, args.static)
    print(f"[imu] up_imu = {up_imu.round(4)}")

    detail = {s.strip() for s in args.detail.split(",") if s.strip()}
    out = []
    for run in args.runs:
        r = analyze(run, up_imu, args)
        if r is None:
            print(f"\n=== {os.path.basename(run.rstrip('/'))}: 无可用 odom")
            continue
        r["loop"] = measure(run, up_imu)
        print_run(r, args.zbin if r["run"] in detail else 0.0)
        out.append(r)

    if out:
        summary_header()
        for r in out:
            print(summary_line(r, r.get("loop")))


if __name__ == "__main__":
    main()
