#!/usr/bin/env python3
"""按楼层提取墙面直线，估计每层相对参考层（起始楼层）的 xyz / yaw 偏差。

动机
----
楼梯间几何退化时累积的漂移，在末端 map.pcd 里会表现成"本该笔直的井道墙变弯"。
如果只看"回到起点"的位姿误差，回程可能被已有地图一路吸住，把第一趟的扭曲
掩盖掉；但地图是逐帧按当时的位姿累积的，每层墙面的位置仍然保留了建图时刻
的漂移，所以逐层测量墙面相对偏移可以把漂移曲线还原出来。

做法
----
1. 从 odom 的 z 平台段自动分楼层（同层多次到访按 z 合并）；
2. 取每层 z±1 m 的点，先用 xy 0.2 m 网格做竖向性筛选（格内 z 跨度 > 0.6 m），
   滤掉楼板/天花板，只留竖直墙面；
3. 在 xy 平面做 RANSAC 直线提取（内点阈值、跨度、密度、覆盖率四重门控）；
4. 把每层直线集与参考层做 2D 刚体配准：穷举 yaw，按法向夹角+偏移就近匹配，
   加权最小二乘解平移；法向张不满时标记不可观测；
5. Δz 用"等层高"假设比较（名义 z = 参考层 z + 层序 × 各层名义层高），
   残差就是 z 方向的累积漂移；
6. 另外输出参考层最强墙面在整栋高度上的"弯曲曲线"（bow），与楼层无关，
   是单个标量指标。

输出（写入 run_dir）
--------------------
walls.csv      每层一行：z、层序、Δx/Δy/Δz/Δyaw、直线数、配准残差、可观测性
wall_bow.csv   每 0.5 m 高度一行：沿参考墙法向的中位偏移与点数

CLI
---
  python3 wall_metrics.py results_xxx/exp0_baseline [--ref-floor-name 10]
"""
import argparse
import csv
import os

import numpy as np
from scipy.ndimage import median_filter

from bag_io import load_odom, read_pcd_xyz, traj_arrays

DEG = 180.0 / np.pi


def yaw_from_quat(qx, qy, qz, qw):
    return np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


# ----------------------------------------------------------------------------
# 楼层检测
# ----------------------------------------------------------------------------
def detect_floor_levels(t, xyz, min_dwell=4.0, slope_thr=0.035, merge_tol=0.9,
                        min_gap=2.0):
    """从 z(t) 的平台段检测楼层。

    平台段 = |dz/dt| 持续小于 slope_thr 且时长 >= min_dwell。
    返回 [{'z','t_first','t_last','dwell'}...]，按 z 升序。
    """
    if len(t) < 10:
        return []
    z = xyz[:, 2]
    dt = float(np.median(np.diff(t)))
    if dt <= 0:
        return []
    win = max(3, int(round(1.0 / dt)) | 1)
    zs = median_filter(z, size=win, mode="nearest")
    vz = np.gradient(zs, t)

    still = np.abs(vz) < slope_thr
    runs = []
    i, n = 0, len(t)
    while i < n:
        if still[i]:
            j = i
            while j + 1 < n and still[j + 1]:
                j += 1
            if t[j] - t[i] >= min_dwell:
                runs.append((i, j))
            i = j + 1
        else:
            i += 1

    lv = []
    for i, j in runs:
        lv.append({
            "z": float(np.median(z[i:j + 1])),
            "t_first": float(t[i] - t[0]),
            "t_last": float(t[j] - t[0]),
            "dwell": float(t[j] - t[i]),
        })
    lv.sort(key=lambda d: d["z"])

    merged = []
    for cur in lv:
        if merged and abs(cur["z"] - merged[-1][-1]["z"]) < merge_tol:
            merged[-1].append(cur)
        else:
            merged.append([cur])

    levels = []
    for grp in merged:
        levels.append({
            "z": float(np.median([g["z"] for g in grp])),
            "t_first": min(g["t_first"] for g in grp),
            "t_last": max(g["t_last"] for g in grp),
            "dwell": sum(g["dwell"] for g in grp),
        })

    # 层间至少 min_gap：太近的合并（保留 dwell 大的那个）
    out = []
    for cur in levels:
        if out and cur["z"] - out[-1]["z"] < min_gap:
            if cur["dwell"] > out[-1]["dwell"]:
                out[-1] = cur
            continue
        out.append(cur)
    return out


# ----------------------------------------------------------------------------
# 点云 -> 竖直墙面直线
# ----------------------------------------------------------------------------
def vertical_filter(pts, cell=0.2, min_span=0.6):
    """只保留"所在 xy 格子内 z 跨度大于 min_span"的点（竖直结构）。"""
    if len(pts) == 0:
        return np.zeros(0, bool)
    xy = pts[:, :2]
    key = np.floor(xy / cell).astype(np.int64)
    kx = key[:, 0] - key[:, 0].min()
    ky = key[:, 1] - key[:, 1].min()
    k = kx * (ky.max() + 1) + ky
    order = np.argsort(k, kind="stable")
    ks = k[order]
    zs = pts[order, 2]
    uk, start = np.unique(ks, return_index=True)
    end = np.append(start[1:], len(ks))
    span = np.maximum.reduceat(zs, start) - np.minimum.reduceat(zs, start)
    good_keys = uk[span > min_span]
    mask = np.zeros(len(pts), bool)
    mask[order[np.isin(ks, good_keys)]] = True
    return mask


def ransac_lines(pts, thr=0.05, iters=400, max_lines=8, min_inliers=150,
                 min_span=2.0, min_density=4.0, min_coverage=0.5, seed=12345):
    """在 xy 平面提取直线。pts: (N,2)。返回 [{'n','rho','count','span','rms'}]。"""
    rng = np.random.default_rng(seed)
    rem = np.asarray(pts, dtype=np.float64)
    lines = []
    for _ in range(max_lines):
        n_pts = len(rem)
        if n_pts < min_inliers:
            break
        sub = rem if n_pts <= 40000 else rem[rng.choice(n_pts, 40000, replace=False)]
        i1 = rng.integers(0, len(sub), iters)
        i2 = rng.integers(0, len(sub), iters)
        p1, p2 = sub[i1], sub[i2]
        d = p2 - p1
        L = np.linalg.norm(d, axis=1)
        ok = L > 1.0
        if not ok.any():
            break
        d, p1, L = d[ok], p1[ok], L[ok]
        nrm = np.stack([-d[:, 1], d[:, 0]], axis=1) / L[:, None]
        rho = (nrm * p1).sum(1)
        flip = (nrm[:, 1] < 0) | ((np.abs(nrm[:, 1]) < 1e-9) & (nrm[:, 0] < 0))
        nrm[flip] *= -1.0
        rho[flip] *= -1.0

        resid = np.abs(sub @ nrm.T - rho[None, :])
        cnt = (resid < thr).sum(0)
        j = int(np.argmax(cnt))
        if cnt[j] < min_inliers:
            break
        n, r = nrm[j], float(rho[j])

        res_all = np.abs(rem @ n - r)
        inl = res_all < thr
        P = rem[inl]
        if len(P) < min_inliers:
            break
        tang = np.array([-n[1], n[0]])
        u = P @ tang
        span = float(u.max() - u.min())
        nb = max(1, int(np.ceil(span / 0.3)))
        hist = np.histogram(u, bins=nb, range=(u.min(), u.max()))[0]
        cover = float((hist > 0).mean())
        dens = len(P) / max(span, 1e-6)
        if span < min_span or cover < min_coverage or dens < min_density:
            # 不是墙（多半是楼板/散点），剔除其内点后继续
            rem = rem[~inl]
            continue
        lines.append({
            "n": n, "rho": r, "count": int(len(P)), "span": span,
            "rms": float(np.sqrt((res_all[inl] ** 2).mean())),
        })
        rem = rem[~inl]
    return lines


def _rot(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def dominant_angle(lines, fold=4):
    """墙面法向的主方向（折叠 fold 次，矩形井道用 4 折即 mod 90°）。

    返回 (theta, strength)：theta ∈ [0, pi/fold)，strength ∈ [0,1] 为方向集中度。
    用 count*span 加权；折叠加权平均对“四面墙都是同一朝向族”的矩形井道稳定。
    """
    if not lines:
        return None, 0.0
    th = np.array([np.arctan2(L["n"][1], L["n"][0]) for L in lines])
    w = np.array([max(L["count"], 1.0) * min(L["span"], 8.0) for L in lines])
    c = float((w * np.cos(fold * th)).sum())
    s = float((w * np.sin(fold * th)).sum())
    strength = float(np.hypot(c, s) / max(w.sum(), 1e-9))
    ang = np.arctan2(s, c) / fold
    period = np.pi / fold
    return float(ang % period), strength


def _wrap_to(x, period):
    return (x + period / 2.0) % period - period / 2.0


def align_to_ref(src, ref, match_deg=3.0, max_shift=3.0, fold=4):
    """把 src 直线集刚体对齐到 ref：p_ref ≈ R(a)·p_src + D。

    yaw 由墙面法向主方向差给出（矩形井道折叠到 ±45° 内），
    平移在给定 yaw 下用直线偏移量的加权最小二乘求解。

    返回的 dyaw_deg / dx,dy 已换算成“本层相对参考层的偏差”（本层 - 参考层，
    逆时针为正；平移在参考层坐标系下）。
    """
    if len(src) < 3 or len(ref) < 2:
        return None
    th_s, st_s = dominant_angle(src, fold)
    th_r, st_r = dominant_angle(ref, fold)
    if th_s is None or th_r is None:
        return None
    period = np.pi / fold
    dyaw = _wrap_to(th_s - th_r, period)
    a = -dyaw

    Rm = _rot(a)
    cos_t = np.cos(np.deg2rad(match_deg))
    rows, bs, ws = [], [], []
    for L in src:
        n2 = Rm @ L["n"]
        cand = None
        for Lr in ref:
            dot = float(n2 @ Lr["n"])
            if abs(dot) < cos_t:
                continue
            s = 1.0 if dot > 0 else -1.0
            b = Lr["rho"] - s * L["rho"]
            if abs(b) > max_shift:
                continue
            if cand is None or abs(b) < abs(cand[1]):
                cand = (s * n2, b)
        if cand is not None:
            rows.append(cand[0])
            bs.append(cand[1])
            ws.append(float(L["count"]))
    if len(rows) < 3:
        return None

    A = np.asarray(rows)
    b = np.asarray(bs)
    w = np.asarray(ws)
    Aw = A * w[:, None]
    D, *_ = np.linalg.lstsq(Aw, b * w, rcond=None)
    resid = np.abs(A @ D - b)
    cost = float((resid * w).sum() / w.sum())
    M = Aw.T @ A
    ev = np.linalg.eigvalsh(M)
    obs = float(ev[0] / max(ev[-1], 1e-9))

    R = _rot(a)
    dxy = -(R.T @ D)          # p_ref = R p_src + D  =>  d = src - ref = -R^T D
    return {
        "align_deg": float(a * DEG),
        "dyaw_deg": float(dyaw * DEG),
        "yaw_strength": float(min(st_s, st_r)),
        "dx": float(dxy[0]),
        "dy": float(dxy[1]),
        "align_cost": cost,
        "n_match": len(rows),
        "obs": obs,
        # 矩形井道有时只看到 3 面墙（第 4 面在视野外）；配准代价低时 3 条线也可信
        "reliable": bool(obs > 0.02 and min(st_s, st_r) > 0.3
                         and (len(rows) >= 4 or (len(rows) == 3 and cost < 0.25))),
    }


def bow_profile(pts, line, half_width=0.4, zbin=0.5, min_count=30):
    """参考层某条墙在整栋高度上的偏移曲线（检验"直墙变弯"）。"""
    n = line["n"]
    sd = pts[:, :2] @ n - line["rho"]
    m = np.abs(sd) < half_width
    if m.sum() < min_count:
        return []
    z = pts[m, 2]
    s = sd[m]
    lo, hi = z.min(), z.max()
    nb = max(1, int(np.ceil((hi - lo) / zbin)))
    idx = np.clip(((z - lo) / max(hi - lo, 1e-6) * nb).astype(int), 0, nb - 1)
    out = []
    for k in range(nb):
        sel = idx == k
        if sel.sum() < min_count:
            continue
        out.append({
            "z": float(lo + (k + 0.5) * (hi - lo) / nb),
            "offset": float(np.median(s[sel])),
            "count": int(sel.sum()),
        })
    return out


# ----------------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------------
def analyze_run(run_dir, ref_floor_name=10, verbose=False):
    odom = load_odom(os.path.join(run_dir, "odom.csv"))
    if odom is None or len(odom) < 50:
        return None
    t_abs, xyz = traj_arrays(odom)
    # 轨迹若已发散（位置跑到 500 m 外），地图也是按错误位姿拼的，分层结果没有意义
    bad = (np.abs(xyz) > 500.0).any(axis=1)
    if bad.any():
        t_abs, xyz = t_abs[~bad], xyz[~bad]
        if len(xyz) < 200:
            return None
    t = t_abs - t_abs[0]

    levels = detect_floor_levels(t, xyz)
    if not levels:
        return None

    # 参考层 = 起始时刻所在楼层
    z0 = float(np.median(xyz[:min(len(xyz), 50), 2]))
    ref_i = int(np.argmin([abs(lv["z"] - z0) for lv in levels]))
    for i, lv in enumerate(levels):
        lv["rel"] = i - ref_i
        # 层间距不一定均匀（没停过的楼层不会被检测到），所以只标"相对起始层第几层停留"，
        # 不臆造 8F/11F 这类绝对楼层名，避免和真实楼层对不上。
        lv["name"] = (f"{ref_floor_name}F" if lv["rel"] == 0 else f"L{lv['rel']:+d}")

    # 细体素实验的地图可达 1e8 级点，这里统一下采样到 6M，墙面直线/RANSAC 本就是抽样统计
    pts = read_pcd_xyz(os.path.join(run_dir, "map.pcd"), max_points=6_000_000)
    if pts is None:
        pts = np.zeros((0, 3))

    # 井道中心：用爬楼段（|vz| 大）的轨迹中位数
    if len(t) > 10 and pts.size:
        vz = np.gradient(median_filter(xyz[:, 2], size=5, mode="nearest"), t)
        fly = np.abs(vz) > 0.08
        center = np.median(xyz[fly, :2], axis=0) if fly.sum() > 20 else np.median(xyz[:, :2], axis=0)
    else:
        center = np.array([0.0, 0.0])

    crop_r = 12.0
    rows = []
    ref_lines = None
    for lv in levels:
        zc = lv["z"]
        sel = (pts[:, 2] > zc - 1.0) & (pts[:, 2] < zc + 1.0)
        band = pts[sel]
        if len(band):
            d2 = ((band[:, :2] - center) ** 2).sum(1)
            band = band[d2 < crop_r * crop_r]
        row = {
            "floor": lv["name"], "rel": lv["rel"], "z": round(zc, 3),
            "t_first": round(lv["t_first"], 1), "dwell": round(lv["dwell"], 1),
            "n_band": int(len(band)),
        }
        wl = []
        if len(band) >= 200:
            vm = vertical_filter(band)
            wall_pts = band[vm]
            row["n_wall"] = int(len(wall_pts))
            if len(wall_pts) >= 200:
                wl = ransac_lines(wall_pts[:, :2])
            row["n_lines"] = len(wl)
            if wl:
                row["line_span"] = round(max(L["span"] for L in wl), 2)
                row["line_rms"] = round(float(np.median([L["rms"] for L in wl])), 3)
        else:
            row["n_wall"] = 0
            row["n_lines"] = 0
        lv["lines"] = wl
        if lv["rel"] == 0:
            ref_lines = wl
        rows.append(row)

    # 逐层对参考层配准
    for lv, row in zip(levels, rows):
        res = None
        if ref_lines and lv.get("lines"):
            if lv["rel"] == 0:
                res = {"align_deg": 0.0, "dyaw_deg": 0.0, "dx": 0.0, "dy": 0.0,
                       "align_cost": 0.0, "n_match": len(ref_lines), "obs": 1.0,
                       "yaw_strength": 1.0, "reliable": True}
            else:
                res = align_to_ref(lv["lines"], ref_lines)
        if res:
            row.update({
                "dyaw_deg": round(res["dyaw_deg"], 3),
                "dx": round(res["dx"], 3), "dy": round(res["dy"], 3),
                "align_cost": round(res["align_cost"], 3),
                "n_match": res["n_match"], "obs": round(res["obs"], 4),
                "yaw_strength": round(res["yaw_strength"], 3),
                "reliable": int(res["reliable"]),
            })
        else:
            row.update({"dyaw_deg": np.nan, "dx": np.nan, "dy": np.nan,
                        "align_cost": np.nan, "n_match": 0, "obs": np.nan,
                        "yaw_strength": np.nan, "reliable": 0})

    # z 方向：仅在"检测到的层间距近似均匀"时，等层高拟合才有意义。
    # 楼梯间往往只在停过的楼层留下平台，中间楼层是连续走下来的，层间隔会差好几倍，
    # 此时 floor_h / dz_res 不可信，直接置 NaN，另给逐层的绝对 z 落差。
    idxs = np.arange(len(levels), dtype=float)
    zv = np.array([lv["z"] for lv in levels])
    gaps = np.abs(np.diff(zv))
    uniform = len(levels) >= 3 and gaps.min() > 0.5 and gaps.max() / gaps.min() < 1.5
    if uniform:
        h, a = np.polyfit(idxs, zv, 1)
        for lv, row in zip(levels, rows):
            row["dz_res"] = round(float(lv["z"] - (a + h * lv["rel"])), 3)
        floor_h = float(h)
    else:
        floor_h = float("nan")
        for row in rows:
            row["dz_res"] = np.nan

    # bow：参考层跨度最大的墙，沿整栋高度统计偏移
    bow = []
    if ref_lines:
        main = max(ref_lines, key=lambda L: L["span"] * L["count"])
        sel = ((pts[:, 0] - center[0]) ** 2 + (pts[:, 1] - center[1]) ** 2) < crop_r * crop_r
        if sel.sum():
            bow = bow_profile(pts[sel], main)
    bow_pp = float("nan")
    if len(bow) >= 4:
        off = np.array([b["offset"] for b in bow])
        bow_pp = float(np.percentile(off, 95) - np.percentile(off, 5))

    # 写盘
    if rows:
        fields = ["floor", "rel", "z", "t_first", "dwell", "n_band", "n_wall",
                  "n_lines", "line_span", "line_rms", "dx", "dy", "dz_res",
                  "dyaw_deg", "align_cost", "n_match", "obs", "yaw_strength",
                  "reliable"]
        with open(os.path.join(run_dir, "walls.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            for r in rows:
                w.writerow({k: r.get(k, "") for k in fields})
    if bow:
        with open(os.path.join(run_dir, "wall_bow.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["z", "offset", "count"])
            w.writeheader()
            for b in bow:
                w.writerow({"z": round(b["z"], 2), "offset": round(b["offset"], 4),
                            "count": b["count"]})

    if verbose:
        print(f"[walls] {os.path.basename(run_dir)}: {len(levels)} 层, "
              f"参考层 {levels[ref_i]['name']} (z={levels[ref_i]['z']:.2f}), "
              f"层高 {floor_h:.2f} m, bow_pp {bow_pp:.3f} m")
        for r in rows:
            print("  {floor:>5s} z={z:8.3f} dx={dx:>7} dy={dy:>7} dz={dz:>7} "
                  "dyaw={dyaw:>8} lines={n_lines:>2} obs={obs}".format(
                      floor=r["floor"], z=r["z"],
                      dx=_fmt(r.get("dx")), dy=_fmt(r.get("dy")),
                      dz=_fmt(r.get("dz_res")), dyaw=_fmt(r.get("dyaw_deg")),
                      n_lines=r.get("n_lines", 0), obs=_fmt(r.get("obs"), 4)))

    by_name = {r["floor"]: r for r in rows}
    lowest = rows[0]["floor"] if rows else None
    highest = rows[-1]["floor"] if rows else None

    def pick(name, key):
        r = by_name.get(name)
        return np.nan if not r else float(r.get(key, np.nan))

    return {
        "floor_count": len(levels),
        "ref_floor": levels[ref_i]["name"],
        "floor_h": floor_h,
        "bow_pp": bow_pp,
        "low_floor": lowest,
        "high_floor": highest,
        "low_rel": levels[0]["rel"], "top_rel": levels[-1]["rel"],
        "low_dz_total": float(levels[0]["z"] - levels[ref_i]["z"]) if levels else np.nan,
        "top_dz_total": float(levels[-1]["z"] - levels[ref_i]["z"]) if levels else np.nan,
        "low_dx": pick(lowest, "dx"), "low_dy": pick(lowest, "dy"),
        "low_dz": pick(lowest, "dz_res"), "low_dyaw": pick(lowest, "dyaw_deg"),
        "low_reliable": pick(lowest, "reliable"),
        "top_dx": pick(highest, "dx"), "top_dy": pick(highest, "dy"),
        "top_dz": pick(highest, "dz_res"), "top_dyaw": pick(highest, "dyaw_deg"),
        "top_reliable": pick(highest, "reliable"),
        "rows": rows,
    }


def _fmt(v, nd=3):
    if v is None:
        return "   -  "
    try:
        if not np.isfinite(v):
            return "   -  "
        return f"{v:.{nd}f}"
    except TypeError:
        return str(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", nargs="+", help="实验目录（含 odom.csv / map.pcd）")
    ap.add_argument("--ref-floor-name", type=int, default=10,
                    help="起始楼层名（默认 10）")
    args = ap.parse_args()
    for d in args.run_dir:
        res = analyze_run(d, args.ref_floor_name, verbose=True)
        if res is None:
            print(f"[walls] {d}: 无可用 odom")


if __name__ == "__main__":
    main()
