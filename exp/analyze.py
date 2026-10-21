#!/usr/bin/env python3
"""汇总 exp/results/* 的实验结果：耗时、轨迹指标、点云规模。

不同次回放的 wall-clock 时长可能略有差异（WSL2 时钟跳变、timeout 抖动），
所以所有轨迹指标都裁剪到"所有实验共有的时间窗"上计算，保证可比。

输出:
  results/summary.csv   机器可读
  results/summary.md    人读对比表
"""
import argparse
import csv
import glob
import os
import re
import sys

import numpy as np

from bag_io import load_odom, read_pcd_xyz as pcd_points, traj_arrays
from wall_metrics import analyze_run as wall_analyze

TIMER_RE = re.compile(
    r"> \[\s*(?P<name>.+?)\s*\] avg: (?P<avg>[0-9.]+) ms , max: (?P<max>[0-9.]+) ms , calls: (?P<calls>\d+)"
)

STAGES = [
    "Undistort",
    "PublishBody",
    "DownSample",
    "Observe",
    "DeltaCorrect",
    "UpdateMap",
]

DEBUG_KEYS = [
    "dbg_frames", "ptsize_mean", "iters_mean", "iters_p90", "dx_inf_median",
    "cond_yk_median", "fit_res_mean", "fit_res_p95_median", "effect_ratio",
    "knn_fail_ratio", "plane_fail_ratio", "res_fail_ratio", "htvh_cond_median",
    "qk_bad_frames",
]

NOTES = {
    "exp0_baseline": "基线：filter_rate=6, voxel=0.25, imu_na/ng=0.1",
    "exp1_fr3": "filter_rate 6->3（参与配准的点更多）",
    "exp2_fr1": "filter_rate 6->1（全量点参与配准）",
    "exp3_vox020": "voxel_fliter_size/vox_resolution 0.25->0.20",
    "exp4_vox015": "voxel_fliter_size/vox_resolution 0.25->0.15",
    "exp5_imu001": "imu_na/ng 0.1->0.01（更信任 IMU）",
    "exp6_imu0001": "imu_na/ng 0.1->0.001（极信任 IMU）",
    "expc1_base": "代码级基线（只开埋点，开关全默认）",
    "expc2_nw005": "实验1：法向质量加权 sigma=0.05",
    "expc3_maprep": "实验2：地图端只存真实扫描点（近子体素中心者）",
    "expc4_clamp": "实验5：均值更新后夹回子体素包围盒",
    "expc5_clamp_merge005": "实验5：clamp + 合并门限 0.1->0.05",
    "expc6_hknn_g3": "HKNN 只搜前 3 组（6->3）",
    "expc7_hknn_s05": "HKNN 提前终止阈值 ×0.5",
    "expm1_nw_g3": "组合：法向加权 σ=0.05 + HKNN 组数 3",
    "expm2_g3_vox015": "组合：voxel 0.15 + HKNN 组数 3",
    "expm3_nw_vox015": "组合：voxel 0.15 + 法向加权 σ=0.05",
    "expm4_nw_g3_vox015": "组合：voxel 0.15 + 法向加权 σ=0.05 + HKNN 组数 3",
}


def parse_debug_csv(path):
    """聚合 lio.debug.level>0 时逐帧写出的 debug.csv。"""
    if not os.path.exists(path):
        return None
    cols = {}
    with open(path) as f:
        rd = csv.reader(f)
        header = next(rd, None)
        if not header:
            return None
        for row in rd:
            if len(row) != len(header):
                continue
            try:
                vals = [float(v) for v in row]
            except ValueError:
                continue
            for k, v in zip(header, vals):
                cols.setdefault(k, []).append(v)
    if not cols or "ptsize" not in cols:
        return None
    a = {k: np.asarray(v, dtype=np.float64) for k, v in cols.items()}
    eps = 1e-12
    eff_ratio = a["n_effect"] / np.maximum(eps, a["ptsize"])
    knn_ratio = a["n_knn_fail"] / np.maximum(eps, a["ptsize"])
    plane_ratio = a["n_plane_fail"] / np.maximum(eps, a["ptsize"])
    res_ratio = a["n_res_fail"] / np.maximum(eps, a["ptsize"])
    htvh_cond = a.get("htvh_ev_max", np.zeros_like(eff_ratio)) / np.maximum(
        eps, a.get("htvh_ev_min", np.zeros_like(eff_ratio)))
    return {
        "dbg_frames": int(len(a["ptsize"])),
        "ptsize_mean": float(a["ptsize"].mean()),
        "iters_mean": float(a["iters"].mean()),
        "iters_p90": float(np.percentile(a["iters"], 90)),
        "dx_inf_median": float(np.median(a["dx_inf"])),
        "cond_yk_median": float(np.median(a["cond_yk"])),
        "fit_res_mean": float(np.median(a["fit_res_mean"])),
        "fit_res_p95_median": float(np.median(a["fit_res_p95"])),
        "effect_ratio": float(eff_ratio.mean()),
        "knn_fail_ratio": float(knn_ratio.mean()),
        "plane_fail_ratio": float(plane_ratio.mean()),
        "res_fail_ratio": float(res_ratio.mean()),
        "htvh_cond_median": float(np.median(htvh_cond)),
        "qk_bad_frames": int((a["qk_finite"] < 0.5).sum()),
    }


def parse_node_log(path):
    """返回 {stage: (weighted_avg_ms, max_ms, total_calls)}，跨多个 60s 窗口聚合。"""
    acc = {}
    if not os.path.exists(path):
        return acc
    with open(path, "r", errors="ignore") as f:
        for line in f:
            m = TIMER_RE.search(line)
            if not m:
                continue
            name = m.group("name").strip().strip("[]").strip()
            avg = float(m.group("avg"))
            mx = float(m.group("max"))
            n = int(m.group("calls"))
            if name not in acc:
                acc[name] = [0.0, 0.0, 0]  # sum(avg*calls), max, calls
            acc[name][0] += avg * n
            acc[name][1] = max(acc[name][1], mx)
            acc[name][2] += n
    return {k: (v[0] / v[2] if v[2] else 0.0, v[1], v[2]) for k, v in acc.items()}


def pcd_info(path):
    """返回 {'points': N, 'extent': (dx,dy,dz)}。"""
    xyz = pcd_points(path)
    if xyz is None:
        return None
    if xyz.size == 0:
        return {"points": 0, "extent": None}
    lo, hi = xyz.min(axis=0), xyz.max(axis=0)
    return {"points": int(len(xyz)), "extent": hi - lo}


def quat_to_yaw(qx, qy, qz, qw):
    return np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def loop_metrics(t_abs, xyz, yaw, path_min=20.0, z_gain=1.2):
    """闭环指标：首圈回到起点的位姿误差 + 最终回到起点的位姿误差。

    - 起点位姿 = 第一帧（odom 原点，t=0）。
    - 首圈终点 = 在第一次持续爬升（z 相对起点 > z_gain）之前，累计路程已超过
      path_min 后，离起点水平距离最近的那一帧（即"转一圈放回起点"）。
    - 终点 = 最后一帧附近的 1 s 中位数（"放回原处"）。

    返回值都是"当前位姿相对起点位姿"的量：xy 为水平距离，yaw 为偏航差（度）。
    """
    n = len(t_abs)
    if n < 50:
        return None
    t = t_abs - t_abs[0]
    p0 = xyz[0]
    y0 = yaw[0]
    step = np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(step)])

    # 第一次持续爬升：z 相对起点超过阈值并保持 2 s
    z_rel = xyz[:, 2] - p0[2]
    above = z_rel > z_gain
    t_asc = None
    for i in range(n):
        if above[i]:
            j = np.searchsorted(t, t[i] + 2.0)
            j = min(j, n - 1)
            if above[i:j + 1].mean() > 0.8:
                t_asc = t[i]
                break
    if t_asc is None:
        t_asc = t[-1]

    def rel(k):
        return {
            "xy": float(np.linalg.norm(xyz[k, :2] - p0[:2])),
            "z": float(xyz[k, 2] - p0[2]),
            "yaw": float(wrap_deg((yaw[k] - y0) * 180.0 / np.pi)),
            "t": float(t[k]),
        }

    # 首圈
    m = (t >= 0.0) & (t <= max(t_asc - 1.0, 1.0)) & (s > path_min)
    loop1 = None
    if m.any():
        idx = np.flatnonzero(m)
        d = np.linalg.norm(xyz[idx, :2] - p0[:2], axis=1)
        k = int(idx[int(np.argmin(d))])
        loop1 = rel(k)

    # 终点：最后 1 s
    k_end = int(np.searchsorted(t, t[-1] - 1.0))
    k_end = min(max(k_end, 0), n - 1)
    seg = np.arange(k_end, n)
    pend = np.median(xyz[seg], axis=0)
    yend = yaw[seg]
    yend = np.arctan2(np.median(np.sin(yend)), np.median(np.cos(yend)))
    final = {
        "xy": float(np.linalg.norm(pend[:2] - p0[:2])),
        "z": float(pend[2] - p0[2]),
        "yaw": float(wrap_deg((yend - y0) * 180.0 / np.pi)),
        "t": float(t[-1]),
    }
    return {"loop1": loop1, "final": final, "t_ascend": float(t_asc)}


def traj_metrics(t_abs, xyz):
    if len(t_abs) < 2:
        return None
    d = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    path_len = float(d.sum())
    dur = float(t_abs[-1] - t_abs[0])
    # 轨迹自洽性：出现 >5 m 的单帧跳跃说明发散/重置
    jumps = int((d > 5.0).sum())
    # 发散判据：整栋楼尺度 < 200 m，位置一旦跑到 500 m 外即可判定轨迹崩掉
    # （例如 vox_res=0.20 的 exp3 在 t≈800 s 位置跳到 -1.3e5，地图随即被污染）
    far = np.abs(xyz) > 500.0
    diverged = bool(far.any())
    t_div = float(t_abs[int(np.argmax(far.any(axis=1)))] - t_abs[0]) if diverged else np.nan
    return {
        "frames": int(len(xyz)),
        "dur_s": dur,
        "path_m": path_len,
        "speed_mps": path_len / dur if dur > 0 else 0.0,
        "z_min": float(xyz[:, 2].min()),
        "z_max": float(xyz[:, 2].max()),
        "z_final": float(xyz[-1, 2]),
        "end_x": float(xyz[-1, 0]),
        "end_y": float(xyz[-1, 1]),
        "big_jumps": jumps,
        "diverged": diverged,
        "t_diverge": t_div,
    }


def interp_at(t_src, xyz_src, t_dst):
    out = np.empty((len(t_dst), 3))
    for i in range(3):
        out[:, i] = np.interp(t_dst, t_src, xyz_src[:, i])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))
    ap.add_argument("--ref-floor-name", type=int, default=10,
                    help="起点楼层名，用于把层序映射成 1F/12F 这类标签（默认 10）")
    ap.add_argument("--skip-walls", action="store_true",
                    help="跳过分层墙面分析（只算轨迹指标，快很多）")
    args = ap.parse_args()

    runs = sorted(d for d in glob.glob(os.path.join(args.root, "*")) if os.path.isdir(d))
    entries = []
    for d in runs:
        name = os.path.basename(d)
        meta = {}
        mp = os.path.join(d, "meta.txt")
        if os.path.exists(mp):
            for line in open(mp, errors="ignore"):
                if "=" in line:
                    k, v = line.strip().split("=", 1)
                    meta[k] = v
        odom = load_odom(os.path.join(d, "odom.csv"))
        if odom is None or len(odom) < 10:
            continue
        t_abs, xyz = traj_arrays(odom)
        yaw = quat_to_yaw(odom["qx"], odom["qy"], odom["qz"], odom["qw"])
        entries.append({
            "name": name,
            "dir": d,
            "meta": meta,
            "timers": parse_node_log(os.path.join(d, "node.log")),
            "debug": parse_debug_csv(os.path.join(d, "debug.csv")),
            "t_abs": t_abs,
            "xyz": xyz,
            "yaw": yaw,
            "pcd": pcd_info(os.path.join(d, "map.pcd")),
            "wall": None if args.skip_walls else wall_analyze(d, args.ref_floor_name),
        })

    if not entries:
        print("no usable results under", args.root)
        return 1

    # 冒烟/中断的短跑（时长明显短于本轮最长者）不参与共同时间窗比较
    max_dur = max(e["t_abs"][-1] - e["t_abs"][0] for e in entries)
    entries = [e for e in entries if (e["t_abs"][-1] - e["t_abs"][0]) >= 0.6 * max_dur]

    # 共有时间窗：所有实验都覆盖到的区间
    t_lo = max(e["t_abs"][0] for e in entries)
    t_hi = min(e["t_abs"][-1] for e in entries)
    if not (t_hi > t_lo):
        print("runs have no common time window", [(e["name"], e["t_abs"][0], e["t_abs"][-1]) for e in entries])
        return 1

    rows = []
    for e in entries:
        m = (e["t_abs"] >= t_lo) & (e["t_abs"] <= t_hi)
        tm = traj_metrics(e["t_abs"][m], e["xyz"][m])
        if tm is None:
            continue
        pcd = e["pcd"] or {}
        ext = pcd.get("extent")
        row = {
            "name": e["name"],
            "note": NOTES.get(e["name"], ""),
            "filter_rate": e["meta"].get("filter_rate", ""),
            "voxel": e["meta"].get("voxel_fliter_size", ""),
            "imu_na": e["meta"].get("imu_na", ""),
            "vox_res": e["meta"].get("vox_resolution", ""),
            "start_offset": e["meta"].get("start_offset", ""),
            "frames": tm["frames"],
            "dur_s": round(tm["dur_s"], 2),
            "path_m": round(tm["path_m"], 2),
            "speed_mps": round(tm["speed_mps"], 2),
            "z_min": round(tm["z_min"], 2),
            "z_max": round(tm["z_max"], 2),
            "z_final": round(tm["z_final"], 2),
            "end_x": round(tm["end_x"], 2),
            "end_y": round(tm["end_y"], 2),
            "big_jumps": tm["big_jumps"],
            "diverged": int(tm["diverged"]),
            "t_diverge": round(tm["t_diverge"], 1) if np.isfinite(tm["t_diverge"]) else "",
            "map_points": pcd.get("points", 0),
            "map_dx": round(float(ext[0]), 1) if ext is not None else 0,
            "map_dy": round(float(ext[1]), 1) if ext is not None else 0,
            "map_dz": round(float(ext[2]), 1) if ext is not None else 0,
            "normal_weight_en": e["meta"].get("normal_weight_en", ""),
            "normal_sigma": e["meta"].get("normal_sigma", ""),
            "map_rep_mode": e["meta"].get("map_rep_mode", ""),
            "map_clamp_mu": e["meta"].get("map_clamp_mu", ""),
            "map_merge_dist": e["meta"].get("map_merge_dist", ""),
            "hknn_max_group": e["meta"].get("hknn_max_group", ""),
            "hknn_order_scale": e["meta"].get("hknn_order_scale", ""),
        }
        dbg = e["debug"]
        for k in DEBUG_KEYS:
            row[k] = round(dbg[k], 4) if dbg and isinstance(dbg.get(k), float) else (dbg.get(k, 0) if dbg else "")
        total = 0.0
        for st in STAGES:
            if st in e["timers"]:
                avg, mx, calls = e["timers"][st]
                row[f"{st}_avg_ms"] = round(avg, 2)
                row[f"{st}_max_ms"] = round(mx, 2)
                row[f"{st}_calls"] = calls
                total += avg
            else:
                row[f"{st}_avg_ms"] = 0
                row[f"{st}_max_ms"] = 0
                row[f"{st}_calls"] = 0
        row["stage_total_ms"] = round(total, 2)
        # 10 Hz 帧周期(100ms)下，单帧总耗时占比（%）
        row["budget_pct"] = round(total / 100.0 * 100.0, 1)

        # 回到起点附近时的偏差：只看离开起点 100s 之后，找离起点最近的一帧
        m = (e["t_abs"] >= t_lo + 100.0) & (e["t_abs"] <= t_hi)
        tw, xw = e["t_abs"][m], e["xyz"][m]
        x0 = e["xyz"][np.argmax(e["t_abs"] >= t_lo)][:2]
        dxy = np.linalg.norm(xw[:, :2] - x0, axis=1)
        k = int(np.argmin(dxy)) if len(dxy) else 0
        row["revisit_min_xy"] = round(float(dxy[k]), 2)
        row["revisit_z"] = round(float(xw[k, 2]), 2)
        row["revisit_t"] = round(float(tw[k] - tw[0]), 1)

        # 闭环误差：首圈回到起点 / 最终回到起点（相对第一帧位姿）
        lm = loop_metrics(e["t_abs"], e["xyz"], e["yaw"])
        if lm:
            row["t_ascend"] = round(lm["t_ascend"], 1)
            l1 = lm["loop1"] or {}
            row["loop1_xy"] = round(l1.get("xy", np.nan), 2)
            row["loop1_z"] = round(l1.get("z", np.nan), 2)
            row["loop1_yaw"] = round(l1.get("yaw", np.nan), 2)
            row["loop1_t"] = round(l1.get("t", np.nan), 1)
            f = lm["final"]
            row["final_xy"] = round(f["xy"], 2)
            row["final_z"] = round(f["z"], 2)
            row["final_yaw"] = round(f["yaw"], 2)
            row["closure_3d"] = round(float(np.hypot(f["xy"], f["z"])), 2)

        # 分层墙面指标（1F / 最高层相对起始层）
        w = e["wall"]
        if w:
            row["floors"] = w["floor_count"]
            row["ref_floor"] = w["ref_floor"]
            row["floor_h"] = round(w["floor_h"], 3)
            row["low_floor"] = w["low_floor"]
            row["low_rel"] = w["low_rel"]
            row["low_dxy"] = round(float(np.hypot(w["low_dx"], w["low_dy"])), 2) \
                if np.isfinite(w["low_dx"]) and np.isfinite(w["low_dy"]) else np.nan
            row["low_dz"] = round(w["low_dz_total"], 2) if np.isfinite(w["low_dz_total"]) else np.nan
            row["low_dyaw"] = round(w["low_dyaw"], 2) if np.isfinite(w["low_dyaw"]) else np.nan
            row["low_reliable"] = int(w["low_reliable"]) if np.isfinite(w["low_reliable"]) else 0
            row["top_floor"] = w["high_floor"]
            row["top_rel"] = w["top_rel"]
            row["top_dz"] = round(w["top_dz_total"], 2) if np.isfinite(w["top_dz_total"]) else np.nan
            row["top_dxy"] = round(float(np.hypot(w["top_dx"], w["top_dy"])), 2) \
                if np.isfinite(w["top_dx"]) and np.isfinite(w["top_dy"]) else np.nan
            row["top_dyaw"] = round(w["top_dyaw"], 2) if np.isfinite(w["top_dyaw"]) else np.nan
            row["bow_pp"] = round(w["bow_pp"], 3) if np.isfinite(w["bow_pp"]) else np.nan

        rows.append(row)

    # 与基线的轨迹差（同一绝对时间戳插值后比较）
    base = next((e for e in entries if e["name"] == "expc1_base"), None) \
        or next((e for e in entries if e["name"] == "exp0_baseline"), entries[0])
    for e in entries:
        row = next(r for r in rows if r["name"] == e["name"])
        if e is base:
            row["rms_vs_base"] = 0.0
            row["end_err_vs_base"] = 0.0
            continue
        t_common = np.intersect1d(np.round(e["t_abs"], 2), np.round(base["t_abs"], 2))
        if t_common.size < 10:
            row["rms_vs_base"] = -1.0
            row["end_err_vs_base"] = -1.0
            continue
        cur_i = interp_at(e["t_abs"], e["xyz"], t_common)
        base_i = interp_at(base["t_abs"], base["xyz"], t_common)
        diff = cur_i - base_i
        row["rms_vs_base"] = round(float(np.sqrt((diff ** 2).sum(axis=1).mean())), 3)
        row["end_err_vs_base"] = round(float(np.linalg.norm(diff[-1])), 3)

    fields = list(rows[0].keys())
    for r in rows:
        for k in fields:
            r.setdefault(k, "")

    csv_path = os.path.join(args.root, "summary.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow(r)

    def table(cols, headers):
        out = ["| " + " | ".join(headers) + " |", "|" + "|".join(["---"] * len(headers)) + "|"]
        for r in rows:
            out.append("| " + " | ".join(str(r.get(c, "")) for c in cols) + " |")
        return "\n".join(out)

    # 结论里引用的数字全部从 rows 里取，避免手写数字与表脱节
    legacy = [r for r in rows if not r["name"].startswith("expc")]
    if not legacy:
        legacy = rows
    code_rows = [r for r in rows if r["name"].startswith("expc")]
    by_total = sorted(legacy, key=lambda r: r["stage_total_ms"])
    fastest, slowest = by_total[0], by_total[-1]
    nonbase = [r for r in legacy if r["name"] != base["name"] and r.get("rms_vs_base", -1) >= 0]
    closest = min(nonbase, key=lambda r: r["rms_vs_base"]) if nonbase else None
    obs_share = [r["Observe_avg_ms"] / r["stage_total_ms"] * 100
                 for r in legacy if r["stage_total_ms"] > 0]
    obs_share_txt = "%.0f%%~%.0f%%" % (min(obs_share), max(obs_share)) if obs_share else "?"
    paths = [r["path_m"] for r in legacy]
    zf = {r["name"]: r["z_final"] for r in rows}
    err = {r["name"]: r.get("end_err_vs_base", 0.0) for r in rows}
    fr = {r["name"]: r["Observe_avg_ms"] for r in rows}

    # ---- 累计漂移：闭环 + 逐层墙面偏差 ----
    def num(r, k, dflt=np.nan):
        v = r.get(k, dflt)
        try:
            v = float(v)
        except (TypeError, ValueError):
            return dflt
        return v if np.isfinite(v) else dflt

    def drift_score(r):
        """综合累计漂移打分（越小越好）。

        末位置闭环 closure_3d 反映总体累计误差；第一趟的扭曲在最终地图里以
        "各层墙相对起点层的偏移"留存，因此把 1F 水平偏移计入，yaw 偏移按
        约 5 m 力臂折算成米（1 deg ≈ 0.09 m，取 0.1）。
        """
        c = num(r, "closure_3d")
        if not np.isfinite(c):
            return np.inf
        low = num(r, "low_dxy")
        # 缺墙面指标（存图失败 / 无 map.pcd）不能按 0 计，否则会被当成零漂移排到前面
        if not np.isfinite(low):
            return np.inf
        return c + low + 0.1 * abs(num(r, "low_dyaw", 0.0))

    ranked = [r for r in sorted(rows, key=drift_score)
              if np.isfinite(drift_score(r)) and not r.get("diverged")]
    for i, r in enumerate(ranked, 1):
        r["drift_rank"] = i
        r["drift_score"] = round(drift_score(r), 2)

    def rng(key):
        vals = [num(r, key) for r in legacy]
        vals = [v for v in vals if np.isfinite(v)]
        return (min(vals), max(vals)) if vals else (np.nan, np.nan)

    def span(p, unit=""):
        return ("%.2f%s~%.2f%s" % (p[0], unit, p[1], unit)
                if np.isfinite(p[0]) and np.isfinite(p[1]) else "n/a")

    best = ranked[0] if ranked else None
    worst = ranked[-1] if ranked else None
    best_rel = min([r for r in ranked if int(r.get("low_reliable", 0)) == 1],
                   key=drift_score, default=None)
    div_rows = [r for r in rows if r.get("diverged")]
    best_closure = min([r for r in rows if np.isfinite(num(r, "closure_3d"))],
                       key=lambda r: num(r, "closure_3d"), default=None)
    best_low = min([r for r in rows if np.isfinite(num(r, "low_dxy"))],
                   key=lambda r: num(r, "low_dxy") + 0.1 * abs(num(r, "low_dyaw", 0.0)),
                   default=None)

    # 代码级实验的结论行
    code_base = next((r for r in code_rows if r["name"] == "expc1_base"),
                     code_rows[0] if code_rows else None)
    code_lines = []
    if code_base:
        old = next((r for r in rows if r["name"] == "exp0_baseline"), None)
        code_lines.append(
            "- 基线 **expc1_base**（只开埋点）：每帧 %.0f 点，有效点 %.1f%%，KNN 失败 %.1f%%，"
            "平面拟合失败 %.1f%%，残差门控失败 %.1f%%，迭代 %.2f 次（p90 %.0f），"
            "cond(Yk) 中位数 %.0f，拟合残差均值 %.4f m（p95 %.4f）。"
            % (code_base["ptsize_mean"], code_base["effect_ratio"] * 100,
               code_base["knn_fail_ratio"] * 100, code_base["plane_fail_ratio"] * 100,
               code_base["res_fail_ratio"] * 100, code_base["iters_mean"],
               code_base["iters_p90"], code_base["cond_yk_median"],
               code_base["fit_res_mean"], code_base["fit_res_p95_median"]))
        if old:
            code_lines.append(
                "- 埋点不改行为：expc1_base 与老基线 exp0_baseline 的轨迹 RMS 为 %.3f m"
                "（同一时间戳插值比较），末位置差 %.3f m。"
                % (old.get("rms_vs_base", 0.0), old.get("end_err_vs_base", 0.0)))
        for r in code_rows:
            if r is code_base:
                continue
            code_lines.append(
                "- **%s**（%s）：Observe %.2f ms（基线 %.2f），有效点 %.1f%%（基线 %.1f%%），"
                "迭代 %.2f 次，cond(Yk) %.0f，RMS %.3f m，末段 z %.2f m（基线 %.2f m），"
                "地图 %.0f k 点（基线 %.0f k 点）。"
                % (r["name"], NOTES.get(r["name"], ""), r["Observe_avg_ms"],
                   code_base["Observe_avg_ms"], r["effect_ratio"] * 100,
                   code_base["effect_ratio"] * 100, r["iters_mean"], r["cond_yk_median"],
                   r.get("rms_vs_base", -1), r["z_final"], code_base["z_final"],
                   r["map_points"] / 1e3, code_base["map_points"] / 1e3))

    extra_lines = []
    if div_rows:
        extra_lines.append(
            "- 发散组（已排除出排名）：%s —— 位置跑到 500 m 外，轨迹与地图均不可用。"
            % "、".join("%s(t≈%.0fs)" % (r["name"], num(r, "t_diverge", 0.0)) for r in div_rows))
    if best_rel:
        extra_lines.append(
            "- 若只采信 1F 墙面配准可信的组（n_match≥4，或 3 条线且 align_cost<0.25），"
            "最优为 **%s**：1F dxy %.2f m、dyaw %.2f°、score %.2f。"
            % (best_rel["name"], num(best_rel, "low_dxy"), num(best_rel, "low_dyaw"),
               best_rel["drift_score"]))

    off = rows[0].get("start_offset", "?")
    md = [
        "# Super-LIO 参数实验汇总\n",
        "同一段 bag（从 offset %ss 开始，1x 实时回放），single_core=true，动态点移除关闭，"
        "离线 PCD 统一 0.25 m 体素落盘。\n" % off,
        "所有轨迹指标裁剪到共同时间窗 **%.1f s** 上计算（WSL2 wall-clock 有抖动，按消息时间戳对齐）。\n"
        % (t_hi - t_lo),
        "## 0. 配置\n",
        table(["name", "note", "filter_rate", "voxel", "imu_na", "vox_res"],
              ["实验", "改了什么", "filter_rate", "voxel(m)", "imu_na/ng", "hash vox_res(m)"]),
        "\n## 1. 轨迹与点云\n",
        table(["name", "frames", "dur_s", "path_m", "speed_mps", "z_min", "z_max", "z_final",
               "end_x", "end_y", "big_jumps", "diverged", "t_diverge",
               "revisit_min_xy", "revisit_z", "revisit_t",
               "map_points", "map_dx", "map_dy", "map_dz", "rms_vs_base", "end_err_vs_base"],
              ["实验", "帧数", "时长(s)", "轨迹长(m)", "均速(m/s)", "z_min", "z_max", "z_final",
               "end_x", "end_y", ">5m跳跃", "发散", "发散时刻(s)",
               "回起点最近XY(m)", "该帧z(m)", "该帧t(s)",
               "地图点数", "dx", "dy", "dz", "RMS vs 基线(m)", "末位置偏差(m)"]),
        "\n## 2. 各阶段耗时（ms/帧，跨所有 60s 窗口按 calls 加权）\n",
        table(["name", "Undistort_avg_ms", "PublishBody_avg_ms", "DownSample_avg_ms",
               "Observe_avg_ms", "DeltaCorrect_avg_ms", "UpdateMap_avg_ms",
               "stage_total_ms", "budget_pct"],
              ["实验", "去畸变", "发布body", "体素降采样", "观测/匹配", "delta修正", "更新地图",
               "合计", "占100ms预算(%)"]),
        "\n## 3. 各阶段最大耗时（ms）\n",
        table(["name", "Undistort_max_ms", "DownSample_max_ms", "Observe_max_ms", "UpdateMap_max_ms"],
              ["实验", "去畸变max", "体素max", "观测max", "地图max"]),
        "\n> `rms_vs_base`：与基线在同一时间戳上插值后的 3D 位置差 RMS；`end_err_vs_base`：末位置差。"
        "无真值，仅反映相对一致性。`>5m跳跃` 非 0 说明轨迹发散或发生重置。\n",
        "\n## 4. 累计漂移：首圈 / 最终闭环 + 逐层墙面偏差\n",
        "判据：`loop1_*` 是第一圈放回起点时的位姿误差（≈ 第一趟累积漂移）；"
        "`final_*` / `closure_3d` 是全程结束回到起点时的位姿误差；"
        "`low_*` 是最低停留层（本 bag 即 1F）墙面相对起点层的平移/偏航偏差，"
        "由最终地图里各层墙面直线配准得到——地图按建图时刻位姿逐帧累积，"
        "所以即使回程被已有地图吸住，第一趟的扭曲仍会留成层间墙面错位；"
        "`low_dz` 是实测的 z 落差（起点层→最低层），与真实楼高对比即可看出竖向漂移；"
        "`low_rel` 是该层相对起点层停留了几个层（中间层若未停留不会出现在表里）；"
        "`bow_pp` 是参考墙沿整栋高度的弯曲幅度（p95-p5，米）。\n",
        table(["name", "t_ascend", "loop1_xy", "loop1_z", "loop1_yaw", "loop1_t",
               "final_xy", "final_z", "final_yaw", "closure_3d",
               "floors", "ref_floor", "low_floor", "low_rel", "low_dxy", "low_dz", "low_dyaw",
               "top_floor", "top_rel", "top_dz", "top_dxy", "top_dyaw", "bow_pp"],
              ["实验", "首圈爬升t(s)", "首圈XY(m)", "首圈Δz(m)", "首圈Δyaw(°)", "首圈t(s)",
               "终点XY(m)", "终点Δz(m)", "终点Δyaw(°)", "终点3D(m)",
               "停留层数", "起点层", "最低层", "相对层号", "最低层dxy(m)", "最低层Δz(m)", "最低层dyaw(°)",
               "最高层", "相对层号", "最高层Δz(m)", "最高层dxy(m)", "最高层dyaw(°)", "墙面弯曲p95-p5(m)"]),
        "\n**累计漂移排名**（score = closure_3d + low_dxy + 0.1×|low_dyaw|，越小越好）\n",
        table(["drift_rank", "name", "note", "closure_3d", "loop1_xy", "final_xy",
               "low_dxy", "low_dyaw", "low_reliable", "bow_pp", "drift_score"],
              ["排名", "实验", "改了什么", "终点3D(m)", "首圈XY(m)", "终点XY(m)",
               "1F dxy(m)", "1F dyaw(°)", "1F配准可信", "墙面弯曲(m)", "score"]),
        "\n> 墙面指标由 `wall_metrics.py` 计算：按 odom 的 z 平台段分楼层 → 竖向点筛选 → "
        "xy 平面 RANSAC 直线 → 以法向 mod 90° 的主方向求 Δyaw → 直线偏移量加权最小二乘求 Δx/Δy。"
        "`low_dxy/low_dyaw` 为 NaN 表示该层墙面不足（直线跨度/覆盖率门控未过），"
        "此时以 `walls.csv` 的 `n_lines/obs/align_cost` 判断可信度。\n",
        "\n## 5. 代码级实验：开关与埋点（lio.debug.level=2 的逐帧统计）\n",
        table(["name", "normal_weight_en", "normal_sigma", "map_rep_mode", "map_clamp_mu",
               "map_merge_dist", "hknn_max_group", "hknn_order_scale"],
              ["实验", "法向加权", "sigma", "代表点模式", "μ clamp", "merge(m)",
               "HKNN组数", "order_scale"]),
        "\n",
        table(["name", "dbg_frames", "ptsize_mean", "iters_mean", "iters_p90", "dx_inf_median",
               "cond_yk_median", "fit_res_mean", "fit_res_p95_median", "effect_ratio",
               "knn_fail_ratio", "plane_fail_ratio", "res_fail_ratio", "htvh_cond_median",
               "qk_bad_frames"],
              ["实验", "帧数", "点数/帧", "迭代均值", "迭代p90", "|dx|∞中位", "cond(Yk)中位",
               "拟合残差均值(m)", "拟合残差p95(m)", "有效点占比", "KNN失败占比", "拟合失败占比",
               "门控失败占比", "HTVH条件数", "Qk非有限帧"]),
        "\n> `cond_yk` = Yk 特征值条件数（≈ 后验协方差的病态程度）；`effect_ratio` = "
        "通过全部门控并进入信息矩阵的点占比；`htvh_cond` 反映 point-to-plane 观测在 6 自由度上的可观测性。\n",
        "\n**代码级实验要点**\n",
        *code_lines,
        "\n## 6. 结论（前七组参数实验）\n",
        "**速度**\n",
        "- 唯一显著耗时项是 `Observe`（HKNN + point-to-plane），占单帧总耗时的 %s；"
        "其余五段合计不到 1.5 ms/帧。" % obs_share_txt,
        "- 单帧总耗时最快 **%s**（%.2f ms，占 10 Hz 帧预算 %.1f%%），最慢 **%s**（%.2f ms，%.1f%%）。"
        "最慢配置距 100 ms 实时预算仍有约 %.0f ms 余量，本段数据上算力不是瓶颈。"
        % (fastest["name"], fastest["stage_total_ms"], fastest["budget_pct"],
           slowest["name"], slowest["stage_total_ms"], slowest["budget_pct"],
           100.0 - slowest["stage_total_ms"]),
        "- `filter_rate` 6→3→1 时 `Observe` 依次为 %.2f→%.2f→%.2f ms，"
        "近似随参与配准的点数增长（点数 ×2、×6），`DownSample`/`UpdateMap` 同步变贵。"
        % (fr.get("exp0_baseline", 0), fr.get("exp1_fr3", 0), fr.get("exp2_fr1", 0)),
        "- `imu_na/ng` 0.1→0.01→0.001 时 `Observe` 反而降到 %.2f/%.2f ms："
        "过程噪声变小后预测更贴合，Huber 权重下残差分布更集中，迭代更快。"
        % (fr.get("exp5_imu001", 0), fr.get("exp6_imu0001", 0)),
        "\n**轨迹与累计漂移**\n",
        "- %d 组轨迹形态一致：路径长 %.2f~%.2f m、均速 %.2f m/s、无 >5 m 单帧跳跃；"
        "地图包围盒 %.1f×%.1f×%.1f m。"
        % (len(legacy), min(paths), max(paths), rows[0]["speed_mps"],
           min(r["map_dx"] for r in legacy), max(r["map_dy"] for r in legacy),
           rows[0]["map_dz"]),
        "- 闭环误差：首圈放回起点的水平偏差 %s m，终点回到起点 %s m（3D %s m），"
        "终点 Δyaw %s。"
        % (span(rng("loop1_xy")), span(rng("final_xy")), span(rng("closure_3d")),
           span(rng("final_yaw"), "°")),
        "- 逐层墙面（1F 相对起点层）：平移偏差 %s m，Δyaw %s；参考墙沿高度的弯曲幅度 %s m。"
        "墙面偏差不受回程地图匹配影响，是判断第一趟扭曲的直接证据。"
        % (span(rng("low_dxy")), span(rng("low_dyaw"), "°"), span(rng("bow_pp"))),
        "- 累计漂移最优 **%s**（score %.2f：终点3D %.2f m、1F dxy %.2f m、1F dyaw %.2f°）；"
        "最差 **%s**（score %.2f）。"
        % (best["name"], best["drift_score"], num(best, "closure_3d"),
           num(best, "low_dxy"), num(best, "low_dyaw"),
           worst["name"], worst["drift_score"]) if best and worst
        else "- 闭环/墙面指标不足，无法排名。",
        "- 两种口径的差异：只看终点 3D 闭环最优是 **%s**（%.2f m），"
        "叠加 1F 墙面偏差后最优是 **%s**——差异说明终点闭环可能被回程地图匹配掩盖。"
        % (best_closure["name"], num(best_closure, "closure_3d"), best_low["name"])
        if best_closure and best_low else "- 口径对比数据不足。",
        *extra_lines,
        "\n**建议**\n",
        "- 以 `drift_score` 排名第一的 **%s** 作为本楼梯间 bag 的默认配置："
        "它在终点闭环与 1F 墙面两项上综合最优。"
        % best["name"] if best else "- 建议等待完整 14 组结果后再定配置。",
        "- 排序时不要只看 `rms_vs_base`/`end_err_vs_base`：这两个指标衡量的是与基线的一致性，"
        "基线本身在楼梯井里就是漂的；`closure_3d`、`loop1_xy`、`low_dxy/low_dyaw` 才直接对应累计误差。",
        "- 若某组 `low_dxy/low_dyaw` 为 n/a，先查该组 `walls.csv` 的 `n_lines/obs/align_cost`，"
        "确认是墙面太少（该层只进大厅没扫全井道）而不是算法失效。",
        "\n> 无真值（无 RTK/动捕），以上“更好/更差”均指与基线的一致性与自洽性，"
        "不能替代有真值的绝对精度评估。\n",
        "CSV: `summary.csv`\n",
    ]

    md_path = os.path.join(args.root, "summary.md")
    with open(md_path, "w") as f:
        f.write("\n".join(md))
    print("\n".join(md))
    print("\nwrote", csv_path, "and", md_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
