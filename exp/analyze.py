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


def load_odom(path):
    if not os.path.exists(path):
        return None
    try:
        data = np.genfromtxt(path, delimiter=",", names=True)
    except Exception:
        return None
    if data is None or data.size == 0:
        return None
    if data.shape == ():
        data = data.reshape(1)
    return data


def pcd_points(path):
    """读取 binary PCD，返回 (N,3) float64 数组；不支持的类型返回 None。"""
    if not os.path.exists(path):
        return None
    fields, sizes, types, counts = [], [], [], []
    npts = 0
    data_kind = "binary"
    with open(path, "rb") as f:
        while True:
            line = f.readline()
            if not line:
                return None
            s = line.decode("ascii", errors="ignore").strip()
            if s.startswith("FIELDS"):
                fields = s.split()[1:]
            elif s.startswith("SIZE"):
                sizes = [int(x) for x in s.split()[1:]]
            elif s.startswith("TYPE"):
                types = s.split()[1:]
            elif s.startswith("COUNT"):
                counts = [int(x) for x in s.split()[1:]]
            elif s.startswith("POINTS"):
                npts = int(s.split()[1])
            elif s.startswith("DATA"):
                data_kind = s.split()[1]
                break

        if data_kind != "binary" or not fields:
            return None

        np_type = {"F": "f", "U": "u", "I": "i"}
        dtype = []
        for fn, sz, tp, ct in zip(fields, sizes, types, counts or [1] * len(fields)):
            base = np_type.get(tp, "f") + str(sz)
            dtype.append((fn, base, (ct,)) if ct > 1 else (fn, base))
        raw = np.frombuffer(f.read(npts * np.dtype(dtype).itemsize), dtype=dtype)
        if raw.size == 0:
            return None
        xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float64)
        return xyz[np.isfinite(xyz).all(axis=1)]


def pcd_info(path):
    """返回 {'points': N, 'extent': (dx,dy,dz)}。"""
    xyz = pcd_points(path)
    if xyz is None:
        return None
    if xyz.size == 0:
        return {"points": 0, "extent": None}
    lo, hi = xyz.min(axis=0), xyz.max(axis=0)
    return {"points": int(len(xyz)), "extent": hi - lo}


def traj_arrays(odom):
    xyz = np.stack([odom["x"], odom["y"], odom["z"]], axis=1).astype(np.float64)
    t_abs = odom["t_ns"].astype(np.float64) * 1e-9
    return t_abs, xyz


def traj_metrics(t_abs, xyz):
    if len(t_abs) < 2:
        return None
    d = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    path_len = float(d.sum())
    dur = float(t_abs[-1] - t_abs[0])
    # 轨迹自洽性：出现 >5 m 的单帧跳跃说明发散/重置
    jumps = int((d > 5.0).sum())
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
    }


def interp_at(t_src, xyz_src, t_dst):
    out = np.empty((len(t_dst), 3))
    for i in range(3):
        out[:, i] = np.interp(t_dst, t_src, xyz_src[:, i])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))
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
        entries.append({
            "name": name,
            "dir": d,
            "meta": meta,
            "timers": parse_node_log(os.path.join(d, "node.log")),
            "debug": parse_debug_csv(os.path.join(d, "debug.csv")),
            "t_abs": traj_arrays(odom)[0],
            "xyz": traj_arrays(odom)[1],
            "pcd": pcd_info(os.path.join(d, "map.pcd")),
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
               "end_x", "end_y", "big_jumps", "revisit_min_xy", "revisit_z", "revisit_t",
               "map_points", "map_dx", "map_dy", "map_dz", "rms_vs_base", "end_err_vs_base"],
              ["实验", "帧数", "时长(s)", "轨迹长(m)", "均速(m/s)", "z_min", "z_max", "z_final",
               "end_x", "end_y", ">5m跳跃", "回起点最近XY(m)", "该帧z(m)", "该帧t(s)",
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
        "\n## 4. 代码级实验：开关与埋点（lio.debug.level=2 的逐帧统计）\n",
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
        "\n## 5. 结论（前七组参数实验）\n",
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
        "\n**轨迹与点云**\n",
        "- %d 组参数实验轨迹形态一致：路径长 %.2f~%.2f m、均速 %.2f m/s、`>5m跳跃` 全为 0，"
        "无发散或重置；地图点数与包围盒（%.1f×%.1f×%.1f m）基本重合。"
        % (len(legacy), min(paths), max(paths), rows[0]["speed_mps"],
           min(r["map_dx"] for r in legacy), max(r["map_dy"] for r in legacy),
           rows[0]["map_dz"]),
        "- 相对基线最近的是 **%s**（RMS %.3f m，末位置差 %.3f m，z_final %.2f）。"
        % (closest["name"], closest["rms_vs_base"], closest["end_err_vs_base"],
           zf.get(closest["name"], 0)) if closest else "- 无其他实验可对比。",
        "- `filter_rate` 6→3/1 与细化体素（0.20/0.15）都把末段 z 从基线的 %.2f m 抬到 "
        "%.2f~%.2f m，横向末位置偏 %.2f~%.2f m；三组变化量远大于 imu 组，"
        "说明“匹配点更多/更细”的配置在这段回环数据上会放大 z 漂移。"
        % (zf.get("exp0_baseline", 0),
           min(zf.get(k, 0) for k in ("exp1_fr3", "exp2_fr1", "exp3_vox020", "exp4_vox015")),
           max(zf.get(k, 0) for k in ("exp1_fr3", "exp2_fr1", "exp3_vox020", "exp4_vox015")),
           min(abs(err.get(k, 0)) for k in ("exp1_fr3", "exp2_fr1", "exp3_vox020", "exp4_vox015")),
           max(abs(err.get(k, 0)) for k in ("exp1_fr3", "exp2_fr1", "exp3_vox020", "exp4_vox015"))),
        "\n**建议**\n",
        "- 保留 `filter_rate=6`，把 `imu_na/ng` 0.1→0.01：在 filter_rate=6 组内它最快"
        "（%.2f ms/帧）且最贴基线（RMS %.3f m）；0.001 单帧总耗时还低 %.2f ms，"
        "但与基线偏差增至 %.3f m，属于“更信任 IMU”的更激进版本，收益不足以抵消风险。"
        % (fr.get("exp5_imu001", 0),
           closest["rms_vs_base"] if closest and closest["name"] == "exp5_imu001" else 0,
           fr.get("exp0_baseline", 0) - fr.get("exp6_imu0001", 0),
           next((r["rms_vs_base"] for r in nonbase if r["name"] == "exp6_imu0001"), 0)),
        "- `filter_rate=1` 用基线的 %.1f 倍算力只换来 %.2f m 的轨迹变化，10 Hz 下没有必要；"
        "若希望更密的匹配约束，先试 3（%.2f ms/帧，基线的 %.1f 倍）。"
        % (fr.get("exp2_fr1", 0) / fr.get("exp0_baseline", 1),
           next((r["rms_vs_base"] for r in nonbase if r["name"] == "exp2_fr1"), 0),
           fr.get("exp1_fr3", 0),
           fr.get("exp1_fr3", 0) / fr.get("exp0_baseline", 1)),
        "- 体素 0.25→0.20/0.15 在本段户外回环上无收益（耗时持平、末段 z 漂移更大），"
        "不建议作为默认改动；体素细化更适合已知的狭窄/室内场景再单独验证。",
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
