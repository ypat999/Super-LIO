#!/usr/bin/env python3
"""M.27/M.29：UpdateMap 尖峰定位——从 node.log 的周期计时窗口提取分段耗时分布。

背景（M.24-8、M.28-7）：端到端 ~10.3 ms，[UpdateMap] 峰值在 1f 上达 92 ms，
同时 [MapTransform] ≤0.05 ms ⇒ 尖峰全部来自哈希插入 [MapInsert]。
M.29 对 grids_ 做 reserve 消除扩容期整表重哈希，本脚本负责前后对比与窗口归因。

注意日志里每分钟有两个 "Printing run time" 头：一个是 time_record_（LIO 各阶段），
另一个是 latency_timer_（[Lidar->CloudWorld] 等端到端延迟）。两者独立计时、独立清零，
所以按"阶段名集合"把窗口分成两组分别编号，避免窗口号错位。

日志行格式（timer.h PrintAll，glog 前缀，双层括号）：
  > [ [UpdateMap] ] avg: 3.21 ms , max: 17.52 ms , calls: 601
   > [ MapStats ] size=... buckets=... new_voxel=... evict=...   （M.29 新增）

用法（exp/ 下）：
  python3 timing_spike_audit.py results/m30_dbg results/f1_dbg results/f1_reserve_anchor
输出：
  - LIO 各阶段 window-max 的 p50/p90/max 与最差窗口
  - [UpdateMap] 最差 5 窗口的 [MapTransform]/[MapInsert]/[MapStats] 同窗对照
"""
import argparse
import glob
import math
import os
import re

# 实际日志是双层括号："> [ [UpdateMap] ] avg: ..."，单层也兼容
LINE_RE = re.compile(
    r">\s*(?:\[\s*)+(?P<name>\w+)\s*(?:\]\s*)+avg:\s*(?P<avg>[\d.]+)\s*ms\s*,\s*"
    r"max:\s*(?P<max>[\d.]+)\s*ms\s*,\s*calls:\s*(?P<calls>\d+)")
MAPSTATS_RE = re.compile(
    r">\s*(?:\[\s*)+MapStats\s*(?:\]\s*)+size=(?P<size>\d+)\s+buckets=(?P<buckets>\d+)\s+"
    r"new_voxel=(?P<new>\d+)\s+evict=(?P<evict>\d+)")

LIO_STAGES = ["Undistort", "DownSample", "Observe", "UpdateMap", "MapTransform",
              "MapInsert", "DeltaCorrect", "PublishBody"]


def parse_blocks(path):
    """把日志切成计时块：[(stages{name:(avg,max,calls)}, mapstats_or_None), ...]"""
    blocks, cur, cur_stats = [], {}, None

    def flush():
        nonlocal cur, cur_stats
        if cur or cur_stats:
            blocks.append((cur, cur_stats))
        cur, cur_stats = {}, None

    with open(path, errors="ignore") as f:
        for line in f:
            if "Printing run time" in line:
                flush()
                continue
            m = LINE_RE.search(line)
            if m:
                cur[m.group("name")] = (float(m.group("avg")), float(m.group("max")),
                                        int(m.group("calls")))
                continue
            s = MAPSTATS_RE.search(line)
            if s:
                cur_stats = (int(s.group("size")), int(s.group("buckets")),
                             int(s.group("new")), int(s.group("evict")))
    flush()
    return blocks


def split_groups(blocks):
    """按计时器分组：含 Observe/UpdateMap 的是 LIO 计时器，其余是延迟计时器。

    返回 {group_key: [(win_idx, stages, mapstats), ...]}
    """
    groups = {}
    for stages, stats in blocks:
        if not stages:
            continue
        key = "LIO" if ("Observe" in stages or "UpdateMap" in stages) else "LATENCY"
        groups.setdefault(key, []).append((len(groups.get(key, [])), stages, stats))
    return groups


def audit(path):
    blocks = parse_blocks(path)
    if not blocks:
        print(f"[{path}] 无计时行（需 lio.eva.timer: true）")
        return
    groups = split_groups(blocks)
    lio = groups.get("LIO", [])
    if not lio:
        print(f"[{path}] 只有延迟计时块，无 LIO 阶段计时")
        return

    print(f"===== {path} =====")
    print(f"{'stage':<14} {'窗数':>4} {'wmax_p50':>8} {'wmax_p90':>8} {'wmax_max':>8} "
          f"{'最差窗':>6} {'该窗avg':>7} {'该窗calls':>8}")

    per_stage = {}
    for win, stages, _ in lio:
        for name, (avg, mx, calls) in stages.items():
            per_stage.setdefault(name, []).append((win, avg, mx, calls))
    names = [n for n in LIO_STAGES if n in per_stage] + \
            sorted(n for n in per_stage if n not in LIO_STAGES)
    for n in names:
        recs = per_stage[n]
        wmaxes = [r[2] for r in recs]
        worst = max(recs, key=lambda r: r[2])
        print(f"{n:<14} {len(recs):>4} {percentile(wmaxes, 50):>8.2f} "
              f"{percentile(wmaxes, 90):>8.2f} {max(wmaxes):>8.2f} "
              f"{worst[0]:>6} {worst[1]:>7.2f} {worst[3]:>8}")

    if "UpdateMap" not in per_stage:
        return
    print("\n[UpdateMap] 最差 5 窗口的分段归因：")
    worst5 = sorted(per_stage["UpdateMap"], key=lambda r: -r[2])[:5]
    for win, _, mx, calls in worst5:
        row = next((b for b in lio if b[0] == win), None)
        stages = row[1] if row else {}
        stats = row[2] if row else None
        def val(k):
            return f"{stages[k][1]:>6.2f}" if k in stages else "   n/a"
        ms = (f"  size={stats[0]:>7} buckets={stats[1]:>8} "
              f"new={stats[2]:>6} evict={stats[3]:>5}") if stats else ""
        print(f"  win#{win:<3} UpdateMap_max={mx:>6.2f}  MapTransform_max={val('MapTransform')}"
              f"  MapInsert_max={val('MapInsert')}  calls={calls}{ms}")
    print("  ⇒ MapInsert 占主导 = 哈希插入（扩容/淘汰）；MapTransform 占主导 = 点数突发。")
    print("     M.29 后：若 MapInsert 峰值显著下降且 buckets 恒定 ⇒ reserve 生效。")

    lat = groups.get("LATENCY", [])
    if lat:
        lat_names = [n for n in lat[-1][1] if n not in LIO_STAGES]
        print("\n[延迟计时器] 最新窗口（端到端）：")
        for n in lat_names:
            avg, mx, calls = lat[-1][1][n]
            print(f"  {n:<22} avg={avg:>7.2f} ms  max={mx:>7.2f} ms  calls={calls}")


def percentile(xs, q):
    if not xs:
        return float("nan")
    xs = sorted(xs)
    k = (len(xs) - 1) * q / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return xs[lo] if lo == hi else xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+",
                    help="run 目录或 node.log 路径（支持通配符）")
    a = ap.parse_args()
    files = []
    for p in a.paths:
        if os.path.isdir(p):
            files.append(os.path.join(p, "node.log"))
        else:
            files.extend(sorted(glob.glob(p)))
    if not files:
        print("没有匹配的 node.log")
        return
    for f in files:
        if os.path.exists(f):
            audit(f)
        else:
            print(f"[skip] {f} 不存在")


if __name__ == "__main__":
    main()
