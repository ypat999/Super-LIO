#!/usr/bin/env python3
"""M.27 细通道/退化诊断：debug.csv 通路验证 + 拒绝点×退化相关性 + [FineCh] 日志。

三个用途（对应工作清单 P2.2 / P3.1 / P3.2）：
  1) --debug-csv      验证 debug.csv 通路（M.25-D 下一步 4）：列齐全、逐帧无缺行、
                      deg_cond 与 [Degeneracy] 日志同源。
  2) --rej-corr       P3.1：统计 n_plane_fail（平面拟合拒绝点）规模、占比，
                      及其与 deg_cond 的逐帧相关性（细特征信息量的间接证据）。
  3) --fine-log       P3.2：解析 node.log 的 [FineCh] 行，给出 cands/used/gain/
                      applied 的时间序列摘要（applied=0 但 degen=1 的帧=门控拦下）。

用法（在 exp/ 下）：
  python3 fine_diag.py --run results/m30_dbg
  python3 fine_diag.py --run results/m30_fine --fine-log
  python3 fine_diag.py --run results/m30_dbg --rej-corr
默认三项全开。debug.csv 列定义见 super_lio.cpp Observe() 埋点段。
"""
import argparse
import os
import re
import sys

import numpy as np


def load_debug_csv(run):
    p = os.path.join(run, "debug.csv")
    if not os.path.exists(p):
        return None
    try:
        d = np.genfromtxt(p, delimiter=",", names=True)
    except Exception as e:
        print(f"[err] 解析 {p} 失败: {e}")
        return None
    return d if getattr(d, "size", 0) > 0 else None


def check_debug_csv(run, d):
    print("== 1) debug.csv 通路验证 ==")
    if d is None:
        print("  debug.csv 缺失或为空 —— 通路未恢复，先查 lio.debug.level 与 "
              "lio.debug.level declare（M.25-B B.0-2 修过一版被合并吞掉的）")
        return
    need = ["t", "ptsize", "n_plane_fail", "n_effect", "deg_cond", "deg_axes",
            "fit_res_mean", "cond_yk"]
    missing = [c for c in need if c not in d.dtype.names]
    if missing:
        print(f"  [warn] 缺列: {missing}")
    n = len(d)
    t = d["t"]
    dt = np.diff(t)
    bad = int((dt <= 0).sum())
    print(f"  rows={n}  cols={len(d.dtype.names)}  dt: min={dt.min():.4f} "
          f"median={np.median(dt):.4f} max={dt.max():.4f}s  非单调跳变={bad}")
    cond = d["deg_cond"]
    nz = cond[cond > 0]
    print(f"  deg_cond: >0 帧占比 {len(nz)/n*100:.1f}%  "
          f"p50={np.percentile(nz,50) if len(nz) else 0:.1f}  "
          f"p90={np.percentile(nz,90) if len(nz) else 0:.1f}  "
          f"max={nz.max() if len(nz) else 0:.1f}  (>100 帧数={int((nz>100).sum())})")


def rej_corr(d):
    print("== 2) P3.1 拒绝点 × 退化相关性 ==")
    if d is None or "n_plane_fail" not in d.dtype.names:
        print("  无 n_plane_fail 列，跳过")
        return
    n_effect = d["n_effect"].astype(float)
    n_fail = d["n_plane_fail"].astype(float)
    ratio = n_fail / np.maximum(n_effect + n_fail, 1.0)
    cond = d["deg_cond"]
    m = cond > 0
    print(f"  n_plane_fail: mean={n_fail.mean():.0f} p95={np.percentile(n_fail,95):.0f} "
          f"max={n_fail.max():.0f}")
    print(f"  拒绝占比 n_plane_fail/(effect+fail): mean={ratio.mean()*100:.1f}% "
          f"p95={np.percentile(ratio,95)*100:.1f}% max={ratio.max()*100:.1f}%")
    if m.sum() > 10:
        c = np.corrcoef(ratio[m], cond[m])[0, 1] if np.std(cond[m]) > 0 else float("nan")
        print(f"  拒绝占比 vs deg_cond 的 Pearson r = {c:+.3f}（>0 支持"
              "『退化段正是细特征丰富段』，细通道有用武之地；≈0 需看空间分布）")
    else:
        print("  deg_cond>0 样本太少，相关性不具统计意义")
    # 退化帧内外的拒绝率对比（比相关系数更直观）
    degen_frames = cond > 100
    if degen_frames.sum() > 5 and (~degen_frames).sum() > 5:
        print(f"  退化帧(cond>100)拒绝率均值 = {ratio[degen_frames].mean()*100:.1f}%  "
              f"非退化帧 = {ratio[~degen_frames].mean()*100:.1f}%")


FINE_RE = re.compile(
    r"\[FineCh\] cands=(\d+) used=(\d+) gain=([\d.eE+-]+) applied=(\d+) "
    r"degen=(\d+) cond=([\d.eE+-]+)")


def fine_log(run):
    print("== 3) [FineCh] 细通道日志 ==")
    p = os.path.join(run, "node.log")
    if not os.path.exists(p):
        print("  node.log 缺失")
        return
    rows = []
    with open(p, errors="ignore") as f:
        for line in f:
            m = FINE_RE.search(line)
            if m:
                rows.append((int(m.group(1)), int(m.group(2)), float(m.group(3)),
                            int(m.group(4)), int(m.group(5)), float(m.group(6))))
    if not rows:
        print("  无 [FineCh] 行 —— fine_channel_en 未开启或未触发")
        return
    a = np.array(rows)
    print(f"  样本 {len(a)} 条（2s 限频）")
    degen = a[:, 4] == 1
    applied = a[:, 3] == 1
    print(f"  degen=1 样本 {int(degen.sum())}/{len(a)}；applied=1 "
          f"{int(applied.sum())} 条（其中 degen=1 内 {int((applied & degen).sum())} 条）")
    if degen.sum():
        g = a[degen, 2]
        print(f"  退化段 gain: p50={np.percentile(g,50):.2f} p90={np.percentile(g,90):.2f} "
              f"max={g.max():.2f}（阈值 1.5，≥1.5 才并入）")
        used = a[degen, 1]
        print(f"  退化段 used: p50={np.percentile(used,50):.0f} max={used.max():.0f}")
        gated = degen & ~applied
        if gated.sum():
            print(f"  门控拦截（degen=1 但 applied=0）{int(gated.sum())} 条 —— "
                  "真空/无细特征段，属预期行为")
    if (~degen).sum():
        print(f"  非退化段 cands 均值 = {a[~degen,0].mean():.0f}（收集但不启用，正常）")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, help="run 目录，如 results/m30_dbg")
    ap.add_argument("--debug-csv", action="store_true")
    ap.add_argument("--rej-corr", action="store_true")
    ap.add_argument("--fine-log", action="store_true")
    a = ap.parse_args()
    run = a.run.rstrip("/")
    d = load_debug_csv(run)
    all_on = not (a.debug_csv or a.rej_corr or a.fine_log)
    if all_on or a.debug_csv:
        check_debug_csv(run, d)
    if all_on or a.rej_corr:
        rej_corr(d)
    if all_on or a.fine_log:
        fine_log(run)


if __name__ == "__main__":
    main()
