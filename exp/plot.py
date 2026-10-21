#!/usr/bin/env python3
"""把 exp/results*/ 的轨迹、地图、闭环误差和逐层墙面偏差画成对比图，输出到 <root>/figs/。

图 1: 全部实验的 XY 轨迹叠加
图 2: 全部实验的 z(t)
图 3: 每个实验的俯视地图 + 轨迹（裁到轨迹附近，避免远处离群点撑爆视野）
图 4: 首圈 vs 最终回到起点的闭环误差柱状图
图 5: 逐层墙面相对起点层的 dx/dy/dyaw + 墙面弯曲 bow_pp
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze import loop_metrics, quat_to_yaw
from bag_io import load_odom, read_pcd_xyz, traj_arrays

EXP_ROOT = os.path.dirname(os.path.abspath(__file__))


def default_root():
    for n in ("results_livox_record_A_building", "results"):
        p = os.path.join(EXP_ROOT, n)
        if os.path.isdir(p):
            return p
    return os.path.join(EXP_ROOT, "results")


def read_walls(path):
    if not os.path.exists(path):
        return []
    out = []
    with open(path) as f:
        for r in csv.DictReader(f):
            out.append(r)
    return out


def fnum(v):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return np.nan
    return v if np.isfinite(v) else np.nan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=default_root())
    ap.add_argument("--min-dur", type=float, default=120.0,
                    help="短于该时长的跑批（冒烟/中断）不参与对比")
    args = ap.parse_args()
    root = args.root
    fig_dir = os.path.join(root, "figs")
    os.makedirs(fig_dir, exist_ok=True)

    names = sorted(
        d for d in os.listdir(root)
        if os.path.isdir(os.path.join(root, d)) and os.path.exists(os.path.join(root, d, "odom.csv"))
    )
    runs = []
    for n in names:
        odom = load_odom(os.path.join(root, n, "odom.csv"))
        if odom is None:
            continue
        t, xyz = traj_arrays(odom)
        t = t - t[0]
        if t[-1] < args.min_dur:
            print("skip short run:", n, f"{t[-1]:.0f} s")
            continue
        if (np.abs(xyz) > 500.0).any():
            print("skip diverged run:", n)
            continue
        yaw = quat_to_yaw(odom["qx"], odom["qy"], odom["qz"], odom["qw"])
        runs.append((n, t, xyz, yaw))
    if not runs:
        print("no data")
        return 1

    cmap = plt.get_cmap("tab20")

    def style(n):
        # 参数实验用实线，代码级实验（expc*）用虚线，避免颜色重复时混淆
        return "--" if n.startswith("expc") else "-"

    # 图 1：XY 轨迹
    fig, ax = plt.subplots(figsize=(9, 8))
    for i, (n, t, xyz, yaw) in enumerate(runs):
        ax.plot(xyz[:, 0], xyz[:, 1], style(n), lw=1.0, color=cmap(i % 20), label=n)
        ax.plot(xyz[0, 0], xyz[0, 1], "o", ms=4, color=cmap(i % 20))
        ax.plot(xyz[-1, 0], xyz[-1, 1], "s", ms=4, color=cmap(i % 20))
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("XY trajectory (solid=param exp, dashed=code exp; circle=start, square=end)")
    ax.grid(True, ls=":", alpha=0.5)
    ax.legend(fontsize=6, ncol=3)
    fig.tight_layout()
    fig.savefig(os.path.join(fig_dir, "traj_xy.png"), dpi=150)
    plt.close(fig)

    # 图 2：z(t)
    fig, ax = plt.subplots(figsize=(11, 5))
    for i, (n, t, xyz, yaw) in enumerate(runs):
        ax.plot(t, xyz[:, 2], style(n), lw=1.0, color=cmap(i % 20), label=n)
    ax.set_xlabel("t [s] from first odom")
    ax.set_ylabel("z [m]")
    ax.set_title("z drift (solid=param exp, dashed=code exp)")
    ax.grid(True, ls=":", alpha=0.5)
    ax.legend(fontsize=6, ncol=3)
    fig.tight_layout()
    fig.savefig(os.path.join(fig_dir, "traj_z.png"), dpi=150)
    plt.close(fig)

    # 图 3：俯视地图
    ncol = 3
    nrow = int(np.ceil(len(runs) / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(6 * ncol, 5 * nrow), squeeze=False)
    for k, (n, t, xyz, yaw) in enumerate(runs):
        ax = axes[k // ncol][k % ncol]
        pts = read_pcd_xyz(os.path.join(root, n, "map.pcd"), max_points=3_000_000)
        if pts is not None and len(pts):
            lo = xyz[:, :2].min(axis=0) - 15
            hi = xyz[:, :2].max(axis=0) + 15
            m = ((pts[:, 0] > lo[0]) & (pts[:, 0] < hi[0]) &
                 (pts[:, 1] > lo[1]) & (pts[:, 1] < hi[1]))
            sub = pts[m]
            if len(sub) > 150000:
                sub = sub[np.random.default_rng(0).choice(len(sub), 150000, replace=False)]
            ax.scatter(sub[:, 0], sub[:, 1], s=0.08, c=sub[:, 2], cmap="viridis", linewidths=0)
        ax.plot(xyz[:, 0], xyz[:, 1], "r-", lw=0.8)
        ax.set_aspect("equal")
        ax.set_title(f"{n}  ({len(pts) if pts is not None else 0} pts)", fontsize=9)
        ax.tick_params(labelsize=7)
    for k in range(len(runs), nrow * ncol):
        axes[k // ncol][k % ncol].axis("off")
    fig.suptitle("top view: map (color=z) + trajectory (red)")
    fig.tight_layout()
    fig.savefig(os.path.join(fig_dir, "map_topview.png"), dpi=100)
    plt.close(fig)

    # 图 4：首圈 / 最终闭环误差
    labels, l1, fin, clo, yend = [], [], [], [], []
    for n, t, xyz, yaw in runs:
        lm = loop_metrics(t, xyz, yaw)
        if not lm:
            continue
        labels.append(n)
        l1.append(lm["loop1"]["xy"] if lm["loop1"] else np.nan)
        fin.append(lm["final"]["xy"])
        clo.append(float(np.hypot(lm["final"]["xy"], lm["final"]["z"])))
        yend.append(lm["final"]["yaw"])
    if labels:
        x = np.arange(len(labels))
        fig, axes = plt.subplots(2, 1, figsize=(max(9, 0.55 * len(labels)), 7),
                                 sharex=True, gridspec_kw={"height_ratios": [3, 2]})
        w = 0.27
        axes[0].bar(x - w, l1, w, label="loop1 (first lap) XY")
        axes[0].bar(x, fin, w, label="final XY")
        axes[0].bar(x + w, clo, w, label="final 3D")
        axes[0].set_ylabel("error [m]")
        axes[0].set_title("closure error back to start (lower = less accumulated drift)")
        axes[0].grid(True, axis="y", ls=":", alpha=0.5)
        axes[0].legend(fontsize=8)
        axes[1].bar(x, yend, 0.4, color="tab:orange")
        axes[1].axhline(0, color="k", lw=0.6)
        axes[1].set_ylabel("final dYaw [deg]")
        axes[1].grid(True, axis="y", ls=":", alpha=0.5)
        axes[1].set_xticks(x)
        axes[1].set_xticklabels(labels, rotation=45, ha="right", fontsize=7)
        fig.tight_layout()
        fig.savefig(os.path.join(fig_dir, "closure.png"), dpi=150)
        plt.close(fig)

    # 图 5：逐层墙面偏差 + bow
    walls = {n: read_walls(os.path.join(root, n, "walls.csv")) for n, *_ in runs}
    walls = {k: v for k, v in walls.items() if v}
    if walls:
        fig, axes = plt.subplots(1, 3, figsize=(16, 5))
        for i, (n, rows) in enumerate(walls.items()):
            rel = np.array([fnum(r["rel"]) for r in rows])
            dx = np.array([fnum(r["dx"]) for r in rows])
            dy = np.array([fnum(r["dy"]) for r in rows])
            dyaw = np.array([fnum(r["dyaw_deg"]) for r in rows])
            dxy = np.hypot(dx, dy)
            axes[0].plot(rel, dxy, "o-", ms=3, lw=1.0, color=cmap(i % 20), label=n)
            axes[1].plot(rel, dyaw, "o-", ms=3, lw=1.0, color=cmap(i % 20), label=n)
            z = np.array([fnum(r["dz_res"]) for r in rows])
            axes[2].plot(rel, z, "o-", ms=3, lw=1.0, color=cmap(i % 20), label=n)
        axes[0].set_title("per-floor wall dXY vs start floor [m]")
        axes[1].set_title("per-floor wall dYaw vs start floor [deg]")
        axes[2].set_title("per-floor z residual (equal floor-height fit) [m]")
        for ax in axes:
            ax.set_xlabel("floor index relative to start (0 = start floor)")
            ax.grid(True, ls=":", alpha=0.5)
        axes[0].legend(fontsize=6, ncol=2)
        fig.tight_layout()
        fig.savefig(os.path.join(fig_dir, "floor_offsets.png"), dpi=150)
        plt.close(fig)

    print("figures ->", fig_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
