#!/usr/bin/env python3
"""把 exp/results/* 的轨迹和地图画成对比图，输出到 results/figs/。

图 1: 全部实验的 XY 轨迹叠加
图 2: 全部实验的 z(t)
图 3: 每个实验的俯视地图 + 轨迹（裁到轨迹附近，避免远处离群点撑爆视野）
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze import load_odom, pcd_points, traj_arrays

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
FIG = os.path.join(ROOT, "figs")


def main():
    os.makedirs(FIG, exist_ok=True)
    names = sorted(
        d for d in os.listdir(ROOT)
        if os.path.isdir(os.path.join(ROOT, d)) and os.path.exists(os.path.join(ROOT, d, "odom.csv"))
    )
    runs = []
    for n in names:
        odom = load_odom(os.path.join(ROOT, n, "odom.csv"))
        if odom is None:
            continue
        t, xyz = traj_arrays(odom)
        runs.append((n, t - t[0], xyz))
    if not runs:
        print("no data")
        return 1

    cmap = plt.get_cmap("tab10")

    # 图 1：XY 轨迹
    fig, ax = plt.subplots(figsize=(9, 8))
    for i, (n, t, xyz) in enumerate(runs):
        ax.plot(xyz[:, 0], xyz[:, 1], lw=1.0, color=cmap(i % 10), label=n)
        ax.plot(xyz[0, 0], xyz[0, 1], "o", ms=4, color=cmap(i % 10))
        ax.plot(xyz[-1, 0], xyz[-1, 1], "s", ms=4, color=cmap(i % 10))
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("XY trajectory (circle=start, square=end)")
    ax.grid(True, ls=":", alpha=0.5)
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "traj_xy.png"), dpi=150)
    plt.close(fig)

    # 图 2：z(t)
    fig, ax = plt.subplots(figsize=(11, 5))
    for i, (n, t, xyz) in enumerate(runs):
        ax.plot(t, xyz[:, 2], lw=1.0, color=cmap(i % 10), label=n)
    ax.set_xlabel("t [s] from first odom")
    ax.set_ylabel("z [m]")
    ax.set_title("z drift")
    ax.grid(True, ls=":", alpha=0.5)
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "traj_z.png"), dpi=150)
    plt.close(fig)

    # 图 3：俯视地图
    ncol = 2
    nrow = int(np.ceil(len(runs) / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(7 * ncol, 6 * nrow), squeeze=False)
    for k, (n, t, xyz) in enumerate(runs):
        ax = axes[k // ncol][k % ncol]
        pts = pcd_points(os.path.join(ROOT, n, "map.pcd"))
        if pts is not None and len(pts):
            lo = xyz[:, :2].min(axis=0) - 15
            hi = xyz[:, :2].max(axis=0) + 15
            m = ((pts[:, 0] > lo[0]) & (pts[:, 0] < hi[0]) &
                 (pts[:, 1] > lo[1]) & (pts[:, 1] < hi[1]))
            sub = pts[m]
            if len(sub) > 300000:
                sub = sub[np.random.default_rng(0).choice(len(sub), 300000, replace=False)]
            ax.scatter(sub[:, 0], sub[:, 1], s=0.08, c=sub[:, 2], cmap="viridis", linewidths=0)
        ax.plot(xyz[:, 0], xyz[:, 1], "r-", lw=0.8)
        ax.set_aspect("equal")
        ax.set_title(f"{n}  ({len(pts) if pts is not None else 0} pts)", fontsize=9)
        ax.tick_params(labelsize=7)
    for k in range(len(runs), nrow * ncol):
        axes[k // ncol][k % ncol].axis("off")
    fig.suptitle("top view: map (color=z) + trajectory (red)")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "map_topview.png"), dpi=130)
    plt.close(fig)

    print("figures ->", FIG)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
