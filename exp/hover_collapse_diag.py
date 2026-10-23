#!/usr/bin/env python3
"""
悬停崩塌 debug 判别（对应 M.22-6 / M.23-6-1）。

输入：一个或多个 run 目录（每个含 DEBUG_LEVEL>=1 生成的 debug.csv）。
只读 debug.csv 自带的 px/py/pz/vx/vy/vz/rv/rv_pred/gx,gy,gz/fit_res_mean 列，
无需 odom.csv / bag，即可：

  1) 按 |v| 阈值自动检出静止窗（odom 速度口径，5 点中值滤波 + 2 s 最短时长）；
  2) 对每个静止窗报告：窗内位姿位移 Δp、观测修正 |c| 峰值、fit_res 均值、
     重力相对初始的偏转角 g_tilt、退化签名 deg_cond/deg_axes（若存在列）；
  3) 标记"崩塌窗"（窗内 Δp > COLP_M，默认 1.0 m），并对崩塌窗逐段打印
     fit_res/g_tilt/|c| 的时间历程，用于区分三选一机制：
       (i)   地图重写拉走（fit_res 在崩塌前突然恶化 + |c| 骤增）
       (ii)  freeze 下竖直失回正（g 冻结但残差 c 的 up 分量单调累积）
       (iii) 观测权重累积后一次性跳变（fit_res/g 平稳，仅 |c| 单点跳）

用法：
  python3 -B exp/hover_collapse_diag.py exp/results_drone/sl_b_h20 \
         exp/results_drone/sl_b_pft05_gsc102_gfz exp/results_drone/sl_nbg003_na03
可选：
  --vth 0.10    静止速度阈值 (m/s)
  --mindur 2.0  最短静止窗时长 (s)
  --colp 1.0    崩塌判定位移阈值 (m)
"""
import sys
import numpy as np


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def so3_exp(phi):
    n = np.linalg.norm(phi)
    if n < 1e-12:
        return np.eye(3)
    K = skew(phi / n)
    return np.eye(3) + np.sin(n) * K + (1.0 - np.cos(n)) * (K @ K)


def so3_log(R):
    c = (np.trace(R) - 1.0) / 2.0
    c = np.clip(c, -1.0, 1.0)
    ang = np.arccos(c)
    if ang < 1e-12:
        return np.zeros(3)
    vv = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return ang / (2.0 * np.sin(ang)) * vv


def median_filter(x, k=5):
    if k % 2 == 0:
        k += 1
    half = k // 2
    y = x.copy()
    n = len(x)
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        y[i] = np.median(x[lo:hi])
    return y


def rotvec_corr(rv, rv_pred):
    R = so3_exp(rv) @ so3_exp(rv_pred).T
    return so3_log(R)


def load_debug(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    return d


def detect_hover(v, t, vth, mindur):
    vf = median_filter(v)
    mask = vf < vth
    segs = []
    n = len(mask)
    i = 0
    while i < n:
        if not mask[i]:
            i += 1
            continue
        j = i
        while j < n and mask[j]:
            j += 1
        if t[j - 1] - t[i] >= mindur:
            segs.append((i, j))
        i = j
    return segs


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    vth = 0.10
    mindur = 2.0
    colp = 1.0
    for a in sys.argv[1:]:
        if a.startswith("--vth="):
            vth = float(a.split("=")[1])
        elif a.startswith("--mindur="):
            mindur = float(a.split("=")[1])
        elif a.startswith("--colp="):
            colp = float(a.split("=")[1])

    for run in args:
        path = run if run.endswith("debug.csv") else run.rstrip("/") + "/debug.csv"
        try:
            d = load_debug(path)
        except Exception as e:
            print(f"[{run}] 读失败: {e}")
            continue

        t = d["t"]
        p = np.stack([d["px"], d["py"], d["pz"]], 1)
        v = np.stack([d["vx"], d["vy"], d["vz"]], 1)
        vel = np.linalg.norm(v, axis=1)

        rv = np.stack([d["rvx"], d["rvy"], d["rvz"]], 1)
        rvp = np.stack([d["rvx_pred"], d["rvy_pred"], d["rvz_pred"]], 1)
        c = np.array([rotvec_corr(rv[i], rvp[i]) for i in range(len(rv))])
        cnorm = np.linalg.norm(c, axis=1)

        g = np.stack([d["gx"], d["gy"], d["gz"]], 1)
        # 初始重力方向（前 50 帧均值）
        g0 = g[:50].mean(axis=0)
        g0 = g0 / np.linalg.norm(g0)
        cosang = np.clip((g @ g0) / np.linalg.norm(g, axis=1), -1, 1)
        gtilt = np.degrees(np.arccos(cosang))

        has_deg = "deg_cond" in d.dtype.names
        deg_cond = d["deg_cond"] if has_deg else np.zeros(len(d))
        deg_axes = d["deg_axes"] if has_deg else np.zeros(len(d))

        fit = d["fit_res_mean"]

        segs = detect_hover(vel, t, vth, mindur)
        print(f"\n===== {run}  帧数={len(d)}  静止窗={len(segs)} =====")
        print("  t0(s)  t1(s)  dur(s)  |dp|(m)   |c|max(deg)  fit_res   g_tilt(deg)  deg_cond  deg_axes")
        for (i, j) in segs:
            dp = np.linalg.norm(p[j - 1] - p[i])
            cmax = np.degrees(cnorm[i:j].max())
            fm = fit[i:j].mean()
            gm = gtilt[i:j].mean()
            dcm = deg_cond[i:j].max()
            da = int(deg_axes[i:j].max()) if has_deg else 0
            flag = "  << 崩塌" if dp > colp else ""
            print(f"  {t[i]:7.1f} {t[j-1]:7.1f} {t[j-1]-t[i]:6.1f} {dp:8.3f} "
                  f"{cmax:11.3f} {fm:9.4f} {gm:10.3f} {dcm:9.1f} {da:7d}{flag}")

        # 对崩塌窗打印时间历程
        collapsed = [(i, j) for (i, j) in segs if np.linalg.norm(p[j - 1] - p[i]) > colp]
        for (i, j) in collapsed:
            print(f"\n  [崩塌窗 t={t[i]:.1f}..{t[j-1]:.1f}s] 逐 1s 历程 (t, dp, |c|deg, fit_res, g_tilt, deg_cond):")
            # 每 1 秒一个 bin
            tb = t[i]
            while tb < t[j - 1]:
                m = (t >= tb) & (t < tb + 1.0)
                if m.any():
                    dpm = np.linalg.norm(p[m][-1] - p[m][0])
                    print(f"    {tb:6.1f}  {dpm:6.3f}  {np.degrees(cnorm[m].max()):8.3f}  "
                          f"{fit[m].mean():8.4f}  {gtilt[m].mean():8.3f}  {deg_cond[m].max():9.1f}")
                tb += 1.0


if __name__ == "__main__":
    main()