#!/usr/bin/env python3
"""M.30 场景自动切换判据离线评估。

背景（见 stairwell_drift_diagnosis.txt M.28/M.29/M.30）：
  A 线（楼梯专用）= nw σ0.035 + hub0.20 + imu_na 0.3 + nbg 3e-3 → 楼梯 0.024 m
  B 线（通用）    = hub0.20 + imu_na 0.01              → 楼梯 1.171 / 1f 3.050 m
  自动切换要回答"当前是哪类场景"。已实测：
    deg_cond 平滑后两 bag 有重叠带（1f 有 ~19% 时间 cond<150 会被误判成楼梯）
    deg_axes>=3 只在 1f 出现（结构性判据，无反馈）
  本脚本检验第三判据 deg_dir_v = |u_min·v̂|（退化方向与运动方向对齐度）：
    走廊：退化轴 ≈ 运动轴 → 接近 1；楼梯：退化轴随螺旋旋转/多为竖直 → 低。
    它只用到雷达 Hessian 与速度，不含先验，因此不存在"改 na → 改变判据"的反馈。

用法：
  python3 autoswitch_diag.py results/m34_dirs_dbg results/f1_dirs_dbg
"""
import argparse
import csv
import statistics


def load(run):
    rows = []
    with open(run.rstrip("/") + "/debug.csv", newline="") as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})
    return rows


def smooth(v, w):
    """滑动中位数；w 为奇数帧数（10 Hz 下 11 帧 ≈ 1.1 s）。"""
    h = w // 2
    out = []
    for i in range(len(v)):
        a, b = max(0, i - h), min(len(v), i + h + 1)
        out.append(statistics.median(v[a:b]))
    return out


def q(v, p):
    s = sorted(v)
    return s[int(p * (len(s) - 1))]


def winfrac(v, w_sec, thr):
    """最近 w_sec 秒内 v > thr 的帧占比（按 10 Hz 帧率折算帧数）。"""
    h = int(w_sec * 10) // 2
    out = []
    for i in range(len(v)):
        a, b = max(0, i - h), min(len(v), i + h)
        out.append(sum(1 for k in range(a, b) if v[k] > thr) / float(b - a))
    return out


def desc(name, v):
    v = [x for x in v if x >= 0]          # -1 = 低速/无重力，分析时剔除
    if not v:
        print("  %-12s (无有效样本)" % name)
        return
    print("  %-12s n=%5d  p5=%.3f p25=%.3f p50=%.3f p75=%.3f p95=%.3f" %
          (name, len(v), q(v, .05), q(v, .25), q(v, .5), q(v, .75), q(v, .95)))


def corridor_frac(rows, cond_thr=150.0, axis_frac=0.3, dir_thr=None, w=11):
    """组合判据：走廊 = cond 中位 > thr  或  窗口内 3 轴帧占比 >= axis_frac
                    或（可选）dir_v 中位 > dir_thr"""
    c = smooth([r["deg_cond"] for r in rows], w)
    dv = smooth([r["deg_dir_v"] for r in rows], w)
    h = w // 2
    out = []
    for i in range(len(rows)):
        a, b = max(0, i - h), min(len(rows), i + h + 1)
        trio = sum(1 for k in range(a, b) if rows[k]["deg_axes"] >= 3) / float(b - a)
        cl = (c[i] > cond_thr) or (trio >= axis_frac)
        if dir_thr is not None and dv[i] >= dir_thr:
            cl = True
        out.append(cl)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stair", help="楼梯 debug run 目录（含 debug.csv）")
    ap.add_argument("f1", help="1f debug run 目录（含 debug.csv）")
    a = ap.parse_args()
    S, F = load(a.stair), load(a.f1)
    print("样本：楼梯 %d 帧 / 1f %d 帧\n" % (len(S), len(F)))

    print("■ 1. 新增指标分布")
    for nm in ["deg_cond_t", "deg_evt_min", "deg_dir_v", "deg_dir_g", "deg_dir_z"]:
        print(" 楼梯")
        desc(nm, [r[nm] for r in S])
        print(" 1f")
        desc(nm, [r[nm] for r in F])
        print()

    print("■ 2. 关键检验：1f 中 cond 低的那些段落，dir_v 是否明显高于楼梯")
    cf = smooth([r["deg_cond"] for r in F], 11)
    low = [F[i]["deg_dir_v"] for i in range(len(F)) if cf[i] < 150.0]
    print("  1f 且 cond<150 的子集（即当前判据的缺口所在）:")
    desc("dir_v", low)
    print("  楼梯全集:")
    desc("dir_v", [r["deg_dir_v"] for r in S])
    if low:
        print("  => 缺口段 dir_v 中位 %.3f；楼梯中位 %.3f" %
              (q([x for x in low if x >= 0], .5) if any(x >= 0 for x in low) else -1,
               q([x for x in [r["deg_dir_v"] for r in S] if x >= 0], .5)))

    print("\n■ 3. 组合判据扫描（判为走廊的比例，理想：楼梯低、1f 高）")
    print("  dir_thr   楼梯误判走廊   1f 正确判走廊")
    for t in [None, 0.5, 0.6, 0.7, 0.8, 0.9]:
        s = corridor_frac(S, dir_thr=t)
        f = corridor_frac(F, dir_thr=t)
        print("  %7s      %6.1f%%          %6.1f%%" %
              ("无" if t is None else "%.1f" % t,
               sum(s) / len(s) * 100, sum(f) / len(f) * 100))

    # ■ 4：窗口化判据（M.30 最终采用方向）。
    # 瞬时/短窗 cond 在两 bag 间重叠，但"最近 W 秒内 cond>thr 的帧占比"把
    # "偶发退化"与"持续退化"区分开：楼梯的 cond 呈 ~40 s 周期振荡、从不超过
    # 223，故该占比恒近 0；1f 走廊持续高 cond，占比稳定在高位。
    print("\n■ 4. 窗口化判据：最近 W 秒内 cond>thr 的帧占比 > gate 则判走廊")
    print("  W(s)  thr  gate   楼梯误判   1f 覆盖")
    for w in [30, 60, 120]:
        for thr in [150, 200, 300]:
            fs = winfrac([r["deg_cond"] for r in S], w, thr)
            ff = winfrac([r["deg_cond"] for r in F], w, thr)
            for gate in [0.5, 0.7]:
                fp = sum(1 for x in fs if x > gate) / len(fs)
                cov = sum(1 for x in ff if x > gate) / len(ff)
                print("  %4d  %4d  %.1f   %6.2f%%   %6.1f%%" %
                      (w, thr, gate, fp * 100, cov * 100))


if __name__ == "__main__":
    main()
