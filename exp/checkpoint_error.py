#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RTK-SLAM Dataset 检查点判据（M.37 §3）。

与 loop_error.py 的分工：loop_error.py 只有"回到同一点"的相对闭环判据；
本脚本用**全站仪实测的地理检查点**（UTM 32N / ETRS89，<1 cm）给出
按时间对应的分段绝对位移判据，因此能测"沿路累积漂移"而不必回到起点。

为什么不直接照抄官方的 absolute_errors：
  官方 metrics.absolute_errors 是完全不对齐的 3D 距离 RMSE，但其
  enu_origin.json + coords.py 的小角度近似隐含"SLAM 的 x 轴 = 真北"。
  这对 RTK-SLAM 成立（GNSS 定北），对纯 LIO 的 Super-LIO 不成立——
  我们的 odom 原点就是首帧机体位姿，x 轴 = 起始航向，与真北差一个未知角 θ。
  本序列水平尺度 ~100 m，θ=1° 就是 ~1.7 m 系统误差，直接算只会量到航向。
  所以主判据用**相邻检查点弦长**（旋转不变量，不需要 θ），
  θ 只在 --yaw 里作为副产品报告，且默认用首段反解，不参与主判据。

近似声明：UTM 坐标差直接当局部 ENU 差用。本场景跨度 <200 m，
  UTM 格网收敛引起的东西向尺度误差 <5e-6，可忽略；高程用 height 差。

用法：
  python3 checkpoint_error.py --gt ground_truth/construction_seq2.csv \
      --run exp/results/m37_c2_bline [--yaw none|first|ls]
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

MAX_DT = 0.5          # 检查点时刻与最近 odom 记录的最大允许时间差（s）
                      # 官方用 2.0 s，太宽：我们偶发的 lidar 断档会把检查点
                      # 静默匹配到陈旧位姿（M.37 §3.1）


def load_odom(path):
    t, p = [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            t.append(float(r['t_ns']) / 1e9)
            p.append([float(r['x']), float(r['y']), float(r['z'])])
    return np.array(t), np.array(p)


def load_gt(path):
    out = []
    with open(path) as f:
        for r in csv.DictReader(f):
            out.append({
                'id': r['point_id'],
                'e': float(r['easting']),
                'n': float(r['northing']),
                'h': float(r['height']),
                'env': r['env'],
                'ts': float(r['timestamp']),
            })
    out.sort(key=lambda d: d['ts'])
    return out


def interp(tarr, parr, tq, tol=MAX_DT):
    """线性插值取位置；时间超出轨迹范围或最近点距离>tol 时返回 None。"""
    if tq < tarr[0] or tq > tarr[-1]:
        return None
    i = int(np.searchsorted(tarr, tq))
    i = min(max(i, 1), len(tarr) - 1)
    if min(abs(tarr[i] - tq), abs(tarr[i - 1] - tq)) > tol:
        return None
    t0, t1 = tarr[i - 1], tarr[i]
    w = (tq - t0) / (t1 - t0) if t1 > t0 else 0.0
    return parr[i - 1] * (1 - w) + parr[i] * w


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gt', required=True)
    ap.add_argument('--run', required=True, help='含 odom.csv 的结果目录')
    ap.add_argument('--yaw', default='none', choices=['none', 'first', 'ls'],
                    help='none=只报旋转不变的弦长判据；first=用首段定 θ 后报绝对误差；'
                         'ls=最小二乘解 θ（含平移）后报残差')
    a = ap.parse_args()

    op = os.path.join(a.run, 'odom.csv')
    if not os.path.exists(op):
        sys.exit(f'缺 {op}')
    tarr, parr = load_odom(op)
    gt = load_gt(a.gt)
    print(f'odom {len(tarr)} 行，时长 {tarr[-1]-tarr[0]:.1f} s；GT 检查点 {len(gt)} 个')

    # 1) 时间匹配（线性插值 + 0.5 s 容差）
    pos, keep, drop = [], [], 0
    for g in gt:
        p = interp(tarr, parr, g['ts'])
        if p is None:
            drop += 1
        else:
            pos.append(p)
            keep.append(g)
    if drop:
        print(f'⚠ 丢弃 {drop} 个检查点（时刻不在轨迹区间内或最近记录时间差 > {MAX_DT} s）')
    if len(keep) < 2:
        sys.exit('匹配到的检查点不足 2 个，无法算弦长')

    gt_enu = np.array([[g['e'], g['n'], g['h']] for g in keep])
    slam = np.array(pos)

    # 2) 主判据：相邻检查点弦长（旋转不变）
    print('\n相邻检查点弦长（不依赖航向/原点）')
    print(' #   段                       env            真值弦长   SLAM弦长     差      相对')
    tot_gt = tot_sl = 0.0
    segs = []
    for i in range(len(keep) - 1):
        dgt = np.linalg.norm(gt_enu[i + 1] - gt_enu[i])
        dsl = np.linalg.norm(slam[i + 1] - slam[i])
        tot_gt += dgt
        tot_sl += dsl
        lab = f"{keep[i]['id']}→{keep[i+1]['id']}"
        env = f"{keep[i]['env']}/{keep[i+1]['env']}"
        segs.append((dgt, dsl, env))
        print(f'{i:2d}  {lab:24s} {env:18s} {dgt:8.2f} {dsl:9.2f} {dsl-dgt:+7.2f} {(dsl-dgt)/dgt*100:+6.1f}%')

    print(f'\n累计真值弦长 {tot_gt:.1f} m，累计 SLAM 弦长 {tot_sl:.1f} m，'
          f'整体尺度比 {tot_sl/tot_gt*100:.2f}%（差 {tot_sl-tot_gt:+.2f} m）')
    print(f'SLAM 自身路径长度 {np.sum(np.linalg.norm(np.diff(slam, axis=0), axis=1)):.1f} m'
          f'（对应区间，含检查点之间的绕行）')

    for tag in ('indoor/indoor', 'outdoor/transition', 'transition/indoor', 'indoor/transition'):
        sub = [(g, s) for g, s, e in segs if e == tag]
        if sub:
            gg = sum(x[0] for x in sub)
            ss = sum(x[1] for x in sub)
            print(f'  按段类型 {tag:20s}: {len(sub):2d} 段  真值 {gg:7.1f} m  SLAM {ss:7.1f} m  '
                  f'比 {ss/gg*100:6.2f}%')

    # 3) 航向：只作为副产品报告
    vgt = gt_enu[1] - gt_enu[0]
    vs1 = slam[1] - slam[0]
    th0 = math.degrees(math.atan2(vgt[1], vgt[0]) - math.atan2(vs1[1], vs1[0]))
    print(f'\n首段方位差（真北 - SLAM x 轴）= {th0:+.2f}°  '
          f'→ 这就是 SLAM 起始航向与真北的夹角，绝对判据必须先处理它')

    if a.yaw != 'none':
        def rot(deg):
            c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

        def resid(deg):
            d = (gt_enu[1:] - gt_enu[:-1]) - (rot(deg) @ (slam[1:] - slam[:-1]).T).T
            return np.linalg.norm(d, axis=1)

        if a.yaw == 'first':
            th = th0
        else:
            cand = np.arange(-180, 180, 0.05)
            th = min(cand, key=lambda d: np.mean(resid(d) ** 2))
        e = np.linalg.norm((gt_enu - gt_enu[0]) -
                           (rot(th) @ (slam - slam[0]).T).T, axis=1)
        print(f'用 θ={th:+.2f}°（{a.yaw}）+ 平移对齐后的逐点误差 '
              f'(m): ' + ' '.join(f'{x:.2f}' for x in e))
        print(f'  RMSE={np.sqrt(np.mean(e**2)):.2f} m  最大={e.max():.2f} m'
              f'   ※ 含对齐，不是官方口径的"零对齐 ATE"')
    else:
        print('  主判据只用上面的弦长比；需要绝对 ATE 时再加 --yaw ls')


if __name__ == '__main__':
    main()
