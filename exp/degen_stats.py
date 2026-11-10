#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""退化指标统计与多 run 对照（M.38）。

读 debug_level>=2 且 DEGENERACY_DETECT_EN=true 的 run 的 debug.csv，
输出 deg_cond / deg_axes / deg_dir_v 的分布，用于回答两件事：
  1) 某个 bag 在我们自己的判据下"退化到什么程度"（与楼梯/1f 同口径对照）；
  2) 若给 --gt，则按数据集检查点的时间与环境标签分段，看退化集中在哪一段。

注意 deg_cond 只度量"信息量少"，不度量"信息互相矛盾"——M.38 的核心结论
就是这两者不是一回事，不要拿 cond 高低去预测漂移大小。

用法：
  python3 degen_stats.py results/m37_c2_degen results/f1_dirs_dbg results/m34_dirs_dbg
  python3 degen_stats.py results/m37_c2_degen --tag "seq2" \
      --gt /path/ground_truth/construction_seq2.csv
"""
import argparse
import csv
import os
import sys


def load(run):
    p = run if run.endswith('.csv') else os.path.join(run, 'debug.csv')
    if not os.path.exists(p):
        return None
    rows = []
    for r in csv.DictReader(open(p)):
        try:
            d = {k: float(v) for k, v in r.items()}
        except ValueError:
            return None          # 该 run 没开 debug，列是空的
        rows.append(d)
    return [r for r in rows if r.get('deg_cond', 0.0) > 0]


def pct(v, p):
    v = sorted(v)
    k = (len(v) - 1) * p / 100.0
    f = int(k)
    c = min(f + 1, len(v) - 1)
    return v[f] + (v[c] - v[f]) * (k - f)


def line(tag, rows):
    if len(rows) < 10:
        return '  %-32s n=%5d  样本不足或未开 deg 诊断' % (tag, len(rows))
    c = [r['deg_cond'] for r in rows]
    a = [r['deg_axes'] for r in rows]
    dv = [r['deg_dir_v'] for r in rows if r.get('deg_dir_v', -1) >= 0]
    s = ('  %-32s n=%5d  cond p50=%7.1f p95=%8.1f max=%9.1f  >150:%5.1f%%  '
         'ax>=2:%5.1f%% ax>=3:%5.1f%%' % (
             tag, len(rows), pct(c, 50), pct(c, 95), max(c),
             100.0 * sum(1 for x in c if x > 150.0) / len(c),
             100.0 * sum(1 for x in a if x >= 2) / len(a),
             100.0 * sum(1 for x in a if x >= 3) / len(a)))
    if dv:
        s += '  dir_v p50=%.2f' % pct(dv, 50)
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--tag', default=None)
    ap.add_argument('--gt', default=None, help='数据集 ground_truth/*.csv，按检查点分段')
    ap.add_argument('--cond-thr', type=float, default=150.0)
    a = ap.parse_args()

    print('同口径对照（cond_thr=%.0f；楼梯/1f 的 M.30 数字即来自此口径）' % a.cond_thr)
    print('%-36s %6s %9s %9s %9s %7s %7s %7s' % ('run', 'n', 'p50', 'p95', 'max',
                                                 '>150', 'ax>=2', 'ax>=3'))
    for run in a.runs:
        rows = load(run)
        tag = a.tag or os.path.basename(run.rstrip('/'))
        if rows is None:
            print('  %-34s 无可用 deg 数据（需 DEBUG_LEVEL>=2 且 DEGENERACY_DETECT_EN=true）' % tag)
            continue
        print(line(tag, rows)[2:])

        if a.gt:
            gt = sorted(csv.DictReader(open(a.gt)), key=lambda r: float(r['timestamp']))
            t0 = rows[0]['t']
            print('\n  按检查点分段（env 标签来自数据集）')
            segs = []
            for i in range(len(gt) - 1):
                ta = float(gt[i]['timestamp']) - t0
                tb = float(gt[i + 1]['timestamp']) - t0
                seg = [r for r in rows if ta <= r['t'] - t0 < tb]
                segs.append((gt[i]['env'], gt[i + 1]['env'], seg))
                print(line('%s->%s %s->%s' % (gt[i]['env'][:9], gt[i + 1]['env'][:9],
                                              gt[i]['point_id'], gt[i + 1]['point_id']), seg))
            print('\n  按环境类型聚合')
            for env in ('outdoor', 'indoor', 'transition'):
                sel = [s for e1, e2, s in segs if e1 == env and e2 == env]
                print(line('%s->%s (%d 段)' % (env, env, len(sel)),
                           [r for s in sel for r in s]))


if __name__ == '__main__':
    main()
