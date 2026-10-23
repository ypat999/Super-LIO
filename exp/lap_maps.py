#!/usr/bin/env python3
"""按圈重建点云：用 odom 的位姿把 bag 里的原始帧累加成"第 1 圈 / 第 2 圈"两张图。

为什么需要它：saveMap() 只落一张合并图，两圈的估计被搅在一起，无法回答
"竖直漂移是每圈持续累积、还是一次性事件"，也无法分别量两圈各自的地面倾角。
本脚本用已落盘的 odom.csv 做位姿、从 bag 读原始帧，按时间窗切开分别累加。

用法:
  python3 -B exp/lap_maps.py --bag <bag> --run <run_dir> \
      --seg "0:300,346:595,595:" [--out <dir>] [--voxel 0.3]
  --seg 给若干时间区间（相对 odom 起点的秒数），逗号分隔，右端留空表示到结尾：
      各区间分别累加成 lap_<k>.pcd。用区间而不是切分点，是为了跳过"停车等
      待"那几段（例如 300~346 s 的停留不该算进任何一圈）。
  需要先 source ROS 与 livox_ros_driver2 的消息包。
"""
import argparse
import os
import sys

import numpy as np


def load_odom(path):
    d = np.genfromtxt(path, delimiter=",", skip_header=1)
    t = d[:, 0].astype(np.float64) * 1e-9
    return t, d[:, 1:4].astype(np.float64), d[:, 4:8].astype(np.float64)


def q_slerp(q0, q1, u):
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    th = np.arccos(np.clip(d, -1, 1))
    return (np.sin((1 - u) * th) * q0 + np.sin(u * th) * q1) / np.sin(th)


def q_to_R(q):
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Pose:
    def __init__(self, t, p, q):
        self.t, self.p, self.q = t, p, q

    def at(self, ts):
        i = int(np.clip(np.searchsorted(self.t, ts) - 1, 0, len(self.t) - 2))
        span = self.t[i + 1] - self.t[i]
        u = 0.0 if span <= 0 else float(np.clip((ts - self.t[i]) / span, 0, 1))
        p = self.p[i] * (1 - u) + self.p[i + 1] * u
        q = q_slerp(self.q[i], self.q[i + 1], u)
        return q_to_R(q), p


def voxel_down(P, vox):
    if len(P) == 0:
        return P
    k = np.floor(P / vox).astype(np.int64)
    key = (k[:, 0] * 73856093) ^ (k[:, 1] * 19349663) ^ (k[:, 2] * 83492791)
    key = key.astype(np.int64) & np.int64(0x7FFFFFFFFFFF)
    _, idx = np.unique(key, return_index=True)
    return P[idx]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--run", required=True)
    ap.add_argument("--seg", required=True,
                    help='时间区间，逗号分隔，如 "0:300,346:595,595:"（右端留空=到结尾）')
    ap.add_argument("--out", default="")
    ap.add_argument("--voxel", type=float, default=0.3)
    ap.add_argument("--topic", default="/livox/lidar")
    a = ap.parse_args()

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from livox_ros_driver2.msg import CustomMsg

    t, p, q = load_odom(os.path.join(a.run, "odom.csv"))
    pose = Pose(t, p, q)
    segs = []
    for s in a.seg.split(","):
        lo, _, hi = s.partition(":")
        segs.append((float(lo), float(hi) if hi.strip() else np.inf))

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[a.topic]))
    t_odom0 = t[0]  # 区间时刻用相对秒数；位姿查表必须用绝对时间（t 是 epoch 秒）

    acc = [[] for _ in segs]
    nframes = [0] * len(segs)
    # 只取局部区间时，读到最后一段末尾就可以停（bag 按时间顺序，省掉整包反序列化）
    t_end = max(hi for _, hi in segs)
    while reader.has_next():
        _, data, stamp = reader.read_next()
        rel = stamp * 1e-9 - t_odom0
        if rel > t_end:
            break
        # 一帧可命中多个区间：区间重叠时一帧进入所有匹配区间（用于 0:w 累积重建）
        hits = [k for k, (lo, hi) in enumerate(segs) if lo <= rel < hi]
        if not hits:
            continue  # 区间外不反序列化：解析一次 CustomMsg 比读一次行贵两个量级
        m = deserialize_message(data, CustomMsg)
        # 每帧抽一半点即可（只用于平面/形状统计），纯 Python 逐点取属性很慢
        sub = m.points[::2]
        n = len(sub)
        if n == 0:
            continue
        pts = np.empty((n, 3))
        pts[:, 0] = np.fromiter((q.x for q in sub), float, n)
        pts[:, 1] = np.fromiter((q.y for q in sub), float, n)
        pts[:, 2] = np.fromiter((q.z for q in sub), float, n)
        R, pos = pose.at(stamp * 1e-9)
        w = pts @ R.T + pos
        for k in hits:
            acc[k].append(w)
            nframes[k] += 1
            if len(acc[k]) >= 300:
                acc[k] = [voxel_down(np.vstack(acc[k]), a.voxel)]

    out = a.out or a.run
    os.makedirs(out, exist_ok=True)
    print(f"[lap_maps] {a.run}  区间={segs}")
    for k in range(len(segs)):
        if not acc[k]:
            print(f"  lap_{k}: 空")
            continue
        P = voxel_down(np.vstack(acc[k]), a.voxel)
        fp = os.path.join(out, f"lap_{k}.pcd")
        with open(fp, "wb") as f:
            f.write(b"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
                    b"FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
            f.write(f"WIDTH {len(P)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n".encode())
            f.write(f"POINTS {len(P)}\nDATA binary\n".encode())
            f.write(P.astype(np.float32).tobytes())
        print(f"  lap_{k}: {nframes[k]} 帧 -> {len(P)} 点  ({fp})")


if __name__ == "__main__":
    main()
