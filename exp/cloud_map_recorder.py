#!/usr/bin/env python3
"""订阅世界系点云（/cloud_registered），在线体素累积，退出时落一张 map.pcd。

用途：给没有 /map_save 服务的框架（point_lio）或 pcl_wait_pub 为空实现的框架
（FAST-LIO 的 publish_map 只在 map_en=true 时累积）导出同口径的建图结果。

与 v1 的差别（v1 在 fastlio_mapping 收到 SIGINT 崩掉后没落盘成功）：
- 不再用 "信号处理里 raise KeyboardInterrupt" 的方式退出（异常可能在 rclpy 的
  C 层回调里被吞掉，进程最终被 kill -9，flush 从未执行）；改成"信号只置标志位，
  主循环 spin_once 轮询"；
- 不再把整包所有帧堆到退出时一次性 concat + lexsort（50M 点峰值 >2 GB）；
  改成每 --compact-every 帧增量体素合并，缓冲区恒定在几百万点；
- 每 --checkpoint-every 帧把当前地图覆盖写一次 map.pcd，即使被硬杀也留有可用结果。

用法: python3 cloud_map_recorder.py --out map.pcd [--topic /cloud_registered]
                                   [--stride 3] [--voxel 0.1]
"""
import argparse
import os
import signal
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2

# 体素 key 打包：每个轴压到 21 bit（偏移 2^20），拼成一个 int64，便于 unique 去重
_OFF = 1 << 20
_MAXK = (1 << 21) - 1

_STOP = {"flag": False}


def _on_signal(signum, frame):
    _STOP["flag"] = True


def cloud_xyz(msg):
    """按 PointCloud2 的 fields 取出 x/y/z（float32）。"""
    names = [f.name for f in msg.fields]
    if not all(k in names for k in ("x", "y", "z")):
        return None
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4"] * 3,
        "offsets": [off["x"], off["y"], off["z"]],
        "itemsize": msg.point_step,
    })
    n = msg.width * msg.height
    arr = np.frombuffer(bytes(msg.data), dtype=dt, count=n)
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1)
    return xyz[np.isfinite(xyz).all(axis=1)]


def voxel_first(pts, voxel):
    """体素降采样：每个体素保留第一个点（真实测量点，不做质心合成）。"""
    if len(pts) == 0:
        return pts
    k = np.floor(pts / voxel).astype(np.int64) + _OFF
    np.clip(k, 0, _MAXK, out=k)
    code = (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]
    _, idx = np.unique(code, return_index=True)
    idx.sort()
    return pts[idx]


def write_pcd(path, pts):
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(pts)}\nDATA binary\n"
    )
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(np.ascontiguousarray(pts.astype(np.float32)).tobytes())
    os.replace(tmp, path)


class CloudMapRecorder(Node):
    def __init__(self, out, topic, stride, voxel, compact_every,
                 checkpoint_every, compact_voxel):
        super().__init__("exp_cloud_map_recorder")
        self.out = out
        self.stride = max(1, stride)
        self.voxel = voxel
        self.compact_voxel = compact_voxel
        self.compact_every = max(1, compact_every)
        self.checkpoint_every = max(1, checkpoint_every)

        self.n_msg = 0          # 收到的消息数
        self.n_used = 0         # 真正收进缓冲的帧数
        self.n_raw = 0          # 累计原始点数
        self.pending = []       # 待合并帧
        self.acc = np.zeros((0, 3), np.float32)   # 已合并（compact_voxel 网格）
        self.done = False

        qos = QoSProfile(depth=20, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PointCloud2, topic, self._cb, qos)
        self.get_logger().info(
            f"accumulating {topic} (stride={self.stride} voxel={self.voxel} "
            f"compact={self.compact_voxel}/{self.compact_every} "
            f"checkpoint={self.checkpoint_every})")

    # ------------------------------------------------------------------ 回调
    def _cb(self, msg):
        self.n_msg += 1
        if self.n_msg % 500 == 0:
            self.get_logger().info(
                f"recv {self.n_msg} msgs, used {self.n_used}, raw {self.n_raw} pts, "
                f"map {len(self.acc)} pts")
        if self.n_msg % self.stride:
            return
        xyz = cloud_xyz(msg)
        if xyz is None or len(xyz) == 0:
            return
        self.pending.append(xyz.copy())
        self.n_used += 1
        self.n_raw += len(xyz)
        if len(self.pending) >= self.compact_every:
            self._compact()
            if self.n_used % self.checkpoint_every < self.compact_every:
                self._write()

    # ------------------------------------------------------------------ 落盘
    def _compact(self):
        if not self.pending:
            return
        buf = np.concatenate([self.acc] + self.pending, axis=0)
        self.pending = []
        self.acc = voxel_first(buf.astype(np.float32), self.compact_voxel)

    def _write(self):
        if len(self.acc) == 0:
            return
        pts = voxel_first(self.acc, self.voxel)
        write_pcd(self.out, pts)
        self.get_logger().info(
            f"checkpoint {self.n_used} frames -> {len(pts)} pts -> {self.out}")

    def flush(self):
        if self.done:
            return
        self.done = True
        self._compact()
        if len(self.acc) == 0:
            self.get_logger().warn(
                f"no data: recv {self.n_msg} msgs (topic never matched?)")
            print("[cloud_map] no data", flush=True)
            return
        pts = voxel_first(self.acc, self.voxel)
        write_pcd(self.out, pts)
        msg = (f"[cloud_map] recv {self.n_msg} msgs, used {self.n_used} frames, "
               f"raw {self.n_raw} pts -> voxel {self.voxel} -> {len(pts)} pts -> {self.out}")
        self.get_logger().info(msg)
        print(msg, flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--topic", default="/cloud_registered")
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--voxel", type=float, default=0.1)
    ap.add_argument("--compact-voxel", type=float, default=0.05)
    ap.add_argument("--compact-every", type=int, default=100)
    ap.add_argument("--checkpoint-every", type=int, default=600)
    args = ap.parse_args()

    rclpy.init()
    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    node = CloudMapRecorder(args.out, args.topic, args.stride, args.voxel,
                            args.compact_every, args.checkpoint_every,
                            args.compact_voxel)
    try:
        while rclpy.ok() and not _STOP["flag"]:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        try:
            node.flush()
        except Exception as exc:                     # noqa: BLE001
            print(f"[cloud_map] flush failed: {exc}", flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
