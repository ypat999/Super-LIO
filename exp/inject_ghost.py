#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M.40 E-3（P-1）：在 seq2 的窄厅段"复制点云 + 固定平移"制造结构矛盾。

做法：
  对 /livox/lidar 的每条 CustomMsg，把整帧点云复制一份并整体平移 (dx,dy,dz)
  （机体系，单位 m），两份合并后写回同一条消息；point_num 与 seq_len 同步 ×2。
  每点 20 B（u32 offset_time, f32 x/y/z, u8 refl, u8 tag, u8 line + 1 B 对齐），
  平移直接改 x/y/z 三个 float32。

  只改指定时间窗内的消息；窗口外的 blob 一字节不动。
  消息类型名沿用 convert_livox_driver.py 产出的 livox_ros_driver2。

用法：
  python3 inject_ghost.py <src_bag_dir> <dst_bag_dir> <t0_rel_s> <t1_rel_s> [dx dy dz]
  t0/t1 相对 bag 起始时间；dst 目录会被创建（先复制 db3 再原地改写）。
"""
import os
import shutil
import sqlite3
import struct
import sys

POINT_BYTES = 20


def parse_layout(blob):
    """返回 (points_off, point_num_off, seq_len_off, point_num)。"""
    off = 4
    off += 8                                     # stamp sec/nsec
    slen, = struct.unpack_from("<I", blob, off)
    off += 4 + slen                              # frame_id 字符串（含末尾 0）
    off = (off + 3) // 4 * 4                     # Livox SDK 64 位只做 4 字节对齐
    off += 8                                     # timebase
    pn_off = off
    point_num, = struct.unpack_from("<I", blob, off)
    off += 4 + 4                                 # point_num + lidar_id/rsvd[3]
    sl_off = off
    seq_len, = struct.unpack_from("<I", blob, off)
    off += 4
    if seq_len != point_num:
        raise ValueError("seq_len != point_num (%d/%d)" % (seq_len, point_num))
    if len(blob) != off + point_num * POINT_BYTES - 1:
        raise ValueError("blob 长度不自洽: %d vs %d"
                         % (len(blob), off + point_num * POINT_BYTES - 1))
    return off, pn_off, sl_off, point_num


def duplicate(blob, d):
    off, pn_off, sl_off, n = parse_layout(blob)
    head = bytearray(blob[:off])
    struct.pack_into("<I", head, pn_off, 2 * n)
    struct.pack_into("<I", head, sl_off, 2 * n)
    pts = blob[off:off + n * POINT_BYTES]
    if len(pts) < n * POINT_BYTES:
        # CDR 把末点末尾的 1 B 对齐填充省掉了，补 0 使两份拷贝都按整 20 B 对齐
        pts = pts + b"\x00" * (n * POINT_BYTES - len(pts))
    dx, dy, dz = d
    ghost = bytearray(pts)
    for i in range(n):
        b = i * POINT_BYTES
        x, y, z = struct.unpack_from("<fff", ghost, b + 4)
        struct.pack_into("<fff", ghost, b + 4, x + dx, y + dy, z + dz)
    out = bytes(head) + pts + bytes(ghost)
    return out[:-1]                              # 末点无尾部填充


def main():
    if len(sys.argv) < 5:
        sys.exit(__doc__)
    src, dst = sys.argv[1], sys.argv[2]
    t0, t1 = float(sys.argv[3]), float(sys.argv[4])
    d = tuple(float(v) for v in sys.argv[5:8]) if len(sys.argv) >= 8 else (1.0, 0.0, 0.0)

    sdb3 = [f for f in os.listdir(src) if f.endswith(".db3")]
    if len(sdb3) != 1:
        sys.exit("源目录应有且仅有一个 .db3: %s" % sdb3)
    name = sdb3[0]
    os.makedirs(dst, exist_ok=True)
    for f in os.listdir(src):
        if f.endswith(".db3") or f == "metadata.yaml":
            shutil.copyfile(os.path.join(src, f), os.path.join(dst, f))

    db = sqlite3.connect(os.path.join(dst, name))
    tid, tstamp = db.execute(
        "SELECT id, 0 FROM topics WHERE name='/livox/lidar'").fetchone()
    t_start, = db.execute("SELECT MIN(timestamp) FROM messages").fetchone()
    rel0 = t_start + int(t0 * 1e9)
    rel1 = t_start + int(t1 * 1e9)
    rows = db.execute(
        "SELECT id, data FROM messages WHERE topic_id=? AND timestamp>=? AND timestamp<=?"
        " ORDER BY id", (tid, rel0, rel1)).fetchall()
    print("窗口 %.1f~%.1f s（bag 相对）  lidar 帧 %d" % (t0, t1, len(rows)))
    if not rows:
        sys.exit("窗口内没有 lidar 消息，检查 t0/t1")

    before = sum(len(b) for _, b in rows)
    for mid, blob in rows:
        db.execute("UPDATE messages SET data=? WHERE id=?", (duplicate(blob, d), mid))
    db.commit()
    after = db.execute(
        "SELECT SUM(LENGTH(data)) FROM messages WHERE topic_id=? AND timestamp>=? AND timestamp<=?",
        (tid, rel0, rel1)).fetchone()[0]
    print("平移 %s m  点云字节 %.1f MB → %.1f MB"
          % (d, before / 1e6, after / 1e6))
    # 抽样复验：窗口内首帧的 point_num 应翻倍
    blob = db.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY id LIMIT 1",
                      (tid,)).fetchone()[0]
    off, _, _, n = parse_layout(blob)
    print("复验首帧 point_num=%d  blob=%d B" % (n, len(blob)))
    db.close()
    print("完成 →", dst)


if __name__ == "__main__":
    main()
