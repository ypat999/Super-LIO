#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 RTK-SLAM Dataset 的 ros2 bag 从 livox_ros_driver(ROS1 包名) 转成
本机可用的 livox_ros_driver2 类型（M.37 接入）。

为什么可以只改类型名而不重编码：
  两边的消息定义逐字段相同（已用源 bag 的 message_definitions 与本机
  driver2 的 .msg 对照确认）：
    std_msgs/Header header
    uint64 timebase
    uint32 point_num
    uint8  lidar_id
    uint8[3] rsvd
    CustomPoint[] points        # u32 offset_time, f32 x,y,z, u8 reflectivity/tag/line
  字段类型与顺序一致 ⇒ CDR 字节布局一致 ⇒ 消息 blob 原样搬运即可。
  实测布局（construction_seq2 第 1 帧，405167 B）：
    slen@12=12, "livox_lidar"@16..27, timebase@28(SDK 按 4 字节对齐，非 8),
    point_num@36=20256, lidar_id+rsvd@40..43, seq_len@44=20256, points@48,
    每点 20 B，末点省略 1 B 尾部填充 ⇒ 48 + 20256*20 - 1 = 405167 ✓

用法：
  python3 convert_livox_driver.py <src_bag_dir> <dst_bag_dir> [--dry-run]
"""
import argparse
import os
import shutil
import sqlite3
import struct
import sys

SRC_TYPE = "livox_ros_driver/msg/"
DST_TYPE = "livox_ros_driver2/msg/"


def rewrite_defs(db):
    """返回被改动的行数，便于校验确实动过。"""
    n = 0
    cols = {r[1] for r in db.execute("PRAGMA table_info(topics)")}
    if "type" in cols:
        sets = ["type = replace(type, ?, ?)"]
        args = [SRC_TYPE, DST_TYPE]
        if "type_description_hash" in cols:
            # 本机自采 bag 是 schema v3，根本没有 hash 列也能正常播放，
            # 故一律置空，避免伪造一个错误的 RIHS 哈希。
            sets.append("type_description_hash = ''")
        cur = db.execute(
            "UPDATE topics SET " + ", ".join(sets) + " WHERE type LIKE ?", args + [SRC_TYPE + "%"])
        n += cur.rowcount
    mdcols = {r[1] for r in db.execute("PRAGMA table_info(message_definitions)")}
    if mdcols:
        cur = db.execute(
            "UPDATE message_definitions SET topic_type = replace(topic_type, ?, ?),"
            " encoded_message_definition = replace(encoded_message_definition, ?, ?),"
            " type_description_hash = '' WHERE topic_type LIKE ?",
            (SRC_TYPE, DST_TYPE, SRC_TYPE, DST_TYPE, SRC_TYPE + "%"))
        n += cur.rowcount
    return n


def verify_blob(db):
    """按已确认的 CDR 布局解析一帧，检查自洽性。"""
    tid, name = db.execute(
        "SELECT id, name FROM topics WHERE name LIKE '%lidar'").fetchone()
    blob = db.execute(
        "SELECT data FROM messages WHERE topic_id=? ORDER BY id LIMIT 1", (tid,)).fetchone()[0]
    off = 4
    sec, nan = struct.unpack_from("<iI", blob, off)
    off += 8
    slen, = struct.unpack_from("<I", blob, off)
    off += 4
    frame = blob[off:off + slen - 1].decode()
    off += slen
    off = (off + 3) // 4 * 4                      # Livox SDK：64 位只做 4 字节对齐
    timebase, point_num, lidar_id = struct.unpack_from("<QIB", blob, off)
    off += 13 + 3                                  # 跳过 lidar_id 之后的 rsvd[3]
    seq_len, = struct.unpack_from("<I", blob, off)
    off += 4
    expect = off + seq_len * 20 - 1                # 末点无尾部填充
    ok = (seq_len == point_num) and (len(blob) == expect)
    print(f"  frame_id={frame!r} stamp={sec}.{nan:09d}")
    print(f"  timebase={timebase} point_num={point_num} lidar_id={lidar_id} seq_len={seq_len}")
    print(f"  blob={len(blob)} B，按布局应为 {expect} B ⇒ {'自洽' if ok else '不自洽'}")
    p0 = struct.unpack_from("<IfffBBB", blob, off)
    print(f"  首点 offset_time={p0[0]} us, r={ (p0[1]**2+p0[2]**2+p0[3]**2) ** 0.5 :.3f} m,"
          f" tag={p0[5]} line={p0[6]}")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    sdb3 = [f for f in os.listdir(a.src) if f.endswith(".db3")]
    if not sdb3:
        sys.exit("源目录里没有 .db3")
    if len(sdb3) > 1:
        sys.exit(f"源目录里有多个 .db3，暂不支持: {sdb3}")
    name = sdb3[0]
    os.makedirs(a.dst, exist_ok=True)
    print(f"源: {a.src}/{name}  ({os.path.getsize(os.path.join(a.src, name)) / 1e9:.2f} GB)")

    if a.dry_run:
        db = sqlite3.connect("file:" + os.path.join(a.src, name) + "?mode=ro", uri=True)
        print("[dry-run] 将要改写的类型：")
        for r in db.execute("SELECT name, type FROM topics"):
            print("   ", r)
        print("[dry-run] 布局自洽性检查：")
        verify_blob(db)
        return

    dst_db3 = os.path.join(a.dst, name)
    if not os.path.exists(dst_db3):
        print("复制 db3（约 10 GB，取决于磁盘）...")
        shutil.copyfile(os.path.join(a.src, name), dst_db3 + ".part")
        os.replace(dst_db3 + ".part", dst_db3)
    db = sqlite3.connect(dst_db3)
    changed = rewrite_defs(db)
    db.commit()
    print(f"改写 {changed} 行类型定义")
    for r in db.execute("SELECT name, type FROM topics"):
        print("   ", r)
    print("布局自洽性检查：")
    ok = verify_blob(db)
    db.close()

    # metadata.yaml：同步类型名（播放与 ros2 bag info 都读它）
    smeta = os.path.join(a.src, "metadata.yaml")
    if os.path.exists(smeta):
        txt = open(smeta, encoding="utf-8").read().replace(SRC_TYPE, DST_TYPE)
        open(os.path.join(a.dst, "metadata.yaml"), "w", encoding="utf-8").write(txt)
        print("metadata.yaml 已同步")
    if not ok:
        sys.exit("布局自洽性未通过，不要使用该产物")
    print(f"完成 → {a.dst}")


if __name__ == "__main__":
    main()
