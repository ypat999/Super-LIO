#!/usr/bin/env python3
"""临时诊断：直接读 bag 的 sqlite，确认消息表可读、话题与时间跨度正常。"""
import glob
import sqlite3
import sys

bag = sys.argv[1] if len(sys.argv) > 1 else "/home/ywj/slam_data/livox_record_drone"
for db in sorted(glob.glob(bag + "/*.db3")):
    con = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
    topics = dict(con.execute("select id, name from topics").fetchall())
    print(db)
    print("  topics:", topics)
    for tid, name in topics.items():
        n, tmin, tmax = con.execute(
            "select count(*), min(timestamp), max(timestamp) from messages where topic_id=?", (tid,)
        ).fetchone()
        print("   %-12s n=%d t=[%d, %d]" % (name, n, tmin, tmax))
