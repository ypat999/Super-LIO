#!/usr/bin/env python3
"""临时诊断：对比两个 bag 的 sqlite 索引与首帧读取耗时。"""
import sqlite3
import sys
import time

for bag in sys.argv[1:]:
    db = bag.rstrip("/") + "/" + bag.rstrip("/").split("/")[-1] + "_0.db3"
    con = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
    print("==", db)
    print("  schema:", [r[0] for r in con.execute(
        "select sql from sqlite_master where type in ('table','index')")])
    t0 = time.time()
    row = con.execute(
        "select timestamp, length(data) from messages order by timestamp asc limit 1").fetchone()
    print("  first row by ts: %s  (%.3fs)" % (row, time.time() - t0))
    t0 = time.time()
    row = con.execute(
        "select timestamp, length(data) from messages where topic_id=1 order by timestamp asc limit 1").fetchone()
    print("  first lidar row: %s  (%.3fs)" % (row, time.time() - t0))
    t0 = time.time()
    n = con.execute("select count(*) from messages where timestamp > ?", (row[0] if row else 0,)).fetchone()
    print("  rows after: %s (%.3fs)" % (n, time.time() - t0))
