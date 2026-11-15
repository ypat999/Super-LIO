#!/usr/bin/env python3
"""临时诊断：打印 bag 记录的各话题 QoS。"""
import sqlite3
import sys

for bag in sys.argv[1:]:
    db = bag.rstrip("/") + "/" + bag.rstrip("/").split("/")[-1] + "_0.db3"
    con = sqlite3.connect("file:%s?mode=ro" % db, uri=True)
    print("==", bag)
    for name, typ, qos in con.execute("select name, type, offered_qos_profiles from topics"):
        print("  %-12s %-40s" % (name, typ))
        print("    " + (qos or "(empty)").replace("\n", " "))
