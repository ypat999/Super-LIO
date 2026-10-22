#!/usr/bin/env python3
"""按进程名采样 LIO 节点的 CPU 时间 / 线程数 / 内存，用于估算单核可承载性。

为什么这样测：日志里的计时器（Super-LIO 的 [Observe]/[Undistort]）只覆盖部分环节，
GenZ 则完全不打点；而真机瓶颈看的是"每帧要吃掉多少 CPU 秒"，与是否多线程无关。
总 CPU 时间 / 帧数 ≈ 把它压到单核上跑所需的时间（忽略并行效率损失，属乐观上界）。

用法:
  python3 -B perf_probe.py <comm_name> <out.csv>      # comm 即 /proc/<pid>/comm，如 genz_lio_node
进程出现后开始累加，消失 3 s 后退出并打印汇总。
"""
import os
import sys
import time

name, out = sys.argv[1], sys.argv[2]
CLK = os.sysconf("SC_CLK_TCK")

prev = {}
total = 0.0
peak_th = 0
peak_rss = 0
seen = False
idle = 0.0
t0 = time.time()
last = t0
rows = []

while True:
    now = time.time()
    pids = []
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            if open("/proc/%s/comm" % d).read().strip() == name:
                pids.append(int(d))
        except Exception:
            continue
    if pids:
        seen = True
        idle = 0.0
    else:
        idle += now - last

    cur = {}
    for pid in pids:
        try:
            st = open("/proc/%d/stat" % pid).read().split()
            cpu = (int(st[13]) + int(st[14])) / CLK
            status = open("/proc/%d/status" % pid).read().splitlines()
            th = int([l for l in status if l.startswith("Threads")][0].split()[1])
            rss = int([l for l in status if l.startswith("VmRSS")][0].split()[1]) // 1024
        except Exception:
            continue
        cur[pid] = cpu
        peak_th = max(peak_th, th)
        peak_rss = max(peak_rss, rss)
        if pid in prev and cpu >= prev[pid]:
            total += cpu - prev[pid]
    prev = cur

    rows.append("%.2f,%d,%.3f,%d,%d" % (now - t0, len(pids), total, peak_th, peak_rss))
    last = now
    if seen and idle > 3.0:
        break
    if not seen and (now - t0) > 90:
        break
    time.sleep(0.2)

with open(out, "w") as f:
    f.write("t,nproc,cpu_s,peak_threads,peak_rss_mb\n")
    f.write("\n".join(rows) + "\n")
print("[probe] %s: total_cpu=%.2fs peak_threads=%d peak_rss=%dMB wall=%.1fs"
      % (name, total, peak_th, peak_rss, now - t0))
