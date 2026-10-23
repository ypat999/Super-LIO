#!/usr/bin/env python3
"""
从 node.log 的周期性计时块中汇总处理耗时与端到端延迟，供跨 run 比较。

计时块格式（timer.h:59，每 60 s 一次，按调用次数加权）：
  > [ [Observe] ] avg: 5.68 ms , max: 12.96 ms , calls: 511

端到端延迟（ROSWrapper 的 latency_timer_，同样按窗口汇总）：
  [Lidar->Odom] / [Lidar->BodyCloud] / [Lidar->CloudWorld] / [IMU->Odom] / [IMU->TF]

用法：
  python3 -B exp/timing_summary.py exp/results_drone/deg_base exp/results_drone/deg_trunc2 ...
  python3 -B exp/timing_summary.py --cols Observe,Undistort,Lidar-\\>Odom results_drone/*
"""
import re
import sys
import os

LINE_RE = re.compile(
    r"> \[\s*\[(?P<name>[^\]]+)\]\s*\] avg:\s*(?P<avg>[-0-9.eE+]+) ms\s*,\s*"
    r"max:\s*(?P<max>[-0-9.eE+]+) ms\s*,\s*calls:\s*(?P<calls>\d+)"
)


def parse_log(path):
    """返回 {name: {'calls': total, 'wsum': sum(avg*calls), 'max': max, 'windows': [(avg,max,calls)]}}"""
    stats = {}
    try:
        with open(path, "r", errors="ignore") as f:
            for line in f:
                m = LINE_RE.search(line)
                if not m:
                    continue
                name = m.group("name").strip()
                avg = float(m.group("avg"))
                mx = float(m.group("max"))
                calls = int(m.group("calls"))
                s = stats.setdefault(name, {"calls": 0, "wsum": 0.0, "max": 0.0,
                                            "windows": []})
                s["calls"] += calls
                s["wsum"] += avg * calls
                s["max"] = max(s["max"], mx)
                s["windows"].append((avg, mx, calls))
    except FileNotFoundError:
        return None
    for s in stats.values():
        s["avg"] = s["wsum"] / s["calls"] if s["calls"] else 0.0
    return stats


def recompute(s, drop_tail):
    """丢弃末尾 drop_tail 个 60 s 窗口后重算（末窗含播包结束/存图，非飞行段）。"""
    if drop_tail <= 0:
        return s["avg"], s["max"]
    ws = s["windows"][:-drop_tail]
    if not ws:
        return s["avg"], s["max"]
    calls = sum(w[2] for w in ws)
    if not calls:
        return 0.0, 0.0
    return sum(w[0] * w[2] for w in ws) / calls, max(w[1] for w in ws)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    cols = ["Observe", "Undistort", "DownSample", "UpdateMap", "DeltaCorrect",
            "Lidar->Odom", "Lidar->CloudWorld"]
    drop_tail = 0
    for a in sys.argv[1:]:
        if a.startswith("--cols="):
            cols = a.split("=", 1)[1].split(",")
        elif a.startswith("--drop-tail="):
            drop_tail = int(a.split("=", 1)[1])

    runs = []
    for a in args:
        d = a.rstrip("/")
        st = parse_log(os.path.join(d, "node.log"))
        if st is None:
            print(f"[skip] {d}: 无 node.log")
            continue
        runs.append((os.path.basename(d), st))

    # 表头
    hdr = f"{'run':22s}" + "".join(f"{c:>16s}" for c in cols)
    print(hdr)
    print("-" * len(hdr))
    for name, st in runs:
        row = f"{name:22s}"
        for c in cols:
            s = st.get(c)
            if s is None:
                row += f"{'-':>16s}"
            else:
                # 显示 avg（调用加权）与 max；drop_tail 用于剔除末尾含存图的窗口
                a_ms, x_ms = recompute(s, drop_tail)
                row += f"{a_ms:7.2f}/{x_ms:>6.1f}"
        print(row)
    print("-" * len(hdr))
    print("格式：平均/峰值 (ms)。平均按各 60 s 窗口的 calls 加权，峰值取全 run 最大值。")


if __name__ == "__main__":
    main()