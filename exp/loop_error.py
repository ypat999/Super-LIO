#!/usr/bin/env python3
"""闭环偏差：起点与终点的位置/姿态差（适用于"一圈回原点"的 bag）。

为什么需要它：楼层间楼梯 bag 是螺旋升/降，末端无法回到起点，测不出闭环误差；
新 bag 是"两条楼梯一下一上、一圈回原点"，于是 |p_end − p_start| 直接就是累积漂移。

竖直方向不取 odom 的 z：本机雷达 30° 俯仰安装，FAST-LIO / GenZ-LIO 的世界系 z 偏离
竖直 ~30°，只有 Super-LIO 自带重力对齐。这里统一从 bag 起始的静态 IMU 均值得
up_imu = −mean(acc)，再用各框架自己的首帧姿态把它转到各自的世界系：
    up_world = R(q0) · up_imu

用法:
  python3 loop_error.py --bag /home/ywj/slam_data/livox_record_drone <run_dir> [<run_dir> ...]
  python3 loop_error.py --bag ... --static 3.0 ...
每个 run_dir 需要 odom.csv（列 t_ns,x,y,z,qx,qy,qz,qw）。
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import load_odom


def quat_to_rot(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def imu_up(bag, static_s=3.0):
    """bag 起始处的重力方向（IMU 系），单位向量，指向上。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/livox/imu"]))
    t0 = reader.get_metadata().starting_time.nanoseconds
    acc, gyr, n = [], [], 0
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if (stamp - t0) > static_s * 1e9:
            break
        m = deserialize_message(data, Imu)
        acc.append([m.linear_acceleration.x, m.linear_acceleration.y,
                    m.linear_acceleration.z])
        gyr.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        n += 1
    if n == 0:
        raise SystemExit("bag 起始没有 IMU 数据")
    a = np.mean(acc, axis=0)
    w = np.mean(gyr, axis=0)
    na = float(np.linalg.norm(a))
    print(f"[imu] 起始 {static_s:.1f} s 内 {n} 帧：|mean acc|={na:.3f}  "
          f"acc std={np.std(acc, axis=0).round(3)}  mean gyro(deg/s)="
          f"{np.degrees(w).round(3)}  gyro std={np.degrees(np.std(gyr, axis=0)).round(3)}")
    # livox 驱动有的版本把加速度按 g 输出（|a|≈1），方向仍可用，只提示不修
    if abs(na - 9.81) > 0.5 and abs(na - 1.0) > 0.2:
        print("[imu] 警告：|acc| 既不是 9.81 也不是 1.0，起始段可能不是静止")
    return -a / na


def rotvec(R):
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    ang = np.arccos(c)
    if ang < 1e-9:
        return np.zeros(3)
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return ax / (2.0 * np.sin(ang)) * ang


def load_run(run):
    """返回 (t, xyz, quat)；优先 odom.csv，缺失时回退 odom_tum.txt。"""
    odom = load_odom(os.path.join(run, "odom.csv"))
    if odom is not None and len(odom) >= 20:
        return (odom["t_ns"].astype(np.float64) * 1e-9,
                np.stack([odom["x"], odom["y"], odom["z"]], axis=1).astype(np.float64),
                np.stack([odom["qx"], odom["qy"], odom["qz"], odom["qw"]],
                         axis=1).astype(np.float64))
    p = os.path.join(run, "odom_tum.txt")
    if not os.path.exists(p):
        return None
    try:
        a = np.loadtxt(p)
    except Exception:
        return None
    if a.ndim != 2 or a.shape[0] < 20 or a.shape[1] < 8:
        return None
    return a[:, 0], a[:, 1:4].astype(np.float64), a[:, 4:8].astype(np.float64)


def measure(run, up_imu, settle=1.0):
    loaded = load_run(run)
    if loaded is None:
        return None
    t, xyz, quat = loaded

    R0 = quat_to_rot(quat[0])
    Rend = quat_to_rot(quat[-1])
    up = R0 @ up_imu

    m0 = (t - t[0]) <= settle
    m1 = (t[-1] - t) <= settle
    p0, p1 = xyz[m0].mean(0), xyz[m1].mean(0)
    d = p1 - p0
    dh = np.linalg.norm(d - (d @ up) * up)
    dv = float(d @ up)

    rv = rotvec(R0.T @ Rend)
    yaw = float(rv @ up_imu)  # 首帧 body 系里 up_imu 就是竖直轴

    step = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    dt = np.diff(t)
    v = step / np.maximum(dt, 1e-6)
    h = xyz @ up
    return {
        "run": os.path.basename(run.rstrip("/")),
        "n": len(xyz),
        "dur": t[-1] - t[0],
        "path": float(step.sum()),
        "v_med": float(np.median(v)),
        "v_max": float(v.max()),
        "d3": float(np.linalg.norm(d)),
        "dh": float(dh),
        "dv": dv,
        "d_pct": float(np.linalg.norm(d) / max(step.sum(), 1e-9) * 100.0),
        "rot": float(np.degrees(np.linalg.norm(rv))),
        "yaw": float(np.degrees(yaw)),
        "zlo": float(h.min()),
        "zhi": float(h.max()),
    }


def detect_stops(t, xyz, v_thr=0.10, min_s=2.0):
    """找静止段，用于"多圈各自回起点"的 bag 逐圈闭环。

    判据：瞬时速度中值滤波后 < v_thr，且连续 >= min_s。
    返回 [(i0, i1)] 闭区间索引；i0/i1 是静止段的首末帧。
    """
    v = np.linalg.norm(np.diff(xyz, axis=0), axis=1) / np.maximum(np.diff(t), 1e-6)
    # 5 点中值滤波，压掉单帧跳变
    k = 5
    if len(v) >= k:
        v = np.array([np.median(v[max(0, i - k // 2):i + k // 2 + 1])
                      for i in range(len(v))])
    still = v < v_thr
    segs, i = [], 0
    while i < len(still):
        if not still[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(still) and still[j + 1]:
            j += 1
        if t[j] - t[i] >= min_s:
            segs.append((i, j))
        i = j + 1
    return segs


def measure_loops(run, up_imu, v_thr=0.10, min_s=2.0):
    """逐圈闭环：以检测到的每个静止段为一个锚点，算相对首个锚点的偏差。"""
    loaded = load_run(run)
    if loaded is None:
        return None
    t, xyz, quat = loaded
    up = quat_to_rot(quat[0]) @ up_imu
    segs = detect_stops(t, xyz, v_thr, min_s)
    if not segs:
        return {"run": os.path.basename(run.rstrip("/")), "anchors": []}

    anchors = []
    for (i0, i1) in segs:
        p = xyz[i0:i1 + 1].mean(0)
        R = quat_to_rot(quat[i0:i1 + 1].mean(0) / np.linalg.norm(quat[i0:i1 + 1].mean(0)))
        anchors.append({"t0": t[i0] - t[0], "t1": t[i1] - t[0],
                        "dur": t[i1] - t[i0], "p": p, "R": R})
    p0, R0 = anchors[0]["p"], anchors[0]["R"]
    step = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    for k, a in enumerate(anchors):
        d = a["p"] - p0
        a["d3"] = float(np.linalg.norm(d))
        a["dh"] = float(np.linalg.norm(d - (d @ up) * up))
        a["dv"] = float(d @ up)
        rv = rotvec(R0.T @ a["R"])
        a["yaw"] = float(np.degrees(rv @ up_imu))
        a["rot"] = float(np.degrees(np.linalg.norm(rv)))
        # 从上一个锚点走过来的里程，作分母
        i_prev = segs[k - 1][0] if k > 0 else 0
        i_now = segs[k][0]
        a["leg"] = float(step[i_prev:i_now].sum())
    return {"run": os.path.basename(run.rstrip("/")),
            "anchors": anchors, "n": len(xyz), "dur": t[-1] - t[0],
            "path": float(step.sum())}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--static", type=float, default=3.0)
    ap.add_argument("--loops", action="store_true",
                    help="按检测到的静止段逐圈算闭环（用于多圈各自回起点的 bag）")
    ap.add_argument("--still", type=float, default=0.10, help="静止判据 m/s")
    ap.add_argument("--still-min", type=float, default=2.0, help="静止最短持续时间 s")
    ap.add_argument("runs", nargs="+")
    a = ap.parse_args()

    up_imu = imu_up(a.bag, a.static)
    print(f"[imu] up_imu = {up_imu.round(4)}")
    if a.loops:
        for run in a.runs:
            r = measure_loops(run, up_imu, a.still, a.still_min)
            if r is None:
                print(f"{os.path.basename(run.rstrip('/'))}: 无 odom.csv 或行数不足")
                continue
            print(f"\n== {r['run']}  dur={r['dur']:.1f}s  path={r['path']:.1f}m  "
                  f"锚点（静止段）{len(r['anchors'])} 个")
            print(f"{'#':>3}{'t0(s)':>8}{'t1(s)':>8}{'停(s)':>7}{'上一段里程':>11}"
                  f"{'闭环3D':>8}{'水平':>7}{'竖直':>7}{'占路径%':>9}{'yaw°':>8}")
            for k, s in enumerate(r["anchors"]):
                pct = 100.0 * s["d3"] / max(s["leg"], 1e-9) if k > 0 else 0.0
                print(f"{k:>3}{s['t0']:>8.1f}{s['t1']:>8.1f}{s['dur']:>7.1f}"
                      f"{s['leg']:>11.1f}{s['d3']:>8.3f}{s['dh']:>7.3f}"
                      f"{s['dv']:>+7.3f}{pct:>9.3f}{s['yaw']:>+8.2f}")
        return
    print(f"\n{'run':<26}{'dur(s)':>7}{'路径m':>8}{'v中':>6}{'闭环3D':>8}"
          f"{'水平':>7}{'竖直':>7}{'占路径%':>9}{'转角°':>8}{'其中yaw°':>10}{'高度m':>9}")
    for run in a.runs:
        r = measure(run, up_imu)
        if r is None:
            print(f"{os.path.basename(run.rstrip('/')):<26}  无 odom.csv 或行数不足")
            continue
        print(f"{r['run']:<26}{r['dur']:>7.1f}{r['path']:>8.1f}{r['v_med']:>6.2f}"
              f"{r['d3']:>8.3f}{r['dh']:>7.3f}{r['dv']:>+7.3f}{r['d_pct']:>9.3f}"
              f"{r['rot']:>8.3f}{r['yaw']:>+10.3f}"
              f"{r['zlo']:>+6.2f}..{r['zhi']:>+5.2f}")


if __name__ == "__main__":
    main()
