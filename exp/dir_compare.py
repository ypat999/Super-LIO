#!/usr/bin/env python3
"""方向对照（实验 A）：同一空间结构在正/反旋转方向下的去畸变点云质量。

动机
----
楼梯间的扭转符号随自身转向翻转。要判断误差最先出现在哪个阶段，先看**去畸变后的
单帧点云**本身是否已经出现与转向相关的厚化或倾斜：
  - 若已出现 => 优先查时序 / 外参 / 运动补偿；
  - 若结构正常 => 才进入"IMU 预测 vs LiDAR 更新"的分离（实验 B）。

做法
----
1. up_world 由 bag 起始静态 IMU 求得：up = R(q0) · up_imu（本机 30 度俯仰安装，
   不能直接用 odom 的 z）；
2. 逐帧读 /livox/lidar，用**原始 IMU 陀螺积分**构造帧内姿态 R(t)：以帧首位姿为锚，
   在 200 Hz 采样间用梯形积分 + 区间内常角速度插值，把每个点按其 offset_time 投到世界系
   （补偿版）。同时生成"整帧统一用帧末位姿"的未补偿版作为对照。
   平移用 odom 线性插值（角运动是帧内拖影的主项）；
3. 每帧在机体系近场环带内拟合主导竖直平面，输出厚度（残差稳健标准差）与倾角
   （平面法向偏离水平），并记录该帧绕 up 的角速度；
4. 按角速度符号 / 幅值分箱比较厚度与倾角，并在固定空间盒内把正转帧与反转帧分别累积，
   对**同一面墙**做二次拟合，直接给出两方向之间的墙面转角差。

用法
----
  source /opt/ros/humble/setup.bash
  python3 -u dir_compare.py --bag /home/ywj/slam_data/livox_record_drone \
      --run results_drone/sl_base_fr6 --window 138,219
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from loop_error import imu_up, load_run, quat_to_rot, rotvec


def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def exp_so3(v):
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    K = skew(v / th)
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * (K @ K)


class RotChain:
    """按 IMU 采样构建的连续姿态链，支持逐点查询。"""

    def __init__(self, t, R, ):
        self.t = t
        self.R = R
        self.d = np.array([rotvec(R[i + 1] @ R[i].T) for i in range(len(R) - 1)])
        self.th = np.linalg.norm(self.d, axis=1)
        self.axis = np.divide(self.d, self.th[:, None],
                              out=np.zeros_like(self.d), where=self.th[:, None] > 1e-12)

    def at(self, tq):
        """返回 (N,3,3)；区间内常角速度插值。"""
        tq = np.atleast_1d(tq)
        i = np.clip(np.searchsorted(self.t, tq) - 1, 0, len(self.t) - 2)
        span = np.maximum(self.t[i + 1] - self.t[i], 1e-9)
        a = np.clip((tq - self.t[i]) / span, 0.0, 1.0)
        out = np.empty((len(tq), 3, 3))
        for iu in np.unique(i):
            m = i == iu
            K = skew(self.axis[iu])
            s = np.sin(self.th[iu] * a[m])[:, None, None]
            c = (1.0 - np.cos(self.th[iu] * a[m]))[:, None, None]
            out[m] = self.R[iu] + s * (K @ self.R[iu]) + c * (K @ K @ self.R[iu])
        return out


def build_imu_chain(t_imu, gyr, bg, t0, t1, R_end, rate_scale=1.0):
    """帧内姿态链：从帧末位姿向帧首反向积分（与节点内部口径一致）。

    节点 odom 的时间戳是 state.timestamp（= 帧内最后一个 IMU 时刻），
    因此 R_end 取 odom 即对应 t1；机体角增量按 R(t_j) = R(t_j+1)·exp(-w·dt) 回推。
    """
    tt = t_imu[(t_imu >= t0) & (t_imu <= t1)]
    tt = np.concatenate(([t0], tt))
    if tt[-1] < t1 - 1e-6:
        tt = np.concatenate((tt, [t1]))
    if len(tt) < 2:
        return RotChain(np.array([t0, t0 + 1e-3]), np.stack([R_end, R_end]))
    mid = 0.5 * (tt[:-1] + tt[1:])
    w = np.stack([np.interp(mid, t_imu, gyr[:, i]) for i in range(3)], axis=1) - bg
    if rate_scale != 1.0:
        w = w * rate_scale
    R = [None] * len(tt)
    R[-1] = R_end
    for j in range(len(tt) - 2, -1, -1):
        R[j] = R[j + 1] @ exp_so3(-w[j] * (tt[j + 1] - tt[j]))
    return RotChain(tt, np.stack(R))


def odom_t_at(t_abs, xyz, tqry):
    tqry = np.atleast_1d(tqry)
    i = np.clip(np.searchsorted(t_abs, tqry) - 1, 0, len(t_abs) - 2)
    span = np.maximum(t_abs[i + 1] - t_abs[i], 1e-9)
    a = np.clip((tqry - t_abs[i]) / span, 0.0, 1.0)
    return xyz[i] + a[:, None] * (xyz[i + 1] - xyz[i])


def _read_bag_raw(bag, windows):
    """返回 (lidar_frames, t_imu, gyr, bag_t0)。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from livox_ros_driver2.msg import CustomMsg
    from sensor_msgs.msg import Imu

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/livox/lidar", "/livox/imu"]))
    t0 = reader.get_metadata().starting_time.nanoseconds * 1e-9
    t_end = max(b for _, b in windows)

    frames, tim, gyr = [], [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic.endswith("/imu"):
            m = deserialize_message(data, Imu)
            ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            tim.append(ts)
            gyr.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
            if ts - t0 > t_end + 1.0:
                break
            continue
        m = deserialize_message(data, CustomMsg)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if st - t0 > t_end + 1.0:
            break
        if not any(a <= st - t0 <= b for a, b in windows):
            continue
        n = len(m.points)
        arr = np.empty((n, 4), dtype=np.float32)
        for k, p in enumerate(m.points):
            arr[k, 0] = p.x
            arr[k, 1] = p.y
            arr[k, 2] = p.z
            arr[k, 3] = p.offset_time
        frames.append((st, arr[:, 3].astype(np.float64) * 1e-9,
                       arr[:, :3].astype(np.float64)))
    return frames, np.array(tim), np.array(gyr), t0


def read_bag(bag, windows, cache_dir=None):
    """带缓存的 read_bag：缓存键由 bag 路径、db 时间戳与窗口共同决定。"""
    import hashlib
    key = None
    if cache_dir:
        db = os.path.join(bag, os.path.basename(bag.rstrip("/")) + "_0.db3")
        try:
            mt = os.path.getmtime(db)
        except OSError:
            mt = 0.0
        raw = f"{bag}|{mt}|{windows}"
        key = hashlib.md5(raw.encode()).hexdigest()[:16]
        os.makedirs(cache_dir, exist_ok=True)
        path = os.path.join(cache_dir, key + ".npz")
        if os.path.exists(path):
            z = np.load(path, allow_pickle=False)
            frames = [(float(s), z[f"o{k}"].astype(np.float64),
                       z[f"p{k}"].astype(np.float64)) for k, s in enumerate(z["st"])]
            return frames, z["tim"], z["gyr"], float(z["t0"])

    frames, tim, gyr, t0 = _read_bag_raw(bag, windows)
    if key:
        d = {"st": np.array([f[0] for f in frames]), "tim": tim, "gyr": gyr,
             "t0": np.array(t0)}
        for k, (_, off, P) in enumerate(frames):
            d[f"o{k}"] = off
            d[f"p{k}"] = P
        np.savez(path, **d)
    return frames, tim, gyr, t0


def fit_vertical_plane(P, h_up, coarse=180, sample=2500, thr=0.06):
    """拟合主导竖直平面：先按水平方位角穷举，再用内点做 3D PCA 精修。"""
    n = len(P)
    if n < 200:
        return None
    e1 = np.array([1.0, 0.0, 0.0])
    e1 = e1 - (e1 @ h_up) * h_up
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(h_up, e1)
    idx = np.random.default_rng(0).choice(n, size=min(sample, n), replace=False)
    Q = P[idx]
    p1, p2 = Q @ e1, Q @ e2
    best = None
    for k in range(coarse):
        th = np.pi * k / coarse
        d = np.cos(th) * p1 + np.sin(th) * p2
        rho = np.median(d)
        cnt = int((np.abs(d - rho) < thr).sum())
        if best is None or cnt > best[0]:
            best = (cnt, th, rho)
    _, th, rho = best
    nrm = np.cos(th) * e1 + np.sin(th) * e2
    m = np.abs(P @ nrm - rho) < thr
    if m.sum() < 150:
        return None
    Q2 = P[m] - P[m].mean(0)
    _, V = np.linalg.eigh(Q2.T @ Q2)
    nn = V[:, 0]
    if nn @ nrm < 0:
        nn = -nn
    dd = P @ nn - (P[m] @ nn).mean()
    inl = np.abs(dd) < thr
    if inl.sum() < 150:
        return None
    # 厚度不做 6 cm 门限截断：在 ±15 cm 窗内取稳健标准差，否则中等拖影会被 gate 削平
    wide = np.abs(dd - np.median(dd[inl])) < 0.15
    th_n = np.degrees(np.arctan2(nn @ e2, nn @ e1)) % 180.0
    return {"theta": float(th_n), "rho": float(np.median(P[inl] @ nn)),
            "n": int(inl.sum()),
            "thick": float(1.4826 * np.median(np.abs(dd[wide] - np.median(dd[inl])))),
            "tilt": float(np.degrees(np.arcsin(min(1.0, abs(nn @ h_up))))),
            "rho_n": float(0.5 * (np.percentile(P[inl] @ nn, 5)
                                  + np.percentile(P[inl] @ nn, 95))),
            "normal": nn}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--run", required=True)
    ap.add_argument("--window", action="append", required=True, help="t0,t1，可重复")
    ap.add_argument("--static", type=float, default=3.0)
    ap.add_argument("--rmin", type=float, default=1.5)
    ap.add_argument("--rmax", type=float, default=7.0)
    ap.add_argument("--hz", type=float, default=2.5)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--box-r", type=float, default=1.5, help="同位置对照的空间盒半径 m")
    ap.add_argument("--wthr", type=float, default=20.0, help="正/反转分组的 |w_up| 阈值 deg/s")
    ap.add_argument("--rate-scale", type=float, default=1.0,
                    help="对照实验：把帧内补偿角速度乘以该系数（正控用，如 1.05）")
    ap.add_argument("--cache-dir", default=os.path.expanduser("~/.cache/slio_dircmp"))
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    windows = [tuple(float(x) for x in w.split(",")) for w in args.window]

    up_imu = imu_up(args.bag, args.static)
    t_abs, xyz, q = load_run(args.run)
    Rq = np.stack([quat_to_rot(qq) for qq in q])
    up = Rq[0] @ up_imu
    print(f"[odom] {len(t_abs)} 帧  up_world={up.round(4)}", flush=True)

    frames, t_imu, gyr, bag_t0 = read_bag(args.bag, windows, args.cache_dir)
    if args.max_frames:
        frames = frames[: args.max_frames]
    print(f"[bag] 窗口内 {len(frames)} 帧, IMU {len(t_imu)} 帧", flush=True)

    bg = gyr[t_imu - t_imu[0] < 3.0].mean(0)
    print(f"[imu] 静态 bg={np.degrees(bg).round(3)} deg/s", flush=True)

    rows = []
    lag = []
    pts = []
    pos = []
    for k, (st, off, P) in enumerate(frames):
        t_end = st + off.max()
        # odom 时间戳 = state.timestamp = 帧内最后一个 IMU 时刻 ≈ 帧末
        i0 = int(np.argmin(np.abs(t_abs - t_end)))
        lag.append(t_abs[i0] - t_end)
        chain = build_imu_chain(t_imu, gyr, bg, st, t_end, Rq[i0], args.rate_scale)
        tp = st + off
        Rp = chain.at(tp)                      # 逐点姿态（补偿）
        tq = odom_t_at(t_abs, xyz, tp)
        world = np.einsum("nij,nj->ni", Rp, P) + tq

        Re = chain.at(np.array([t_end]))[0]            # 帧末位姿（未补偿）
        te = odom_t_at(t_abs, xyz, np.array([t_end]))[0]
        world_naive = P @ Re.T + te

        ps = odom_t_at(t_abs, xyz, np.array([st + 0.5 * off.max()]))[0]
        hs = float(ps @ up)
        rng = np.linalg.norm(world - ps, axis=1)
        dz = np.abs(world @ up - hs)
        sel = (rng > args.rmin) & (rng < args.rmax) & (dz < args.hz)
        if sel.sum() < 300:
            continue
        A = fit_vertical_plane(world[sel], up)
        B = fit_vertical_plane(world_naive[sel], up)
        if A is None or B is None:
            continue
        Rs = chain.at(np.array([st]))[0]               # 帧首位姿
        wup = float(rotvec(Re @ Rs.T) @ up) / max(off.max(), 1e-3)

        # 半帧一致性：同一面墙按 offset_time 前后半分，正确去畸变下两者方位角应一致
        q = world[sel]
        band = np.abs(q @ A["normal"] - A["rho_n"]) < 0.30
        dth = np.nan
        if band.sum() > 400:
            to = off[sel]
            h0 = band & (to <= np.median(to[band]))
            h1 = band & ~h0
            if h0.sum() > 200 and h1.sum() > 200:
                F0 = fit_vertical_plane(q[h0], up)
                F1 = fit_vertical_plane(q[h1], up)
                if F0 is not None and F1 is not None \
                        and abs(F0["rho_n"] - F1["rho_n"]) < 0.10:
                    dth = (F1["theta"] - F0["theta"] + 90.0) % 180.0 - 90.0

        rows.append([st - bag_t0, np.degrees(wup), hs, A["thick"], A["tilt"], A["n"],
                     B["thick"], B["tilt"], B["n"], A["theta"], A["rho"], dth])
        pts.append(world[sel][:: max(1, int(sel.sum() // 2500))])
        pos.append(ps)
        if len(frames) <= 40:
            print(f"  t={st-bag_t0:7.2f} w_up={np.degrees(wup):+7.1f} h={hs:+6.2f} "
                  f"厚 {1e3*A['thick']:5.1f}/{1e3*B['thick']:5.1f}mm "
                  f"倾 {A['tilt']:5.2f}/{B['tilt']:5.2f}deg n={A['n']}/{B['n']}", flush=True)
        if (k + 1) % 200 == 0:
            print(f"  {k+1}/{len(frames)}", flush=True)
    if lag:
        print(f"[odom] 帧末与最近 odom 时间差 中位 {1e3*np.median(lag):.1f} ms  "
              f"p90 {1e3*np.percentile(np.abs(lag), 90):.1f} ms", flush=True)

    a = np.array(rows)
    if len(a) == 0:
        raise SystemExit("没有可用帧")
    out = args.out or os.path.join(args.run, "dir_compare.csv")
    np.savetxt(out, a, delimiter=",", fmt="%.6f",
               header="t,w_up_deg,h,thick_comp_m,tilt_comp_deg,n_comp,"
                      "thick_raw_m,tilt_raw_deg,n_raw,theta_deg,rho_m,dtheta_half_deg")
    print(f"[out] {out}", flush=True)

    print("\n分组中位（厚度 mm / 倾角 deg）")
    print(f"{'组':16s} {'帧':>4s} {'|w|':>6s} {'厚_补偿':>8s} {'厚_未补偿':>9s} "
          f"{'倾角_补偿':>9s} {'倾角_未补偿':>11s} {'比值':>6s} {'半帧Δθ':>8s}")
    for lo, hi in ((0, 5), (5, 20), (20, 50), (50, 120), (120, 1e9)):
        for sgn, lab in ((1, "正转"), (-1, "反转")):
            m = (np.abs(a[:, 1]) >= lo) & (np.abs(a[:, 1]) < hi) & (np.sign(a[:, 1]) == sgn)
            if m.sum() < 10:
                continue
            dh = a[m, 11][~np.isnan(a[m, 11])]
            dhs = f"{np.median(dh):+8.3f}" if len(dh) >= 10 else "       -"
            print(f"{lab+f' {lo}-{hi}':16s} {m.sum():4d} {np.median(np.abs(a[m,1])):6.1f} "
                  f"{1e3*np.median(a[m,3]):8.1f} {1e3*np.median(a[m,6]):9.1f} "
                  f"{np.median(a[m,4]):9.2f} {np.median(a[m,7]):11.2f} "
                  f"{np.median(a[m,3])/max(1e-9,np.median(a[m,6])):6.2f} {dhs}")

    # ---- 同一空间盒内的正/反转对照：先定参考墙，再按墙内切片分组重拟合 ----
    P0 = np.stack(pos)
    ctr = np.median(P0, axis=0)
    inbox = np.linalg.norm(P0 - ctr, axis=1) < args.box_r
    print(f"\n同位置盒 中心[{ctr[0]:+.2f} {ctr[1]:+.2f} {ctr[2]:+.2f}] "
          f"半径 {args.box_r} m  含 {inbox.sum()}/{len(a)} 帧  |w_up|>{args.wthr:.0f} 分组")
    if inbox.sum() < 10:
        return
    Qall = np.vstack([pts[i] for i in np.nonzero(inbox)[0]])
    F0 = fit_vertical_plane(Qall, up, thr=0.15)
    if F0 is None or F0["thick"] > 0.10 or F0["n"] < 0.15 * len(Qall):
        print(f"  参考墙质量不足（厚 {1e3*(F0['thick'] if F0 else np.nan):.0f} mm），"
              f"跳过跨帧对照；以逐帧半帧 Δθ 的符号统计为准")
        return
    slab = np.abs(Qall @ F0["normal"] - F0["rho"]) < 0.15
    print(f"  参考墙 theta {F0['theta']:7.2f}  厚 {1e3*F0['thick']:5.1f} mm  "
          f"墙内点 {slab.sum()}/{len(Qall)}")
    grp = {"正转": inbox & (a[:, 1] > args.wthr),
           "反转": inbox & (a[:, 1] < -args.wthr)}
    fits = {}
    for lab, m in grp.items():
        if m.sum() < 3:
            continue
        Q = np.vstack([pts[i] for i in np.nonzero(m)[0]])
        Q = Q[np.abs(Q @ F0["normal"] - F0["rho"]) < 0.15]
        if len(Q) < 500:
            continue
        F = fit_vertical_plane(Q, up, thr=0.15)
        if F is None:
            continue
        fits[lab] = F
        print(f"  {lab} {m.sum():3d} 帧 {len(Q):6d} 点  "
              f"厚 {1e3*F['thick']:5.1f} mm  倾 {F['tilt']:5.2f} deg  "
              f"theta {F['theta']:7.2f}  rho {F['rho']:+.3f} m")
    if "正转" in fits and "反转" in fits:
        d = (fits["反转"]["theta"] - fits["正转"]["theta"]) % 180.0
        print(f"  → |Δtheta| = {min(d, 180.0 - d):.2f} deg  "
              f"Δ厚 {1e3*(fits['反转']['thick']-fits['正转']['thick']):+.1f} mm  "
              f"Δ倾 {fits['反转']['tilt']-fits['正转']['tilt']:+.2f} deg  "
              f"Δrho {fits['反转']['rho']-fits['正转']['rho']:+.3f} m")


if __name__ == "__main__":
    main()
