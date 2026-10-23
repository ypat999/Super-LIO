#!/usr/bin/env python3
"""姿态估计误差（俯仰/横滚，绕世界 x/y 轴）与"姿态误差→平移误差"的定量核对。

坐标系（2026-10-23 修正）
------------------------
统一到「对齐后」的系：竖直升 = +z，起点朝向的水平投影 = +x（见 align_to_up）。
这样 h = +z 是真高度（旧版本 h=-z 是深度，导致上行/下行标签相反，已改正）。
Super-LIO 世界 z 本来就是竖直的（偏 0.3°）；GenZ-LIO / FAST-LIO 不做重力对齐，
世界 z 偏竖直 28.6°，且对齐后还差 180° 朝向，必须一起补上才能跟 Super-LIO 比。

为什么要换口径
--------------
直接用 `R_t · up_imu` 与世界竖直的夹角，会得到 120~140° 的"倾斜"；
但同一时段的加速度计原始数据显示设备本身就被握成前倾 119°，姿态估计也说 119°
—— 这部分是**真实姿态**（人拿着扫描器下楼的姿势），不是估计误差。

所以改为：把加速度计测到的比力转进世界系，看它偏离 +z 多少：
    a_world(t) = R_t · C · a_imu(t)      （静止时 ≈ +g·z）
    a_hat = a_world/|a_world|
    εx = a_hat_x   εy = a_hat_y          （小角下就是"估计竖直"相对真竖直的倾角）
若估计系绕世界 y 倾了 δ，则真实上升 Δh 在估计系里多出 +x 分量 δ·Δh，故
    dx_pred = ∫ εx dh,  dy_pred = ∫ εy dh
用逐段均值乘该段 Δh 求和，噪声比逐帧积分小得多。

常值外参 C（odom 发布系 ↔ IMU 系）用陀螺标定，不依赖任何“水平”假设：
    ω_pub(t) = rotvec(R_t^T R_{t+1})/dt   ≈  C · ω_imu(t)
    C = Rotation.align_vectors(ω_pub, ω_imu)
陀螺先扣掉起始静止段偏置。

用法
----
  python3 attitude_drift.py --bag /home/ywj/slam_data/livox_record_drone --all
  python3 attitude_drift.py --bag ... --detail sl_dbg2 --xy
  python3 attitude_drift.py --bag ... --align-up --fl  <gz 与 sl 混合对比>
"""
import argparse
import os
import sys

import numpy as np
from scipy.ndimage import median_filter
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from loop_error import imu_up, load_run
from floor_yaw import find_legs, find_plateaus

# imu_up() 返回的其实是「向下」（-a/|a|），所以对齐后 (0,0,-1) = 重力向下，
# 高度轴 H = +z。旧版本把 (0,0,-1) 当“上”，h 实际是深度。
UP = np.array([0.0, 0.0, -1.0])   # 对齐后的「下」
H = np.array([0.0, 0.0, 1.0])     # 对齐后的高度轴


def load_imu(bag):
    """读整包 IMU：返回 (t_s, acc(3,N), gyr(3,N))。acc 单位 g。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu

    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("cdr", "cdr"))
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/livox/imu"]))
    t0 = rd.get_metadata().starting_time.nanoseconds
    T, A, W = [], [], []
    while rd.has_next():
        _, d, s = rd.read_next()
        m = deserialize_message(d, Imu)
        T.append((s - t0) * 1e-9)
        A.append([m.linear_acceleration.x, m.linear_acceleration.y,
                  m.linear_acceleration.z])
        W.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
    return np.array(T), np.array(A).T, np.array(W).T


def up_rotation(up_world):
    """把 up_world 转到 UP=(0,0,-1) 的最小旋转。"""
    # 不能直接 Rotation.align_vectors([UP],[up_world])：单向量对使 Wahba 问题
    # 秩亏（绕公共轴任意），scipy 会给出 151° 之类的解。这里显式取最小旋转。
    u = np.asarray(up_world, float)
    u = u / np.linalg.norm(u)
    ax = np.cross(u, UP)
    s, c = float(np.linalg.norm(ax)), float(u @ UP)
    if s < 1e-9:
        return Rotation.identity() if c > 0 else Rotation.from_rotvec([np.pi, 0.0, 0.0])
    return Rotation.from_rotvec(ax / s * np.arctan2(s, c))


def align_to_up(xyz, quat, up_world):
    """把该 run 的世界系旋到「竖直 = ±z、起点朝向 = +x」。

    GenZ-LIO / FAST-LIO 不做重力对齐（世界 z 沿首帧 IMU z，实测偏 28.6°），
    Super-LIO 首帧对齐（偏 0.3°）。不统一坐标系，逐腿倾角在 gz/fw1 上无意义。
    用最小旋转把 up_world 转到 (0,0,-1) 后，gz 系还会残留 180° 的朝向差
    （实测 gz 的起点 body-x 水平投影指 -x，Super-LIO 指 +x），
    再绕竖轴把它转到 +x，各框架的 倾x/倾y 才可直接比较。
    """
    A0 = up_rotation(up_world)
    A = A0
    q = quat / np.linalg.norm(quat, axis=1, keepdims=True)
    q0 = q[0] / np.linalg.norm(q[0])
    hb = A.apply(Rotation.from_quat(q0).apply([1.0, 0.0, 0.0]))
    hb[2] = 0.0
    n = np.linalg.norm(hb)
    if n > 1e-6:
        th = float(np.arctan2(hb[1], hb[0]))
        Rz = Rotation.from_rotvec([0.0, 0.0, -th])
        A = Rz * A
    return (A.apply(xyz), (A * Rotation.from_quat(q)).as_quat(),
            float(np.degrees(A0.magnitude())))


def analyze(run, imu, wmin=20.0, up_imu=None):
    loaded = load_run(run)
    if loaded is None:
        return None
    t, xyz, quat = loaded
    t = t - t[0]
    if t[-1] < 200.0:
        return None

    align_deg = 0.0
    if up_imu is not None:
        R0 = Rotation.from_quat(quat[0] / np.linalg.norm(quat[0]))
        xyz, quat, align_deg = align_to_up(xyz, quat, R0.apply(up_imu))

    Rq = Rotation.from_quat(quat / np.linalg.norm(quat, axis=1, keepdims=True))
    R = Rq.as_matrix()

    ti, acc, gyr = imu
    bias = gyr[:, ti < 3.0].mean(axis=1)
    w_i = np.stack([np.interp(t, ti, gyr[i]) for i in range(3)], axis=1) - bias
    a_i = np.stack([np.interp(t, ti, acc[i]) for i in range(3)], axis=1)

    # odom 角速度（发布系）
    dt = np.maximum(np.diff(t), 1e-4)
    w_p = (Rq[:-1].inv() * Rq[1:]).as_rotvec() / dt[:, None]
    w_p = np.vstack([w_p, w_p[-1]])
    w_im = w_i[:-1]
    m = np.linalg.norm(w_im, axis=1) > np.radians(wmin)
    C, _ = Rotation.align_vectors(w_p[:-1][m], w_im[m])   # ω_pub ≈ C·ω_imu
    a_pub = C.apply(a_i)
    a_w = np.einsum("nij,nj->ni", R, a_pub)

    h = xyz @ H
    win = max(5, int(round(1.5 / np.median(np.diff(t)))) | 1)
    hs = savgol_filter(median_filter(h, size=win, mode="nearest"),
                       win if win % 2 else win + 1, 2, mode="nearest")
    hdot = np.gradient(hs, t)

    # εx/εy：估计竖直相对真竖直的倾角（deg）。静止时 a_w/|a_w| ≈ +z，
    # 绕 y 倾 δ ⇒ a_hat_x ≈ δ；绕 x 倾 ⇒ a_hat_y ≈ -δ。
    a_hat = a_w / np.maximum(np.linalg.norm(a_w, axis=1), 1e-6)[:, None]
    eps_x = np.degrees(a_hat[:, 0])
    eps_y = np.degrees(a_hat[:, 1])

    # 无偏的姿态一致性检验：只比较“重力方向之间的夹角”与“估计竖直之间的夹角”。
    # 两者都以参考帧为基准，任何常值外参/加速度计零偏都在夹角中抵消。
    meas_up = -a_i / np.maximum(np.linalg.norm(a_i, axis=1), 1e-6)[:, None]
    est_up = np.einsum("nji,j->ni", R, UP)
    meas_up = np.stack([median_filter(meas_up[:, i], size=max(5, int(2.0 / np.median(np.diff(t))) | 1),
                                      mode="nearest") for i in range(3)], axis=1)
    est_up = np.stack([median_filter(est_up[:, i], size=max(5, int(2.0 / np.median(np.diff(t))) | 1),
                                     mode="nearest") for i in range(3)], axis=1)
    ref_m = (t >= 3.0) & (t <= 20.0)
    ref_idx = ref_m
    mu = meas_up[ref_m].mean(0)
    eu = est_up[ref_m].mean(0)
    mu /= np.linalg.norm(mu)
    eu /= np.linalg.norm(eu)
    ang_true = np.degrees(np.arccos(np.clip(meas_up @ mu, -1, 1)))
    ang_est = np.degrees(np.arccos(np.clip(est_up @ eu, -1, 1)))

    m0, m1 = t <= 1.0, (t[-1] - t) <= 1.0
    d = xyz[m1].mean(0) - xyz[m0].mean(0)
    dxyz = np.array([d[0], d[1], d[2]])      # 对齐后世界系就是 x,y,高

    ph = phases(t, h)
    ph_pitch, ph_roll, ph_hd, ph_w = [], [], [], []
    ph_tilt = []          # 每条腿的 (dx/dh, dy/dh) 等效倾角 deg（+ = 升高时往该轴正方向漂）
    ph_fl = []            # 每条腿的 (沿前进方向, 沿右手方向) 等效倾角 deg
    dx_pred = dy_pred = 0.0
    wn = np.degrees(np.linalg.norm(w_i, axis=1))
    X = xyz[:, 0]
    Y = xyz[:, 1]
    for _, i0, i1 in ph:
        if i1 - i0 < 5:
            ph_pitch.append(np.nan)
            ph_roll.append(np.nan)
            ph_hd.append(0.0)
            ph_w.append(np.nan)
            ph_tilt.append((np.nan, np.nan))
            ph_fl.append((np.nan, np.nan))
            continue
        p = float(np.mean(eps_y[i0:i1]))     # 预测 dy/dh
        r = float(np.mean(eps_x[i0:i1]))     # 预测 dx/dh
        dh = float(h[i1] - h[i0])
        ph_pitch.append(p)
        ph_roll.append(r)
        ph_hd.append(dh)
        ph_w.append(float(np.mean(wn[i0:i1])))
        dx_pred += np.radians(r) * dh
        dy_pred += np.radians(p) * dh
        # 该腿内 x(h)、y(h) 的线性斜率 = 竖直位移被搬到水平的比例
        hh = h[i0:i1]
        if hh.max() - hh.min() > 3.0:
            bx = np.polyfit(hh, X[i0:i1], 1)[0]
            by = np.polyfit(hh, Y[i0:i1], 1)[0]
            ph_tilt.append((float(np.degrees(np.arctan(bx))),
                            float(np.degrees(np.arctan(by)))))
        else:
            ph_tilt.append((np.nan, np.nan))
        # 同一斜率，但沿「这条腿的前进方向 / 右手方向」分解：
        # 与走位、世界系 yaw 无关，跨框架可比。
        if hh.max() - hh.min() > 3.0:
            P = xyz[i0:i1, :2] - xyz[i0:i1, :2].mean(0)
            _, _, vt = np.linalg.svd(P, full_matrices=False)
            f = vt[0] / np.linalg.norm(vt[0])
            # SVD 主方向的正负是任意的：用“前 1/3 -> 后 1/3”的位移定朝向
            k3 = max(1, len(P) // 3)
            if f @ (P[-k3:].mean(0) - P[:k3].mean(0)) < 0:
                f = -f
            if np.linalg.norm(P @ f) > 2.0:
                rgt = np.array([f[1], -f[0]])      # 水平右转 90°
                bf = np.polyfit(hh, xyz[i0:i1, :2] @ f, 1)[0]
                bl = np.polyfit(hh, xyz[i0:i1, :2] @ rgt, 1)[0]
                ph_fl.append((float(np.degrees(np.arctan(bf))),
                              float(np.degrees(np.arctan(bl)))))
            else:
                ph_fl.append((np.nan, np.nan))
        else:
            ph_fl.append((np.nan, np.nan))

    return dict(run=os.path.basename(run.rstrip("/")), t=t, xyz=xyz, h=h,
                pitch=eps_y, roll=eps_x, hdot=hdot,
                d=dxyz, dx_pred=float(dx_pred), dy_pred=float(dy_pred),
                phases=ph, ph_pitch=ph_pitch, ph_roll=ph_roll, ph_dh=ph_hd,
                ph_w=ph_w, ph_tilt=ph_tilt, ph_fl=ph_fl,
                meas_up=meas_up, est_up=est_up, ref_idx=ref_idx,
                ang_true=ang_true, ang_est=ang_est, align_deg=align_deg,
                Cangle=float(np.degrees(np.linalg.norm(C.as_rotvec()))))


def phases(t, h):
    out = [(f"plat{k}", a, b) for k, (a, b) in enumerate(find_plateaus(t, h))]
    out += [(("up" if s > 0 else "dn"), a, b) for a, b, s in find_legs(t, h)]
    return sorted(out, key=lambda x: x[1])


def _prep(run, up_imu):
    """读一个 run 并对齐到「竖直=z、起点朝向=+x」，返回 t, xyz, yaw。"""
    t, xyz, q = load_run(run)
    t = t - t[0]
    R0 = Rotation.from_quat(q[0] / np.linalg.norm(q[0]))
    xyz, q, _ = align_to_up(xyz, q, R0.apply(up_imu))
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    R = Rotation.from_quat(q)
    yaw = np.unwrap(R.as_euler("zyx")[:, 0])
    return t, xyz, yaw


def _circ_mean(a):
    a = np.asarray(a, float)
    return float(np.degrees(np.arctan2(np.sin(a).mean(), np.cos(a).mean())))


def _rel_yaw(P0, P1, step, win_s=20.0):
    """滑窗 2D Procrustes：两轨迹同一时间窗内的相对旋转角（deg）。

    比四元数 yaw 稳（不依赖设备姿态，设备前倾 119° 时 body-x 近竖直、极敏感），
    也比瞬时速度方向稳（剪刀梯每层换向 180°，瞬时航向噪声极大）。
    只取窗口内轨迹确实有形状（去中心后半径 > 1 m）的窗口。
    """
    w = int(round(win_s / step))
    hop = max(1, w // 4)
    out = []
    for i in range(0, len(P0) - w + 1, hop):
        a = P0[i:i + w, :2] - P0[i:i + w, :2].mean(0)
        b = P1[i:i + w, :2] - P1[i:i + w, :2].mean(0)
        if np.linalg.norm(a) < 1.0 or np.linalg.norm(b) < 1.0:
            continue
        ang = np.degrees(np.arctan2((a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum(),
                                    (a * b).sum()))
        out.append((0.5 * (i + i + w) * step, ang))
    return np.array(out) if out else np.zeros((0, 2))


def compare(ref, run, up_imu, step=0.1):
    """同一 bag 内按时间直接对齐，量 run 相对 ref 的位移/朝向差随高度的变化。

    两 run 处理的是同一段 bag，时间轴天然对应（各自减首帧时间）。
    逐高度做地图互相关会被窗外远景和相对 yaw 污染，本函数用位姿直接相减：
        d(t) = p_run(t) − p_ref(t)
    再对 d 的 x/y 分量相对 ref 高度做线性拟合，斜率即两图之间的「相对弯腰」。

    yaw 有两个口径：
      - 四元数 yaw：设备前倾 ~119° 时 body-x 接近竖直，对竖直方向误差极敏感；
      - 速度方向 heading：与设备姿态无关，只要两人走的是同一条路就相同。
    两者不一致时以后者为准。
    """
    t0, x0, y0 = _prep(ref, up_imu)
    t1, x1, y1 = _prep(run, up_imu)
    a, b = max(t0[0], t1[0]), min(t0[-1], t1[-1])
    if b - a < 60.0:
        return None
    tg = np.arange(a, b, step)
    P0 = np.stack([np.interp(tg, t0, x0[:, i]) for i in range(3)], axis=1)
    P1 = np.stack([np.interp(tg, t1, x1[:, i]) for i in range(3)], axis=1)
    Y0 = np.interp(tg, t0, y0)
    Y1 = np.interp(tg, t1, y1)
    d = P1 - P0
    dyaw = np.degrees(Y1 - Y0)
    dyaw = (dyaw + 180.0) % 360.0 - 180.0
    ryw = _rel_yaw(P0, P1, step)
    h = P0[:, 2]

    legs = []
    for i0, i1, s in find_legs(tg, h):
        if i1 - i0 < 50:
            continue
        hh = h[i0:i1]
        if hh.max() - hh.min() < 3.0:
            continue
        ta, tb = tg[i0], tg[i1 - 1]
        kx = np.polyfit(hh, d[i0:i1, 0], 1)[0]
        ky = np.polyfit(hh, d[i0:i1, 1], 1)[0]
        mw = (ryw[:, 0] >= ta) & (ryw[:, 0] <= tb)
        legs.append(dict(kind="up" if s > 0 else "dn",
                         t0=float(ta), t1=float(tb),
                         h0=float(hh[0]), h1=float(hh[-1]),
                         dx=float(d[i1 - 1, 0] - d[i0, 0]),
                         dy=float(d[i1 - 1, 1] - d[i0, 1]),
                         tilt=(float(np.degrees(np.arctan(kx))),
                               float(np.degrees(np.arctan(ky))),
                               float(np.degrees(np.arctan(np.hypot(kx, ky))))),
                         dyaw=float(dyaw[i1 - 1] - dyaw[i0]),
                         ryaw=float(np.median(ryw[mw, 1])) if mw.sum() >= 2 else np.nan))
    return dict(run=os.path.basename(run.rstrip("/")), legs=legs,
                d0=d[0], d1=d[-1], dyaw0=float(dyaw[0]), dyaw1=float(dyaw[-1]),
                span=float(b - a))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--detail", default=None)
    ap.add_argument("--xy", action="store_true")
    ap.add_argument("--fl", action="store_true",
                    help="额外打印按「前进/右手」方向分解的等效倾角（跨框架可比）")
    ap.add_argument("--nbin", type=int, default=10)
    ap.add_argument("--wmin", type=float, default=20.0)
    ap.add_argument("--align-up", action="store_true",
                    help="先按 up_world=R(q0)·up_imu 把各 run 旋到竖直=z，再量倾角"
                         "（GenZ/FAST-LIO 世界系偏 28.6°，不对齐则倾角无意义）")
    ap.add_argument("--cmp", default=None,
                    help="以该 run 为参照，按时间对齐后量各 run 相对它的位移/朝向差"
                         "（同一 bag，时间轴天然对应）")
    ap.add_argument("runs", nargs="*")
    a = ap.parse_args()

    print("[imu] 读整包加速度/陀螺 ...", flush=True)
    imu = load_imu(a.bag)
    print(f"[imu] 帧数={len(imu[0])} 时长={imu[0][-1]:.1f}s")

    up_imu = None
    if a.align_up:
        a0 = imu[1][:, imu[0] < 3.0].mean(axis=1)
        up_imu = -a0 / np.linalg.norm(a0)
        print(f"[imu] up_imu={up_imu.round(4)}  |acc|={np.linalg.norm(a0):.3f}")

    runs = list(a.runs)
    if a.all:
        base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results_drone")
        runs += [os.path.join(base, d) for d in sorted(os.listdir(base))
                 if os.path.exists(os.path.join(base, d, "odom.csv"))]

    print(f"\n{'run':<22}{'dx':>8}{'dy':>8}{'dh':>8}{'|dxy|':>8}{'对齐°':>7}"
          f"{'降εy':>8}{'升εy':>8}"
          f"{'降倾x':>9}{'降倾y':>9}{'升倾x':>9}{'升倾y':>9}"
          f"{'模型dx':>9}{'模型dy':>9}")
    rows = []
    for run in runs:
        r = analyze(run, imu, a.wmin, up_imu)
        if r is None:
            print(f"{os.path.basename(run.rstrip('/')):<22}  跳过（无 odom 或过短）")
            continue
        rows.append(r)
        pu = pd_ = np.nan
        tu = td = (np.nan, np.nan)
        for k, (nm, _, _) in enumerate(r["phases"]):
            if nm == "up":
                pu, tu = r["ph_pitch"][k], r["ph_tilt"][k]
            elif nm == "dn":
                pd_, td = r["ph_pitch"][k], r["ph_tilt"][k]
        print(f"{r['run']:<22}{r['d'][0]:>+8.3f}{r['d'][1]:>+8.3f}{r['d'][2]:>+8.3f}"
              f"{np.hypot(r['d'][0], r['d'][1]):>8.3f}{r['align_deg']:>7.1f}"
              f"{pd_:>+8.2f}{pu:>+8.2f}"
              f"{td[0]:>+9.2f}{td[1]:>+9.2f}{tu[0]:>+9.2f}{tu[1]:>+9.2f}"
              f"{r['dx_pred']:>+9.3f}{r['dy_pred']:>+9.3f}")

    if a.fl:
        print("\n沿「前进方向(f)/右手方向(r)」分解的等效倾角（与走位、世界系 yaw 无关）")
        print(f"{'run':<22}{'降f':>8}{'降r':>8}{'降|倾|':>9}"
              f"{'升f':>8}{'升r':>8}{'升|倾|':>9}{'对齐°':>7}")
        for r in rows:
            u = d = (np.nan, np.nan)
            for k, (nm, _, _) in enumerate(r["phases"]):
                if nm == "up":
                    u = r["ph_fl"][k]
                elif nm == "dn":
                    d = r["ph_fl"][k]
            print(f"{r['run']:<22}{d[0]:>+8.2f}{d[1]:>+8.2f}"
                  f"{np.hypot(*d):>9.2f}{u[0]:>+8.2f}{u[1]:>+8.2f}"
                  f"{np.hypot(*u):>9.2f}{r['align_deg']:>7.1f}")

    if a.cmp:
        if up_imu is None:
            print("\n[--cmp] 需要 --align-up（否则各框架世界系不正交于重力）")
            return
        print(f"\n相对参照 {os.path.basename(a.cmp.rstrip('/'))} 的位移/朝向差"
              f"（同一 bag 按时间对齐；d = 本 run − 参照）")
        print(f"{'run':<20}{'d起点':>16}{'d终点':>18}{'总Δyaw':>8}")
        for run in runs:
            c = compare(a.cmp, run, up_imu)
            if c is None:
                print(f"{os.path.basename(run.rstrip('/')):<20} 跳过")
                continue
            s0 = f"({c['d0'][0]:+.2f},{c['d0'][1]:+.2f})"
            s1 = f"({c['d1'][0]:+.2f},{c['d1'][1]:+.2f})"
            print(f"{c['run']:<20}{s0:>16}{s1:>18}{c['dyaw1']:>+8.2f}")
            for L in c["legs"]:
                print(f"  {L['kind']:<4}t {L['t0']:>6.1f}-{L['t1']:<6.1f}"
                      f" h {L['h0']:>6.1f}->{L['h1']:<6.1f}"
                      f" Δd=({L['dx']:>+6.2f},{L['dy']:>+6.2f})"
                      f"  相对倾 x={L['tilt'][0]:>+6.2f} y={L['tilt'][1]:>+6.2f}"
                      f" |{L['tilt'][2]:>5.2f}|  Δyaw={L['dyaw']:>+6.2f}"
                      f"  Procrustes={L['ryaw']:>+6.2f}")

    if a.detail:
        for r in rows:
            if r["run"] == a.detail:
                detail(r, a.nbin, a.xy)
                break


def detail(r, nbin, xy):
    t, h = r["t"], r["h"]
    print(f"\n== {r['run']}  闭环 dx={r['d'][0]:+.3f} dy={r['d'][1]:+.3f} "
          f"dh={r['d'][2]:+.3f}  模型 dx={r['dx_pred']:+.3f} dy={r['dy_pred']:+.3f}")
    print(f"{'phase':<8}{'t0':>7}{'t1':>7}{'h0':>7}{'h1':>7}"
          f"{'εy均':>8}{'εx均':>8}{'εy段内p10':>12}{'p90':>8}")
    for (nm, i0, i1), p, rr in zip(r["phases"], r["ph_pitch"], r["ph_roll"]):
        seg = r["pitch"][i0:i1]
        print(f"{nm:<8}{t[i0]:>7.1f}{t[i1]:>7.1f}{h[i0]:>7.1f}{h[i1]:>7.1f}"
              f"{p:>+8.2f}{rr:>+8.2f}"
              f"{np.percentile(seg, 10):>+12.2f}{np.percentile(seg, 90):>+8.2f}")

    print("  时间剖面（每 10 s，εx/εy 为瞬时值，别当误差看）")
    print(f"  {'t':>6}{'h':>7}{'εy':>8}{'εx':>8}")
    k = 0
    for tt in np.arange(t[0], t[-1] + 1e-9, 10.0):
        while k + 1 < len(t) and t[k] < tt:
            k += 1
        print(f"  {t[k]:>6.1f}{h[k]:>7.1f}{r['pitch'][k]:>+8.2f}{r['roll'][k]:>+8.2f}")

    # 无偏检验：相对起始段(3~20 s)，真实转了多大 vs 估计转了多大
    print("  姿态一致性（相对起始静止段；夹角差 = 估计把姿态多转/少转的量）")
    print(f"  {'t':>6}{'h':>7}{'真实角':>8}{'估计角':>8}{'差°':>7}")
    k = 0
    for tt in np.arange(3.0, t[-1] + 1e-9, 10.0):
        while k + 1 < len(t) and t[k] < tt:
            k += 1
        d = r["ang_est"][k] - r["ang_true"][k]
        print(f"  {t[k]:>6.1f}{h[k]:>7.1f}{r['ang_true'][k]:>8.2f}"
              f"{r['ang_est'][k]:>8.2f}{d:>+7.2f}")

    if not xy:
        return
    print("  分高度层 xy（dx/dy 相对该腿最下层；+ = 升高时往 x+/y+ 漂）")
    for i0, i1, s in find_legs(t, h):
        hh = h[i0:i1]
        ee = r["xyz"][i0:i1, 0]
        nn = r["xyz"][i0:i1, 1]
        edges = np.linspace(hh.min(), hh.max(), nbin + 1)
        print(f"  leg {'升' if s > 0 else '降'} h {hh.min():.1f}->{hh.max():.1f}"
              f"  t {t[i0]:.1f}-{t[i1]:.1f}")
        print(f"    {'h中':>7}{'x':>9}{'y':>9}{'dx':>8}{'dy':>8}")
        x0 = y0 = None
        for i in range(nbin):
            msk = (hh >= edges[i]) & (hh < edges[i + 1])
            if msk.sum() < 3:
                continue
            x, y = ee[msk].mean(), nn[msk].mean()
            if x0 is None:
                x0, y0 = x, y
            print(f"    {(edges[i]+edges[i+1])/2:>7.1f}{x:>9.3f}{y:>9.3f}"
                  f"{x-x0:>+8.3f}{y-y0:>+8.3f}")


if __name__ == "__main__":
    main()
