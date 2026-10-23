#!/usr/bin/env python3
"""两张点云做全局配准，量"同一片静止场景被两次重建"的位姿不一致。

用途
----
把 f1 bag 的「起飞前静置」与「落地后静置」两段分别用 odom 位姿重建（lap_maps.py），
再对这两朵云做 FPFH + RANSAC + ICP。场景是静止的，所以：
    配准结果 ≈ 单位阵 ⇔ 世界系在这 640 s 里既没转也没平移（位姿是自洽的）
    配准的旋转 = 世界系的姿态漂移（水平轴分量即"世界系相对真实地面歪了多少"）
    配准的平移 = 该时刻的绝对位置误差（不再依赖"无人机是否回到原点"这个假设）
它比闭环误差更硬：闭环误差要求"起终点物理重合"，这里只要求"场景没动"。

用法:
  python3 -B exp/cloud_reg.py <src.pcd> <dst.pcd> [--voxel 0.3] [--init-x 0 ...]
  把 src 配到 dst，输出 4x4 变换、旋转轴角、平移。
"""
import argparse
import os

import numpy as np
import open3d as o3d


def load(path, voxel):
    pc = o3d.io.read_point_cloud(path)
    if voxel > 0:
        pc = pc.voxel_down_sample(voxel)
    pc, _ = pc.remove_statistical_outlier(20, 1.5)
    return pc


def prep(pc, voxel):
    pc = o3d.geometry.PointCloud(pc)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(voxel * 4, 40))
    f = o3d.pipelines.registration.compute_fpfh_feature(
        pc, o3d.geometry.KDTreeSearchParamHybrid(voxel * 10, 100))
    return pc, f


def rotvec(R):
    ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = np.linalg.norm(ax)
    return ax / s * ang if s > 1e-12 else np.zeros(3), ang


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--voxel", type=float, default=0.3)
    ap.add_argument("--crop", type=float, default=0.0,
                    help="只保留距世界系原点水平半径 r 内的点（比较局部场景时用，"
                         "避免远处不重叠的几何把 ICP 带偏）")
    ap.add_argument("--init", type=float, nargs=6, default=None,
                    metavar=("x", "y", "z", "rx", "ry", "rz"),
                    help="初始猜测（米 / 度），不给则从单位阵开始")
    a = ap.parse_args()

    s = load(a.src, a.voxel)
    d = load(a.dst, a.voxel)
    if a.crop > 0:
        def crop(pc):
            P = np.asarray(pc.points)
            m = np.hypot(P[:, 0], P[:, 1]) < a.crop
            return pc.select_by_index(np.nonzero(m)[0])
        s, d = crop(s), crop(d)
    print(f"[reg] src {os.path.basename(a.src)} {len(s.points)} 点  "
          f"dst {os.path.basename(a.dst)} {len(d.points)} 点  voxel {a.voxel}")

    init = np.eye(4)
    if a.init:
        r = np.radians(a.init[3:])
        Rx = np.array([[1, 0, 0], [0, np.cos(r[0]), -np.sin(r[0])], [0, np.sin(r[0]), np.cos(r[0])]])
        Ry = np.array([[np.cos(r[1]), 0, np.sin(r[1])], [0, 1, 0], [-np.sin(r[1]), 0, np.cos(r[1])]])
        Rz = np.array([[np.cos(r[2]), -np.sin(r[2]), 0], [np.sin(r[2]), np.cos(r[2]), 0], [0, 0, 1]])
        init[:3, :3] = Rz @ Ry @ Rx
        init[:3, 3] = a.init[:3]

    if np.allclose(init, np.eye(4)):
        # 无先验：先 FPFH 全局配准，给 ICP 一个好起点（两朵云可能相距数米）
        sp, sf = prep(s, a.voxel)
        dp, df = prep(d, a.voxel)
        coarse = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            sp, dp, sf, df, True, a.voxel * 3,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
            [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
             o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(a.voxel * 3)],
            o3d.pipelines.registration.RANSACConvergenceCriteria(400000, 0.999))
        print(f"[reg] RANSAC fitness={coarse.fitness:.3f} rmse={coarse.inlier_rmse:.3f}")
        init = coarse.transformation

    d.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(a.voxel * 4, 40))
    reg = o3d.pipelines.registration.registration_icp(
        s, d, a.voxel * 2.0, init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
    T = reg.transformation
    rv, ang = rotvec(T[:3, :3])
    tilt = np.degrees(np.arccos(np.clip(T[2, 2], -1, 1)))
    print(f"[reg] ICP  fitness={reg.fitness:.3f}  inlier_rmse={reg.inlier_rmse * 100:.1f} cm")
    print(f"[reg] 旋转 {ang:.3f}°  轴角(世界系)=({rv[0]:+.3f},{rv[1]:+.3f},{rv[2]:+.3f})°")
    print(f"[reg] 平移 =({T[0,3]:+.3f},{T[1,3]:+.3f},{T[2,3]:+.3f}) m  "
          f"|水平|={np.hypot(T[0,3],T[1,3]):.3f} m")
    print(f"[reg] ⇒ 世界系竖直偏差 {tilt:.3f}°  (含 yaw 的整体系 {ang:.3f}°)")


if __name__ == "__main__":
    main()
