/**
 * @file parameters.hpp
 * @author WangLiansheng (lswang@mail.ecust.edu.cn)
 * @date 2023-03-14
 * @copyright Copyright (c) 2023
 */


#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_


#include <atomic>
#include <string>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "basic/alias.h"
#include "basic/Manifold.h"


namespace LI2Sup{
  
  extern const std::string g_root_dir;
  extern std::atomic<bool> g_flag_run;
  extern bool g_flg_map_init;

  /// evaluation
  extern bool g_time_eva;

  extern bool g_save_map;
  extern bool g_if_filter;
  extern std::string g_map_name;
  extern std::string g_save_map_dir;
  extern float g_map_ds_size;
  extern int   g_pcd_save_interval;
  extern std::string g_pcd_prefix;
  
  extern std::string g_imu_topic;
  extern std::string g_lidar_topic;
  
  /// QoS settings
  extern bool g_imu_qos_reliable;
  extern bool g_lidar_qos_reliable;

  extern int   g_lidar_type;       // 1: mid360, 2: hesai16, 3: velo16, 4: velo32, 5: vel_nclt, 6: ls16 
  extern float g_blind2;
  extern float g_maxrange2;
  extern int   g_filter_rate;
  extern int   g_filter_offset; // 余数偏移，用于下次采样的起始点
  /// 启用对称振荡采样：filter_rate>1 时交替从两端选取偏移，
  /// 覆盖更均匀，避免始终遗漏同一组点
  extern bool  g_enable_filter_offset;
  extern bool  g_enable_downsample;
  extern float g_voxel_fliter_size;
  extern bool  g_intensity_filter_en;
  extern float g_intensity_min;
  extern int   g_lidar_channels;         // multi-line lidar channels (e.g. Airy=96) for full_column_interval
  extern int   g_full_column_interval;   // keep full column every N columns, 0=disabled

  extern int    g_imu_type;
  extern double g_gravity_norm;
  extern double g_imu_na;
  extern double g_imu_ng;
  extern double g_imu_nba;
  extern double g_imu_nbg;
  // 陀螺 scale 修正系数（乘在原始角速度上，默认 1.0）。
  // 螺旋楼梯道每层转 360°，陀螺 scale 误差 δ 会折算成每层恒定 δ×360° 的
  // yaw 偏转（同向同角度），且往返一圈后归零 —— 这是"逐层同向偏转"的系统性
  // 来源，靠配准观测只能部分压住，需要在此显式标定。
  extern double g_imu_gyr_scale;

  // 雷达逐点时间基准偏移（秒，默认 0）。只作用于点的去畸变时刻，不改帧 end_time。
  // 若雷达时间戳相对 IMU 时钟有固定滞后 Δ，去畸变时每帧点云会整体多转 ω·Δ，
  // 配准把整帧云当刚体对齐 ⇒ 位姿每帧偏 ω·Δ，沿螺旋累计成"与转角成正比"的 yaw 偏置。
  extern double g_lidar_time_offset;

  extern BASIC::SE3 g_lidar_imu;      // lidar in imu frame
  extern BASIC::SE3 g_odom_robo;      // lidar in robot frame
  extern BASIC::M3  g_lidar_robo_yaw; // lidar in robot frame rotation only yaw

  /// hash_map
  extern std::size_t g_ivox_capacity;
  extern float       g_ivox_resolution;
  
  /// kf
  extern int g_kf_type;            // 1: ESKF, 2: InESKF.
  extern int g_kf_max_iterations;
  extern bool g_kf_align_gravity;
  extern bool g_kf_freeze_gravity;   // true: 冻结重力方向（不把 dx 加到 g 上）
  extern int g_ref_gravity_axis;   // 0: +X, 1: +Y, 2: -Z (default)
  extern double g_kf_quit_eps;

  /// submaps
  extern double g_submap_resolution;
  extern int    g_submap_capacity;
  
  /// output  
  extern bool g_2_robot;
  extern bool g_2_plan_env_world;
  extern bool g_2_plan_env_body;
  extern bool g_2_ml_map;
  extern bool g_visual_map;
  extern bool g_visual_dense;
  extern bool g_visual_map_body;
  extern bool g_visual_dense_body;
  extern int  g_pub_step;
  extern bool g_footprint_pub_en;
  extern std::string g_tf_base_footprint_frame;
  extern std::string g_world_frame;
  extern std::string g_imu_frame;

  /// map save service topic
  extern std::string g_map_save_service_topic;

  /// for planner
  extern bool g_planner_enable;

  /// lio only undistort mode
  extern bool g_lio_only_undistort;

  /// downsample only mode (highest priority)
  extern bool g_downsample_only;

  /// Define the hybrid residual formulation.
  enum ResidualType{
    PROB = 1,     // Probabilistic residual
    P2P  = 2,     // Point-to-plane residual
    MIX  = 3      // Hybrid residual (probabilistic + point-to-plane)
  };
  extern ResidualType g_residual_type;


  /// for relocation
  extern bool g_update_map;
  extern double g_init_px, g_init_py, g_init_pz, g_init_roll, g_init_pitch, g_init_yaw;

  /// observe
  /// 平面拟合阈值，值越大保留的有效点越多（退化场景下可适当放宽）
  extern double g_plane_fit_threshold;

  /// 观测权重，等价于 1/σ²，其中 σ 为点到面残差的标准差(米)
  /// 高精度雷达(如 Mid-360): 1000 (σ≈0.032m)
  /// 低精度雷达(如 Airy):    200  (σ≈0.071m, 5m处法向偏差±0.1m)
  extern double g_obs_weight;

  /// Huber 核基础阈值(米)：点到面残差小于此值时使用满权重，超过时线性降权
  /// 应设为 1.5~2 倍的近处(5m)残差标准差
  /// 高精度雷达: 0.05, 低精度雷达: 0.10
  extern double g_huber_delta_base;

  /// Huber 核距离缩放系数(米/米)：阈值随点距离线性增长
  /// delta = g_huber_delta_base + g_huber_delta_scale * point_distance
  /// 远处点测量噪声更大，需要更大的阈值避免误降权
  /// 推荐值: 0.003 (30m处阈值增加0.09m)
  extern double g_huber_delta_scale;

  /// ===== 退化检测 + 参数自适应（源自 ros2 分支 degeneracy detection，默认全关 = 原行为）=====
  /// 对观测信息矩阵 HTVH 做特征分解，判定是否存在退化方向（只检测 + 记录，不改行为）
  extern bool   g_degeneracy_detect_en;
  /// 退化判定阈值：cond(HTVH) = max_eig/min_eig 超过此值即认为存在退化方向
  extern double g_degeneracy_threshold;
  /// 退化"响应"之一（ros2 分支原实现）：把退化方向的 LiDAR 约束清零，交 IMU 预积分接管
  extern bool   g_degeneracy_truncate_en;
  /// Tikhonov 地板系数：λ = g_tikhonov_lambda * mean_eig，保证截断后信息矩阵可逆。
  /// 注意 ros2 分支默认 0.1（=均特征值 10%）是语义 bug，正确语义约 1e-3（=0.1%）
  extern double g_tikhonov_lambda;
  /// 退化判据模式（实验 1.1）：0=全 6 维分解（原行为）；1=只对平移 3x3 块分解；
  /// 2=对角预条件后 6 维分解。旋转块与平移块量纲不同，混排会产生假退化。
  extern int    g_degen_trans_only;
  /// 退化状态去抖：cond 在楼梯 bag 上中位 40、p90 138，阈值 100 时逐帧反复翻转
  /// （实测 116 次翻转、中位持续 2 帧）。任何直接吃该信号的响应都会注入不连续权重，
  /// 因此用"滞回 + 最短持续帧数"把状态机稳定下来：
  ///   进入：cond > threshold            连续 g_degen_enter_frames 帧
  ///   退出：cond < threshold/hysteresis 连续 g_degen_exit_frames  帧
  extern double g_degeneracy_hysteresis;
  extern int    g_degeneracy_enter_frames;
  extern int    g_degeneracy_exit_frames;
  /// 退化"响应"之二（新）：检测到退化时把观测参数切到"退化档"，未检出则回静态基线
  extern bool   g_adaptive_param_en;

  /// ===== 坏帧护栏 =====
  /// 实测（楼梯 bag 悬停崩塌）：处理停顿会让 imu_buffer_ 欠载，一帧 100 ms 的雷达
  /// 只有 55 ms 的 IMU 覆盖，去畸变用残缺轨迹把点云扭坏，配准一次跳变 1~3.7 m。
  /// 开该护栏后，帧间隔过大或 IMU 头部覆盖不足的帧不参与配准/建图，只喂 IMU 预测。
  extern bool   g_frame_gap_guard_en;
  /// 相邻雷达帧 end_time 间隔上限（s），超过视为丢帧
  extern double g_frame_gap_max;
  /// 允许的 IMU 头部缺口（s）：meas.imu.front().secs - lidar.start_time
  extern double g_imu_cover_max;
  /// 退化档：强制开启法向质量加权及其 sigma
  extern bool   g_adaptive_normal_weight_en;
  extern double g_adaptive_normal_sigma;
  /// 退化档：plane_fit / huber 覆盖值；<=0 表示退化时沿用静态配置值
  extern double g_adaptive_plane_fit;
  extern double g_adaptive_huber;

  /// ===== 实验埋点 =====
  /// 0=关闭; 1=每帧写 <save_map_dir>/debug.csv; 2=额外计算 fit_res p95 与 cond_yk
  extern int g_debug_level;

  /// 法向质量加权（实验 1）：按平面拟合残差 max|dist| 降权
  extern bool   g_normal_weight_en;
  extern double g_normal_sigma;

  /// 地图端代表点 / 门控（实验 2/5）
  extern int    g_map_rep_mode;      // 0=增量均值(论文), 1=离子体素中心最近的真实点
  extern bool   g_map_clamp_mu;      // 均值更新后夹回子体素包围盒
  extern int    g_map_max_points;    // 每个子体素最多融合点数
  extern double g_map_merge_dist;    // 与 μ 的合并距离门限（m）

  /// HKNN 分组搜索显式化
  extern int    g_hknn_max_group;    // -1=全部 6 组；1..6=只搜前 N 组（有效半径递减）
  extern double g_hknn_order_scale;  // orders_min_dis2 乘子（随 vox_res 缩放用）

  /// 实验 C：固定地图 / 冻结地图更新
  /// freeze=true 时 UpdateMap() 不写 ivox，地图只保留 load_path 载入的先验，
  /// 用于隔离"地图反馈是否放大方向相关误差"（只作机制定位，不作性能评价）
  extern bool   g_map_freeze;
  /// 非空时启动即载入该 pcd 作为地图，跳过 map_init（首帧点云不再入图）
  extern std::string g_map_load_path;
  /// 载入地图时的初始位姿 {x,y,z,roll,pitch,yaw(deg)}；未配置时为单位位姿
  extern bool   g_map_load_use_pose;
  extern double g_map_load_pose[6];
  /// 安全阀：连续跳过达到该帧数后强制放行一帧，避免雷达频率低于阈值时永久跳过
  extern int    g_guard_max_consecutive;

  /// for dynamic point removal
  extern bool g_dynamic_removal_enable;
  extern int  g_dynamic_removal_method;     // 0: Temporal, 1: Raycast
  extern float g_dynamic_removal_grid_size;
  extern int   g_dynamic_removal_min_neighbors;
  extern int   g_dynamic_removal_frame_window;
  extern int   g_dynamic_removal_raycast_min_hits;
  extern bool  g_dynamic_removal_isolated_removal;

  /// for SC-PGO offline processing output
  extern bool  g_sc_pgo_enable;
  extern float g_sc_pgo_keyframe_gap;
  extern float g_sc_pgo_keyframe_deg_gap;

  /// Single-core mode: disables all TBB parallelism in LIO pipeline,
  /// independent threads (OutputThread, SaveThread) remain unaffected.
  extern bool g_single_core;

  /// Fast TF: publish tf (world->imu, world->base_footprint) at IMU frequency
  /// to reduce tf latency for downstream consumers.
  extern bool g_fast_tf;


}

#endif
