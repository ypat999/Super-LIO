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

  /// ===== M.29 地图哈希尖峰治理 =====
  /// 构造时对 grids_ 做 reserve：只改桶数量、不改键集与 LRU 淘汰顺序，
  /// 因此位姿输出应逐字节不变；作用是消除扩容期整表重哈希造成的偶发尖峰
  /// （实测 1f UpdateMap 单窗口 max 92 ms，其中 MapInsert 独占 92.07 ms）。
  extern bool   g_map_hash_reserve_en;
  /// reserve 的目标桶数；<=0 表示直接用 g_ivox_capacity
  extern int    g_map_hash_reserve_size;

  /// 平面拟合邻域大小 K（4..8，默认 8）。
  /// KNNHeap 的容量仍是模板参数 8（数组尺寸），这里只控制"填到几个算满"
  /// 以及 getTopK 的提前退出判据，因此 K<8 时数组尾部保持 0 不参与。
  extern int    g_plane_knn_k;

  /// kf
  extern int g_kf_type;            // 1: ESKF, 2: InESKF.
  extern int g_kf_max_iterations;
  extern bool g_kf_align_gravity;
  extern bool g_kf_freeze_gravity;   // true: 冻结重力方向（不把 dx 加到 g 上）
  extern int g_ref_gravity_axis;   // 0: +X, 1: +Y, 2: -Z (default)
  extern double g_kf_quit_eps;
  extern bool g_kf_dynamic_start;  // true: 开机即运动，跳过静态零偏标定（bg0 取 0）

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
  extern bool g_odom_cov_en;   // 是否填充 odom 协方差（默认关，全 0 输出）
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

  /// 旋转通道观测权重放大系数（1.0 = 原行为，只作用于 Hessian 的旋转 3x3 块）。
  /// home3 0.25 实测：走廊段快速转向（陀螺峰值 156°/s）时偏航滑掉 9°，
  /// 随后姿态误差把重力泄漏成水平加速度（拒绝观测 2.4 s 内 |v| 0.8→3.9 m/s）。
  /// 偏航只靠两侧墙约束、权重偏低时最易滑，这里单独放大旋转信息。
  extern double g_obs_rot_weight;

  /// 实验：距离自适应测量方差（对齐官方 FAST-LIO2 的 beam_err/dept_err）：
  /// σ²(r)=dept_err²+(beam_err·r)²，以 r=5m 处标定为 g_obs_weight。
  /// 远处点平面拟合误差随距离增大，常数权重会过度信任 30~150m 点导致逐帧抖动。
  /// g_beam_err=0 时完全退回原常数权重行为；与 2.B 的 g_obs_dist_k 属同一族，勿同开。
  extern double g_beam_err;
  extern double g_dept_err;

  /// Huber 核基础阈值(米)：点到面残差小于此值时使用满权重，超过时线性降权
  /// 应设为 1.5~2 倍的近处(5m)残差标准差
  /// 高精度雷达: 0.05, 低精度雷达: 0.10
  extern double g_huber_delta_base;

  /// Huber 核距离缩放系数(米/米)：阈值随点距离线性增长
  /// delta = g_huber_delta_base + g_huber_delta_scale * point_distance
  /// 远处点测量噪声更大，需要更大的阈值避免误降权
  /// 推荐值: 0.003 (30m处阈值增加0.09m)
  extern double g_huber_delta_scale;

  /// ===== 阶段 2 机制开关（默认全关 = 原行为）=====
  /// 2.A 地面/重力软约束：对 |n·up|>0.95 的地板/天花板点追加"法向对齐重力"的
  /// 旋转残差，权重独立于 obs_weight（0 = 关闭）。这是新信息，不是重加权。
  extern double g_ground_weight;
  /// 2.B 观测噪声按距离建模：w /= (1 + (g_obs_dist_k * range)^2)，0 = 关闭
  extern double g_obs_dist_k;
  /// 2.D 阈值随运动幅度自适应：huber_delta *= (1 + g_thresh_motion_k * |omega|)，0 = 关闭
  extern double g_thresh_motion_k;
  /// 2.E 按点时刻分配 ΔT：ApplyDeltaCorrection 用帧内 3 段插值代替整帧刚性修正
  extern bool   g_delta_interp_en;

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

  /// 退化"响应"之三（M.33）：沿**运动方向**截断。
  /// 依据 drone bag 275 s 漂移：最弱平移方向与速度方向夹角余弦 0.90~1.00，
  /// 但 6 维 cond 只有 10~45、平移块 cond_t 只有 3~6，永远达不到 threshold=100。
  /// 判据改为"运动方向上的约束强度占比" λ_v/λ_max(H_tt)（λ_v = v̂ᵀH_tt v̂），
  /// 低于 ratio 时把该方向的 LiDAR 约束投影掉，交 IMU 预积分接管。
  /// 默认关；关时与既有行为完全一致。
  extern bool   g_degen_motion_dir_en;
  extern double g_degen_motion_dir_ratio;
  extern double g_degen_motion_dir_vmin;

  /// ===== 法向质量加权（实验 1，默认关 = 原行为）=====
  /// 平面拟合残差 max|dist| 越大（平面越"脏"），该点观测权重越低：
  ///   w *= 1 / (1 + (fit_res / g_normal_sigma)^2)
  /// 权重按点缓存在帧内，首轮拟合算出，后续迭代沿用。
  extern bool   g_normal_weight_en;
  extern double g_normal_sigma;

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

  /// ===== 细通道（M.27）：退化帧把"平面拟合拒绝点"以 point-to-point 并入 =====
  /// 背景：墙角/扶手/窗框等细尺度点因 plane_fit 硬阈值被整点丢弃（地图里其实有），
  /// 退化时这些点是粗通道之外唯一的新信息。设计见 M.27：粗通道判退化（去抖状态
  /// degeneracy_active_）→ 细通道把拒绝点沿"到最近地图原始点连线"建伪平面残差
  /// → λ_min 增益门控通过才并入（区分"有扶手可救"与"真空走廊"）。
  /// last_HTVH 保持纯平面 H，退化检测不包含细通道贡献，防自激振荡。
  extern bool   g_fine_channel_en;
  /// 细通道观测权重（独立于 obs_weight；细点噪声大，默认远小于 1000）
  extern double g_fine_channel_weight;
  /// 每帧候选点上限（超出不再收集）
  extern int    g_fine_channel_max_points;
  /// 关联距离上限（米）：到最近地图原始点超过此值不作为候选
  extern double g_fine_max_dist;
  /// λ_min 增益门控阈值：λ_min(H+H_f)/λ_min(H) ≥ 此值才并入（1.0=无增益即弃用）
  extern double g_fine_channel_min_gain;

  /// ===== M.29 地图哈希尖峰治理 =====
  /// 构造时对 grids_ 做 reserve：只改桶数量、不改键集与 LRU 淘汰顺序，
  /// 因此位姿输出应逐字节不变；作用是消除扩容期整表重哈希造成的偶发尖峰
  /// （实测 1f UpdateMap 单窗口 max 92 ms，其中 MapInsert 独占 92.07 ms）。
  extern bool   g_map_hash_reserve_en;
  /// reserve 的目标桶数；<=0 表示直接用 g_ivox_capacity（默认 2e6）
  extern int    g_map_hash_reserve_size;

  /// ===== M.31 场景自动档（启动窗口判定一次，全程固定） =====
  /// 依据 M.30 实测：瞬时/短窗退化指标在两场景间重叠，但"最近 W 秒内
  /// deg_cond > thr 的帧占比"能干净分离（楼梯 cond 是 ~40 s 周期振荡、永不超
  /// 223；1f 走廊持续高位）。实测 W=120 s/thr=150/gate=0.5 时楼梯误判 0.00%、
  /// 1f 覆盖 94.9%。此处只用它**判定一次**，不做逐帧切换——A 线是刀刃平衡，
  /// 反复抖动风险远大于收益。
  extern bool   g_auto_scene_en;
  /// 判定窗口（秒）；窗口未满时不切换
  extern double g_auto_scene_window;
  /// 退化阈值：deg_cond 超过此值记为"高退化帧"
  extern double g_auto_scene_cond_thr;
  /// 门限：窗口内高退化帧占比 > 此值 ⇒ 判为走廊
  extern double g_auto_scene_gate;
  /// 楼梯档（A 线；窗口未满时也用它——误用 B 线在楼梯的代价远大于反之）
  extern double g_auto_scene_stair_na;
  extern bool   g_auto_scene_stair_nw;
  extern double g_auto_scene_stair_nw_sigma;
  /// 走廊档（B 线）：na 与法向权重
  extern double g_auto_scene_corridor_na;
  extern bool   g_auto_scene_corridor_nw;

  /// ===== M.32 在线时延估计（侧车，借鉴 Ultra-Fusion TimeSync + 锁定状态机）=====
  /// 定位：自动诊断器 + 实机保险——回答"lidar↔IMU 时延到底多大"（M.10/5d 的
  /// 扫参只覆盖 ±3 ms 且已证时延不是楼梯 bag 的漂移主因，扫参无法发现更大的
  /// 真实时延；换固件/换雷达后台行为变化时这是最常见的隐性坑）。
  /// 模型：body=雷达系（wrapper 已把 IMU 旋进雷达系）。残差对时延修正 δ 的
  ///       雅可比 c = ∂r/∂δ = nb^T·(ω×p_body + v_body)，即 r(δ) ≈ r + c·δ，
  ///       最小二乘解 δ_est = −Σc·r / Σc²（渐消加权）。
  /// 状态机：ESTIMATING → LOCKED（滑窗极差 < tol 且信息帧够）；
  ///        apply_en 时写入 g_lidar_time_offset_extra（单位延迟反馈到下一帧点时间戳）。
  ///        锁定值默认只打印不应用（Ultra-Fusion apply_locked_result_to_slam
  ///        默认 false 的同款设计：标定结果先人工确认再生效）。
  extern bool   g_calib_td_en;
  /// 渐消遗忘因子（0<λ<1，越小窗口越短；0.995 ≈ 有效窗口 200 帧 ≈ 20 s）
  extern double g_calib_td_lambda;
  /// 帧信息量门限：本帧 RMS(c) 低于 sqrt(此值) 不并入（静止/低速帧 c≈0，
  /// 硬解只有噪声）。单位 (m/s)²，默认 0.01 ⇒ 需 RMS 速度相关项 ≥ 0.1 m/s。
  extern double g_calib_td_cc_gate;
  /// 外点剔除：|r| 超过此值（米）的点不并入（动态物体/错关联）
  extern double g_calib_td_outlier;
  /// 锁定窗口（帧）：最近 N 个信息帧 δ_est 的极差持续小于 tol 则 LOCKED
  extern int    g_calib_td_lock_win;
  /// 锁定极差容限（秒）
  extern double g_calib_td_lock_tol;
  /// 锁定后自动应用（默认 false：只打印 [TdCalib] LOCKED，人工确认后置 true）
  extern bool   g_calib_td_apply_en;
  /// M.32 时延侧车的运行时修正量（锁定且 apply_en 时写入；wrapper 点时间戳
  /// 读取处追加该项。x + 0.0 逐位恒等 ⇒ 默认关闭时不改变任何计算结果，md5 不变）。
  /// 跨线程（process 线程写 / lidar 回调线程读），故用 atomic。
  extern std::atomic<double> g_lidar_time_offset_extra;

  /// ===== 实验埋点 =====
  /// 0=关闭; 1=每帧写 <save_map_dir>/debug.csv; 2=额外计算 fit_res p95 与 cond_yk
  extern int g_debug_level;

  /// P0.1 可观测性诊断的特征长度 L（m）：归一化扰动 δq=[δt, Lδθ] 用，
  /// 使信息矩阵六个特征值统一到"米"量纲。仅 level>=2 的 observability.csv 使用。
  extern double g_debug_feature_length;

  /// 影子模式：按归一化信息谱计算方向权重并落盘，**不接入滤波**（阶段 2 前置验证）
  extern bool   g_shadow_en;
  extern double g_shadow_lambda_ref;  // 权重函数 λ/(λ+λ_ref) 的参考值
  extern double g_shadow_w_min;       // 权重下限
  extern double g_shadow_alpha;       // 帧间低通系数 [0,1]
  extern double g_shadow_max_dw;      // 每帧权重最大变化量（速率限制）

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

  /// 平面拟合邻域大小 K（M.35；4..8，默认 8 = 67c0acd 的现行为）。
  /// 历史上 K 写死在 KNNHeap<K,...> 模板参数里，改一次要重编译一次，
  /// 无法在同一二进制上对照 K=5/K=8 ⇒ 提到运行时。
  /// 堆的**容量**仍是模板参数 8（数组尺寸），这里只控制"填到几个算满"
  /// 以及 getTopK 的提前退出判据，因此 K<8 时数组尾部保持 0 不参与。
  extern int g_plane_knn_k;

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

  /// ===== 断档冻结（IMU 积分 dt 上限）=====
  /// 雷达/IMU 链路瞬时中断（重着陆导致接触不良等）后，接回时第一条 IMU 的 dt
  /// 等于整段断档时长；无上限实现会用「断档前的冲击加速度尖峰」把这段时长积分掉，
  /// 产生米级位置跳变与数 m/s 速度跳变。超过该值的 dt 只对齐时间基准、不积分。
  extern double g_max_imu_dt;

  /// ===== 位姿信任域 / LOST =====
  /// 开启后：单帧修正跳变超过阈值时撤销该次激光修正（保留 IMU 推算），暂停建图，
  /// 但里程计/TF/点云仍按当前位姿全量输出 —— 空中掉线时以飞行连续性优先。
  extern bool   g_trust_guard_en;
  /// 单帧允许的位置/姿态跳变
  extern double g_trust_jump_max;
  extern double g_trust_jump_rot_max;
  /// 状态速度硬门（m/s，0 = 关闭）。单帧跳变门只看单帧位移，挡不住
  /// "每帧 <1 m 的持续拖拽"（≈5 m/s 匀速拖走）；该门在速度层面兜底，
  /// 超限直接进 LOST 由 lost_vel_damp / lost_vel_max 钉住速度。
  extern double g_trust_vel_max;
  /// 地图新点比例门（0 = 关闭）。本帧点云投到 world 后落在"尚无粗体素"的点
  /// 占比超过该值、且本帧修正量超过 *_jump_min，判为错误匹配：
  /// 撤销本帧修正 + 暂停建图（SUSPECT，下一帧自动重评）。
  extern double g_map_novelty_max;
  extern double g_map_novelty_jump_min;
  extern double g_map_novelty_jump_min_rot;
  extern int    g_map_novelty_streak;
  /// 早期漂移前兆门（0 = 关闭）：新点比例只需超过 map_novelty_early 且
  /// IMU 预测速度超过 trust_vel_early(m/s) 连续 novelty_early_streak 帧即判前兆。
  /// 依据 drone bag：健康段（含 40-60 s 大量新覆盖）从未出现"新点比例高 + 速度高"
  /// 同时成立，而 275.3 s 发散前该组合先于 novelty_max 判据约 1 s 出现。
  extern double g_map_novelty_early;
  extern double g_trust_vel_early;
  extern int    g_novelty_early_streak;
  /// 开阔区门控（0 = 关闭）：本帧最远点距离超过 open_guard_max_range (m) 或
  /// 机体系 z 跨度超过 open_guard_z_span (m) 时，新点比例与平移锯齿两条判据
  /// 不生效。home3 430 s 起进入大开阔区：地图无对应结构，novelty 恒高、
  /// 冻结 Anchor 的目标图与新区域零重叠，GICP 无法做绝对校验，判据一旦成立
  /// 就会硬重锚并把位姿钉在 3 s 前，反而制造漂移（476 s |v|>2、497 s 下沉）。
  extern double g_open_guard_max_range;
  extern double g_open_guard_z_span;
  /// LOST 期间重新接受观测所需的单帧修正量上限（影子配准）
  extern double g_trust_corr_max;
  extern double g_trust_corr_rot_max;

  /// ===== 信息量门：H 近奇异时放弃本帧激光更新（只不融合，不冻结位姿）=====
  extern bool   g_info_gate_en;
  extern double g_lam_min;    // 归一化最小特征值下限
  extern double g_cond_max;   // 归一化条件数上限

  /// ===== 重力观测（加速度计约束 roll/pitch，始终启用）=====
  extern bool   g_gravity_obs_en;
  extern double g_acc_max;        // 权重半衰点 m/s^2（按平台配置）
  extern double g_acc_lpf_tau;    // 加速度低通时间常数 s
  extern double g_gravity_sigma;  // 等效观测噪声 m/s^2

  /// ===== 诊断（默认关闭；仅记录，不参与估计）=====
  /// 0=关闭; 2=每帧写 observability.csv（归一化信息矩阵全谱）
  extern int    g_debug_level;
  /// 归一化扰动 δq=[δt, Lδθ] 的特征长度 L（m）：旋转列换算到米制
  extern double g_debug_feature_length;

  /// ===== 影子模式（默认关闭）：只计算并记录方向权重，不接入滤波 =====
  extern bool   g_shadow_en;
  extern double g_shadow_lambda_ref;
  extern double g_shadow_w_min;
  extern double g_shadow_alpha;
  extern double g_shadow_max_dw;
  /// LOST 退出所需的连续合格帧数
  extern int    g_lost_recover_frames;
  /// LOST 期间的限速再锚定步长：每帧最多把位姿往配准结果拉多少
  extern double g_lost_slew_p;
  extern double g_lost_slew_r;
  /// LOST 期间是否继续向 IVox 建图（默认关，防止污染用于配准的地图）
  extern bool   g_lost_map_en;
  /// LOST 退出必须同时满足"本帧确有观测"：有效平面残差点数下限。
  /// 防止 n_effect==0 时 corr_p 恒为 0 被误判成收敛（假收敛）。
  extern int    g_lost_min_effect_pts;
  /// LOST 期间速度衰减系数（1.0 = 不衰减，<1 每帧乘一次）
  extern double g_lost_vel_damp;
  /// LOST 期间速度硬上限（m/s，0 = 关闭）。姿态/零偏误差下加速度计的重力泄漏
  /// 可达 ~5 m/s²，仅靠衰减系数仍会在帧间反弹，需再用上限兜住。
  extern double g_lost_vel_max;
  /// 单帧 LiDAR 姿态修正上限（deg/帧，0 = 关闭）。
  /// 针对"渐进拖歪"失效：单帧跳变门（25 deg）拦不住每帧 1~3 deg 的持续拖拽，
  /// 而 10~20 deg/s 的姿态拖动会把重力方向拖歪、产生虚假水平加速度。
  extern double g_trust_rot_clamp_deg;
  /// 姿态修正净速率门（deg/s，0 = 关闭）：窗口内平均修正速率超限 → SUSPECT。
  /// SUSPECT 只撤销该帧激光修正并暂停写图，不进入 LOST（可自恢复）。
  extern double g_trust_corr_rate_max;
  /// 上述速率统计窗口（帧数）
  extern int    g_trust_corr_rate_window;
  /// 修正方向一致性下限 C_θ = |Σδθ| / (Σ|δθ|+ε)（0 = 不加该条件）。
  /// 健康段修正方向来回摆动 → C_θ 低；拖歪时每帧同向 → C_θ 接近 1。
  extern double g_trust_corr_coh_min;
  /// 升级 LOST 的净速率阈值（deg/s，0 = 只做 SUSPECT、永不升级 LOST）
  extern double g_trust_corr_rate_lost;
  /// 升级 LOST 需连续超阈的累计时长（秒）
  extern double g_trust_corr_lost_hold_s;
  /// 只在该 bag 相对时间之后才允许判 SUSPECT（秒，0 = 不限制）。
  /// 用途：跳过起飞/手持上机段的合法大幅修正，直接验证真实漂移段的接管。
  extern double g_trust_suspect_after_s;
  /// SUSPECT 持续超过该时长仍未恢复，强制接受 Voxmap 观测并恢复建图（秒，0 = 关闭）。
  /// 防止 GICP 无法接管时（锚无重叠）系统永久瘫痪。
  extern double g_trust_suspect_max_s;
  /// LOST 持续超过该时长仍未恢复，强制接受 Voxmap 观测并恢复建图（秒，0 = 关闭）。
  /// 防止 GICP 长期 conv=0（源云与目标无重叠）时系统永久滞留 LOST、停止建图。
  extern double g_trust_lost_max_s;
  /// 超时兜底放行的残差门：观测与当前位姿差超过该值（米/度）时拒绝接受，
  /// 改为清空 IVox + Anchor、以当前 IMU 位姿重锚新建地图段。
  /// 依据：实测把 24.4 m 的单帧跳变直接吃进 ESKF 后速度冲到 125 m/s、轨迹发散。
  extern double g_trust_force_accept_max_p;
  extern double g_trust_force_accept_max_r;
  /// 实验：bag 相对时间超过该值后停止向 IVox 写图（秒，0 = 关闭）。
  /// 用途：判定漂移根因是"地图自污染反馈"还是"纯匹配错误"。
  extern double g_map_freeze_after_s;
  /// 可信 Anchor 延迟提交窗口（秒，0 = 关闭该特性）。
  /// 每帧点云先进入可撤销队列，超过该窗口且期间未触发 SUSPECT/LOST 才提交；
  /// 触发异常时丢弃未提交部分，保证 Anchor 不含漂移期的污染点云。
  extern double g_anchor_delay_commit_s;
  /// 可信 Anchor 保留窗口（秒）：只保留最近这段时间的已提交帧，控制内存。
  extern double g_anchor_window_s;
  /// 可信 Anchor GICP 重定位总开关（默认关）。
  /// 开启后：检测器判 SUSPECT/LOST 时，用「当前帧去畸变机体系点云」对
  /// 「冻结 Anchor 的可信世界系点云」做 GICP，得到独立的 world<-body 位姿观测，
  /// 通过质量门后直接覆盖 ESKF 状态（而不是撤销本次 LiDAR 修正）。
  /// 常态零开销：仅在异常帧才构建目标地图/运行配准。
  extern bool   g_anchor_gicp_en;
  /// 接受一次 GICP 位姿所需的最小内点比例（内点=最近邻距离 < 0.3 m）
  extern double g_gicp_min_inlier_ratio;
  /// 接受一次 GICP 位姿的最大匹配 RMSE（米制，sqrt(均方最近邻距离)）
  extern double g_gicp_max_rmse;
  /// 相对 IMU 预测位姿允许的最大平移/旋转修正（GICP 是绝对观测，修正量过大多半是错解）
  extern double g_gicp_max_dpos;
  extern double g_gicp_max_drot;
  /// GICP 连续失败多少帧后退出接管、回到 Voxmap 观测
  extern int    g_gicp_fail_max;
  /// Anchor 目标地图体素降采样尺寸（米）
  extern double g_gicp_target_voxel;
  /// Anchor 目标地图只保留距最近已提交帧该半径内的点（米），控制目标规模
  extern double g_gicp_target_radius;
  /// 构建目标地图时只取最近这段时间内的已提交帧（秒），0 = 用全部保留窗口
  extern double g_gicp_target_window_s;
  /// 目标地图基准帧抽稀间隔（秒）：窗口内每隔该时长取一帧稠密帧，控制帧数与点数
  extern double g_gicp_target_keyframe_dt;
  /// 目标地图最终点数上限（体素重滤直到满足）
  extern int    g_gicp_target_max_pts;
  /// GICP 降频：接管期每 N 个雷达帧才跑一次配准，其余帧纯 IMU 传播。
  /// 1 = 每帧都跑（旧行为）；实测单帧 200~800 ms，3~4 帧一次才能跟上实时。
  extern int    g_gicp_frame_interval;
  /// 源/目标改用全量稠密去畸变云（而非被 filter_rate 抽稀的 scan_undistort_full_）
  extern bool   g_gicp_dense_en;
  /// 单帧 GICP 源云点数上限：超过则自动加大体素（防止全量云把配准拖到秒级）
  extern int    g_gicp_source_max_pts;
  /// 单帧 Anchor 稠密体副本的点数上限（独立于源云上限）。
  /// 目标图由约 10 帧基准帧拼接，每帧点数直接决定目标密度：
  /// 实测每帧 800 点时目标只有 ~3.6k 点，GICP 内点比例 34 s 内从 0.68 掉到 0.15。
  extern int    g_gicp_anchor_body_max_pts;
  /// ===== 接管期 → 交还 Voxmap =====
  /// 交还要求连续 N 次 GICP 帧同时满足：Voxmap 观测健康（有效点数、修正量小）
  /// 且与本帧 GICP 位姿一致。满足即退出接管、按 Voxmap 观测继续建图。
  extern int    g_gicp_handback_frames;
  extern double g_gicp_handback_corr_p;    // Voxmap 相对 IMU 预测的修正量上限 (m)
  extern double g_gicp_handback_corr_r;    // 同上，姿态 (deg)
  extern double g_gicp_handback_agree_p;   // Voxmap 与 GICP 位姿差上限 (m)
  extern double g_gicp_handback_agree_r;   // 同上，姿态 (deg)

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
