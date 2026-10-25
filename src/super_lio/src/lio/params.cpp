

#include "lio/params.h"

using namespace std;
using namespace BASIC;

namespace LI2Sup{

  const std::string g_root_dir = std::string(ROOT);
  std::atomic<bool> g_flag_run = true; 
  bool g_flg_map_init = true;

  /// evaluation
  bool g_time_eva = false;

  bool   g_save_map;
  bool   g_if_filter; 
  string g_save_map_dir;
  string g_map_name;
  float  g_map_ds_size;
  int    g_pcd_save_interval;
  string g_pcd_prefix;
  
  string g_imu_topic;
  string g_lidar_topic;
  
  /// QoS settings
  bool g_imu_qos_reliable = true;
  bool g_lidar_qos_reliable = true;

  int    g_lidar_type;
  float  g_blind2;
  float  g_maxrange2;
  int    g_filter_rate;
  int    g_filter_offset = 0;
  bool   g_enable_filter_offset = true;
  bool   g_enable_downsample;
  float  g_voxel_fliter_size;
  bool   g_intensity_filter_en = false;
  float  g_intensity_min = 0.0f;
  int    g_lidar_channels = 0;
  int    g_full_column_interval = 0;

  int    g_imu_type;
  double g_gravity_norm = 9.7946;
  double g_imu_na;
  double g_imu_ng;
  double g_imu_nba;
  double g_imu_nbg = 0.003;   // 陀螺零偏随机游走：实测 bg 一趟可漂 0.025 rad/s，
                              // 1e-4 只允许其 1/500，滤波器跟不上；3e-3 才可自收敛
  double g_imu_gyr_scale = 1.0;
  double g_lidar_time_offset = 0.0;

  SE3 g_lidar_imu;
  SE3 g_odom_robo;
  M3  g_lidar_robo_yaw;

  /// hash_map
  std::size_t g_ivox_capacity = 100000;
  float       g_ivox_resolution = 0.5;

  /// kf
  int g_kf_type = 1;                // 1: ESKF, 2: InESKF
  int g_kf_max_iterations = 4;
  bool g_kf_align_gravity = true;
  bool g_kf_freeze_gravity = false;
  int g_ref_gravity_axis = 2;       // 0: +X, 1: +Y, 2: -Z (default)
  double g_kf_quit_eps;

  /// submap 
  double g_submap_resolution;
  int    g_submap_capacity;

  /// output
  bool g_2_robot    = false;
  bool g_2_plan_env_world = false; 
  bool g_2_plan_env_body  = false;
  bool g_2_ml_map = false;
  bool g_visual_map = true;
  bool g_visual_dense = false;
  bool g_visual_map_body = false;
  bool g_visual_dense_body = false;
  int  g_pub_step;
  bool g_footprint_pub_en = true;
  string g_tf_base_footprint_frame = "base_footprint";
  string g_world_frame = "world";
  string g_imu_frame = "imu";
  string g_map_save_service_topic = "/map_save";

  /// for planner
  bool g_planner_enable;

  /// lio only undistort mode
  bool g_lio_only_undistort = false;

  /// downsample only mode (highest priority)
  bool g_downsample_only = false;

  ResidualType g_residual_type = PROB;

  /// for relocation
  bool g_update_map = false;
  double g_init_px, g_init_py, g_init_pz, g_init_roll, g_init_pitch, g_init_yaw;

  /// observe
  double g_plane_fit_threshold = 0.15;

  double g_obs_weight = 1000.0;
  double g_huber_delta_base = 0.05;
  double g_huber_delta_scale = 0.003;
  double g_ground_weight = 0.0;
  double g_obs_dist_k = 0.0;
  double g_thresh_motion_k = 0.0;
  bool   g_delta_interp_en = false;

  /// 退化检测 + 参数自适应（默认全关 = 原行为）
  bool   g_degeneracy_detect_en = false;
  double g_degeneracy_threshold = 100.0;
  bool   g_degeneracy_truncate_en = false;
  double g_tikhonov_lambda = 1e-3;
  int    g_degen_trans_only = 0;
  double g_degeneracy_hysteresis = 0.5;
  int    g_degeneracy_enter_frames = 3;
  int    g_degeneracy_exit_frames = 10;
  bool   g_adaptive_param_en = false;
  /// 坏帧护栏
  bool   g_frame_gap_guard_en = false;
  double g_frame_gap_max = 0.2;
  double g_imu_cover_max = 0.03;
  bool   g_adaptive_normal_weight_en = true;
  double g_adaptive_normal_sigma = 0.035;
  double g_adaptive_plane_fit = 0.0;
  double g_adaptive_huber = 0.0;

  /// 细通道（M.27，默认全关 = 原行为）
  bool   g_fine_channel_en = false;
  double g_fine_channel_weight = 100.0;
  int    g_fine_channel_max_points = 150;
  double g_fine_max_dist = 0.5;
  double g_fine_channel_min_gain = 1.5;

  /// M.29 地图哈希尖峰治理（默认开：只改桶数量，不改行为）
  bool   g_map_hash_reserve_en = true;
  int    g_map_hash_reserve_size = 0;

  /// 实验埋点 / 代码级实验开关（默认全部保持原行为）
  int    g_debug_level = 0;
  bool   g_normal_weight_en = false;
  double g_normal_sigma = 0.05;
  int    g_map_rep_mode = 0;
  bool   g_map_clamp_mu = false;
  int    g_map_max_points = 20;
  double g_map_merge_dist = 0.1;
  int    g_hknn_max_group = -1;
  double g_hknn_order_scale = 1.0;

  /// 实验 C：固定地图 / 冻结地图更新
  bool   g_map_freeze = false;
  string g_map_load_path;
  bool   g_map_load_use_pose = false;
  double g_map_load_pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  int    g_guard_max_consecutive = 9;

  /// for dynamic point removal
  bool g_dynamic_removal_enable = false;
  int  g_dynamic_removal_method = 0;
  float g_dynamic_removal_grid_size = 0.2f;
  int   g_dynamic_removal_min_neighbors = 2;
  int   g_dynamic_removal_frame_window = 1;
  int   g_dynamic_removal_raycast_min_hits = 2;
  bool  g_dynamic_removal_isolated_removal = true;

  /// for SC-PGO offline processing output
  bool  g_sc_pgo_enable = false;
  float g_sc_pgo_keyframe_gap = 5.0f;
  float g_sc_pgo_keyframe_deg_gap = 10.0f;

  /// Single-core mode
  bool g_single_core = false;

  /// Fast tf
  bool g_fast_tf = false;

}