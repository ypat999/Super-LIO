

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
  double g_imu_nbg;

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

  /// 使用本地时钟替代话题原始时间戳
  bool g_use_local_timestamp = true;

}