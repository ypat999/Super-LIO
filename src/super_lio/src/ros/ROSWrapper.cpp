
#include "ros/ROSWrapper.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "lio/super_lio.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <thread>
#include <chrono>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

#ifdef LIVOX_SUPPORT
#include "livox_ros_driver2/msg/custom_msg.hpp"
#endif


using namespace BASIC;

namespace LI2Sup{

namespace {
/// 发布节流开关（默认值与历史行为完全一致，不影响 bag / 离线复现）。
/// 实时部署时这些 DDS 发布是常驻负载：200 Hz 的 imu odom + 2 条 TF、10 Hz 的
/// 全量/世界系点云，在 RK3588 这类平台上会直接抢走 executor 的处理时间，
/// 表现为图像/点云在 DDS 侧被覆盖（下游只收到一半）。
int g_imu_odom_hz = 200;             ///< 高频 imu odom/TF 发布频率，0 = 关闭
bool g_pub_cloud_body = true;        ///< /lio/cloud_body（抽稀点云）
bool g_pub_cloud_world = true;       ///< /lio/cloud_world（世界系点云）
bool g_pub_cloud_body_dense = true;  ///< /lio/cloud_body_dense（全量点云）
double g_imu_odom_last_sec = 0.0;    ///< 上次入队时刻（消息时间）

/// 实时性保护：lidar_buffer_ 允许积压的帧数上限（0 = 不限制，保持历史 FIFO 行为）。
/// LIO 单帧耗时超过雷达帧间隔时（RK3588 实测约 141 ms vs 100 ms），FIFO 会让
/// 输出时间戳无界滞后（实测 70 s 累积到 10 s），彩色支路随即全部配不上图。
int g_max_lidar_backlog = 2;

/// 按频率决定本帧 IMU 是否要发高频 odom/TF。
bool shouldPublishImuOdom(double secs) {
  if (g_imu_odom_hz <= 0) {
    return false;
  }
  const double period = 1.0 / static_cast<double>(g_imu_odom_hz);
  if (g_imu_odom_last_sec <= 0.0 || secs - g_imu_odom_last_sec >= period - 1e-4) {
    g_imu_odom_last_sec = secs;
    return true;
  }
  return false;
}
}  // namespace

void LoadParamFromRos(rclcpp::Node& node)
{
  // ---- 发布节流（实时部署用；默认保持历史行为） ----
  node.declare_parameter<int>("lio.publish.imu_odom_hz", 200);
  node.get_parameter("lio.publish.imu_odom_hz", g_imu_odom_hz);
  node.declare_parameter<bool>("lio.publish.cloud_body", true);
  node.get_parameter("lio.publish.cloud_body", g_pub_cloud_body);
  node.declare_parameter<bool>("lio.publish.cloud_world", true);
  node.get_parameter("lio.publish.cloud_world", g_pub_cloud_world);
  node.declare_parameter<bool>("lio.publish.cloud_body_dense", true);
  node.get_parameter("lio.publish.cloud_body_dense", g_pub_cloud_body_dense);
  LOG(INFO) << GREEN << " ---> [Param] publish: imu_odom_hz=" << g_imu_odom_hz
            << " cloud_body=" << g_pub_cloud_body << " cloud_world=" << g_pub_cloud_world
            << " cloud_body_dense=" << g_pub_cloud_body_dense << RESET;

  node.declare_parameter<int>("lio.ros.max_lidar_backlog", 2);
  node.get_parameter("lio.ros.max_lidar_backlog", g_max_lidar_backlog);
  LOG(INFO) << GREEN << " ---> [Param] ros.max_lidar_backlog=" << g_max_lidar_backlog
            << " (0=不限制)" << RESET;

  node.declare_parameter<bool>("lio.map.save_map", false);
  node.get_parameter("lio.map.save_map", g_save_map);

  LOG(INFO) << GREEN << " ---> [Param] map/save_map: "
            << (g_save_map ? "true" : "false") << RESET;

  node.declare_parameter<bool>("lio.eva.timer", false);
  node.get_parameter("lio.eva.timer", g_time_eva);

  node.declare_parameter<bool>("lio.map.if_filter", false);
  node.get_parameter("lio.map.if_filter", g_if_filter);

  node.declare_parameter<std::string>("lio.map.save_map_dir", "");
  node.get_parameter("lio.map.save_map_dir", g_save_map_dir);
  // 支持 "~/..." 与 "$HOME/..."：配置随机器走，不再写死 /home/<user>
  if (!g_save_map_dir.empty()) {
    const char* home = std::getenv("HOME");
    if (home != nullptr) {
      const std::string h(home);
      if (g_save_map_dir == "~") {
        g_save_map_dir = h;
      } else if (g_save_map_dir.rfind("~/", 0) == 0) {
        g_save_map_dir = h + g_save_map_dir.substr(1);
      } else if (g_save_map_dir.rfind("$HOME/", 0) == 0) {
        g_save_map_dir = h + g_save_map_dir.substr(5);
      }
    }
  }
  LOG(INFO) << "[Param] map.save_map_dir: '" << g_save_map_dir << "'";

  node.declare_parameter<std::string>("lio.map.map_name", "default");
  node.get_parameter("lio.map.map_name", g_map_name);

  node.declare_parameter<double>("lio.map.ds_size", 0.5);
  node.get_parameter("lio.map.ds_size", g_map_ds_size);

  node.declare_parameter<int>("lio.map.save_interval", 1);
  node.get_parameter("lio.map.save_interval", g_pcd_save_interval);

  node.declare_parameter<std::string>("lio.map.pcd_prefix", "");
  node.get_parameter("lio.map.pcd_prefix", g_pcd_prefix);

  // 实验 C：固定地图 / 冻结地图更新（默认关闭，保持原行为）
  node.declare_parameter<bool>("lio.map.freeze", false);
  node.get_parameter("lio.map.freeze", g_map_freeze);
  // 实验：bag 相对时间超过该值后停止写图（0 = 关闭），用于判定地图自污染
  node.declare_parameter<double>("lio.map.freeze_after_s", 0.0);
  node.get_parameter("lio.map.freeze_after_s", g_map_freeze_after_s);
  // 可信 Anchor 延迟提交（0 = 关闭）；见 params.h
  node.declare_parameter<double>("lio.anchor.delay_commit_s", 0.0);
  node.get_parameter("lio.anchor.delay_commit_s", g_anchor_delay_commit_s);
  node.declare_parameter<double>("lio.anchor.window_s", 40.0);
  node.get_parameter("lio.anchor.window_s", g_anchor_window_s);
  // 可信 Anchor GICP 重定位（默认关）：异常时用冻结 Anchor 产生第二套位姿观测
  node.declare_parameter<bool>("lio.anchor.gicp_en", false);
  node.get_parameter("lio.anchor.gicp_en", g_anchor_gicp_en);
  node.declare_parameter<double>("lio.anchor.gicp_min_inlier_ratio", 0.5);
  node.get_parameter("lio.anchor.gicp_min_inlier_ratio", g_gicp_min_inlier_ratio);
  node.declare_parameter<double>("lio.anchor.gicp_max_rmse", 0.15);
  node.get_parameter("lio.anchor.gicp_max_rmse", g_gicp_max_rmse);
  node.declare_parameter<double>("lio.anchor.gicp_max_dpos", 1.5);
  node.get_parameter("lio.anchor.gicp_max_dpos", g_gicp_max_dpos);
  node.declare_parameter<double>("lio.anchor.gicp_max_drot", 15.0);
  node.get_parameter("lio.anchor.gicp_max_drot", g_gicp_max_drot);
  node.declare_parameter<int>("lio.anchor.gicp_fail_max", 3);
  node.get_parameter("lio.anchor.gicp_fail_max", g_gicp_fail_max);
  node.declare_parameter<double>("lio.anchor.gicp_target_voxel", 0.20);
  node.get_parameter("lio.anchor.gicp_target_voxel", g_gicp_target_voxel);
  node.declare_parameter<double>("lio.anchor.gicp_target_radius", 30.0);
  node.get_parameter("lio.anchor.gicp_target_radius", g_gicp_target_radius);
  node.declare_parameter<double>("lio.anchor.gicp_target_window_s", 10.0);
  node.get_parameter("lio.anchor.gicp_target_window_s", g_gicp_target_window_s);
  node.declare_parameter<double>("lio.anchor.gicp_target_keyframe_dt", 1.0);
  node.get_parameter("lio.anchor.gicp_target_keyframe_dt", g_gicp_target_keyframe_dt);
  node.declare_parameter<int>("lio.anchor.gicp_target_max_pts", 40000);
  node.get_parameter("lio.anchor.gicp_target_max_pts", g_gicp_target_max_pts);
  // GICP 降频 + 稠密源云 + 接管期交还 Voxmap
  node.declare_parameter<int>("lio.anchor.gicp_frame_interval", 3);
  node.get_parameter("lio.anchor.gicp_frame_interval", g_gicp_frame_interval);
  node.declare_parameter<bool>("lio.anchor.gicp_dense_en", true);
  node.get_parameter("lio.anchor.gicp_dense_en", g_gicp_dense_en);
  node.declare_parameter<int>("lio.anchor.gicp_source_max_pts", 3000);
  node.get_parameter("lio.anchor.gicp_source_max_pts", g_gicp_source_max_pts);
  node.declare_parameter<int>("lio.anchor.gicp_body_max_pts", 3000);
  node.get_parameter("lio.anchor.gicp_body_max_pts", g_gicp_anchor_body_max_pts);
  node.declare_parameter<int>("lio.anchor.gicp_handback_frames", 10);
  node.get_parameter("lio.anchor.gicp_handback_frames", g_gicp_handback_frames);
  node.declare_parameter<double>("lio.anchor.gicp_handback_corr_p", 0.15);
  node.get_parameter("lio.anchor.gicp_handback_corr_p", g_gicp_handback_corr_p);
  node.declare_parameter<double>("lio.anchor.gicp_handback_corr_r", 1.5);
  node.get_parameter("lio.anchor.gicp_handback_corr_r", g_gicp_handback_corr_r);
  node.declare_parameter<double>("lio.anchor.gicp_handback_agree_p", 0.30);
  node.get_parameter("lio.anchor.gicp_handback_agree_p", g_gicp_handback_agree_p);
  node.declare_parameter<double>("lio.anchor.gicp_handback_agree_r", 5.0);
  node.get_parameter("lio.anchor.gicp_handback_agree_r", g_gicp_handback_agree_r);
  node.declare_parameter<std::string>("lio.map.load_path", "");
  node.get_parameter("lio.map.load_path", g_map_load_path);
  std::vector<double> map_init_pose;
  node.declare_parameter<std::vector<double>>("lio.map.init_pose", std::vector<double>{});
  node.get_parameter("lio.map.init_pose", map_init_pose);
  if (map_init_pose.size() == 6) {
    g_map_load_use_pose = true;
    for (int i = 0; i < 6; ++i) g_map_load_pose[i] = map_init_pose[i];
  }
  LOG(INFO) << GREEN << " ---> [Param] map/freeze: " << g_map_freeze
            << "  load_path: " << g_map_load_path
            << "  init_pose: " << (g_map_load_use_pose ? "set" : "identity") << RESET;

  LOG(INFO) << GREEN << " ---> [Param] map/pcd_prefix: "
            << g_pcd_prefix << RESET;

  LOG(INFO) << GREEN << " ---> [Param] anchor: delay_commit=" << g_anchor_delay_commit_s
            << " s window=" << g_anchor_window_s
            << " s gicp=" << g_anchor_gicp_en
            << " (inlier>=" << g_gicp_min_inlier_ratio
            << " rmse<=" << g_gicp_max_rmse << " m dpos<=" << g_gicp_max_dpos
            << " m drot<=" << g_gicp_max_drot << " deg fail_max=" << g_gicp_fail_max
            << " target_voxel=" << g_gicp_target_voxel
            << " radius=" << g_gicp_target_radius << " m"
            << " target_win=" << g_gicp_target_window_s << " s/"
            << g_gicp_target_keyframe_dt << " s dt, max="
            << g_gicp_target_max_pts
            << " dense=" << g_gicp_dense_en
            << " max_pts=" << g_gicp_source_max_pts
            << " body_max=" << g_gicp_anchor_body_max_pts
            << " interval=" << g_gicp_frame_interval
            << " handback=" << g_gicp_handback_frames
            << " frames/" << g_gicp_handback_corr_p << " m/"
            << g_gicp_handback_corr_r << " deg)" << RESET;

  node.declare_parameter<std::string>("lio.ros.lidar_topic", "/lidar");
  node.get_parameter("lio.ros.lidar_topic", g_lidar_topic);

  node.declare_parameter<std::string>("lio.ros.imu_topic", "/imu");
  node.get_parameter("lio.ros.imu_topic", g_imu_topic);

  node.declare_parameter<bool>("lio.ros.imu_qos_reliable", false);
  node.get_parameter("lio.ros.imu_qos_reliable", g_imu_qos_reliable);

  node.declare_parameter<bool>("lio.ros.lidar_qos_reliable", false);
  node.get_parameter("lio.ros.lidar_qos_reliable", g_lidar_qos_reliable);

  LOG(INFO) << GREEN << " ---> [Param] ros/imu_qos_reliable: "
            << (g_imu_qos_reliable ? "true" : "false") << RESET;

  LOG(INFO) << GREEN << " ---> [Param] ros/lidar_qos_reliable: "
            << (g_lidar_qos_reliable ? "true" : "false") << RESET;

  node.declare_parameter<std::string>("lio.ros.map_save_service_topic", "/map_save");
  node.get_parameter("lio.ros.map_save_service_topic", g_map_save_service_topic);

  node.declare_parameter<int>("lio.sensor.lidar_type", 0);
  node.get_parameter("lio.sensor.lidar_type", g_lidar_type);

  double temp_range_dis;
  node.declare_parameter<double>("lio.sensor.blind", 0.0);
  node.get_parameter("lio.sensor.blind", temp_range_dis);
  g_blind2 = temp_range_dis * temp_range_dis;

  node.declare_parameter<double>("lio.sensor.maxrange", 100.0);
  node.get_parameter("lio.sensor.maxrange", temp_range_dis);
  g_maxrange2 = temp_range_dis * temp_range_dis;

  node.declare_parameter<int>("lio.sensor.filter_rate", 1);
  node.get_parameter("lio.sensor.filter_rate", g_filter_rate);

  node.declare_parameter<int>("lio.debug.level", 0);
  node.get_parameter("lio.debug.level", g_debug_level);
  node.declare_parameter<double>("lio.debug.feature_length", 1.0);
  node.get_parameter("lio.debug.feature_length", g_debug_feature_length);

  node.declare_parameter<bool>("lio.observe.info_gate_en", false);
  node.get_parameter("lio.observe.info_gate_en", g_info_gate_en);
  node.declare_parameter<double>("lio.observe.lam_min", 1.0e3);
  node.get_parameter("lio.observe.lam_min", g_lam_min);
  node.declare_parameter<double>("lio.observe.cond_max", 10000.0);
  node.get_parameter("lio.observe.cond_max", g_cond_max);

  node.declare_parameter<bool>("lio.imu.gravity_obs_en", false);
  node.get_parameter("lio.imu.gravity_obs_en", g_gravity_obs_en);
  node.declare_parameter<double>("lio.imu.acc_max", 2.5);
  node.get_parameter("lio.imu.acc_max", g_acc_max);
  node.declare_parameter<double>("lio.imu.acc_lpf_tau", 0.08);
  node.get_parameter("lio.imu.acc_lpf_tau", g_acc_lpf_tau);
  node.declare_parameter<double>("lio.imu.gravity_sigma", 0.5);
  node.get_parameter("lio.imu.gravity_sigma", g_gravity_sigma);

  node.declare_parameter<bool>("lio.shadow.enable", false);
  node.get_parameter("lio.shadow.enable", g_shadow_en);
  node.declare_parameter<double>("lio.shadow.lambda_ref", 1.0e4);
  node.get_parameter("lio.shadow.lambda_ref", g_shadow_lambda_ref);
  node.declare_parameter<double>("lio.shadow.w_min", 0.05);
  node.get_parameter("lio.shadow.w_min", g_shadow_w_min);
  node.declare_parameter<double>("lio.shadow.alpha", 0.3);
  node.get_parameter("lio.shadow.alpha", g_shadow_alpha);
  node.declare_parameter<double>("lio.shadow.max_dw", 0.25);
  node.get_parameter("lio.shadow.max_dw", g_shadow_max_dw);

  node.declare_parameter<bool>("lio.sensor.enable_filter_offset", true);
  node.get_parameter("lio.sensor.enable_filter_offset", g_enable_filter_offset);

  node.declare_parameter<bool>("lio.sensor.enable_downsample", false);
  node.get_parameter("lio.sensor.enable_downsample", g_enable_downsample);

  node.declare_parameter<double>("lio.sensor.voxel_fliter_size", 0.2);
  node.get_parameter("lio.sensor.voxel_fliter_size", g_voxel_fliter_size);

  node.declare_parameter<bool>("lio.sensor.intensity_filter_en", false);
  node.get_parameter("lio.sensor.intensity_filter_en", g_intensity_filter_en);

  node.declare_parameter<double>("lio.sensor.intensity_min", 0.0);
  node.get_parameter("lio.sensor.intensity_min", g_intensity_min);

  node.declare_parameter<int>("lio.sensor.lidar_channels", 0);
  node.get_parameter("lio.sensor.lidar_channels", g_lidar_channels);

  node.declare_parameter<int>("lio.sensor.full_column_interval", 0);
  node.get_parameter("lio.sensor.full_column_interval", g_full_column_interval);

  node.declare_parameter<double>("lio.sensor.gravity_norm", 9.81);
  node.get_parameter("lio.sensor.gravity_norm", g_gravity_norm);

  node.declare_parameter<int>("lio.sensor.imu_type", 0);
  node.get_parameter("lio.sensor.imu_type", g_imu_type);

  node.declare_parameter<double>("lio.sensor.imu_na", 0.0);
  node.get_parameter("lio.sensor.imu_na", g_imu_na);

  node.declare_parameter<double>("lio.sensor.imu_ng", 0.0);
  node.get_parameter("lio.sensor.imu_ng", g_imu_ng);

  node.declare_parameter<double>("lio.sensor.imu_nba", 0.0);
  node.get_parameter("lio.sensor.imu_nba", g_imu_nba);

  node.declare_parameter<double>("lio.sensor.imu_nbg", 0.003);
  node.get_parameter("lio.sensor.imu_nbg", g_imu_nbg);

  // 陀螺 scale 修正（1.0 为不修正）。螺旋楼梯道里 yaw 净转角与层数成正比，
  // 任何陀螺 scale 误差 δ 都会变成"每层固定偏转 δ*360°"，属于系统性偏置。
  node.declare_parameter<double>("lio.sensor.imu_gyr_scale", 1.0);
  node.get_parameter("lio.sensor.imu_gyr_scale", g_imu_gyr_scale);

  // 雷达逐点时间基准偏移（秒，默认 0）。只作用于点的去畸变时刻，不改 end_time，
  // 用于标定"雷达时间戳相对 IMU 时钟的固定滞后"：Δ 让每帧点云整体多转 ω·Δ，
  // 沿螺旋楼梯累计后就是与转角成正比的 yaw 偏置（每层同向同角度）。
  node.declare_parameter<double>("lio.sensor.lidar_time_offset", 0.0);
  node.get_parameter("lio.sensor.lidar_time_offset", g_lidar_time_offset);

  // ================= extrinsic =================
  std::vector<double> extrinsic_lidar_imu;
  node.declare_parameter<std::vector<double>>(
      "lio.extrinsic.lidar_imu", std::vector<double>(12, 0.0));
  node.get_parameter("lio.extrinsic.lidar_imu", extrinsic_lidar_imu);

  V3 __t(extrinsic_lidar_imu[0],
         extrinsic_lidar_imu[1],
         extrinsic_lidar_imu[2]);
  std::vector<scalar> r_data(9);
  for (int i = 0; i < 9; ++i) {
    r_data[i] = static_cast<scalar>(extrinsic_lidar_imu[3 + i]);
  }
  M3 __R(r_data.data());
  g_lidar_imu = SE3(__R, __t);

  std::vector<double> extrinsic_odom_robo;
  node.declare_parameter<std::vector<double>>(
      "lio.extrinsic.odom_robo", std::vector<double>(6, 0.0));
  node.get_parameter("lio.extrinsic.odom_robo", extrinsic_odom_robo);

  __t = V3(extrinsic_odom_robo[0],
           extrinsic_odom_robo[1],
           extrinsic_odom_robo[2]);

  auto temp_R =
      Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0,
                          Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_odom_robo[4] * M_PI / 180.0,
                          Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_odom_robo[3] * M_PI / 180.0,
                          Eigen::Vector3d::UnitX());

  g_odom_robo.R_ = temp_R.cast<scalar>();
  g_odom_robo.R_ = g_odom_robo.R_.transpose().eval();
  g_odom_robo = SE3(g_odom_robo.R_, __t);

  auto temp_R_yaw =
      Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0,
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  g_lidar_robo_yaw = temp_R_yaw.cast<scalar>();

  // ================= hash map =================
  node.declare_parameter<int>("lio.hash_map.hash_capacity", 1000000);
  node.get_parameter("lio.hash_map.hash_capacity", g_ivox_capacity);

  node.declare_parameter<double>("lio.hash_map.vox_resolution", 0.5);
  node.get_parameter("lio.hash_map.vox_resolution", g_ivox_resolution);

  // M.29 哈希预分配（消除扩容期整表重哈希尖峰；只改桶数，不改行为）
  node.declare_parameter<bool>("lio.hash_map.reserve_en", true);
  node.get_parameter("lio.hash_map.reserve_en", g_map_hash_reserve_en);
  node.declare_parameter<int>("lio.hash_map.reserve_size", 0);
  node.get_parameter("lio.hash_map.reserve_size", g_map_hash_reserve_size);
  LOG(INFO) << "[Param] hash_map.reserve: en=" << g_map_hash_reserve_en
            << " size=" << g_map_hash_reserve_size << " (0=用 hash_capacity)";

  // M.31 场景自动档：启动窗口判定一次后全程固定（默认关，关时行为不变）
  node.declare_parameter<bool>("lio.auto_scene.en", false);
  node.get_parameter("lio.auto_scene.en", g_auto_scene_en);
  node.declare_parameter<double>("lio.auto_scene.window", 120.0);
  node.get_parameter("lio.auto_scene.window", g_auto_scene_window);
  node.declare_parameter<double>("lio.auto_scene.cond_thr", 150.0);
  node.get_parameter("lio.auto_scene.cond_thr", g_auto_scene_cond_thr);
  node.declare_parameter<double>("lio.auto_scene.gate", 0.5);
  node.get_parameter("lio.auto_scene.gate", g_auto_scene_gate);
  node.declare_parameter<double>("lio.auto_scene.stair_na", 0.3);
  node.get_parameter("lio.auto_scene.stair_na", g_auto_scene_stair_na);
  node.declare_parameter<bool>("lio.auto_scene.stair_nw", true);
  node.get_parameter("lio.auto_scene.stair_nw", g_auto_scene_stair_nw);
  node.declare_parameter<double>("lio.auto_scene.stair_nw_sigma", 0.035);
  node.get_parameter("lio.auto_scene.stair_nw_sigma", g_auto_scene_stair_nw_sigma);
  node.declare_parameter<double>("lio.auto_scene.corridor_na", 0.01);
  node.get_parameter("lio.auto_scene.corridor_na", g_auto_scene_corridor_na);
  node.declare_parameter<bool>("lio.auto_scene.corridor_nw", false);
  node.get_parameter("lio.auto_scene.corridor_nw", g_auto_scene_corridor_nw);
  LOG(INFO) << "[Param] auto_scene: en=" << g_auto_scene_en
            << " win=" << g_auto_scene_window << "s thr=" << g_auto_scene_cond_thr
            << " gate=" << g_auto_scene_gate
            << " stair(na=" << g_auto_scene_stair_na << ",nw=" << g_auto_scene_stair_nw
            << ",sig=" << g_auto_scene_stair_nw_sigma << ")"
            << " corridor(na=" << g_auto_scene_corridor_na
            << ",nw=" << g_auto_scene_corridor_nw << ")";

  // M.32 在线时延估计（侧车；默认关）
  node.declare_parameter<bool>("lio.calib_td.en", false);
  node.get_parameter("lio.calib_td.en", g_calib_td_en);
  node.declare_parameter<double>("lio.calib_td.lambda", 0.995);
  node.get_parameter("lio.calib_td.lambda", g_calib_td_lambda);
  node.declare_parameter<double>("lio.calib_td.cc_gate", 0.01);
  node.get_parameter("lio.calib_td.cc_gate", g_calib_td_cc_gate);
  node.declare_parameter<double>("lio.calib_td.outlier", 0.20);
  node.get_parameter("lio.calib_td.outlier", g_calib_td_outlier);
  node.declare_parameter<int>("lio.calib_td.lock_win", 300);
  node.get_parameter("lio.calib_td.lock_win", g_calib_td_lock_win);
  node.declare_parameter<double>("lio.calib_td.lock_tol", 1.0e-3);
  node.get_parameter("lio.calib_td.lock_tol", g_calib_td_lock_tol);
  node.declare_parameter<bool>("lio.calib_td.apply_en", false);
  node.get_parameter("lio.calib_td.apply_en", g_calib_td_apply_en);
  LOG(INFO) << "[Param] calib_td: en=" << g_calib_td_en
            << " lambda=" << g_calib_td_lambda
            << " cc_gate=" << g_calib_td_cc_gate
            << " outlier=" << g_calib_td_outlier
            << " lock_win=" << g_calib_td_lock_win
            << " lock_tol=" << g_calib_td_lock_tol * 1e3 << "ms"
            << " apply_en=" << g_calib_td_apply_en
            << " base_toff=" << g_lidar_time_offset * 1e3 << "ms";

  // kf
  node.declare_parameter<int>("lio.kf.kf_type", 0);
  node.get_parameter("lio.kf.kf_type", g_kf_type);

  node.declare_parameter<int>("lio.kf.kf_max_iterations", 0);
  node.get_parameter("lio.kf.kf_max_iterations", g_kf_max_iterations);

  node.declare_parameter<bool>("lio.kf.kf_align_gravity", false);
  node.get_parameter("lio.kf.kf_align_gravity", g_kf_align_gravity);

  node.declare_parameter<bool>("lio.kf.freeze_gravity", false);
  node.get_parameter("lio.kf.freeze_gravity", g_kf_freeze_gravity);

  node.declare_parameter<int>("lio.kf.ref_gravity_axis", 2);
  node.get_parameter("lio.kf.ref_gravity_axis", g_ref_gravity_axis);

  node.declare_parameter<double>("lio.kf.kf_quit_eps", 0.0);
  node.get_parameter("lio.kf.kf_quit_eps", g_kf_quit_eps);

  node.declare_parameter<bool>("lio.kf.dynamic_start", false);
  node.get_parameter("lio.kf.dynamic_start", g_kf_dynamic_start);

  LOG(INFO) << GREEN << " ---> [Param] kf/dynamic_start: "
            << (g_kf_dynamic_start ? "true (跳过静态零偏标定, bg0=0)" : "false") << RESET;

  // observe
  node.declare_parameter<double>("lio.observe.plane_fit_threshold", 0.15);
  node.get_parameter("lio.observe.plane_fit_threshold", g_plane_fit_threshold);

  LOG(INFO) << GREEN << " ---> [Param] observe/plane_fit_threshold: "
            << g_plane_fit_threshold << RESET;

  node.declare_parameter<double>("lio.observe.obs_weight", 1000.0);
  node.get_parameter("lio.observe.obs_weight", g_obs_weight);

  node.declare_parameter<double>("lio.observe.rot_weight", 1.0);
  node.get_parameter("lio.observe.rot_weight", g_obs_rot_weight);

  node.declare_parameter<double>("lio.observe.huber_delta_base", 0.05);
  node.get_parameter("lio.observe.huber_delta_base", g_huber_delta_base);

  node.declare_parameter<double>("lio.observe.huber_delta_scale", 0.003);
  node.get_parameter("lio.observe.huber_delta_scale", g_huber_delta_scale);

  // 实验：距离自适应测量方差 σ²(r)=dept_err²+(beam_err·r)²（beam_err=0 关闭）
  node.declare_parameter<double>("lio.observe.beam_err", 0.0);
  node.get_parameter("lio.observe.beam_err", g_beam_err);

  node.declare_parameter<double>("lio.observe.dept_err", 0.02);
  node.get_parameter("lio.observe.dept_err", g_dept_err);


  LOG(INFO) << GREEN << " ---> [Param] observe/obs_weight: " << g_obs_weight << RESET;
  LOG(INFO) << GREEN << " ---> [Param] observe/rot_weight: " << g_obs_rot_weight << RESET;
  LOG(INFO) << GREEN << " ---> [Param] observe/huber_delta_base: " << g_huber_delta_base << RESET;
  LOG(INFO) << GREEN << " ---> [Param] observe/huber_delta_scale: " << g_huber_delta_scale << RESET;
  LOG(INFO) << GREEN << " ---> [Param] observe/beam_err: " << g_beam_err << RESET;
  LOG(INFO) << GREEN << " ---> [Param] observe/dept_err: " << g_dept_err << RESET;

  // ================= 阶段 2 机制开关（默认全关） =================
  node.declare_parameter<double>("lio.observe.ground_weight", 0.0);
  node.get_parameter("lio.observe.ground_weight", g_ground_weight);
  node.declare_parameter<double>("lio.observe.obs_dist_k", 0.0);
  node.get_parameter("lio.observe.obs_dist_k", g_obs_dist_k);
  node.declare_parameter<double>("lio.observe.thresh_motion_k", 0.0);
  node.get_parameter("lio.observe.thresh_motion_k", g_thresh_motion_k);
  node.declare_parameter<bool>("lio.observe.delta_interp_en", false);
  node.get_parameter("lio.observe.delta_interp_en", g_delta_interp_en);
  LOG(INFO) << GREEN << " ---> [Param] observe/ground_weight: " << g_ground_weight
            << " obs_dist_k: " << g_obs_dist_k
            << " thresh_motion_k: " << g_thresh_motion_k
            << " delta_interp_en: " << g_delta_interp_en << RESET;

  node.declare_parameter<bool>("lio.observe.normal_weight_en", false);
  node.get_parameter("lio.observe.normal_weight_en", g_normal_weight_en);

  node.declare_parameter<double>("lio.observe.normal_sigma", 0.05);
  node.get_parameter("lio.observe.normal_sigma", g_normal_sigma);

  node.declare_parameter<bool>("lio.observe.degeneracy_detect_en", false);
  node.get_parameter("lio.observe.degeneracy_detect_en", g_degeneracy_detect_en);
  node.declare_parameter<double>("lio.observe.degeneracy_threshold", 100.0);
  node.get_parameter("lio.observe.degeneracy_threshold", g_degeneracy_threshold);
  node.declare_parameter<bool>("lio.observe.degeneracy_truncate_en", false);
  node.get_parameter("lio.observe.degeneracy_truncate_en", g_degeneracy_truncate_en);
  node.declare_parameter<double>("lio.observe.tikhonov_lambda", 1e-3);
  node.get_parameter("lio.observe.tikhonov_lambda", g_tikhonov_lambda);
  node.declare_parameter<int>("lio.observe.degen_trans_only", 0);
  node.get_parameter("lio.observe.degen_trans_only", g_degen_trans_only);
  node.declare_parameter<double>("lio.observe.degeneracy_hysteresis", 0.5);
  node.get_parameter("lio.observe.degeneracy_hysteresis", g_degeneracy_hysteresis);
  node.declare_parameter<int>("lio.observe.degeneracy_enter_frames", 3);
  node.get_parameter("lio.observe.degeneracy_enter_frames", g_degeneracy_enter_frames);
  node.declare_parameter<int>("lio.observe.degeneracy_exit_frames", 10);
  node.get_parameter("lio.observe.degeneracy_exit_frames", g_degeneracy_exit_frames);
  node.declare_parameter<bool>("lio.observe.adaptive_param_en", false);
  node.get_parameter("lio.observe.adaptive_param_en", g_adaptive_param_en);
  node.declare_parameter<bool>("lio.observe.adaptive_normal_weight_en", true);
  node.get_parameter("lio.observe.adaptive_normal_weight_en", g_adaptive_normal_weight_en);
  node.declare_parameter<double>("lio.observe.adaptive_normal_sigma", 0.035);
  node.get_parameter("lio.observe.adaptive_normal_sigma", g_adaptive_normal_sigma);
  node.declare_parameter<double>("lio.observe.adaptive_plane_fit", 0.0);
  node.get_parameter("lio.observe.adaptive_plane_fit", g_adaptive_plane_fit);
  node.declare_parameter<double>("lio.observe.adaptive_huber", 0.0);
  node.get_parameter("lio.observe.adaptive_huber", g_adaptive_huber);

  // M.33 沿运动方向截断（默认关 = 原行为）
  node.declare_parameter<bool>("lio.observe.degen_motion_dir_en", false);
  node.get_parameter("lio.observe.degen_motion_dir_en", g_degen_motion_dir_en);
  node.declare_parameter<double>("lio.observe.degen_motion_dir_ratio", 0.15);
  node.get_parameter("lio.observe.degen_motion_dir_ratio", g_degen_motion_dir_ratio);
  node.declare_parameter<double>("lio.observe.degen_motion_dir_vmin", 0.30);
  node.get_parameter("lio.observe.degen_motion_dir_vmin", g_degen_motion_dir_vmin);

  // 细通道（M.27，默认全关 = 原行为）
  node.declare_parameter<bool>("lio.observe.fine_channel_en", false);
  node.get_parameter("lio.observe.fine_channel_en", g_fine_channel_en);
  node.declare_parameter<double>("lio.observe.fine_channel_weight", 100.0);
  node.get_parameter("lio.observe.fine_channel_weight", g_fine_channel_weight);
  node.declare_parameter<int>("lio.observe.fine_channel_max_points", 150);
  node.get_parameter("lio.observe.fine_channel_max_points", g_fine_channel_max_points);
  node.declare_parameter<double>("lio.observe.fine_max_dist", 0.5);
  node.get_parameter("lio.observe.fine_max_dist", g_fine_max_dist);
  node.declare_parameter<double>("lio.observe.fine_channel_min_gain", 1.5);
  node.get_parameter("lio.observe.fine_channel_min_gain", g_fine_channel_min_gain);
  LOG(INFO) << GREEN << " ---> [Param] observe/fine_channel: en=" << g_fine_channel_en
            << " weight=" << g_fine_channel_weight
            << " max_points=" << g_fine_channel_max_points
            << " max_dist=" << g_fine_max_dist
            << " min_gain=" << g_fine_channel_min_gain << RESET;

  // 坏帧护栏
  node.declare_parameter<bool>("lio.observe.frame_gap_guard_en", true);
  node.get_parameter("lio.observe.frame_gap_guard_en", g_frame_gap_guard_en);
  node.declare_parameter<double>("lio.observe.frame_gap_max", 0.2);
  node.get_parameter("lio.observe.frame_gap_max", g_frame_gap_max);
  node.declare_parameter<double>("lio.observe.imu_cover_max", 0.03);
  node.get_parameter("lio.observe.imu_cover_max", g_imu_cover_max);

  node.declare_parameter<int>("lio.hash_map.rep_mode", 0);
  node.get_parameter("lio.hash_map.rep_mode", g_map_rep_mode);

  node.declare_parameter<bool>("lio.hash_map.clamp_mu", false);
  node.get_parameter("lio.hash_map.clamp_mu", g_map_clamp_mu);

  node.declare_parameter<int>("lio.hash_map.max_points", 20);
  node.get_parameter("lio.hash_map.max_points", g_map_max_points);

  node.declare_parameter<double>("lio.hash_map.merge_dist", 0.1);
  node.get_parameter("lio.hash_map.merge_dist", g_map_merge_dist);

  node.declare_parameter<int>("lio.hash_map.hknn_max_group", -1);
  node.get_parameter("lio.hash_map.hknn_max_group", g_hknn_max_group);

  node.declare_parameter<double>("lio.hash_map.hknn_order_scale", 1.0);
  node.get_parameter("lio.hash_map.hknn_order_scale", g_hknn_order_scale);

  // M.35：平面拟合邻域大小（此前写死在 KNNHeap<K,...> 模板参数里）
  node.declare_parameter<int>("lio.hash_map.plane_knn_k", 8);
  node.get_parameter("lio.hash_map.plane_knn_k", g_plane_knn_k);
  // 下限 4：少于 4 点拟合不出平面（calc_plane_coeff 之前有 count<4 的丢弃）
  // 上限 8：等于 KNNHeap 的模板容量，超了会写越界（try_insert 内另有兜底夹取）
  if (g_plane_knn_k < 4) g_plane_knn_k = 4;
  if (g_plane_knn_k > 8) g_plane_knn_k = 8;

  LOG(INFO) << GREEN << " ---> [Param] debug/level: " << g_debug_level << RESET;
  LOG(INFO) << GREEN << " ---> [Param] observe/normal_weight_en: " << g_normal_weight_en
            << " sigma: " << g_normal_sigma << RESET;
  LOG(INFO) << GREEN << " ---> [Param] hash_map/rep_mode: " << g_map_rep_mode
            << " clamp_mu: " << g_map_clamp_mu
            << " max_points: " << g_map_max_points
            << " merge_dist: " << g_map_merge_dist << RESET;
  LOG(INFO) << GREEN << " ---> [Param] hash_map/hknn_max_group: " << g_hknn_max_group
            << " order_scale: " << g_hknn_order_scale << RESET;
  LOG(INFO) << GREEN << " ---> [Param] hash_map/plane_knn_k: " << g_plane_knn_k << RESET;
  node.declare_parameter<int>("lio.observe.guard_max_consecutive", 9);
  node.get_parameter("lio.observe.guard_max_consecutive", g_guard_max_consecutive);

  LOG(INFO) << GREEN << " ---> [Param] observe/frame_gap_guard_en: " << g_frame_gap_guard_en
            << " gap_max: " << g_frame_gap_max << " imu_cover_max: " << g_imu_cover_max
            << " max_consecutive: " << g_guard_max_consecutive << RESET;

  // 断档冻结：IMU 积分 dt 上限
  node.declare_parameter<double>("lio.observe.max_imu_dt", 0.05);
  node.get_parameter("lio.observe.max_imu_dt", g_max_imu_dt);

  // 位姿信任域 / LOST
  node.declare_parameter<bool>("lio.observe.trust_guard_en", true);
  node.get_parameter("lio.observe.trust_guard_en", g_trust_guard_en);
  node.declare_parameter<double>("lio.observe.trust_jump_max", 1.0);
  node.get_parameter("lio.observe.trust_jump_max", g_trust_jump_max);
  node.declare_parameter<double>("lio.observe.trust_jump_rot_max", 15.0);
  node.get_parameter("lio.observe.trust_jump_rot_max", g_trust_jump_rot_max);
  // 状态速度硬门（0 = 关闭）：挡住"每帧 <1 m 的持续拖拽"型漂移
  node.declare_parameter<double>("lio.observe.trust_vel_max", 0.0);
  node.get_parameter("lio.observe.trust_vel_max", g_trust_vel_max);
  // 地图新点比例门（0 = 关闭）：错误匹配的早期指标
  node.declare_parameter<double>("lio.observe.map_novelty_max", 0.0);
  node.get_parameter("lio.observe.map_novelty_max", g_map_novelty_max);
  node.declare_parameter<double>("lio.observe.map_novelty_jump_min", 0.10);
  node.get_parameter("lio.observe.map_novelty_jump_min", g_map_novelty_jump_min);
  node.declare_parameter<double>("lio.observe.map_novelty_jump_min_rot", 1.0);
  node.get_parameter("lio.observe.map_novelty_jump_min_rot", g_map_novelty_jump_min_rot);
  node.declare_parameter<int>("lio.observe.map_novelty_streak", 2);
  node.get_parameter("lio.observe.map_novelty_streak", g_map_novelty_streak);
  // 早期漂移前兆门（任一为 0 则关闭）：新点比例 + 速度同时抬头
  node.declare_parameter<double>("lio.observe.map_novelty_early", 0.0);
  node.get_parameter("lio.observe.map_novelty_early", g_map_novelty_early);
  node.declare_parameter<double>("lio.observe.trust_vel_early", 0.0);
  node.get_parameter("lio.observe.trust_vel_early", g_trust_vel_early);
  node.declare_parameter<int>("lio.observe.novelty_early_streak", 2);
  node.get_parameter("lio.observe.novelty_early_streak", g_novelty_early_streak);
  node.declare_parameter<double>("lio.observe.open_guard_max_range", 0.0);
  node.get_parameter("lio.observe.open_guard_max_range", g_open_guard_max_range);
  node.declare_parameter<double>("lio.observe.open_guard_z_span", 0.0);
  node.get_parameter("lio.observe.open_guard_z_span", g_open_guard_z_span);
  node.declare_parameter<double>("lio.observe.trust_corr_max", 0.5);
  node.get_parameter("lio.observe.trust_corr_max", g_trust_corr_max);
  node.declare_parameter<double>("lio.observe.trust_corr_rot_max", 10.0);
  node.get_parameter("lio.observe.trust_corr_rot_max", g_trust_corr_rot_max);
  node.declare_parameter<int>("lio.observe.lost_recover_frames", 5);
  node.get_parameter("lio.observe.lost_recover_frames", g_lost_recover_frames);
  node.declare_parameter<double>("lio.observe.lost_slew_p", 0.10);
  node.get_parameter("lio.observe.lost_slew_p", g_lost_slew_p);
  node.declare_parameter<double>("lio.observe.lost_slew_r", 1.0);
  node.get_parameter("lio.observe.lost_slew_r", g_lost_slew_r);
  node.declare_parameter<bool>("lio.observe.lost_map_en", false);
  node.get_parameter("lio.observe.lost_map_en", g_lost_map_en);
  node.declare_parameter<int>("lio.observe.lost_min_effect_pts", 100);
  node.get_parameter("lio.observe.lost_min_effect_pts", g_lost_min_effect_pts);
  node.declare_parameter<double>("lio.observe.lost_vel_damp", 1.0);
  node.get_parameter("lio.observe.lost_vel_damp", g_lost_vel_damp);
  node.declare_parameter<double>("lio.observe.lost_vel_max", 1.0);
  node.get_parameter("lio.observe.lost_vel_max", g_lost_vel_max);
  node.declare_parameter<double>("lio.observe.trust_rot_clamp_deg", 0.0);
  node.get_parameter("lio.observe.trust_rot_clamp_deg", g_trust_rot_clamp_deg);
  node.declare_parameter<double>("lio.observe.trust_corr_rate_max", 0.0);
  node.get_parameter("lio.observe.trust_corr_rate_max", g_trust_corr_rate_max);
  node.declare_parameter<int>("lio.observe.trust_corr_rate_window", 20);
  node.get_parameter("lio.observe.trust_corr_rate_window", g_trust_corr_rate_window);
  node.declare_parameter<double>("lio.observe.trust_corr_coh_min", 0.0);
  node.get_parameter("lio.observe.trust_corr_coh_min", g_trust_corr_coh_min);
  node.declare_parameter<double>("lio.observe.trust_corr_rate_lost", 0.0);
  node.get_parameter("lio.observe.trust_corr_rate_lost", g_trust_corr_rate_lost);
  node.declare_parameter<double>("lio.observe.trust_corr_lost_hold_s", 1.0);
  node.get_parameter("lio.observe.trust_corr_lost_hold_s", g_trust_corr_lost_hold_s);
  node.declare_parameter<double>("lio.observe.trust_suspect_after_s", 0.0);
  node.get_parameter("lio.observe.trust_suspect_after_s", g_trust_suspect_after_s);
  node.declare_parameter<double>("lio.observe.trust_suspect_max_s", 10.0);
  node.get_parameter("lio.observe.trust_suspect_max_s", g_trust_suspect_max_s);
  node.declare_parameter<double>("lio.observe.trust_lost_max_s", 20.0);
  node.get_parameter("lio.observe.trust_lost_max_s", g_trust_lost_max_s);
  node.declare_parameter<double>("lio.observe.trust_force_accept_max_p", 3.0);
  node.get_parameter("lio.observe.trust_force_accept_max_p", g_trust_force_accept_max_p);
  node.declare_parameter<double>("lio.observe.trust_force_accept_max_r", 20.0);
  node.get_parameter("lio.observe.trust_force_accept_max_r", g_trust_force_accept_max_r);

  LOG(INFO) << GREEN << " ---> [Param] observe/max_imu_dt: " << g_max_imu_dt << " s" << RESET;
  LOG(INFO) << GREEN << " ---> [Param] observe/trust_guard_en: " << g_trust_guard_en
            << " jump_max: " << g_trust_jump_max << " jump_rot_max: " << g_trust_jump_rot_max
            << " corr_max: " << g_trust_corr_max << " corr_rot_max: " << g_trust_corr_rot_max
            << " recover_frames: " << g_lost_recover_frames
            << " slew_p: " << g_lost_slew_p << " m slew_r: " << g_lost_slew_r << " deg"
            << " lost_map_en: " << g_lost_map_en
            << " rot_clamp: " << g_trust_rot_clamp_deg << " deg/frame"
            << " corr_rate_max: " << g_trust_corr_rate_max << " deg/s @"
            << g_trust_corr_rate_window << " frames"
            << " coh_min: " << g_trust_corr_coh_min
            << " rate_lost: " << g_trust_corr_rate_lost
            << " hold: " << g_trust_corr_lost_hold_s << " s"
            << " suspect_after: " << g_trust_suspect_after_s << " s"
            << " suspect_max: " << g_trust_suspect_max_s << " s"
            << " lost_max: " << g_trust_lost_max_s << " s"
            << " force_accept<=" << g_trust_force_accept_max_p << " m/"
            << g_trust_force_accept_max_r << " deg" << RESET;
  LOG(INFO) << GREEN << " ---> [Param] map/freeze_after_s: "
            << g_map_freeze_after_s << " s" << RESET;

  // submaps
  node.declare_parameter<double>("lio.submap.submap_resolution", 0.0);
  node.get_parameter("lio.submap.submap_resolution", g_submap_resolution);

  node.declare_parameter<int>("lio.submap.submap_capacity", 0);
  node.get_parameter("lio.submap.submap_capacity", g_submap_capacity);

  // visual
  node.declare_parameter<bool>("lio.output.robot", false);
  node.get_parameter("lio.output.robot", g_2_robot);

  node.declare_parameter<bool>("lio.output.planner", false);
  node.get_parameter("lio.output.planner", g_planner_enable);

  node.declare_parameter<bool>("lio.output.plan_env_world", false);
  node.get_parameter("lio.output.plan_env_world", g_2_plan_env_world);

  node.declare_parameter<bool>("lio.output.plan_env_body", false);
  node.get_parameter("lio.output.plan_env_body", g_2_plan_env_body);

  node.declare_parameter<bool>("lio.output.ml_map", false);
  node.get_parameter("lio.output.ml_map", g_2_ml_map);

  node.declare_parameter<bool>("lio.output.map", false);
  node.get_parameter("lio.output.map", g_visual_map);

  node.declare_parameter<bool>("lio.output.dense", false);
  node.get_parameter("lio.output.dense", g_visual_dense);

  node.declare_parameter<bool>("lio.output.map_body", false);
  node.get_parameter("lio.output.map_body", g_visual_map_body);

  node.declare_parameter<bool>("lio.output.dense_body", false);
  node.get_parameter("lio.output.dense_body", g_visual_dense_body);

  node.declare_parameter<int>("lio.output.pub_step", 0);
  node.get_parameter("lio.output.pub_step", g_pub_step);

  node.declare_parameter<bool>("lio.output.footprint_pub_en", true);
  node.get_parameter("lio.output.footprint_pub_en", g_footprint_pub_en);

  // 该键原先只在 on-set 回调里读取、从未 declare，params-file 里的值不会生效
  // （实测 yaml 写 true，日志仍打印 false）。补齐 declare/get。
  node.declare_parameter<bool>("lio.output.odom_cov_en", false);
  node.get_parameter("lio.output.odom_cov_en", g_odom_cov_en);

  node.declare_parameter<std::string>("lio.output.tf_base_footprint_frame", "base_footprint");
  node.get_parameter("lio.output.tf_base_footprint_frame", g_tf_base_footprint_frame);

  node.declare_parameter<std::string>("lio.output.world_frame", "world");
  node.get_parameter("lio.output.world_frame", g_world_frame);

  node.declare_parameter<std::string>("lio.output.imu_frame", "imu");
  node.get_parameter("lio.output.imu_frame", g_imu_frame);

  LOG(INFO) << GREEN << " ---> [Param] output/footprint_pub_en: "
            << (g_footprint_pub_en ? "true" : "false") << RESET;

  LOG(INFO) << GREEN << " ---> [Param] output/odom_cov_en: "
            << (g_odom_cov_en ? "true" : "false") << RESET;

  LOG(INFO) << GREEN << " ---> [Param] output/tf_base_footprint_frame: "
            << g_tf_base_footprint_frame << RESET;

  LOG(INFO) << GREEN << " ---> [Param] output/world_frame: "
            << g_world_frame << RESET;

  LOG(INFO) << GREEN << " ---> [Param] output/imu_frame: "
            << g_imu_frame << RESET;

  // ================= relocation =================
  node.declare_parameter<bool>("lio.relocation.update_map", false);
  node.get_parameter("lio.relocation.update_map", g_update_map);

  std::vector<double> init_pose;
  node.declare_parameter<std::vector<double>>(
      "lio.relocation.init_pose", std::vector<double>(6, 0.0));
  node.get_parameter("lio.relocation.init_pose", init_pose);

  g_init_px    = init_pose[0];
  g_init_py    = init_pose[1];
  g_init_pz    = init_pose[2];
  g_init_roll  = init_pose[3];
  g_init_pitch = init_pose[4];
  g_init_yaw   = init_pose[5];

  // ================= dynamic removal =================
  node.declare_parameter<bool>("lio.dynamic_removal.enable", false);
  node.get_parameter("lio.dynamic_removal.enable", g_dynamic_removal_enable);

  node.declare_parameter<int>("lio.dynamic_removal.method", 0);
  node.get_parameter("lio.dynamic_removal.method", g_dynamic_removal_method);

  node.declare_parameter<double>("lio.dynamic_removal.grid_size", 0.2);
  double temp_grid_size;
  node.get_parameter("lio.dynamic_removal.grid_size", temp_grid_size);
  g_dynamic_removal_grid_size = static_cast<float>(temp_grid_size);

  node.declare_parameter<int>("lio.dynamic_removal.min_neighbors", 2);
  node.get_parameter("lio.dynamic_removal.min_neighbors", g_dynamic_removal_min_neighbors);

  node.declare_parameter<int>("lio.dynamic_removal.frame_window", 1);
  node.get_parameter("lio.dynamic_removal.frame_window", g_dynamic_removal_frame_window);

  node.declare_parameter<int>("lio.dynamic_removal.raycast_min_hits", 2);
  node.get_parameter("lio.dynamic_removal.raycast_min_hits", g_dynamic_removal_raycast_min_hits);

  node.declare_parameter<bool>("lio.dynamic_removal.isolated_removal", true);
  node.get_parameter("lio.dynamic_removal.isolated_removal", g_dynamic_removal_isolated_removal);

  LOG(INFO) << GREEN << " ---> [Param] dynamic_removal/enable: "
            << (g_dynamic_removal_enable ? "true" : "false") << RESET;

  LOG(INFO) << GREEN << " ---> [Param] dynamic_removal/method: "
            << (g_dynamic_removal_method == 0 ? "Temporal" : "Raycast") << RESET;

  LOG(INFO) << GREEN << " ---> [Param] dynamic_removal/grid_size: "
            << g_dynamic_removal_grid_size << RESET;

  LOG(INFO) << GREEN << " ---> [Param] dynamic_removal/min_neighbors: "
            << g_dynamic_removal_min_neighbors << RESET;

  LOG(INFO) << GREEN << " ---> [Param] dynamic_removal/frame_window: "
            << g_dynamic_removal_frame_window << RESET;

  LOG(INFO) << GREEN << " ---> [Param] dynamic_removal/raycast_min_hits: "
            << g_dynamic_removal_raycast_min_hits << RESET;

  LOG(INFO) << GREEN << " ---> [Param] dynamic_removal/isolated_removal: "
            << (g_dynamic_removal_isolated_removal ? "true" : "false") << RESET;

  // ================= SC-PGO offline output =================
  node.declare_parameter<bool>("lio.sc_pgo.enable", false);
  node.get_parameter("lio.sc_pgo.enable", g_sc_pgo_enable);

  node.declare_parameter<double>("lio.sc_pgo.keyframe_gap", 5.0);
  double temp_kf_gap;
  node.get_parameter("lio.sc_pgo.keyframe_gap", temp_kf_gap);
  g_sc_pgo_keyframe_gap = static_cast<float>(temp_kf_gap);

  node.declare_parameter<double>("lio.sc_pgo.keyframe_deg_gap", 10.0);
  double temp_kf_deg_gap;
  node.get_parameter("lio.sc_pgo.keyframe_deg_gap", temp_kf_deg_gap);
  g_sc_pgo_keyframe_deg_gap = static_cast<float>(temp_kf_deg_gap);

  LOG(INFO) << GREEN << " ---> [Param] sc_pgo/enable: "
            << (g_sc_pgo_enable ? "true" : "false") << RESET;

  LOG(INFO) << GREEN << " ---> [Param] sc_pgo/keyframe_gap: "
            << g_sc_pgo_keyframe_gap << RESET;

  LOG(INFO) << GREEN << " ---> [Param] sc_pgo/keyframe_deg_gap: "
            << g_sc_pgo_keyframe_deg_gap << RESET;

  // ================= single core mode =================
  node.declare_parameter<bool>("lio.single_core", false);
  node.get_parameter("lio.single_core", g_single_core);

  LOG(INFO) << GREEN << " ---> [Param] single_core: "
            << (g_single_core ? "true" : "false") << RESET;

  // ================= fast tf mode =================
  node.declare_parameter<bool>("lio.fast_tf", false);
  node.get_parameter("lio.fast_tf", g_fast_tf);

  LOG(INFO) << GREEN << " ---> [Param] fast_tf: "
            << (g_fast_tf ? "true" : "false") << RESET;

  // ================= lio only undistort mode =================
  node.declare_parameter<bool>("lio.lio_only_undistort", false);
  node.get_parameter("lio.lio_only_undistort", g_lio_only_undistort);

  LOG(INFO) << GREEN << " ---> [Param] lio_only_undistort: "
            << (g_lio_only_undistort ? "true" : "false") << RESET;

  // ================= downsample only mode (highest priority) =================
  node.declare_parameter<bool>("lio.downsample_only", false);
  node.get_parameter("lio.downsample_only", g_downsample_only);

  LOG(INFO) << GREEN << " ---> [Param] downsample_only: "
            << (g_downsample_only ? "true" : "false") << RESET;

  LOG(INFO) << GREEN << " ---> [Params]: Load from ROS2 parameter server."
            << RESET;
}


#ifdef LIVOX_SUPPORT
void livox2pcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg, CloudPtr& point_cloud){
  point_cloud->clear();
  CloudPtr cloud_full(new PointCloudType());
  int plsize = msg->point_num;
  cloud_full->resize(plsize);
  point_cloud->reserve(plsize);
  std::vector<bool> is_valid_pt(plsize, false);
  std::vector<std::size_t> index(plsize - 1);
  std::iota(std::begin(index), std::end(index), 1);

  std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
    if((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)
    {
      // if (i % g_filter_rate == 0) 
      {
        cloud_full->at(i).x = msg->points[i].x;
        cloud_full->at(i).y = msg->points[i].y;
        cloud_full->at(i).z = msg->points[i].z;
        cloud_full->at(i).intensity = msg->points[i].reflectivity;

        if ((abs(cloud_full->at(i).x - cloud_full->at(i - 1).x) > 1e-7) ||
            (abs(cloud_full->at(i).y - cloud_full->at(i - 1).y) > 1e-7) ||
            (abs(cloud_full->at(i).z - cloud_full->at(i - 1).z) > 1e-7))
        {
          double normal_dis = cloud_full->at(i).x * cloud_full->at(i).x + 
                              cloud_full->at(i).y * cloud_full->at(i).y +
                              cloud_full->at(i).z * cloud_full->at(i).z;
          if(normal_dis > g_blind2 and normal_dis < g_maxrange2){
            is_valid_pt[i] = true;
          }
        }
      }
    }
  });

  for (int i = 1; i < plsize; i++) {
    if (is_valid_pt[i]) {
      point_cloud->points.push_back(cloud_full->at(i));
    }
  }
}
#endif


std::string lidarTypeToString(int type) {
  if (type <= 0 || type >= static_cast<int>(LID_TYPE_NAMES.size())) return "UNKNOWN";
  return LID_TYPE_NAMES[type];
}


inline bool validPoint(double x, double y, double z)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    return false;

  double d2 = x * x + y * y + z * z;
  return (d2 > g_blind2 && d2 < g_maxrange2);
}


inline double stampToSec(const builtin_interfaces::msg::Time& t)
{
  return static_cast<double>(t.sec) +
         static_cast<double>(t.nanosec) * 1e-9;
}


inline builtin_interfaces::msg::Time toRosTime(double t_sec)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(std::floor(t_sec));
  t.nanosec = static_cast<uint32_t>((t_sec - t.sec) * 1e9);
  return t;
}


ROSWrapper::ROSWrapper(const rclcpp::NodeOptions& options)
: rclcpp::Node("super_lio", options)
{
  LoadParamFromRos(*this);
  LOG(INFO) << GREEN << " ---> Using Lidar type: "
            << lidarTypeToString(g_lidar_type) << RESET;

  // 参数运行时修改回调
  param_handler_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      for (const auto & p : params) {
        const auto & name = p.get_name();
        if (name == "lio.map.save_map") g_save_map = p.as_bool();
        else if (name == "lio.eva.timer") g_time_eva = p.as_bool();
        else if (name == "lio.map.if_filter") g_if_filter = p.as_bool();
        else if (name == "lio.map.ds_size") g_map_ds_size = p.as_double();
        else if (name == "lio.map.save_interval") g_pcd_save_interval = p.as_int();
        else if (name == "lio.sensor.filter_rate") g_filter_rate = p.as_int();
        else if (name == "lio.debug.level") g_debug_level = p.as_int();
        else if (name == "lio.debug.feature_length") g_debug_feature_length = p.as_double();
        else if (name == "lio.observe.info_gate_en") g_info_gate_en = p.as_bool();
        else if (name == "lio.observe.lam_min") g_lam_min = p.as_double();
        else if (name == "lio.observe.cond_max") g_cond_max = p.as_double();
        else if (name == "lio.imu.gravity_obs_en") g_gravity_obs_en = p.as_bool();
        else if (name == "lio.imu.acc_max") g_acc_max = p.as_double();
        else if (name == "lio.imu.acc_lpf_tau") g_acc_lpf_tau = p.as_double();
        else if (name == "lio.imu.gravity_sigma") g_gravity_sigma = p.as_double();
        else if (name == "lio.shadow.enable") g_shadow_en = p.as_bool();
        else if (name == "lio.shadow.lambda_ref") g_shadow_lambda_ref = p.as_double();
        else if (name == "lio.shadow.w_min") g_shadow_w_min = p.as_double();
        else if (name == "lio.shadow.alpha") g_shadow_alpha = p.as_double();
        else if (name == "lio.shadow.max_dw") g_shadow_max_dw = p.as_double();
        else if (name == "lio.sensor.enable_filter_offset") g_enable_filter_offset = p.as_bool();
        else if (name == "lio.sensor.enable_downsample") g_enable_downsample = p.as_bool();
        else if (name == "lio.sensor.voxel_fliter_size") g_voxel_fliter_size = p.as_double();
        else if (name == "lio.sensor.intensity_filter_en") g_intensity_filter_en = p.as_bool();
        else if (name == "lio.sensor.intensity_min") g_intensity_min = p.as_double();
        else if (name == "lio.sensor.full_column_interval") g_full_column_interval = p.as_int();
        else if (name == "lio.output.robot") g_2_robot = p.as_bool();
        else if (name == "lio.output.planner") g_planner_enable = p.as_bool();
        else if (name == "lio.output.plan_env_world") g_2_plan_env_world = p.as_bool();
        else if (name == "lio.output.plan_env_body") g_2_plan_env_body = p.as_bool();
        else if (name == "lio.output.ml_map") g_2_ml_map = p.as_bool();
        else if (name == "lio.output.map") g_visual_map = p.as_bool();
        else if (name == "lio.output.dense") g_visual_dense = p.as_bool();
        else if (name == "lio.output.map_body") g_visual_map_body = p.as_bool();
        else if (name == "lio.output.dense_body") g_visual_dense_body = p.as_bool();
        else if (name == "lio.output.pub_step") g_pub_step = p.as_int();
        else if (name == "lio.output.footprint_pub_en") g_footprint_pub_en = p.as_bool();
        else if (name == "lio.output.odom_cov_en") g_odom_cov_en = p.as_bool();
        else if (name == "lio.dynamic_removal.enable") g_dynamic_removal_enable = p.as_bool();
        else if (name == "lio.dynamic_removal.method") g_dynamic_removal_method = p.as_int();
        else if (name == "lio.dynamic_removal.grid_size") g_dynamic_removal_grid_size = p.as_double();
        else if (name == "lio.dynamic_removal.min_neighbors") g_dynamic_removal_min_neighbors = p.as_int();
        else if (name == "lio.dynamic_removal.frame_window") g_dynamic_removal_frame_window = p.as_int();
        else if (name == "lio.dynamic_removal.raycast_min_hits") g_dynamic_removal_raycast_min_hits = p.as_int();
        else if (name == "lio.dynamic_removal.isolated_removal") g_dynamic_removal_isolated_removal = p.as_bool();
        else if (name == "lio.sc_pgo.enable") g_sc_pgo_enable = p.as_bool();
        else if (name == "lio.single_core") g_single_core = p.as_bool();
        else if (name == "lio.fast_tf") g_fast_tf = p.as_bool();
        else if (name == "lio.lio_only_undistort") g_lio_only_undistort = p.as_bool();
        else if (name == "lio.downsample_only") g_downsample_only = p.as_bool();
        else if (name == "lio.observe.plane_fit_threshold") g_plane_fit_threshold = p.as_double();
        else if (name == "lio.observe.obs_weight") g_obs_weight = p.as_double();
        else if (name == "lio.observe.rot_weight") g_obs_rot_weight = p.as_double();
        else if (name == "lio.observe.huber_delta_base") g_huber_delta_base = p.as_double();
        else if (name == "lio.observe.huber_delta_scale") g_huber_delta_scale = p.as_double();
        else if (name == "lio.observe.beam_err") g_beam_err = p.as_double();
        else if (name == "lio.observe.dept_err") g_dept_err = p.as_double();
        else if (name == "lio.debug.level") g_debug_level = p.as_int();
        else if (name == "lio.debug.feature_length") g_debug_feature_length = p.as_double();
        else if (name == "lio.shadow.enable") g_shadow_en = p.as_bool();
        else if (name == "lio.shadow.lambda_ref") g_shadow_lambda_ref = p.as_double();
        else if (name == "lio.shadow.w_min") g_shadow_w_min = p.as_double();
        else if (name == "lio.shadow.alpha") g_shadow_alpha = p.as_double();
        else if (name == "lio.shadow.max_dw") g_shadow_max_dw = p.as_double();
        else if (name == "lio.observe.normal_weight_en") g_normal_weight_en = p.as_bool();
        else if (name == "lio.observe.normal_sigma") g_normal_sigma = p.as_double();
        else if (name == "lio.hash_map.rep_mode") g_map_rep_mode = p.as_int();
        else if (name == "lio.hash_map.clamp_mu") g_map_clamp_mu = p.as_bool();
        else if (name == "lio.hash_map.max_points") g_map_max_points = p.as_int();
        else if (name == "lio.hash_map.merge_dist") g_map_merge_dist = p.as_double();
        else if (name == "lio.hash_map.hknn_max_group") g_hknn_max_group = p.as_int();
        else if (name == "lio.hash_map.hknn_order_scale") g_hknn_order_scale = p.as_double();
        else if (name == "lio.hash_map.plane_knn_k") {
          g_plane_knn_k = std::min<int>(8, std::max<int>(4, static_cast<int>(p.as_int())));
        }
        else if (name == "lio.kf.kf_max_iterations") g_kf_max_iterations = p.as_int();
        else if (name == "lio.kf.kf_quit_eps") g_kf_quit_eps = p.as_double();
        else if (name == "lio.kf.freeze_gravity") g_kf_freeze_gravity = p.as_bool();
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });

  msg2uav_.header.frame_id = g_world_frame;
  path_.header.frame_id = g_world_frame;

  setupIO();
  setupServices();
  
  // 启动 IMU 输出线程，将发布从 IMU 回调解耦
  imu_output_running_ = true;
  imu_output_thread_ = std::thread(&ROSWrapper::imuOutputThread, this);
}


void ROSWrapper::setupServices(){
  // 创建保存地图服务
  save_map_service_ = this->create_service<std_srvs::srv::Trigger>(
      g_map_save_service_topic,
      std::bind(&ROSWrapper::saveMapServiceCallback, this, 
                std::placeholders::_1, std::placeholders::_2));
  
  LOG(INFO) << GREEN << " ---> [Service] Save map service created: " << g_map_save_service_topic << RESET;
}


void ROSWrapper::setupIO(){
  //// input ======================================
  // 创建独立的回调组，实现 IMU/lidar/process 三级并行
  cb_sensor_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_imu_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_lidar_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_process_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_report_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_opt_imu;
  sub_opt_imu.callback_group = cb_imu_;
  rclcpp::SubscriptionOptions sub_opt_lidar;
  sub_opt_lidar.callback_group = cb_lidar_;

  auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(500))
                 .durability_volatile();
  if (g_imu_qos_reliable) {
    imu_qos.reliable();
  } else {
    imu_qos.best_effort();
  }

  auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(20))
                   .durability_volatile();
  if (g_lidar_qos_reliable) {
    lidar_qos.reliable();
  } else {
    lidar_qos.best_effort();
  }

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      g_imu_topic,
      imu_qos,
      std::bind(&ROSWrapper::imuHandler, this, std::placeholders::_1),
      sub_opt_imu);

#ifdef LIVOX_SUPPORT
  if (g_lidar_type == LID_TYPE::LIVOX) {
    sub_lidar_ =
        this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            g_lidar_topic,
            lidar_qos,
            std::bind(&ROSWrapper::livoxHandler, this, std::placeholders::_1),
            sub_opt_lidar);
  } else
#endif
  {
    sub_lidar_std_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            g_lidar_topic,
            lidar_qos,
            std::bind(&ROSWrapper::stdMsgHandler, this, std::placeholders::_1),
            sub_opt_lidar);
  }

  /// output ======================================
  // 里程计/轨迹必须 RELIABLE：SC-PGO(aft_mapped_to_init)、Nav2 里程计、RViz
  // 显示全部以默认 RELIABLE 订阅，BEST_EFFORT 发布会因 QoS 不兼容而完全收不到。
  // 发布阻塞已由 imuOutputThread（有界队列）从 IMU 回调解耦，不再影响 LIO。
  auto viz_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable()
    .durability_volatile();

  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "lio/odom", viz_qos);

  pub_imu_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "lio/imu/odom", viz_qos);

  pub_robo_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "lio/robo/odom", viz_qos);

  // odom 重置计数：TRANSIENT_LOCAL + KEEP_LAST(1)，
  // 让晚于 LIO 启动的下游（飞控通信节点）也能立即拿到当前计数值。
  pub_odom_reset_counter_ = this->create_publisher<std_msgs::msg::UInt32>(
      "lio/odom_reset_counter",
      rclcpp::QoS(rclcpp::KeepLast(1))
          .reliable()
          .transient_local());
  {
    std_msgs::msg::UInt32 m;
    m.data = odom_reset_counter_.load();
    pub_odom_reset_counter_->publish(m);
  }

  pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
      "lio/path", viz_qos);

  auto pointcloud_qos = rclcpp::QoS(rclcpp::KeepLast(2))
    .best_effort()
    .durability_volatile();

  pub_cloud_world_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "lio/cloud_world", pointcloud_qos);

  pub_cloud_body_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "lio/cloud_body", pointcloud_qos);

  pub_cloud_body_dense_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "lio/cloud_body_dense", pointcloud_qos);

  // 退化签名：BEST_EFFORT，与视觉支路（方案 B）的 rclcpp::SensorDataQoS() 订阅兼容；
  // 离线回放时即便丢几帧也只影响注入强度，不影响正确性。
  pub_degeneracy_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "lio/degeneracy",
      rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile());

  // TF 必须 RELIABLE：RViz / Nav2 / tf2 均以默认 RELIABLE 订阅 /tf，
  // BEST_EFFORT 发布会因 QoS 不兼容导致 RViz 一条 TF 都收不到。
  // WiFi 弱网时的发布阻塞已由 imuOutputThread（有界队列）解耦，不再阻塞 IMU 回调。
  auto tf_qos = rclcpp::QoS(rclcpp::KeepLast(100))
    .reliable()
    .durability_volatile();
  tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(this, tf_qos);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


void ROSWrapper::imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg){
  auto t0 = std::chrono::high_resolution_clock::now();

  // 方案F: IMU 到达延迟诊断（ROS 时钟 vs 消息时间戳，sim/实时时钟均适用）
  const double now_secs = this->now().seconds();
  const double msg_secs = stampToSec(msg->header.stamp);
  const double arrival_lat = now_secs - msg_secs;
  static double last_arrival_warn = 0.0;
  if (arrival_lat > 0.05 && (now_secs - last_arrival_warn) > 1.0) {
    LOG(WARNING) << "[IMU Delay] arrival latency = " << arrival_lat * 1000.0
                 << " ms (msg.stamp=" << msg_secs << ", ros_now=" << now_secs << ")";
    last_arrival_warn = now_secs;
  }

  IMUData data;
  data.secs = msg_secs;

  V3 acc_raw(msg->linear_acceleration.x,
             msg->linear_acceleration.y,
             msg->linear_acceleration.z);
  V3 gyr_raw(msg->angular_velocity.x,
             msg->angular_velocity.y,
             msg->angular_velocity.z);
  // 先按 IMU 本体系做 scale 标定，再转到 lidar 系（两者只差旋转，标定应按传感器轴）
  if (g_imu_gyr_scale != 1.0) gyr_raw = gyr_raw * g_imu_gyr_scale;

  M3 R_IMU_to_Lidar = g_lidar_imu.R_.transpose();
  data.acc = R_IMU_to_Lidar * acc_raw;
  data.gyr = R_IMU_to_Lidar * gyr_raw;

  // 真回退（bag 重播/设备时钟重置）与轻微乱序必须区别对待：
  // 真回退要让整条时间轴重来；而"晚到一拍的旧样本"只是传输乱序，
  // 清空缓冲会把已经攒下的有效样本全扔掉——实测一次瞬时乱序就把 200 Hz
  // IMU 缓冲打成"永远只有 1 个样本"，雷达帧随后全部被坏帧护栏跳过，
  // Anchor 窗口塌成 4 s，整轮实验报废。这里只丢晚到的那个样本。
  constexpr double kImuLoopBackResetSec = 1.0;
  bool loop_back = false;
  bool ooo_drop = false;
  {
    std::lock_guard<std::mutex> lock(buffers_mutex_);
    const double back = last_timestamp_imu_ - data.secs;
    if (back > kImuLoopBackResetSec) {
      LOG(WARNING) << "imu loop back " << back * 1000.0 << " ms (>" 
                   << kImuLoopBackResetSec * 1000.0 << " ms), clear buffer";
      imu_buffer_.clear();
      imu_buffer_.push_back(data);
      last_timestamp_imu_ = data.secs;
      loop_back = true;
    } else if (back > 0.0) {
      ++imu_ooo_drop_;
      if (imu_ooo_drop_ % 100 == 1) {
        LOG(WARNING) << "imu out-of-order sample dropped: back=" << back * 1000.0
                     << " ms, total_dropped=" << imu_ooo_drop_
                     << ", buffered=" << imu_buffer_.size();
      }
      ooo_drop = true;
    } else {
      imu_buffer_.push_back(data);
      last_timestamp_imu_ = data.secs;
    }
  }
  if (loop_back || ooo_drop) {
    // eskf_->Reset();   // todo:
    return;
  }

  DynamicState imu_state, robo_state;
  if(eskf_->Predict(data, imu_state, robo_state)){
    // 高频 odom/TF 节流：默认 200 Hz（历史行为）；实时部署可降到 50 或直接关闭。
    if (shouldPublishImuOdom(data.secs)) {
      ImuOutputData out;
      out.imu_state = std::move(imu_state);
      out.robo_state = std::move(robo_state);
      out.timestamp = data.secs;
      out.receive_time = std::chrono::duration<double>(
          std::chrono::high_resolution_clock::now().time_since_epoch()).count();
      // 高频路不带协方差（实测 200Hz 协方差对 PX4 负担过大）：快照位恒置 0。
      // 带协方差的 10Hz /lio/odom 在 pub_odom 内直接取 eskf_ 后验。
      out.cov_diag_pv.setZero();
      out.cov_diag_rot.setZero();
      out.need_converge = true;
      std::lock_guard<std::mutex> lock(imu_output_mutex_);
      if(imu_output_queue_.size() > 10) {
        // 原先这里静默丢最旧帧：会掩盖 LiDAR/IMU 处理堆积，而堆积正是飞控侧
        // vision_data_stopped 的根因。改为计数 + 限频告警。
        imu_output_queue_.pop();
        const uint64_t dropped = imu_output_drops_.fetch_add(1) + 1;
        if (dropped % 50 == 1) {
          LOG(WARNING) << YELLOW << " ---> [odom queue] 积压丢帧累计 " << dropped
                       << " 帧，odom 时间戳相对实时已滞后" << RESET;
        }
      }
      imu_output_queue_.push(std::move(out));
    }
    imu_output_cv_.notify_one();
    
    auto t1 = std::chrono::high_resolution_clock::now();
    double lat_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    recordLatency("[IMU->TF]", lat_ms);
  }
}


#ifdef LIVOX_SUPPORT
void ROSWrapper::livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg){
  if(msg->point_num < 10) return;
  // 诊断：雷达帧从传感器时间戳到本进程接收的滞后漂移。
  // 雷达时间戳与墙钟基准不同（固定差约 2.05e8 s），因此只输出相对首帧的漂移：
  // 漂移持续增长 ⇒ 驱动/DDS 侧已积压（上游问题）；漂移平稳 ⇒ 滞后来自 LIO 处理。
  {
    static std::atomic<std::uint64_t> lag_n{0};
    static std::atomic<double> lag_first{0.0};
    const double now_s = std::chrono::duration<double>(
                             std::chrono::high_resolution_clock::now().time_since_epoch())
                             .count();
    const double lag = now_s - stampToSec(msg->header.stamp);
    const auto n = lag_n.fetch_add(1, std::memory_order_relaxed) + 1;
    double first = lag_first.load(std::memory_order_relaxed);
    if (n == 1) {
      lag_first.store(lag, std::memory_order_relaxed);
      first = lag;
    }
    if (n % 20 == 1) {
      LOG(INFO) << "[LidarIn] n=" << n << " stamp=" << stampToSec(msg->header.stamp)
                << " lag=" << lag << "s drift=" << (lag - first) << "s pts=" << msg->point_num;
    }
  }
  LidarData lidar_data;
  lidar_data.receive_time = std::chrono::duration<double>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();
  std::size_t ptsize = msg->point_num;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  lidar_data.pc->reserve(ptsize / g_filter_rate + 1);
  lidar_data.pc_full.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  lidar_data.pc_full->reserve(ptsize + 1);

  double offset_time = 0.0;
  // M.32 时延侧车修正量：每个点云只读一次原子量，避免逐点原子读开销。
  // 未开启标定/未锁定时恒为 0.0 ⇒ x + 0.0 逐位恒等，不改变任何计算结果。
  const double td_extra = g_lidar_time_offset_extra.load();
  // 一次遍历同时填充全量点(pc_full)与抽稀点(pc)，避免重复扫描 msg->points。
  // 全量点与抽稀点共用同一套 range/intensity 过滤，仅抽样比例不同。
  for(std::size_t _i = 0; _i < ptsize; ++_i){
    auto& pt = msg->points[_i];
    auto tag = pt.tag & 0x30;
    if (tag == 0x10 || tag == 0x00){
      auto dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if(dis > g_blind2 && dis < g_maxrange2){
        if(g_intensity_filter_en && pt.reflectivity < g_intensity_min) continue;
        offset_time = pt.offset_time * 1e-9;   // 帧尾基准不变
        const double pt_time = offset_time + g_lidar_time_offset + td_extra;
        lidar_data.pc_full->emplace_back(pt.x, pt.y, pt.z, pt.reflectivity, pt_time);
        // filter_rate 抽稀：与原先 for(_i=offset; _i<ptsize; _i+=rate) 等价
        if(_i >= (std::size_t)g_filter_offset &&
           (_i - (std::size_t)g_filter_offset) % (std::size_t)g_filter_rate == 0){
          lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.reflectivity, pt_time);
        }
      }
    }
  }
  // 更新偏移量（对称振荡采样：交替从两端选取，覆盖更均匀）
  if(g_filter_rate > 1 && g_enable_filter_offset) {
    static int g_filter_osc = 0;
    int half = g_filter_osc / 2;
    g_filter_offset = (g_filter_osc & 1) ? (g_filter_rate - 1 - half) : half;
    g_filter_osc = (g_filter_osc + 1) % g_filter_rate;
  }
  lidar_data.start_time = stampToSec(msg->header.stamp);
  lidar_data.end_time   = lidar_data.start_time + offset_time;
  lidar_data.frame_id = msg->header.frame_id;
  {
    std::lock_guard<std::mutex> lock(buffers_mutex_);
    lidar_buffer_.push_back(lidar_data);
  }
}
#endif


void ROSWrapper::stdMsgHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  if(msg->data.size() < 10) return;
  
  LidarData lidar_data;
  lidar_data.receive_time = std::chrono::duration<double>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  lidar_data.pc_full.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());

  double offset_time = 0.0;
  double dis = 0.0;

  // Zero-copy: resolve field offsets once, then iterate msg->data directly
  const auto& fields = msg->fields;
  const uint32_t point_step = msg->point_step;
  const uint32_t row_step = msg->row_step;
  const uint32_t height = msg->height;
  const uint32_t width = msg->width;
  const size_t num_points = static_cast<size_t>(height) * width;

  // Find field offsets
  uint32_t off_x = 0, off_y = 0, off_z = 0, off_intensity = 0, off_time = 0;
  bool has_intensity = false, has_time = false;
  uint8_t time_type = 0; // 0=none, 1=float, 2=double, 3=uint32, 4=uint64
  for (const auto& f : fields) {
    if (f.name == "x") off_x = f.offset;
    else if (f.name == "y") off_y = f.offset;
    else if (f.name == "z") off_z = f.offset;
    else if (f.name == "intensity") { off_intensity = f.offset; has_intensity = true; }
    else if (f.name == "timestamp" || f.name == "time" || f.name == "t") {
      off_time = f.offset;
      has_time = true;
      if (f.datatype == sensor_msgs::msg::PointField::FLOAT64) time_type = 2;
      else if (f.datatype == sensor_msgs::msg::PointField::FLOAT32) time_type = 1;
      else if (f.datatype == sensor_msgs::msg::PointField::UINT32) time_type = 3;
      else if (f.datatype == 7) time_type = 4;  // UINT64=7 (not defined in Humble PointField)
    }
  }

  const uint8_t* data_ptr = msg->data.data();

  // Helper to read a point field from raw data
  auto read_float = [&](size_t idx, uint32_t offset) -> float {
    float val;
    memcpy(&val, data_ptr + idx * point_step + offset, sizeof(float));
    return val;
  };

  lidar_data.pc->reserve(num_points / g_filter_rate + 1);
  lidar_data.pc_full->reserve(num_points + 1);
  lidar_data.start_time = stampToSec(msg->header.stamp);

  // Find min time for relative offset calculation
  double time_begin = 0.0;
  bool need_scan_min_max = (g_lidar_type == LID_TYPE::ROBOSENSE_AIRY && has_time);
  if (need_scan_min_max) {
    // ROBOSENSE: need global min time across all points
    double min_time = std::numeric_limits<double>::max();
    for (size_t i = 0; i < num_points; ++i) {
      double t = 0.0;
      if (time_type == 2) { memcpy(&t, data_ptr + i * point_step + off_time, sizeof(double)); }
      else if (time_type == 1) { float ft; memcpy(&ft, data_ptr + i * point_step + off_time, sizeof(float)); t = ft; }
      if (t < min_time && t > 0) min_time = t;
    }
    time_begin = min_time;
  } else if (has_time && g_lidar_type != LID_TYPE::GAZEBO) {
    // Read first valid point's time as reference
    bool ref_found = false;
    for (size_t i = 0; i < std::min(num_points, size_t(10)); ++i) {
      float x = read_float(i, off_x);
      float y = read_float(i, off_y);
      float z = read_float(i, off_z);
      if (validPoint(x, y, z)) {
        if (time_type == 2) {
          double t; memcpy(&t, data_ptr + i * point_step + off_time, sizeof(double));
          time_begin = t;
        } else if (time_type == 1) {
          float t; memcpy(&t, data_ptr + i * point_step + off_time, sizeof(float));
          time_begin = t;
        } else if (time_type == 3) {
          uint32_t t; memcpy(&t, data_ptr + i * point_step + off_time, sizeof(uint32_t));
          time_begin = static_cast<double>(t);
        } else if (time_type == 4) {
          uint64_t t; memcpy(&t, data_ptr + i * point_step + off_time, sizeof(uint64_t));
          time_begin = static_cast<double>(t);
        }
        ref_found = true;
        break;
      }
    }
    // 兜底：Livox 实测有 44/1602 帧的前 10 点全是无效点（距离为 0）。此时 time_begin
    // 会留在 0，而 Livox 逐点时间是绝对纳秒（≈1.79e18）⇒ max_offset_time≈1.8e9 s，
    // end_time 远超 IMU 时间戳，sync_measure 永远 false；更糟的是坏帧卡在 lidar_buf
    // 队首后，其后所有帧一起停摆（frame_num 从此不再增长）。这里整帧扫一遍取最小原始
    // 时间戳作为基准，只在坏帧触发，正常帧无额外开销。
    if (!ref_found) {
      double mn = std::numeric_limits<double>::max();
      for (size_t i = 0; i < num_points; ++i) {
        double t = 0.0;
        if (time_type == 2) {
          memcpy(&t, data_ptr + i * point_step + off_time, sizeof(double));
        } else if (time_type == 1) {
          float ft; memcpy(&ft, data_ptr + i * point_step + off_time, sizeof(float));
          t = static_cast<double>(ft);
        } else if (time_type == 3) {
          uint32_t ut; memcpy(&ut, data_ptr + i * point_step + off_time, sizeof(uint32_t));
          t = static_cast<double>(ut);
        } else if (time_type == 4) {
          uint64_t ut; memcpy(&ut, data_ptr + i * point_step + off_time, sizeof(uint64_t));
          t = static_cast<double>(ut);
        } else {
          continue;
        }
        if (std::isfinite(t) && t < mn) mn = t;
      }
      if (mn < std::numeric_limits<double>::max()) time_begin = mn;
    }
  }

  double max_offset_time = 0.0;
  // M.32 时延侧车修正量（每帧读一次；未标定时恒 0.0，逐位恒等）
  const double td_extra = g_lidar_time_offset_extra.load();

  // Helper: read offset_time for point at index k
  auto read_offset_time = [&](size_t k) -> double {
    if (has_time) {
      switch (time_type) {
        case 2: { double t; memcpy(&t, data_ptr + k * point_step + off_time, sizeof(double));
          // Livox/Ouster 的 FLOAT64 逐点时间是**纳秒**：Livox 直接给绝对 epoch 纳秒
          // （实测 ≈1.79e18），Ouster 给 0..1e8 的相对值。原实现只做 t-time_begin
          // 不做换算，max_offset_time 就变成 1e8 s，end_time 远超 IMU 时间戳，
          // sync_measure 永远 false（表现为只有 [IMU Lag]、frame_num 恒 0）。
          const double rel = t - time_begin;
          if (g_lidar_type == LID_TYPE::OUSTER || std::fabs(t) > 1e12) return rel * 1e-9;
          return rel; }
        case 1: { float t; memcpy(&t, data_ptr + k * point_step + off_time, sizeof(float));
          if (g_lidar_type == LID_TYPE::VELO16 || g_lidar_type == LID_TYPE::VELO32) return static_cast<double>(t);
          else return static_cast<double>(t) - time_begin; }
        case 3: { uint32_t t; memcpy(&t, data_ptr + k * point_step + off_time, sizeof(uint32_t));
          // Ouster 的逐点时间是 uint32 **纳秒**（实测 0..1e8，即 0..100 ms），
          // 与 time_begin（读首点原始 uint32）单位一致；原实现按微秒换算（t*1e-6），
          // 会让 end_time 放大 1000 倍（≈100 s），sync 永远等不到 IMU。
          // 仅在 OUSTER 下改为纳秒；其它雷达的 uint32 时间保持原微秒语义，避免误伤。
          if (g_lidar_type == LID_TYPE::OUSTER)
            return (static_cast<double>(t) - time_begin) * 1e-9;
          return t * 1e-6 - time_begin; }
        case 4: { uint64_t t; memcpy(&t, data_ptr + k * point_step + off_time, sizeof(uint64_t)); return t * 1e-9 - time_begin; }
        default: return 0.0;
      }
    } else {
      return static_cast<double>(k) / num_points * 0.1;
    }
  };

  // Helper: try to add point at index k, returns true if added
  auto try_add_point = [&](size_t k) {
    float x = read_float(k, off_x);
    float y = read_float(k, off_y);
    float z = read_float(k, off_z);
    if (!validPoint(x, y, z)) return false;
    float intensity = has_intensity ? read_float(k, off_intensity) : 1.0f;
    if (g_intensity_filter_en && intensity < g_intensity_min) return false;
    double ot = read_offset_time(k);
    lidar_data.pc->emplace_back(x, y, z, intensity, ot + g_lidar_time_offset + td_extra);
    if (ot > max_offset_time) max_offset_time = ot;
    return true;
  };

  const bool full_col_en = (g_full_column_interval > 0 && g_lidar_channels > 0);

  if (full_col_en) {
    // Column-aware iteration: process column by column to maintain time order.
    // For retained columns (every g_full_column_interval), emit all channels;
    // for other columns, only emit points matching filter_rate.
    size_t num_cols = num_points / g_lidar_channels;
    size_t col_idx = 0;
    size_t in_col_offset = g_filter_offset % g_lidar_channels;
    for (; col_idx < num_cols; ++col_idx) {
      size_t col_start = col_idx * g_lidar_channels;
      if (col_idx % (size_t)g_full_column_interval == 0) {
        // Retained column: emit all channels
        for (size_t k = col_start; k < col_start + g_lidar_channels && k < num_points; ++k) {
          try_add_point(k);
        }
      } else {
        // Normal column: emit only filter_rate-selected points
        for (size_t k = col_start + in_col_offset; k < col_start + g_lidar_channels && k < num_points; k += g_filter_rate) {
          try_add_point(k);
        }
      }
    }
    // Remaining points beyond full columns
    size_t remainder_start = num_cols * g_lidar_channels;
    for (size_t k = remainder_start + in_col_offset; k < num_points; k += g_filter_rate) {
      try_add_point(k);
    }
  } else {
    // No full column retention: simple filter_rate iteration
    for (size_t i = g_filter_offset; i < num_points; i += g_filter_rate) {
      try_add_point(i);
    }
  }

  // 全量点：遍历所有点（不做 filter_rate 抽稀），供 cloud_*_dense 持久化输出。
  // 复用与抽稀点相同的 range/intensity 过滤，仅抽样比例不同。
  for (size_t i = 0; i < num_points; ++i) {
    float x = read_float(i, off_x);
    float y = read_float(i, off_y);
    float z = read_float(i, off_z);
    if (!validPoint(x, y, z)) continue;
    float intensity = has_intensity ? read_float(i, off_intensity) : 1.0f;
    if (g_intensity_filter_en && intensity < g_intensity_min) continue;
    double ot = read_offset_time(i);
    if (ot > max_offset_time) max_offset_time = ot;
    lidar_data.pc_full->emplace_back(x, y, z, intensity, ot + g_lidar_time_offset + td_extra);
  }

  // 更新偏移量（对称振荡采样：交替从两端选取，覆盖更均匀）
  if(g_filter_rate > 1 && g_enable_filter_offset) {
    static int g_filter_osc = 0;
    int half = g_filter_osc / 2;
    g_filter_offset = (g_filter_osc & 1) ? (g_filter_rate - 1 - half) : half;
    g_filter_osc = (g_filter_osc + 1) % g_filter_rate;
  }
  lidar_data.end_time = lidar_data.start_time + max_offset_time;
  lidar_data.frame_id = msg->header.frame_id;
  {
    std::lock_guard<std::mutex> lock(buffers_mutex_);
    lidar_buffer_.push_back(lidar_data);
  }
}


ROSWrapper::~ROSWrapper() {
  imu_output_running_ = false;
  imu_output_cv_.notify_one();
  if (imu_output_thread_.joinable()) {
    imu_output_thread_.join();
  }
}


void ROSWrapper::notifyOdomReset(const std::string& reason)
{
  // 计数单调递增，绝不清零；PX4 侧 reset_counter 是 uint8，由下游做 &0xFF 自然回绕。
  const uint32_t n = odom_reset_counter_.fetch_add(1) + 1;
  LOG(WARNING) << YELLOW << " ---> [odom reset] 声明 odom 内部状态/参考系重置 #" << n
               << " 原因: " << reason << RESET;
  if (pub_odom_reset_counter_) {
    std_msgs::msg::UInt32 m;
    m.data = n;
    pub_odom_reset_counter_->publish(m);
  }
}


void ROSWrapper::zeroOdometryCovariance(nav_msgs::msg::Odometry& odom)
{
  // 高频路（/lio/imu/odom、/lio/robo/odom）不带协方差：实测 200Hz 协方差输出
  // 对 PX4 造成巨大计算负担并带来不可预计行为。下游（通信节点）发给 PX4 的
  // 是这一路，全 0 协方差由 PX4 EKF2 用 max(EKF2_EVP_NOISE², 0.01²) 抬回
  // 名义噪声，不会被当成"绝对真值"。带协方差的 10Hz /lio/odom 供 safety 监看。
  odom.pose.covariance.fill(0.0);
  odom.twist.covariance.fill(0.0);
}


void ROSWrapper::fillOdometryCovariance(nav_msgs::msg::Odometry& odom,
                                       const BASIC::M3& R_world_body,
                                       const BASIC::V6& cov_diag_pv,
                                       const BASIC::V3& cov_diag_rot,
                                       bool need_converge,
                                       double lidar_n_eff)
{
  if (!g_odom_cov_en) {
    // 协方差输出关闭（默认）：保持全 0，不做任何后验方差计算。
    // 下游（通信节点）不再做 Σ_NED=RΣRᵀ 变换，PX4 ev_pos_control 用
    // max(EKF2_EVP_NOISE², 0.01²) 抬回名义噪声，不会被当成"绝对真值"。
    odom.pose.covariance.fill(0.0);
    odom.twist.covariance.fill(0.0);
    return;
  }

  // ---- 地板值（不可省）----
  // 信息式更新在退化方向（长走廊、单面墙、单向特征）上会让 P 收敛到极小，
  // 把 1e-9 发给飞控等于告诉它"该方向 odom 是绝对真值"，会直接把估计器带崩。
  constexpr double POS_FLOOR  = 1e-4;   // (0.01 m)^2
  constexpr double VEL_FLOOR  = 1e-3;   // (0.03 m/s)^2
  constexpr double ROT_FLOOR  = 1e-4;   // (0.01 rad)^2
  constexpr double GYRO_FLOOR = 1e-2;   // 角速度无后验可用，给保守常数，绝不留 0

  // 迭代用满仍未收敛 => 该帧后验不可信，膨胀后再发。
  // 只需让该帧在 EKF2_EVP_GATE=5 / EVV_GATE=3 下超门即可，不必给极大值：
  // 保留数值有限还能继续约束姿态，同时不满足 §4.4 第5条"test_ratio<0.1"的
  // 速度接管条件 —— 即退化帧会自动拒绝速度接管，但仍贡献位置/姿态信息。
  // 失效帧膨胀：P 是滤波器"自认为"的不确定度，激光稀疏但仍被接受时 P 会被
  // 持续压低，报给飞控的仍是 1e-4（1cm），PX4 的 EVP_GATE 不会拒绝。实测
  // home1 270-300s 真实速度已 10 m/s，cov_px 中位仍恒为 1e-4，属过度自信。
  // 用外部可观测性（有效点数）强制膨胀：n_eff>=200 时不膨胀，越稀疏越接近 1e4。
  const double h_neff = std::min(1.0, std::max(0.0, lidar_n_eff / 400.0));
  const double gap = std::max(0.0, (0.5 - h_neff) / 0.5);
  const double deg_infl = 1.0 + 9999.0 * gap * gap;

  const double infl = (need_converge ? 4.0 : 1.0) * deg_infl;

  // 非有限值的兜底（必须显式判）：std::max(a,b) 等价于 (a<b)?b:a，NaN 参与
  // 比较恒为 false，于是会原样返回 NaN —— 地板值对 NaN 完全无效。而 P_ 是
  // float 矩阵，ESKF 里 P_pred/J_prior/Yk 三处裸 .inverse() 在病态/退化输入下
  // 就可能产出 inf 或 NaN，并沿对角一路进到 odom.covariance。
  // 这里不给地板值（那等于宣称"高度可信"），给一个明确的"不可用"量级，
  // 让下游有依据把这一帧降权。
  constexpr double INVALID_VAR = 1e4;
  auto floored = [infl](double v, double f) {
    if (!std::isfinite(v)) return INVALID_VAR;
    return std::max(v * infl, f);
  };

  // ROS 6x6 行主序: pose=[x y z roll pitch yaw], twist=[vx vy vz wx wy wz]
  auto& pc = odom.pose.covariance;
  pc.fill(0.0);
  pc[0 * 6 + 0] = floored(cov_diag_pv(0), POS_FLOOR);
  pc[1 * 6 + 1] = floored(cov_diag_pv(1), POS_FLOOR);
  pc[2 * 6 + 2] = floored(cov_diag_pv(2), POS_FLOOR);
  // 注意: P 的 0..2 是机体系左扰动旋转误差，与 ROS 的世界系欧拉角约定不一致。
  // PX4 只使用 orientation_variance[0..2]，此处按对角近似；勿直接喂给 Nav2 等下游。
  pc[3 * 6 + 3] = floored(cov_diag_rot(0), ROT_FLOOR);
  pc[4 * 6 + 4] = floored(cov_diag_rot(1), ROT_FLOOR);
  pc[5 * 6 + 5] = floored(cov_diag_rot(2), ROT_FLOOR);

  // 速度块: P 的 v 分量是世界系，而 REP-103 要求 twist 表达在 child_frame(body)。
  // 必须与 v_body = R^T * v_world 做同样的合同变换，否则姿态一动协方差主轴就错位，
  // 下游会看到"速度和它的不确定性不在同一坐标系"。
  auto& tc = odom.twist.covariance;
  tc.fill(0.0);
  Eigen::Matrix<double, 3, 3> Pvv_world = Eigen::Matrix<double, 3, 3>::Zero();
  Pvv_world(0, 0) = static_cast<double>(cov_diag_pv(3));
  Pvv_world(1, 1) = static_cast<double>(cov_diag_pv(4));
  Pvv_world(2, 2) = static_cast<double>(cov_diag_pv(5));
  const Eigen::Matrix<double, 3, 3> Rw = R_world_body.template cast<double>();
  const Eigen::Matrix<double, 3, 3> Pvv_body = Rw.transpose() * Pvv_world * Rw;

  tc[0 * 6 + 0] = floored(Pvv_body(0, 0), VEL_FLOOR);
  tc[1 * 6 + 1] = floored(Pvv_body(1, 1), VEL_FLOOR);
  tc[2 * 6 + 2] = floored(Pvv_body(2, 2), VEL_FLOOR);
  tc[3 * 6 + 3] = GYRO_FLOOR;
  tc[4 * 6 + 4] = GYRO_FLOOR;
  tc[5 * 6 + 5] = GYRO_FLOOR;
}


void ROSWrapper::imuOutputThread() {
  while (imu_output_running_) {
    ImuOutputData data;
    {
      std::unique_lock<std::mutex> lock(imu_output_mutex_);
      imu_output_cv_.wait(lock, [this] {
        return !imu_output_queue_.empty() || !imu_output_running_;
      });
      if (!imu_output_running_ && imu_output_queue_.empty()) {
        return;
      }
      data = std::move(imu_output_queue_.front());
      imu_output_queue_.pop();
    }

    // 在独立线程中构建并发布 odom + TF，不阻塞 IMU 回调
    const V3 v_imu_body = data.imu_state.R.transpose() * data.imu_state.v;

    nav_msgs::msg::Odometry odom_imu;
    odom_imu.header.stamp = toRosTime(data.timestamp);
    odom_imu.header.frame_id = g_world_frame;
    odom_imu.child_frame_id = g_imu_frame;
    odom_imu.pose.pose.position.x = data.imu_state.p(0);
    odom_imu.pose.pose.position.y = data.imu_state.p(1);
    odom_imu.pose.pose.position.z = data.imu_state.p(2);
    {
      Quat q(data.imu_state.R);
      q.normalize();
      odom_imu.pose.pose.orientation.x = q.x();
      odom_imu.pose.pose.orientation.y = q.y();
      odom_imu.pose.pose.orientation.z = q.z();
      odom_imu.pose.pose.orientation.w = q.w();
    }
    odom_imu.twist.twist.linear.x = v_imu_body[0];
    odom_imu.twist.twist.linear.y = v_imu_body[1];
    odom_imu.twist.twist.linear.z = v_imu_body[2];
    odom_imu.twist.twist.angular.x = data.imu_state.w(0);
    odom_imu.twist.twist.angular.y = data.imu_state.w(1);
    odom_imu.twist.twist.angular.z = data.imu_state.w(2);
    zeroOdometryCovariance(odom_imu);   // 高频路不带协方差（见函数注释）
    pub_imu_odom_->publish(odom_imu);

    // robot body-frame odom
    {
      const V3 r_imu_to_robo = -g_odom_robo.R_ * g_odom_robo.t_;
      const V3 v_robo_body = g_odom_robo.R_.transpose() * (v_imu_body + data.imu_state.w.cross(r_imu_to_robo));
      const V3 w_robo_body = g_odom_robo.R_.transpose() * data.imu_state.w;

      nav_msgs::msg::Odometry odom_robo;
      odom_robo.header.stamp = toRosTime(data.timestamp);
      odom_robo.header.frame_id = g_world_frame;
      odom_robo.child_frame_id = "base_link";
      odom_robo.pose.pose.position.x = data.robo_state.p(0);
      odom_robo.pose.pose.position.y = data.robo_state.p(1);
      odom_robo.pose.pose.position.z = data.robo_state.p(2);
      {
        Quat q(data.robo_state.R);
        q.normalize();
        odom_robo.pose.pose.orientation.x = q.x();
        odom_robo.pose.pose.orientation.y = q.y();
        odom_robo.pose.pose.orientation.z = q.z();
        odom_robo.pose.pose.orientation.w = q.w();
      }
      odom_robo.twist.twist.linear.x = v_robo_body[0];
      odom_robo.twist.twist.linear.y = v_robo_body[1];
      odom_robo.twist.twist.linear.z = v_robo_body[2];
      odom_robo.twist.twist.angular.x = w_robo_body[0];
      odom_robo.twist.twist.angular.y = w_robo_body[1];
      odom_robo.twist.twist.angular.z = w_robo_body[2];
      // 通信节点发给 PX4 的就是这一路 (/lio/robo/odom)：协方差恒为 0，
      // 由 PX4 EKF2 抬回名义噪声；带协方差的 10Hz /lio/odom 供 safety 监看。
      zeroOdometryCovariance(odom_robo);
      pub_robo_odom_->publish(odom_robo);
      if(data.receive_time > 0.0){
        auto now_s = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        double lat_ms = (now_s - data.receive_time) * 1000.0;
        recordLatency("[IMU->Odom]", lat_ms);
      }
    }

    // Fast TF: base_footprint
    if (g_fast_tf && g_footprint_pub_en) {
      geometry_msgs::msg::TransformStamped tf_footprint;
      tf_footprint.header.stamp = toRosTime(data.timestamp);
      tf_footprint.header.frame_id = g_world_frame;
      tf_footprint.child_frame_id = g_tf_base_footprint_frame;
      tf_footprint.transform.translation.x = data.imu_state.p(0);
      tf_footprint.transform.translation.y = data.imu_state.p(1);
      tf_footprint.transform.translation.z = data.imu_state.p(2);

      Eigen::Vector3f world_up;
      if (g_ref_gravity_axis == 0)      world_up = Eigen::Vector3f(-1, 0, 0);
      else if (g_ref_gravity_axis == 1) world_up = Eigen::Vector3f(0, -1, 0);
      else                              world_up = Eigen::Vector3f(0, 0, 1);

      Eigen::Quaternionf q_imu(data.imu_state.R);
      q_imu.normalize();

      Eigen::Vector3f lidar_fwd_local;
      if (g_ref_gravity_axis == 0)      lidar_fwd_local = Eigen::Vector3f::UnitZ();
      else if (g_ref_gravity_axis == 1) lidar_fwd_local = Eigen::Vector3f::UnitZ();
      else                              lidar_fwd_local = Eigen::Vector3f::UnitX();
      Eigen::Vector3f lidar_fwd_world = q_imu * lidar_fwd_local;

      Eigen::Vector3f fwd_proj = lidar_fwd_world - (lidar_fwd_world.dot(world_up)) * world_up;
      if (fwd_proj.norm() < 1e-6) {
        Eigen::Vector3f alt_local = (g_ref_gravity_axis == 2) ? Eigen::Vector3f::UnitY() : Eigen::Vector3f::UnitX();
        Eigen::Vector3f alt_world = q_imu * alt_local;
        fwd_proj = alt_world - (alt_world.dot(world_up)) * world_up;
        if (fwd_proj.norm() < 1e-6) continue;
      }
      fwd_proj.normalize();

      Eigen::Vector3f foot_x = fwd_proj;
      Eigen::Vector3f foot_z = world_up;
      Eigen::Vector3f foot_y = foot_z.cross(foot_x);
      if (foot_y.norm() < 1e-6) continue;
      foot_y.normalize();

      Eigen::Matrix3f foot_mat;
      foot_mat.col(0) = foot_x;
      foot_mat.col(1) = foot_y;
      foot_mat.col(2) = foot_z;

      Eigen::Quaternionf q_foot(foot_mat);
      q_foot.normalize();

      tf_footprint.transform.rotation.x = q_foot.x();
      tf_footprint.transform.rotation.y = q_foot.y();
      tf_footprint.transform.rotation.z = q_foot.z();
      tf_footprint.transform.rotation.w = q_foot.w();

      tf_broadcaster_->sendTransform(tf_footprint);
    }
  }
}


bool ROSWrapper::sync_measure(MeasureGroup& meas){
  // process 线程与 IMU/lidar 回调线程并发，整个缓冲区操作持锁
  std::lock_guard<std::mutex> lock(buffers_mutex_);

  if (lidar_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }

  if (!lidar_pushed_) {
    // 实时性保护：处理能力低于雷达帧率时，积压超过上限就丢弃最旧帧，
    // 把输出时间戳拉回当前时刻（否则滞后无界增长，彩色支路永远配不上图）。
    // 丢帧仍保有 7~10 Hz 的连续帧，IMU 区间由 sync_measure 按新帧 end_time 消费，
    // 不会留下空洞。
    if (g_max_lidar_backlog > 0) {
      static std::atomic<std::uint64_t> dropped_total{0};
      while (lidar_buffer_.size() > static_cast<std::size_t>(g_max_lidar_backlog)) {
        lidar_buffer_.pop_front();
        const auto n = dropped_total.fetch_add(1, std::memory_order_relaxed) + 1;
        if (n % 50 == 1) {
          LOG(WARNING) << "[Lidar Drop] 积压超限丢最旧帧：累计=" << n
                       << " 上限=" << g_max_lidar_backlog;
        }
      }
    }
    meas.lidar = lidar_buffer_.front();
    lidar_pushed_ = true;
  }

  if(last_timestamp_lidar_ > meas.lidar.end_time){
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    return false;
  }

  if (last_timestamp_imu_ < meas.lidar.end_time) {
    // 方案F: IMU 延迟诊断 — 当 IMU 数据落后于 Lidar 时说明 IMU 处理被阻塞
    static double last_warn_stamp = 0;
    double imu_lag = meas.lidar.end_time - last_timestamp_imu_;
    if (imu_lag > 0.05 && (meas.lidar.end_time - last_warn_stamp) > 1.0) {
      char buf[256];
      snprintf(buf, sizeof(buf), "[IMU Lag] IMU lags behind lidar by %.1f ms. IMU buf: %zu, lidar buf: %zu",
               imu_lag * 1000, imu_buffer_.size(), lidar_buffer_.size());
      LOG(WARNING) << buf;
      last_warn_stamp = meas.lidar.end_time;
    }
    return false;
  }

  double imu_time = imu_buffer_.front().secs;
  meas.imu.clear();
  while ((!imu_buffer_.empty()) && (imu_time < meas.lidar.end_time)) {
    imu_time = imu_buffer_.front().secs;
    if (imu_time > meas.lidar.end_time) break;
    meas.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  last_timestamp_lidar_ = meas.lidar.end_time;
  lidar_buffer_.pop_front();
  lidar_pushed_ = false;
  return true;
}


/// 看门狗专用：try_lock 拿不到就立刻返回 false，避免看门狗自己被卡住。
bool ROSWrapper::getBufferStats(size_t& lidar_n, size_t& imu_n,
                                double& last_imu_stamp, double& last_lidar_stamp){
  std::unique_lock<std::mutex> lock(buffers_mutex_, std::try_to_lock);
  if(!lock.owns_lock()){ return false; }
  lidar_n = lidar_buffer_.size();
  imu_n = imu_buffer_.size();
  last_imu_stamp = last_timestamp_imu_;
  last_lidar_stamp = last_timestamp_lidar_;
  return true;
}


void ROSWrapper::pub_odom(const NavState& state, const V3& body_omega){
  // /lio/odom 与 IMU 频率的 /lio/robo/odom 输出同一坐标系（base_link 本体），
  // 对激光修正后的 IMU 系后验施加 lio.extrinsic.odom_robo。
  // 这样 10Hz 的激光修正结果与 200Hz 的 IMU 推算结果可直接互换，不再差一个本体偏移。
  const V3 r_imu_to_robo = -g_odom_robo.R_ * g_odom_robo.t_;
  const V3 robo_position = state.R.R_ * r_imu_to_robo + state.p;
  const M3 robo_rotation = state.R.R_ * g_odom_robo.R_;

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = g_world_frame;
  odom.child_frame_id = "base_link";

  odom.header.stamp = toRosTime(state.timestamp);
  odom.pose.pose.position.x = robo_position[0];
  odom.pose.pose.position.y = robo_position[1];
  odom.pose.pose.position.z = robo_position[2];

  // IMU 系四元数仍用于下面 world->imu 的 TF 广播
  V4 temp_q = state.R.coeffs();
  {
    Quat robo_quat(robo_rotation);
    robo_quat.normalize();
    odom.pose.pose.orientation.x = robo_quat.x();
    odom.pose.pose.orientation.y = robo_quat.y();
    odom.pose.pose.orientation.z = robo_quat.z();
    odom.pose.pose.orientation.w = robo_quat.w();
  }

  // REP 103: twist must be in child_frame (base_link body frame)
  const V3 v_imu_body = state.R.R_.transpose() * state.v;
  const V3 v_body = g_odom_robo.R_.transpose() * (v_imu_body + body_omega.cross(r_imu_to_robo));
  odom.twist.twist.linear.x = v_body[0];
  odom.twist.twist.linear.y = v_body[1];
  odom.twist.twist.linear.z = v_body[2];

  const V3 w_body = g_odom_robo.R_.transpose() * body_omega;
  odom.twist.twist.angular.x = w_body[0];
  odom.twist.twist.angular.y = w_body[1];
  odom.twist.twist.angular.z = w_body[2];

  // /lio/odom(10Hz) 是全系统唯一带协方差的 odom 路，供 safety 做 LIO 健康监看
  // （POSITION/VELOCITY/EV_OUTPUT 能力位判据）。此处在 process 线程内即时快照，
  // 与 state.timestamp 同属最近一次观测更新后的后验。高频路协方差恒为 0。
  if (eskf_) {
    fillOdometryCovariance(odom, robo_rotation, eskf_->GetCovDiagPV(),
                           eskf_->GetCovDiagRot(), eskf_->NeedConverge(),
                           eskf_->GetLidarNEff());
  }
  pub_odom_->publish(odom);    // base_link frame -> lidar frequency

  // 退化签名与 /lio/odom 同频（10 Hz），同源取自 degeneracySnapshot()：
  // 放在这里而不是函数末尾，避免 base_footprint 分支提前 return 时漏发。
  pub_degeneracy();

  if(g_2_robot){
    static auto pub_msg2uav_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", 10);
    msg2uav_.header.stamp = odom.header.stamp;
    msg2uav_.pose.position.x = robo_position[0];
    msg2uav_.pose.position.y = robo_position[1];
    msg2uav_.pose.position.z = robo_position[2];
    Quat robo_quat(robo_rotation);
    msg2uav_.pose.orientation.w = robo_quat.w();
    msg2uav_.pose.orientation.x = robo_quat.x();
    msg2uav_.pose.orientation.y = robo_quat.y();
    msg2uav_.pose.orientation.z = robo_quat.z();
    pub_msg2uav_->publish(msg2uav_);
  }

  if((last_path_point_ - robo_position).norm() > 0.1)
  {
    path_.header.stamp = odom.header.stamp;
    geometry_msgs::msg::PoseStamped point;
    point.pose = odom.pose.pose;
    path_.poses.push_back(point);
    pub_path_->publish(path_);
    last_path_point_ = robo_position;
  }

  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.stamp = odom.header.stamp;
  tf_msg.header.frame_id = g_world_frame;
  tf_msg.child_frame_id = g_imu_frame;

  tf_msg.transform.translation.x = state.p[0];
  tf_msg.transform.translation.y = state.p[1];
  tf_msg.transform.translation.z = state.p[2];

  tf_msg.transform.rotation.x = temp_q.x();
  tf_msg.transform.rotation.y = temp_q.y();
  tf_msg.transform.rotation.z = temp_q.z();
  tf_msg.transform.rotation.w = temp_q.w();

  tf_broadcaster_->sendTransform(tf_msg);

  // Publish base_footprint transform
  // 在 pub_odom 函数末尾添加以下逻辑
if (g_footprint_pub_en) {
    geometry_msgs::msg::TransformStamped tf_footprint;
    tf_footprint.header.stamp = odom.header.stamp;
    tf_footprint.header.frame_id = g_world_frame;
    tf_footprint.child_frame_id = g_tf_base_footprint_frame;

    // 1. 位置保持不变
    tf_footprint.transform.translation.x = state.p[0];
    tf_footprint.transform.translation.y = state.p[1];
    tf_footprint.transform.translation.z = state.p[2];

    // 2. 根据重力配置确定世界系下的“天顶”向量 (Up Vector)
    // 根据你的矩阵逻辑：
    // case 0 (+X方向重力): Up 是 -X
    // case 1 (+Y方向重力): Up 是 -Y
    // case 2 (-Z方向重力): Up 是 +Z
    Eigen::Vector3f world_up;
    if (g_ref_gravity_axis == 0)      world_up = Eigen::Vector3f(-1, 0, 0);
    else if (g_ref_gravity_axis == 1) world_up = Eigen::Vector3f(0, -1, 0);
    else                              world_up = Eigen::Vector3f(0, 0, 1);

    // 3. 获取雷达在世界系下的“前方”向量
    // 前两个代码块中也有相同的逻辑，需要同步修改
    // 注意：不要使用与 g_ref_gravity_axis 相同的轴，否则重力对齐后该轴与 world_up 平行
    // 导致投影到水平面为零向量，normalize() 产生 NaN 使 TF 被丢弃
    // 重力对齐后 g_ref_gravity_axis 对应的轴与 world_up 平行，必须避让
    Eigen::Vector3f lidar_fwd_local;
    if (g_ref_gravity_axis == 0)      lidar_fwd_local = Eigen::Vector3f::UnitZ();  // gravity on X -> Z
    else if (g_ref_gravity_axis == 1) lidar_fwd_local = Eigen::Vector3f::UnitZ();  // gravity on Y -> Z
    else                              lidar_fwd_local = Eigen::Vector3f::UnitX();  // gravity on Z -> X (default)
    Eigen::Vector3f lidar_fwd_world = Eigen::Quaternionf(state.R.R_) * lidar_fwd_local;

    // 4. 将“前方”投影到水平面上 (剔除掉重力方向的分量)
    Eigen::Vector3f fwd_proj = lidar_fwd_world - (lidar_fwd_world.dot(world_up)) * world_up;
    if (fwd_proj.norm() < 1e-6) {
        // 备选：尝试另一个非重力轴
        Eigen::Vector3f alt_local = (g_ref_gravity_axis == 2) ? Eigen::Vector3f::UnitY() : Eigen::Vector3f::UnitX();
        Eigen::Vector3f alt_world = Eigen::Quaternionf(state.R.R_) * alt_local;
        fwd_proj = alt_world - (alt_world.dot(world_up)) * world_up;
        if (fwd_proj.norm() < 1e-6) return;  // 仍不可用则跳过本次发布
    }
    fwd_proj.normalize();

    // 5. 构造 base_footprint 的旋转矩阵 (正交基)
    // X 轴 = 投影后的前方
    // Z 轴 = 世界系天顶
    // Y 轴 = Z 叉乘 X
    Eigen::Vector3f foot_x = fwd_proj;
    Eigen::Vector3f foot_z = world_up;
    Eigen::Vector3f foot_y = foot_z.cross(foot_x);
    if (foot_y.norm() < 1e-6) return;
    foot_y.normalize();

    Eigen::Matrix3f foot_mat;
    foot_mat.col(0) = foot_x;
    foot_mat.col(1) = foot_y;
    foot_mat.col(2) = foot_z;

    // 6. 转换为四元数发布
    Eigen::Quaternionf q_foot(foot_mat);
    q_foot.normalize();

    tf_footprint.transform.rotation.x = q_foot.x();
    tf_footprint.transform.rotation.y = q_foot.y();
    tf_footprint.transform.rotation.z = q_foot.z();
    tf_footprint.transform.rotation.w = q_foot.w();

    tf_broadcaster_->sendTransform(tf_footprint);
}

  // tf_msg.child_frame_id = "god";
  // tf_msg.transform.rotation.x = 0.0;
  // tf_msg.transform.rotation.y = 0.0;
  // tf_msg.transform.rotation.z = 0.0;
  // tf_msg.transform.rotation.w = 1.0;
  // tf_broadcaster_->sendTransform(tf_msg);

}


void ROSWrapper::pub_degeneracy(){
  if (!pub_degeneracy_) {
    return;
  }
  // 布局与 superlivo_common::DegeneracySignature 对齐：
  //   [cond6, axes, active, cond_t, evt_min, ux, uy, uz]
  // cond6 / cond_t 都填快照里的同一个条件数：本仓 lio.observe.degen_trans_only=1 时
  // 该值即平移 3x3 块的条件数；该标志为 0 时则是完整 6x6。发布端不做二次分解，
  // 避免与 LIO 内部退化检测各算一套。evt_min 未被视觉支路消费，恒 0。
  std_msgs::msg::Float64MultiArray msg;
  msg.data.assign(8, 0.0);
  if (super_lio_) {
    const auto snap = super_lio_->degeneracySnapshot();
    if (snap.ok && snap.has_info) {
      msg.data[0] = snap.cond;
      msg.data[1] = static_cast<double>(snap.axes);
      msg.data[2] = snap.active ? 1.0 : 0.0;
      msg.data[3] = snap.cond;
      msg.data[4] = 0.0;
      const Eigen::Vector3d dir = snap.dir_w;
      if (dir.norm() > 1e-6) {
        const Eigen::Vector3d u = dir.normalized();
        msg.data[5] = u.x();
        msg.data[6] = u.y();
        msg.data[7] = u.z();
      }
    }
  }
  // Float64MultiArray 没有 header 字段；B 侧按消息到达时间判超时（degen_timeout_），
  // 因此这里只填载荷。
  pub_degeneracy_->publish(msg);
}

void ROSWrapper::pub_cloud_world(const CloudPtr& pc, double time){
  if (!g_pub_cloud_world) return;
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = g_world_frame;
  cloud.header.stamp = toRosTime(time);
  pub_cloud_world_->publish(cloud);
}


void ROSWrapper::pub_cloud_world_undistort_only(const CloudPtr& pc, double time, const std::string& lidar_frame){
  if (!g_pub_cloud_world) return;
  try{
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf_buffer_->lookupTransform(
        g_world_frame, lidar_frame, tf2::TimePointZero);
    
    sensor_msgs::msg::PointCloud2 cloud_in, cloud_out;
    pcl::toROSMsg(*pc, cloud_in);
    cloud_in.header.frame_id = lidar_frame;
    cloud_in.header.stamp = toRosTime(time);
    
    tf2::doTransform(cloud_in, cloud_out, transform_stamped);
    cloud_out.header.frame_id = g_world_frame;
    pub_cloud_world_->publish(cloud_out);
  } catch (const tf2::TransformException& ex) {
    LOG(WARNING) << YELLOW << " ---> [Undistort] TF lookup failed: " << ex.what() 
                 << ", publishing without transform to " << g_world_frame << RESET;
    sensor_msgs::msg::PointCloud2 cloud;
    pcl::toROSMsg(*pc, cloud);
    cloud.header.frame_id = g_world_frame;
    cloud.header.stamp = toRosTime(time);
    pub_cloud_world_->publish(cloud);
  }
}


void ROSWrapper::pub_cloud_body(const CloudPtr& pc, double time){
  if (!g_pub_cloud_body) return;
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = g_imu_frame;
  cloud.header.stamp = toRosTime(time);
  pub_cloud_body_->publish(cloud);
}


void ROSWrapper::pub_cloud_body_dense(const CloudPtr& pc, double time){
  if (!g_pub_cloud_body_dense) return;
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = g_imu_frame;
  cloud.header.stamp = toRosTime(time);
  pub_cloud_body_dense_->publish(cloud);
}


void ROSWrapper::pub_cloud_undistort_only(const CloudPtr& pc, double time, const std::string& lidar_frame){
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = lidar_frame;
  cloud.header.stamp = toRosTime(time);
  pub_cloud_body_->publish(cloud);
}


void ROSWrapper::pub_cloud2planner(const CloudPtr& pc, double time){
  static auto pub_cloud2robot_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "lio/robo/cloud_world", rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile());
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = "world";
  cloud.header.stamp = toRosTime(time);
  pub_cloud2robot_->publish(cloud);
}


void ROSWrapper::pub_cloud_body_pose(const CloudPtr& pc, 
  const NavState& state)
{
  static auto pub_cloud_body_pose_ =
    this->create_publisher<super_lio::msg::CloudPose>(
        "/lio/body/cloud_pose", rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile());
  super_lio::msg::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = toRosTime(state.timestamp); 
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];

  pub_cloud_body_pose_->publish(cloud_pose);
}


void ROSWrapper::pub_cloud_world_pose(const CloudPtr& pc, 
   const NavState& state)
{
  static auto pub_cloud_world_pose_ =
    this->create_publisher<super_lio::msg::CloudPose>(
        "/lio/world/cloud_pose", rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile());
  super_lio::msg::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = toRosTime(state.timestamp);  
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];
  pub_cloud_world_pose_->publish(cloud_pose);
}


void ROSWrapper::pub_processing_time(double time, 
  double current_time, double mean_time, double std_time)
{
  static auto pub_processing_time_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lio/processing_time",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile());
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = toRosTime(time);
  msg.pose.position.x = current_time;
  msg.pose.position.y = mean_time;
  msg.pose.position.z = std_time;
  pub_processing_time_->publish(msg);
}

void ROSWrapper::recordLatency(const std::string& name, double latency_ms){
  latency_timer_.Record(name, latency_ms);
}

void ROSWrapper::printLatencies(){
  latency_timer_.PrintAll();
}

void ROSWrapper::clearLatencies(){
  latency_timer_.Clear();
}


void ROSWrapper::set_global_map(const BASIC::CloudPtr& global_map){
  pcl::toROSMsg(*global_map, global_map_msg_);
  global_map_msg_.header.frame_id = "world";

  static auto global_map_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "lio/global_map", rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile());

  static auto global_map_timer =
    this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        static int count = -1;
        static int publish_interval = 1;

        count++;
        if (count % publish_interval != 0) {
          return;
        }

        count = 0;
        publish_interval++;
        if (publish_interval > 10) {
          publish_interval = 10;
        }
        global_map_msg_.header.stamp = this->now();
        global_map_pub->publish(global_map_msg_);
      });
}


void ROSWrapper::set_initial_data(BASIC::SE3& init_pose, bool& flg_get_init_guess, bool flg_finish_init)
{
  static auto init_pose_sub =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        [this, &init_pose, &flg_get_init_guess](
          const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
        {
          V3 init_translation;
          init_translation << msg->pose.pose.position.x,
                              msg->pose.pose.position.y,
                              0.2;

          double x = msg->pose.pose.orientation.x;
          double y = msg->pose.pose.orientation.y;
          double z = msg->pose.pose.orientation.z;
          double w = msg->pose.pose.orientation.w;

          Quat init_rotation(w, x, y, z);

          init_pose = BASIC::SE3(SO3(init_rotation.toRotationMatrix()), init_translation);

          flg_get_init_guess = true;

          // 外部注入初始位姿 = 世界系原点变了，对下游（PX4 EKF2）是一次参考系重置
          notifyOdomReset("initialpose received");

          LOG(INFO) << YELLOW
                  << " ---> GET Initial guess: "
                  << init_translation.transpose()
                  << " yaw: "
                  << init_rotation.toRotationMatrix()
                          .eulerAngles(0, 1, 2)
                          .transpose()
                  << RESET;
        });

  if (flg_finish_init) {
    init_pose_sub.reset();
  }
}


void ROSWrapper::saveMapServiceCallback(const std_srvs::srv::Trigger::Request::SharedPtr request, 
                                        const std_srvs::srv::Trigger::Response::SharedPtr response)
{
  LOG(INFO) << GREEN << " ---> [Service] Save map service called" << RESET;
  
  if (super_lio_) {
    // 作用域守卫：无论 saveMap() 是正常返回、提前 return（g_save_map=false 等）
    // 还是抛异常，都必须恢复 LIO 处理。否则 paused_ 会永久置位，表现为
    // 「无日志 / 无 PCD / 服务无响应」的整条链路静默停摆。
    struct ResumeGuard {
      SuperLIO* lio = nullptr;
      bool active = true;
      ~ResumeGuard() {
        if (active && lio) {
          LOG(WARNING) << YELLOW << " ---> [Service] 未走到正常恢复分支，强制恢复 LIO 处理" << RESET;
          lio->resumeProcessing();
        }
      }
    };
    ResumeGuard resume_guard;
    resume_guard.lio = super_lio_.get();

    LOG(INFO) << YELLOW << " ---> [Service] Pausing LIO processing..." << RESET;
    super_lio_->pauseProcessing();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    LOG(INFO) << YELLOW << " ---> [Service] Saving map..." << RESET;
    super_lio_->saveMap();
    super_lio_->printTimeRecord();
    
    LOG(INFO) << YELLOW << " ---> [Service] Resuming LIO processing..." << RESET;
    super_lio_->resumeProcessing();
    resume_guard.active = false;
    
    LOG(INFO) << YELLOW << " ---> [Service] Resetting IMU pre-integration..." << RESET;
    super_lio_->resetIMUIntegration();
    // 重置计数在 SuperLIO::resetIMUIntegration() 内统一上报（单一收敛点），此处不重复计数
    
    LOG(INFO) << GREEN << " ---> [Service] Map saved successfully, LIO resumed" << RESET;
    response->success = true;
    response->message = "Map saved successfully, LIO processing resumed and IMU pre-integration reset";
  } else {
    LOG(ERROR) << RED << " ---> [Service] SuperLIO instance not set" << RESET;
    response->success = false;
    response->message = "SuperLIO instance not set";
  }
}


} // namespace END.