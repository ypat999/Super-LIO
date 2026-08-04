
#include "ros/ROSWrapper.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "lio/super_lio.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <thread>
#include <chrono>

#ifdef LIVOX_SUPPORT
#include "livox_ros_driver2/msg/custom_msg.hpp"
#endif


using namespace BASIC;

namespace LI2Sup{

void LoadParamFromRos(rclcpp::Node& node)
{
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
  // g_save_map_dir = g_root_dir + g_save_map_dir;

  node.declare_parameter<std::string>("lio.map.map_name", "default");
  node.get_parameter("lio.map.map_name", g_map_name);

  node.declare_parameter<double>("lio.map.ds_size", 0.5);
  node.get_parameter("lio.map.ds_size", g_map_ds_size);

  node.declare_parameter<int>("lio.map.save_interval", 1);
  node.get_parameter("lio.map.save_interval", g_pcd_save_interval);

  node.declare_parameter<std::string>("lio.map.pcd_prefix", "");
  node.get_parameter("lio.map.pcd_prefix", g_pcd_prefix);

  LOG(INFO) << GREEN << " ---> [Param] map/pcd_prefix: "
            << g_pcd_prefix << RESET;

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

  node.declare_parameter<double>("lio.sensor.imu_nbg", 0.0);
  node.get_parameter("lio.sensor.imu_nbg", g_imu_nbg);

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

  // kf
  node.declare_parameter<int>("lio.kf.kf_type", 0);
  node.get_parameter("lio.kf.kf_type", g_kf_type);

  node.declare_parameter<int>("lio.kf.kf_max_iterations", 0);
  node.get_parameter("lio.kf.kf_max_iterations", g_kf_max_iterations);

  node.declare_parameter<bool>("lio.kf.kf_align_gravity", false);
  node.get_parameter("lio.kf.kf_align_gravity", g_kf_align_gravity);

  node.declare_parameter<int>("lio.kf.ref_gravity_axis", 2);
  node.get_parameter("lio.kf.ref_gravity_axis", g_ref_gravity_axis);

  node.declare_parameter<double>("lio.kf.kf_quit_eps", 0.0);
  node.get_parameter("lio.kf.kf_quit_eps", g_kf_quit_eps);

  // observe
  node.declare_parameter<double>("lio.observe.plane_fit_threshold", 0.15);
  node.get_parameter("lio.observe.plane_fit_threshold", g_plane_fit_threshold);

  LOG(INFO) << GREEN << " ---> [Param] observe/plane_fit_threshold: "
            << g_plane_fit_threshold << RESET;

  node.declare_parameter<double>("lio.observe.obs_weight", 1000.0);
  node.get_parameter("lio.observe.obs_weight", g_obs_weight);

  node.declare_parameter<double>("lio.observe.huber_delta_base", 0.05);
  node.get_parameter("lio.observe.huber_delta_base", g_huber_delta_base);

  node.declare_parameter<double>("lio.observe.huber_delta_scale", 0.003);
  node.get_parameter("lio.observe.huber_delta_scale", g_huber_delta_scale);

  LOG(INFO) << GREEN << " ---> [Param] observe/obs_weight: " << g_obs_weight << RESET;
  LOG(INFO) << GREEN << " ---> [Param] observe/huber_delta_base: " << g_huber_delta_base << RESET;
  LOG(INFO) << GREEN << " ---> [Param] observe/huber_delta_scale: " << g_huber_delta_scale << RESET;

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

  node.declare_parameter<std::string>("lio.output.tf_base_footprint_frame", "base_footprint");
  node.get_parameter("lio.output.tf_base_footprint_frame", g_tf_base_footprint_frame);

  node.declare_parameter<std::string>("lio.output.world_frame", "world");
  node.get_parameter("lio.output.world_frame", g_world_frame);

  node.declare_parameter<std::string>("lio.output.imu_frame", "imu");
  node.get_parameter("lio.output.imu_frame", g_imu_frame);

  LOG(INFO) << GREEN << " ---> [Param] output/footprint_pub_en: "
            << (g_footprint_pub_en ? "true" : "false") << RESET;

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
        else if (name == "lio.observe.huber_delta_base") g_huber_delta_base = p.as_double();
        else if (name == "lio.observe.huber_delta_scale") g_huber_delta_scale = p.as_double();
        else if (name == "lio.kf.kf_max_iterations") g_kf_max_iterations = p.as_int();
        else if (name == "lio.kf.kf_quit_eps") g_kf_quit_eps = p.as_double();
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
  auto viz_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .best_effort()
    .durability_volatile();

  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "lio/odom", viz_qos);

  pub_imu_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "lio/imu/odom", viz_qos);

  pub_robo_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "lio/robo/odom", viz_qos);

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
        "lio/body/cloud", pointcloud_qos);

  // TF 发布同样必须 best_effort：Humble 的 TransformBroadcaster 默认
  // DynamicBroadcasterQoS(=RELIABLE)，远程 RViz 在 WiFi 差/断连时
  // RELIABLE /tf 仍会触发重传积压，必须显式改为 best_effort + volatile
  auto tf_qos = rclcpp::QoS(rclcpp::KeepLast(100))
    .best_effort()
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

  M3 R_IMU_to_Lidar = g_lidar_imu.R_.transpose();
  data.acc = R_IMU_to_Lidar * acc_raw;
  data.gyr = R_IMU_to_Lidar * gyr_raw;

  bool loop_back = false;
  {
    std::lock_guard<std::mutex> lock(buffers_mutex_);
    if (data.secs < last_timestamp_imu_) {
      LOG(WARNING) << "imu loop back, clear buffer";
      imu_buffer_.clear();
      imu_buffer_.push_back(data);
      last_timestamp_imu_ = data.secs;
      loop_back = true;
    } else {
      imu_buffer_.push_back(data);
      last_timestamp_imu_ = data.secs;
    }
  }
  if (loop_back) {
    // eskf_->Reset();   // todo:
    return;
  }

  DynamicState imu_state, robo_state;
  if(eskf_->Predict(data, imu_state, robo_state)){
    // 推入队列，由独立线程 imuOutputThread 发布，避免 WiFi 差时 RELIABLE 发布阻塞回调
    {
      ImuOutputData out;
      out.imu_state = std::move(imu_state);
      out.robo_state = std::move(robo_state);
      out.timestamp = data.secs;
      std::lock_guard<std::mutex> lock(imu_output_mutex_);
      if(imu_output_queue_.size() > 10) {
        imu_output_queue_.pop();
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
  LidarData lidar_data;
  lidar_data.receive_time = std::chrono::duration<double>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();
  std::size_t ptsize = msg->point_num;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  lidar_data.pc->reserve(ptsize / g_filter_rate + 1);

  double offset_time = 0.0;
  for(std::size_t _i = g_filter_offset; _i < ptsize; _i += g_filter_rate){
    auto& pt = msg->points[_i];
    auto tag = pt.tag & 0x30;
    if (tag == 0x10 || tag == 0x00){
      auto dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if(dis > g_blind2 && dis < g_maxrange2){
        if(g_intensity_filter_en && pt.reflectivity < g_intensity_min) continue;
        offset_time = pt.offset_time * 1e-9;
        lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.reflectivity, offset_time);
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
        break;
      }
    }
  }

  double max_offset_time = 0.0;

  // Helper: read offset_time for point at index k
  auto read_offset_time = [&](size_t k) -> double {
    if (has_time) {
      switch (time_type) {
        case 2: { double t; memcpy(&t, data_ptr + k * point_step + off_time, sizeof(double)); return t - time_begin; }
        case 1: { float t; memcpy(&t, data_ptr + k * point_step + off_time, sizeof(float));
          if (g_lidar_type == LID_TYPE::VELO16 || g_lidar_type == LID_TYPE::VELO32) return static_cast<double>(t);
          else return static_cast<double>(t) - time_begin; }
        case 3: { uint32_t t; memcpy(&t, data_ptr + k * point_step + off_time, sizeof(uint32_t)); return t * 1e-6 - time_begin; }
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
    lidar_data.pc->emplace_back(x, y, z, intensity, ot);
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
      pub_robo_odom_->publish(odom_robo);
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


void ROSWrapper::pub_odom(const NavState& state, const V3& body_omega){
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = g_world_frame;
  odom.child_frame_id = g_imu_frame;

  odom.header.stamp = toRosTime(state.timestamp);
  odom.pose.pose.position.x = state.p[0];
  odom.pose.pose.position.y = state.p[1];
  odom.pose.pose.position.z = state.p[2];

  V4 temp_q = state.R.coeffs();
  odom.pose.pose.orientation.x = temp_q[0];
  odom.pose.pose.orientation.y = temp_q[1];
  odom.pose.pose.orientation.z = temp_q[2];
  odom.pose.pose.orientation.w = temp_q[3];

  // REP 103: twist must be in child_frame (body/imu frame)
  const V3 v_body = state.R.R_.transpose() * state.v;
  odom.twist.twist.linear.x = v_body[0];
  odom.twist.twist.linear.y = v_body[1];
  odom.twist.twist.linear.z = v_body[2];

  odom.twist.twist.angular.x = body_omega[0];
  odom.twist.twist.angular.y = body_omega[1];
  odom.twist.twist.angular.z = body_omega[2];

  pub_odom_->publish(odom);    // imu frame -> lidar frequency

  V3 robo_position = state.R.R_ * ( - g_odom_robo.R_ * g_odom_robo.t_) + state.p;

  if(g_2_robot){
    static auto pub_msg2uav_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", 10);
    M3 robo_rotation = state.R.R_ * g_odom_robo.R_;
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


void ROSWrapper::pub_cloud_world(const CloudPtr& pc, double time){
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = g_world_frame;
  cloud.header.stamp = toRosTime(time);
  pub_cloud_world_->publish(cloud);
}


void ROSWrapper::pub_cloud_world_undistort_only(const CloudPtr& pc, double time, const std::string& lidar_frame){
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
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = g_imu_frame;
  cloud.header.stamp = toRosTime(time);
  pub_cloud_body_->publish(cloud);
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
    LOG(INFO) << YELLOW << " ---> [Service] Pausing LIO processing..." << RESET;
    super_lio_->pauseProcessing();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    LOG(INFO) << YELLOW << " ---> [Service] Saving map..." << RESET;
    super_lio_->saveMap();
    super_lio_->printTimeRecord();
    
    LOG(INFO) << YELLOW << " ---> [Service] Resuming LIO processing..." << RESET;
    super_lio_->resumeProcessing();
    
    LOG(INFO) << YELLOW << " ---> [Service] Resetting IMU pre-integration..." << RESET;
    super_lio_->resetIMUIntegration();
    
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