
#include "lio/super_lio.h"

#include <sys/resource.h>
#include <sched.h>
#include <pthread.h>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>


using namespace BASIC;

namespace {

// 高效 R*t 点云变换（避免 pcl::transformPointCloud 的 4x4 矩阵开销）
// 仅变换 x,y,z，直接拷贝 intensity
inline void transformPointCloudRt(const pcl::PointCloud<pcl::PointXYZI>& src,
                                   pcl::PointCloud<pcl::PointXYZI>& dst,
                                   const Eigen::Matrix3f& R,
                                   const Eigen::Vector3f& t) {
  dst.resize(src.size());
  const float* r0 = R.data(); // Eigen column-major: r0=col0, r1=col1, r2=col2
  const float* r1 = r0 + 3;
  const float* r2 = r1 + 3;
  for (size_t i = 0; i < src.size(); ++i) {
    const auto& p = src.points[i];
    auto& q = dst.points[i];
    // Eigen column-major: r0=col0, r1=col1, r2=col2
    // R*p = col0*px + col1*py + col2*pz
    q.x = r0[0]*p.x + r1[0]*p.y + r2[0]*p.z + t[0];
    q.y = r0[1]*p.x + r1[1]*p.y + r2[1]*p.z + t[1];
    q.z = r0[2]*p.x + r1[2]*p.y + r2[2]*p.z + t[2];
    q.intensity = p.intensity;
  }
  dst.header = src.header;
  dst.width = dst.size();
  dst.height = 1;
  dst.is_dense = src.is_dense;
}

} // anonymous namespace

namespace LI2Sup{

bool SuperLIO::set_realtime_priority(int priority)
{
  struct sched_param param;
  param.sched_priority = priority;
  
  int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  if (ret != 0) {
    LOG(WARNING) << "Failed to set SCHED_FIFO priority " << priority 
               << ", error: " << std::strerror(ret);
    return false;
  }
  
  LOG(INFO) << GREEN << " ---> [SuperLIO]: Set SCHED_FIFO priority to " << priority << RESET;
  return true;
}

/// 平面拟合：用 N 个点拟合平面 ax+by+cz+1=0，返回法向量系数 abcd
inline bool calc_plane_coeff(const int N, const std::array<V3, 5>& points, std::array<double, 4>& abcd)
{
  Eigen::Vector3d normvec;
  if (N == 5) {
    Eigen::Matrix<double, 5, 3> A;
    Eigen::Matrix<double, 5, 1> b;
    for (int j = 0; j < 5; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }
  else {
    Eigen::Matrix<double, 4, 3> A;
    Eigen::Matrix<double, 4, 1> b;

    for (int j = 0; j < N; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }

  double n = normvec.norm();
  if (n < 1e-6f) return false;

  abcd[3] = 1.0 / n;
  normvec *= abcd[3];
  abcd[0] = normvec[0];
  abcd[1] = normvec[1];
  abcd[2] = normvec[2];
  
  for (int i = 0; i < N; ++i) {
    const V3& p = points[i];
    auto dist = abcd[0] * p(0) + abcd[1] * p(1) + abcd[2] * p(2) + abcd[3];
    if (std::abs(dist) > g_plane_fit_threshold) return false;
  }
  return true;
}


inline bool compute_error(
  const std::array<double, 4>& abcd, const V3& point, 
  const float length, scalar& error)
{
  error = abcd[0] * point[0] + abcd[1] * point[1] + abcd[2] * point[2] + abcd[3];
  return length > 81 * error * error;
}


/// 初始化：创建 IVox 地图、ESKF 滤波器、加载参数
void SuperLIO::init(){
  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
  kf_.reset(new ESKF());
  data_wrapper_->setESKF(kf_);
  
  scan_undistort_full_.reset(new PointCloudType());
  ds_undistort_.reset(new PointCloudType());
  world_pc_.reset(new PointCloudType());
  ds_world_.reset(new PointCloudType());

  if(g_save_map){
    point_map_.reset(new PointCloudType());
  }
  
  points_world_v3_.reserve(21000);
  abcd_vec_.resize(20000);
  effect_knn_idxs_.resize(20000);
  voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

  state_fn_ = &SuperLIO::stateWaitKFInit;

  output_running_ = true;
  output_thread_ = std::thread(&SuperLIO::OutputThread, this);

  if(g_save_map){
    namespace fs = std::filesystem;
    
    std::string save_map_dir = g_save_map_dir;
    if (!save_map_dir.empty() && save_map_dir[0] != '/') {
      save_map_dir = g_root_dir + save_map_dir;
    }
    
    std::string pcd_folder = save_map_dir + "/PCD";
    
    if(fs::exists(pcd_folder)){
      int deleted_count = 0;
      for(const auto& entry : fs::directory_iterator(pcd_folder)){
        std::string filename = entry.path().filename().string();
        // Match original PCD fragments: {prefix}scans_*.pcd  and filtered fragments: filtered_{prefix}scans_*.pcd
        bool is_scan_pcd = (entry.path().extension() == ".pcd" &&
                           (filename.find(g_pcd_prefix + "scans_") != std::string::npos ||
                            filename.find("filtered_" + g_pcd_prefix + "scans_") != std::string::npos));
        if(is_scan_pcd){
          try{
            fs::remove(entry.path());
            deleted_count++;
            LOG(INFO) << GREEN << " ---> Deleted old PCD fragment: " << filename << RESET;
          } catch(const std::exception& e){
            LOG(WARNING) << RED << " ---> Failed to delete " << filename 
                        << ": " << e.what() << RESET;
          }
        }
        if(entry.path().extension() == ".txt" &&
           filename.find(g_pcd_prefix + "scans_") != std::string::npos){
          try{
            fs::remove(entry.path());
            deleted_count++;
            LOG(INFO) << GREEN << " ---> Deleted old pose file: " << filename << RESET;
          } catch(const std::exception& e){
            LOG(WARNING) << RED << " ---> Failed to delete " << filename 
                        << ": " << e.what() << RESET;
          }
        }
      }
      if(deleted_count > 0){
        LOG(INFO) << YELLOW << " ---> Deleted " << deleted_count 
                  << " old PCD fragments with prefix '" << g_pcd_prefix << "'" << RESET;
      }
    }
    
    save_running_ = true;
    save_thread_ = std::thread(&SuperLIO::SaveThread, this);

    // SC-PGO offline output init
    if(g_sc_pgo_enable){
      std::string scans_dir = save_map_dir + "/Scans";
      
      // Clean old Scans directory
      if(fs::exists(scans_dir)){
        for(const auto& entry : fs::directory_iterator(scans_dir)){
          try{
            fs::remove(entry.path());
          } catch(const std::exception& e){
            LOG(WARNING) << RED << " ---> Failed to delete old SC-PGO file: " 
                        << entry.path().filename().string() << " : " << e.what() << RESET;
          }
        }
      } else {
        fs::create_directories(scans_dir);
      }
      
      // Open odom_poses.txt (truncate)
      sc_pgo_odom_file_.open(save_map_dir + "/odom_poses.txt", std::ios::out | std::ios::trunc);
      if(!sc_pgo_odom_file_.is_open()){
        LOG(WARNING) << RED << " ---> Failed to open odom_poses.txt for SC-PGO output" << RESET;
      }
      
      sc_pgo_index_ = 0;
      sc_pgo_first_ = true;
      
      LOG(INFO) << GREEN << " ---> [SC-PGO] Output enabled, saving to " 
                << scans_dir << RESET;
    }
  }

  if (g_single_core) {
    LOG(INFO) << YELLOW << " ---> [SuperLIO]: Single-core mode" << RESET;
  }

  LOG(INFO) << GREEN << " ---> [SuperLIO]: initialized." << RESET;
}

SuperLIO::~SuperLIO(){
  output_running_ = false;
  output_cv_.notify_all();
  if(output_thread_.joinable()){
    output_thread_.join();
  }

  save_running_ = false;
  save_cv_.notify_all();
  if(save_thread_.joinable()){
    save_thread_.join();
  }
}


void SuperLIO::stateWaitKFInit()
{
  // downsample_only mode: skip KF and map initialization
  if(g_downsample_only){
    kf_->init_ = true;
    state_fn_ = &SuperLIO::stateProcess;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: Downsample-only mode, skip KF and map init" << RESET;
    return;
  }
  
  if (kf_init()) {
    state_fn_ = &SuperLIO::stateWaitMapInit;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: KF init done" << RESET;
  }
}

void SuperLIO::stateWaitMapInit()
{
  // downsample_only mode should not reach here, but add check for safety
  if(g_downsample_only){
    kf_->init_ = true;
    state_fn_ = &SuperLIO::stateProcess;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: Downsample-only mode, skip map init" << RESET;
    return;
  }
  
  if(g_lio_only_undistort){
    kf_->init_ = true;
    state_fn_ = &SuperLIO::stateProcess;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: Undistort-only mode, skip map init" << RESET;
    return;
  }
  if (map_init()) {
    kf_->init_ = true;
    state_fn_ = &SuperLIO::stateProcess;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: Map init done" << RESET;
  }
}

/// 主处理流程：状态机驱动，依次执行 IMU 传播→去畸变→观测→更新地图→输出
void SuperLIO::process(){
  if(paused_.load()){
    return;
  }

  static bool priority_set = false;
  if (!priority_set) {
    set_realtime_priority(98);
    priority_set = true;
  }
  
  if(!data_wrapper_->sync_measure(measures_)){
    return;
  }
  current_lidar_frame_ = measures_.lidar.frame_id;
  (this->*state_fn_)();
}


bool SuperLIO::kf_init(){
  static int imu_cout = 0;
  static V3 mean_gyro = V3::Zero();
  static V3 mean_acce = V3::Zero();

  for(auto& imu: measures_.imu){
    imu_cout ++;
    mean_gyro += (imu.gyr - mean_gyro) / imu_cout;
    mean_acce += (imu.acc - mean_acce) / imu_cout;
  }

  /// 100 Hz for 1 second.
  if(imu_cout < 50){
    return false;
  }

  V3 gravity = - mean_acce * g_gravity_norm / mean_acce.norm();
  V3 ref_gravity;
  switch(g_ref_gravity_axis) {
    case 0:  ref_gravity = V3(g_gravity_norm, 0, 0); break;   // +X
    case 1:  ref_gravity = V3(0, g_gravity_norm, 0); break;   // +Y
    case 2:  
    default: ref_gravity = V3(0, 0, -g_gravity_norm); break;  // -Z (default)
  }
  M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
  V3 n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  LOG(INFO) << GREEN << " ---> [SuperLIO]: Gravity Alignment Results:" << RESET;
  LOG(INFO) << GREEN << "      Mean Acceleration: [" << mean_acce.transpose() << "]" << RESET;
  LOG(INFO) << GREEN << "      Gravity Norm: " << g_gravity_norm << RESET;
  LOG(INFO) << GREEN << "      Measured Gravity: [" << gravity.transpose() << "]" << RESET;
  LOG(INFO) << GREEN << "      Reference Gravity: [" << ref_gravity.transpose() << "]" << RESET;
  LOG(INFO) << GREEN << "      Yaw Angle: " << yaw * 180.0 / M_PI << " degrees" << RESET;
  LOG(INFO) << GREEN << "      IMU Scale: " << g_gravity_norm / mean_acce.norm() << RESET;

  M3 R_yaw_inv = Eigen::AngleAxis<scalar>(-yaw, V3::UnitZ()).toRotationMatrix(); 

  // Gravity-aligned initial orientation (pure imu/lidar → world, no odom_robo offset).
  // odom_robo only affects robot odom output (lio/robo/odom), not imu frame.
  M3 rot = R_yaw_inv * init_rot;

  ESKF::Options options;
  options.gyro_var_ = g_imu_ng;
  options.acce_var_ = g_imu_na;
  options.bias_gyro_var_ = g_imu_nbg;
  options.bias_acce_var_ = g_imu_nba;
  options.num_iterations_ = g_kf_max_iterations;
  options.quit_eps_ = g_kf_quit_eps;

  float imu_scale = g_gravity_norm / mean_acce.norm();
  kf_->SetInitialConditions(options, mean_gyro, V3::Zero(), imu_scale, ref_gravity);
  auto state = kf_->GetSysState();
  state.R = SO3(rot);
  state.p = V3::Zero();             // World origin at initial imu/lidar position.
  state.timestamp = measures_.imu.back().secs;
  kf_->SetX(state);
  sys_init_pose_ = kf_->GetSE3();
  return true;
}


bool SuperLIO::map_init(){
  frame_num_++;

  std::size_t ptsize = measures_.lidar.pc->size();
  points_world_v3_.resize(ptsize);

  const SE3 transform = sys_init_pose_;

  for (size_t idx = 0; idx < ptsize; ++idx) {
    auto& point_pcl = measures_.lidar.pc->points[idx];
    V3 point_body(point_pcl.x, point_pcl.y, point_pcl.z);
    points_world_v3_[idx] = transform * point_body;
  }

  ivox_->insert(points_world_v3_);
  kf_->SetLastObsTime(measures_.lidar.end_time);

  // 20 Hz for 1.0 seconds. Integral coverage area > 70%
  if(frame_num_ > 3){
    g_flg_map_init = false;
    return true;
  }
  return false;
}


/// 核心处理：IMU 前向传播 + 点云去畸变 + ESKF 观测更新
void SuperLIO::stateProcess(){
  frame_num_++;

  // downsample_only mode: no deskew, no ESKF, no delta correction
  if(g_downsample_only){
    if(g_time_eva){
      time_record_.Evaluate([this]() { DownSampleOnly(); }, "[DownSampleOnly]");
    }else{
      DownSampleOnly();
    }
    PublishBodyCloud();   // immediate body publish (low latency)
    Output();              // world cloud via queue
    caceData();
    caceSCPGOData();
    return;
  }

  // Normal & lio_only_undistort: both run deskew + ESKF observation,
  // so delta correction applies to both.
  // Body cloud emitted immediately after deskew (before ESKF) for minimal latency.
  if(g_time_eva){
    time_record_.Evaluate([this](){Propagation_Undistort();},  "[Undistort]");
    time_record_.Evaluate([this]() { PublishBodyCloud(); },    "[PublishBody]");
    time_record_.Evaluate([this]() { DownSample(); },          "[DownSample]");
    time_record_.Evaluate([this]() { Observe(); },             "[Observe]");
    time_record_.Evaluate([this]() { ApplyDeltaCorrection(); },"[DeltaCorrect]");
    time_record_.Evaluate([this]() { UpdateMap(); },           "[UpdateMap]");
  }else{
    Propagation_Undistort();
    PublishBodyCloud();   // immediate, pre-ESKF (minimum latency)
    DownSample();
    Observe();
    ApplyDeltaCorrection();
    UpdateMap();
  }
  Output();                // world cloud via queue (high precision)
  caceData();
  caceSCPGOData();
}


/// 缓存当前帧数据：变换点云到世界系，存入 cace_map_ 供后续输出和建图
void SuperLIO::caceData(){
  if(!g_save_map) return;

  auto state = kf_->GetNavState();

  if(g_lio_only_undistort){
    if(g_if_filter){
      *world_pc_ = *ds_undistort_;
    }else{
      *world_pc_ = *scan_undistort_full_;
    }
  }else{
    // Reuse Output's already-transformed cloud if available
    if(last_transformed_world_pc_ && !last_transformed_world_pc_->empty()){
      *world_pc_ = *last_transformed_world_pc_;
    }else{
      const Eigen::Matrix3f Rf = state.R.R_.cast<float>();
      const Eigen::Vector3f tf = state.p.cast<float>();
      if(g_if_filter){
        transformPointCloudRt(*ds_undistort_, *world_pc_, Rf, tf);
      }else{
        transformPointCloudRt(*scan_undistort_full_, *world_pc_, Rf, tf);
      }
    }
  }

  static int scan_wait_num = 0;
  if(!world_pc_->empty()){
    *point_map_ += *world_pc_;
    scan_wait_num++;
  }

  if(g_pcd_save_interval < 0) {
    scan_wait_num = 0;
    return;
  }

  static bool rm_PCD_dir = false;
  if(!rm_PCD_dir){
    rm_PCD_dir = true;
    std::string save_map_dir = g_save_map_dir;
    if (!save_map_dir.empty() && save_map_dir[0] != '/') {
      save_map_dir = g_root_dir + save_map_dir;
    }
    std::string cmd = "rm -rf " + save_map_dir + "/PCD";
    [[maybe_unused]] int res;
    res = system(cmd.c_str());
    cmd = "mkdir -p " + save_map_dir + "/PCD";
    res = system(cmd.c_str());
  }

  if (point_map_->size() > 0 && scan_wait_num >= g_pcd_save_interval) {
    pcd_index_++;
    
    SaveData save_data;
    save_data.cloud_to_save.reset(new PointCloudType(*point_map_));
    save_data.pcd_index = pcd_index_;
    save_data.timestamp = state.timestamp;
    save_data.position = state.p.cast<float>();
    save_data.orientation = BASIC::Quat(state.R.R_.cast<float>());
    
    {
      std::lock_guard<std::mutex> lock(save_mutex_);
      if(save_queue_.size() > 3){
        save_queue_.pop();
      }
      save_queue_.push(std::move(save_data));
    }
    save_cv_.notify_one();
    
    point_map_->clear();
    scan_wait_num = 0;
  }
}

void SuperLIO::SaveThread(){
  while(save_running_){
    SaveData data;
    {
      std::unique_lock<std::mutex> lock(save_mutex_);
      save_cv_.wait(lock, [this]{
        return !save_queue_.empty() || !save_running_;
      });
      
      if(!save_running_ && save_queue_.empty()){
        break;
      }
      
      if(save_queue_.empty()){
        continue;
      }
      
      data = std::move(save_queue_.front());
      save_queue_.pop();
      save_cv_.notify_one();
    }
    
    if(data.cloud_to_save && !data.cloud_to_save->empty()){
      std::string save_map_dir = g_save_map_dir;
      if (!save_map_dir.empty() && save_map_dir[0] != '/') {
        save_map_dir = g_root_dir + save_map_dir;
      }
      std::string map_name(std::string(save_map_dir + "/PCD/" + g_pcd_prefix + "scans_") + std::to_string(data.pcd_index) +
                                 std::string(".pcd"));
      LOG(INFO) << GREEN << " ---> current scan saved to /PCD/" << g_pcd_prefix << "scans_" << data.pcd_index 
                << "  size:  " << data.cloud_to_save->size() << RESET;
      pcl::io::savePCDFileBinary(map_name, *data.cloud_to_save);
      
      std::string odom_name(std::string(save_map_dir + "/PCD/" + g_pcd_prefix + "scans_") + std::to_string(data.pcd_index) +
                                 std::string(".txt"));
      std::ofstream odom_file(odom_name);
      if(odom_file.is_open()){
        odom_file << std::fixed << std::setprecision(6);
        odom_file << data.timestamp << " "
                  << data.position.x() << " " << data.position.y() << " " << data.position.z() << " "
                  << data.orientation.x() << " " << data.orientation.y() << " " 
                  << data.orientation.z() << " " << data.orientation.w() << std::endl;
        odom_file.close();
      }
    }
  }
}


void SuperLIO::caceSCPGOData(){
  if(!g_sc_pgo_enable) return;
  if(!g_save_map) return;
  
  auto state = kf_->GetNavState();
  BASIC::SE3 current_pose(state.R.R_, state.p);

  // Initialize first pose
  if(sc_pgo_first_){
    sc_pgo_pose_prev_ = current_pose;
    sc_pgo_trans_accum_ = 0.0f;
    sc_pgo_rot_accum_ = 0.0f;
    sc_pgo_first_ = false;

    // Save first frame always as keyframe
    std::string save_map_dir = g_save_map_dir;
    if (!save_map_dir.empty() && save_map_dir[0] != '/') {
      save_map_dir = g_root_dir + save_map_dir;
    }

    // Save body-frame scan
    if(scan_undistort_full_ && !scan_undistort_full_->empty()){
      std::stringstream ss;
      ss << std::setw(6) << std::setfill('0') << sc_pgo_index_;
      std::string pcd_path = save_map_dir + "/Scans/" + ss.str() + ".pcd";
      pcl::io::savePCDFileBinary(pcd_path, *scan_undistort_full_);
      sc_pgo_index_++;
    }

    // Write KITTI-format pose
    if(sc_pgo_odom_file_.is_open()){
      Eigen::Matrix3f R = state.R.R_.cast<float>();
      Eigen::Vector3f t = state.p.cast<float>();
      sc_pgo_odom_file_ << std::fixed << std::setprecision(6)
                        << R(0,0) << " " << R(0,1) << " " << R(0,2) << " " << t(0) << " "
                        << R(1,0) << " " << R(1,1) << " " << R(1,2) << " " << t(1) << " "
                        << R(2,0) << " " << R(2,1) << " " << R(2,2) << " " << t(2) << "\n";
      sc_pgo_odom_file_.flush();
    }
    return;
  }

  // Compute delta from previous keyframe pose (absolute difference)
  BASIC::SE3 delta = sc_pgo_pose_prev_.inverse() * current_pose;
  V3 trans = delta.t();
  float dx = std::abs(trans(0));
  float dy = std::abs(trans(1));
  float dz = std::abs(trans(2));
  float dtrans = std::sqrt(dx*dx + dy*dy + dz*dz);

  // Extract rotation angle via angle-axis representation
  Eigen::Matrix3f R = delta.R_.cast<float>();
  float angle_rad = Eigen::AngleAxisf(R).angle();

  float kf_rad_gap = g_sc_pgo_keyframe_deg_gap * M_PI / 180.0f;

  // Use absolute difference directly (not accumulated)
  if(dtrans > g_sc_pgo_keyframe_gap || angle_rad > kf_rad_gap){
    sc_pgo_pose_prev_ = current_pose;

    std::string save_map_dir = g_save_map_dir;
    if (!save_map_dir.empty() && save_map_dir[0] != '/') {
      save_map_dir = g_root_dir + save_map_dir;
    }

    // Save body-frame undistorted scan
    if(scan_undistort_full_ && !scan_undistort_full_->empty()){
      std::stringstream ss;
      ss << std::setw(6) << std::setfill('0') << sc_pgo_index_;
      std::string pcd_path = save_map_dir + "/Scans/" + ss.str() + ".pcd";
      pcl::io::savePCDFileBinary(pcd_path, *scan_undistort_full_);
      sc_pgo_index_++;
    }

    // Append KITTI-format pose
    if(sc_pgo_odom_file_.is_open()){
      Eigen::Matrix3f Rm = state.R.R_.cast<float>();
      Eigen::Vector3f tm = state.p.cast<float>();
      sc_pgo_odom_file_ << std::fixed << std::setprecision(6)
                        << Rm(0,0) << " " << Rm(0,1) << " " << Rm(0,2) << " " << tm(0) << " "
                        << Rm(1,0) << " " << Rm(1,1) << " " << Rm(1,2) << " " << tm(1) << " "
                        << Rm(2,0) << " " << Rm(2,1) << " " << Rm(2,2) << " " << tm(2) << "\n";
      sc_pgo_odom_file_.flush();
    }
  }
}


void SuperLIO::ProcessCaceMap(const std::string& output_name, bool filtered){
  namespace fs = std::filesystem;

  std::string save_map_dir = g_save_map_dir;
  if (!save_map_dir.empty() && save_map_dir[0] != '/') {
    save_map_dir = g_root_dir + save_map_dir;
  }

  std::string pcd_folder = save_map_dir + "/PCD";

  // When filtered is true, merge filtered_ files; otherwise merge original scans_
  std::string scan_prefix = filtered ?
      ("filtered_" + g_pcd_prefix + "scans_") : (g_pcd_prefix + "scans_");

  std::string output_map_name = save_map_dir + "/" + output_name;

  // Collect and sort matching PCD files
  std::vector<std::string> pcd_files;
  for (const auto& entry : fs::directory_iterator(pcd_folder)) {
    if (entry.path().extension() == ".pcd" &&
        entry.path().filename().string().find(scan_prefix) != std::string::npos) {
      pcd_files.push_back(entry.path().string());
    }
  }
  std::sort(pcd_files.begin(), pcd_files.end());

  if (pcd_files.empty()) {
    LOG(WARNING) << RED << " ---> No matching PCD fragments found with prefix '" << scan_prefix << "'" << RESET;
    return;
  }

  LOG(INFO) << YELLOW << " ---> Merging " << pcd_files.size() << " PCD fragments in: " << pcd_folder << RESET;

  if (!g_if_filter) {
    // Streaming binary merge: read header to count total points, then concatenate binary data
    // This avoids loading all point clouds into memory at once
    LOG(INFO) << YELLOW << " ---> Streaming merge (no downsample) ..." << RESET;

    // Step 1: Count total points and extract header template from first file
    size_t total_points = 0;
    std::string header_template;
    bool first = true;

    for (const auto& file : pcd_files) {
      pcl::PCLPointCloud2 cloud2;
      pcl::PCDReader reader;
      if (reader.readHeader(file, cloud2) == 0) {
        total_points += cloud2.width * cloud2.height;
        if (first) {
          // Read the ASCII header from the first file to use as template
          std::ifstream ifs(file, std::ios::binary);
          std::string line;
          while (std::getline(ifs, line)) {
            header_template += line + "\n";
            if (line.find("DATA") == 0) break;
          }
          first = false;
        }
      } else {
        LOG(WARNING) << RED << " ---> Failed to read header: " << file << RESET;
      }
    }

    if (total_points == 0) {
      LOG(WARNING) << RED << " ---> No points to merge" << RESET;
      return;
    }

    // Step 2: Write output file header with correct total point count
    std::string updated_header;
    std::istringstream header_stream(header_template);
    std::string line;
    while (std::getline(header_stream, line)) {
      if (line.find("POINTS ") == 0) {
        updated_header += "POINTS " + std::to_string(total_points) + "\n";
      } else if (line.find("WIDTH ") == 0) {
        updated_header += "WIDTH " + std::to_string(total_points) + "\n";
      } else {
        updated_header += line + "\n";
      }
    }

    std::ofstream ofs(output_map_name, std::ios::binary);
    if (!ofs.is_open()) {
      LOG(ERROR) << RED << " ---> Failed to open output file: " << output_map_name << RESET;
      return;
    }
    ofs << updated_header;

    // Step 3: Append binary data from each file without loading entire cloud into memory
    for (const auto& file : pcd_files) {
      std::ifstream ifs(file, std::ios::binary);
      if (!ifs.is_open()) {
        LOG(WARNING) << RED << " ---> Failed to open: " << file << RESET;
        continue;
      }

      // Seek to end of header ("DATA binary\n" or "DATA ascii\n")
      std::string data_line;
      while (std::getline(ifs, data_line)) {
        if (data_line.find("DATA") == 0) break;
      }
      size_t data_offset = static_cast<size_t>(ifs.tellg());

      // Get binary data size
      ifs.seekg(0, std::ios::end);
      size_t file_size = static_cast<size_t>(ifs.tellg());
      if (file_size <= data_offset) continue;

      size_t binary_size = file_size - data_offset;

      // Read and write binary data in chunks
      ifs.seekg(static_cast<std::streamoff>(data_offset), std::ios::beg);
      const size_t chunk_size = 1024 * 1024; // 1MB chunks
      std::vector<char> buffer(chunk_size);
      size_t remaining = binary_size;
      while (remaining > 0) {
        size_t to_read = std::min(chunk_size, remaining);
        ifs.read(buffer.data(), to_read);
        ofs.write(buffer.data(), to_read);
        remaining -= to_read;
      }
    }
    ofs.close();

    LOG(INFO) << GREEN << " ---> Streaming merge done: " << total_points << " points" << RESET;
  } else {
    // With downsample: load one-by-one into merged_map (voxel grid needs all points)
    LOG(INFO) << YELLOW << " ---> Merge with downsampling ..." << RESET;

    PointCloudType::Ptr merged_map(new PointCloudType());
    int count = 0;
    for (const auto& file : pcd_files) {
      PointCloudType::Ptr tmp_cloud(new PointCloudType());
      if (pcl::io::loadPCDFile<PointType>(file, *tmp_cloud) == 0) {
        *merged_map += *tmp_cloud;
        count++;
      } else {
        LOG(WARNING) << RED << " ---> Failed to load: " << file << RESET;
      }
    }

    LOG(INFO) << YELLOW << " ---> Total merged fragments: " << count << RESET;

    PointCloudType filtered_map;
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    voxel_filter.setInputCloud(merged_map);
    voxel_filter.filter(filtered_map);

    if (filtered_map.size() > 0) {
      filtered_map.width = filtered_map.size();
      filtered_map.height = 1;
      filtered_map.is_dense = false;
    }
    pcl::io::savePCDFileBinary(output_map_name, filtered_map);
    LOG(INFO) << GREEN << " ---> Final map size: " << filtered_map.size() << RESET;
  }

  LOG(INFO) << GREEN << " ---> Final map saved to: " << output_map_name << RESET;
}


void SuperLIO::saveMap(){
  namespace fs = std::filesystem;
  if(!g_save_map) return;
  if(g_pcd_save_interval > 0){
    LOG(INFO) << YELLOW << " ---> Saving last cace ... " << RESET;
    if (point_map_->size() > 0) {
      pcd_index_++;
      
      auto state = kf_->GetNavState();
      SaveData save_data;
      save_data.cloud_to_save.reset(new PointCloudType(*point_map_));
      save_data.pcd_index = pcd_index_;
      save_data.timestamp = state.timestamp;
      save_data.position = state.p.cast<float>();
      save_data.orientation = BASIC::Quat(state.R.R_.cast<float>());
      
      {
        std::lock_guard<std::mutex> lock(save_mutex_);
        save_queue_.push(std::move(save_data));
      }
      save_cv_.notify_one();
      
      point_map_->clear();
    }
    
    {
      std::unique_lock<std::mutex> lock(save_mutex_);
      save_cv_.wait(lock, [this]{
        return save_queue_.empty();
      });
    }
    
    LOG(INFO) << GREEN << " ---> Save last cace success. " << RESET;

    // Step 1: Run dynamic point removal on scans_*.pcd -> filtered_scans_*.pcd
    if(g_dynamic_removal_enable){
      LOG(INFO) << YELLOW << " ---> Running dynamic point removal (per-frame mode) ... " << RESET;
      std::string save_map_dir = g_save_map_dir;
      if (!save_map_dir.empty() && save_map_dir[0] != '/') {
        save_map_dir = g_root_dir + save_map_dir;
      }
      std::string pcd_folder = save_map_dir + "/PCD";
      
      std::stringstream cmd;
      cmd << "taskset -c 0,1,2,3,4,5,6 ros2 run super_lio dynamic_remove_node"
          << " --input_dir " << pcd_folder
          << " --output_dir " << pcd_folder
          << " --grid_size " << g_dynamic_removal_grid_size
          << " --min_neighbors " << g_dynamic_removal_min_neighbors
          << " --method " << g_dynamic_removal_method;
      
      if(!g_pcd_prefix.empty()) {
        cmd << " --scans_prefix " << g_pcd_prefix;
      }
      
      if(g_dynamic_removal_method == 0) {
        cmd << " --frame_window " << g_dynamic_removal_frame_window;
      } else {
        cmd << " --raycast_min_hits " << g_dynamic_removal_raycast_min_hits;
      }
      
      if(!g_dynamic_removal_isolated_removal) {
        cmd << " --disable_isolated";
      }
      
      if(g_single_core) {
        cmd << " --single_core";
      }
      
      LOG(INFO) << YELLOW << " ---> Executing: " << cmd.str() << RESET;
      int ret = system(cmd.str().c_str());
      if(ret == 0) {
        LOG(INFO) << GREEN << " ---> Dynamic point removal success. Filtered PCDs saved to: " << pcd_folder << RESET;
      } else {
        LOG(WARNING) << RED << " ---> Dynamic point removal failed with code: " << ret << RESET;
      }
    }

    // Step 2: Merge scans into final map
    if(g_dynamic_removal_enable){
      // Dynamic removal enabled: merge filtered_scans_*.pcd -> test.pcd
      LOG(INFO) << YELLOW << " ---> Merging filtered scans into " << g_map_name << " ... " << RESET;
      ProcessCaceMap(g_map_name, true);

      // Save original unfiltered scans as test_ori.pcd backup directly
      size_t dot_pos = g_map_name.find_last_of('.');
      std::string ori_name = (dot_pos != std::string::npos)
          ? g_map_name.substr(0, dot_pos) + "_ori" + g_map_name.substr(dot_pos)
          : g_map_name + "_ori";
      LOG(INFO) << YELLOW << " ---> Saving original unfiltered scans as " << ori_name << " ..." << RESET;
      ProcessCaceMap(ori_name, false);
    } else {
      // Dynamic removal disabled: directly merge original scans_*.pcd -> test.pcd
      LOG(INFO) << YELLOW << " ---> Merging original scans into " << g_map_name << " ... " << RESET;
      ProcessCaceMap(g_map_name, false);
    }

    LOG(INFO) << GREEN << " ---> Process cace map success. " << RESET;
    
    // Close SC-PGO output file
    if(g_sc_pgo_enable && sc_pgo_odom_file_.is_open()){
      sc_pgo_odom_file_.close();
      LOG(INFO) << GREEN << " ---> [SC-PGO] Output complete: " 
                << sc_pgo_index_ << " keyframes saved" << RESET;
    }
    return;
  }

  LOG(INFO) << YELLOW << " ---> Saving map..... " << RESET;
  if(!point_map_->empty()){
    std::string save_map_dir = g_save_map_dir;
    if (!save_map_dir.empty() && save_map_dir[0] != '/') {
      save_map_dir = g_root_dir + save_map_dir;
    }
    std::string map_name = save_map_dir + "/" + g_map_name;
    LOG(INFO) << YELLOW << " ---> Save map to: " << map_name << RESET;
    pcl::VoxelGrid<PointType> voxel_fliter;
    PointCloudType latst_map;
    voxel_fliter.setInputCloud(point_map_);
    voxel_fliter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    voxel_fliter.filter(latst_map);
    if(latst_map.size() > 0){
      latst_map.width = latst_map.size();
      latst_map.height = 1;
      latst_map.is_dense = false;
    }
    pcl::io::savePCDFileBinary(map_name, latst_map);
    LOG(INFO) << GREEN << " ---> Save map success. File: " << map_name << RESET;
    LOG(INFO) << GREEN << " ---> Map size: " << latst_map.size() << RESET;
  }
  
  // Close SC-PGO output file
  if(g_sc_pgo_enable && sc_pgo_odom_file_.is_open()){
    sc_pgo_odom_file_.close();
    LOG(INFO) << GREEN << " ---> [SC-PGO] Output complete: " 
              << sc_pgo_index_ << " keyframes saved" << RESET;
  }
}


inline double get_cpu_time_seconds() {
  struct rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 +
         usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
}

/// IMU 前向传播 + 点云去畸变：在 IMU 采样点间线性插值旋转/平移，
/// 将每个点校正到扫描结束时刻的位姿，使用小角度近似避免 SLERP 开销
void SuperLIO::Propagation_Undistort(){
  propagate_states_.clear();
  propagate_states_.emplace_back(kf_->GetDynamicState());
  kf_->SetObsTime(measures_.lidar.end_time);
  for (auto &imu : measures_.imu) {
    kf_->Predict(imu);
    propagate_states_.emplace_back(kf_->GetDynamicState());
  }

  const SE3 T_end = kf_->GetSE3();
  T_predicted_end_ = T_end;  // save for delta correction after ESKF update
  const M3  R_inv = T_end.R_.transpose();
  const V3  T_end_t = T_end.t_;
  const double start_time = measures_.lidar.start_time;
  auto& raw_pc = measures_.lidar.pc;

  std::size_t ptsize = raw_pc->points.size();
  scan_undistort_full_->resize(ptsize);

  const size_t M = propagate_states_.size();
  if (M < 2) {
    // No IMU intervals to interpolate between; copy raw points directly
    for (size_t i = 0; i < ptsize; ++i) {
      const auto& pt = raw_pc->points[i];
      auto& pt_full = scan_undistort_full_->points[i];
      pt_full.x = pt.x;
      pt_full.y = pt.y;
      pt_full.z = pt.z;
      pt_full.intensity = pt.intensity;
    }
    return;
  }

  // Pre-compute per-interval constants to avoid redundant work.
  // For each IMU interval [j, j+1], precompute:
  //   R_end_inv_R_h = R_inv * R_h          (3x3, shared by all points in interval)
  //   t_base = R_inv * (p_h - T_end_t)     (3x1, shared)
  //   v_base = R_inv * v_h                  (3x1, shared)
  //   acc_base = R_inv * acc_t              (3x1, shared)
  //   omega_body = R_h^T * omega            (3x1, body-frame angular velocity)
  //   dt_inv = 1.0 / dt                     (scalar, shared)
  // Then for each point with parameter tau:
  //   R_i ≈ R_h * Exp(omega * tau) ≈ R_h * (I + hat(omega * tau))  [small angle]
  //   R_inv * R_i ≈ R_end_inv_R_h * (I + hat(omega_body * tau))
  //   p_i = p_h + v_h * tau + 0.5 * acc_t * tau^2
  //   result = R_inv * (R_i * raw + p_i - T_end_t)
  //          = R_end_inv_R_h * raw + R_end_inv_R_h * hat(omega_body * tau) * raw
  //            + t_base + v_base * tau + 0.5 * acc_base * tau^2
  // The hat(omega_body*tau)*raw = omega_body*tau × raw, which is cheap.

  struct IntervalCache {
    M3 R_end_inv_R_h;    // R_inv * R_h
    V3 t_base;           // R_inv * (p_h - T_end_t)
    V3 v_base;           // R_inv * v_h
    V3 acc_base;         // R_inv * acc_t * 0.5
    V3 omega_body;       // body-frame angular velocity for small-angle Exp
    double dt_inv;       // 1.0 / dt
    double time_start;   // interval start time
    double time_end;     // interval end time
  };

  std::vector<IntervalCache> interval_cache(M - 1);
  for (size_t j = 0; j + 1 < M; ++j) {
    const auto& s0 = propagate_states_[j];
    const auto& s1 = propagate_states_[j + 1];
    auto& ic = interval_cache[j];
    ic.R_end_inv_R_h = R_inv * s0.R;
    ic.t_base = R_inv * (s0.p - T_end_t);
    ic.v_base = R_inv * s0.v;
    ic.acc_base = 0.5 * R_inv * s1.a;
    // Use raw gyro from the IMU that drove this interval, not state-derived w.
    // State-derived w lags by one Predict call and causes skew during
    // direction reversals (state.w still reflects old rotation direction).
    ic.omega_body = measures_.imu[j].gyr;
    ic.dt_inv = 1.0 / (s1.time - s0.time);
    ic.time_start = s0.time;
    ic.time_end = s1.time;
  }

  // 预提取 offset_time 到连续数组，改善 cache 局部性
  // （AOS 布局下 offset_time 与 x/y/z 交错，遍历时 cache 命中率差）
  const double back_time = propagate_states_.back().time;
  std::vector<double> offset_times(ptsize);
  for (size_t i = 0; i < ptsize; ++i) {
    offset_times[i] = start_time + raw_pc->points[i].offset_time;
  }

  size_t j = 0; // cached interval index

  for (size_t idx = 0; idx < ptsize; ++idx) {
    auto& pt_full = scan_undistort_full_->points[idx];
    const auto& pt = raw_pc->points[idx];
    pt_full.intensity = pt.intensity;

    double query_time = offset_times[idx];
    if (query_time > back_time) {
      pt_full.x = pt.x;
      pt_full.y = pt.y;
      pt_full.z = pt.z;
      continue;
    }

    // Advance j while next interval starts before query_time
    while (j + 1 < M - 1 && interval_cache[j + 1].time_start < query_time) ++j;

    const auto& ic = interval_cache[j];
    const double tau = query_time - ic.time_start;

    // 2nd-order Exp(omega*tau) approximation:
    //   Exp(phi) = I + hat(phi) + 0.5*hat(phi)^2 + O(phi^3)
    //   1st-order (I+hat) drops the cos term → over-rotates points during fast turns.
    //   2nd-order residual < 0.01° at 300°/s — imperceptible even at long range.
    const float px = pt.x, py = pt.y, pz = pt.z;
    const float ox = ic.omega_body[0], oy = ic.omega_body[1], oz = ic.omega_body[2];
    const float tau_f = tau;
    const float htau2 = 0.5f * tau_f * tau_f;
    // c = omega × raw
    const float cx = oy*pz - oz*py;
    const float cy = oz*px - ox*pz;
    const float cz = ox*py - oy*px;
    // c2 = omega × (omega × raw)
    const float c2x = oy*cz - oz*cy;
    const float c2y = oz*cx - ox*cz;
    const float c2z = ox*cy - oy*cx;
    // m = raw + c*tau + 0.5*c2*tau²
    const float mx = px + cx*tau_f + c2x*htau2;
    const float my = py + cy*tau_f + c2y*htau2;
    const float mz = pz + cz*tau_f + c2z*htau2;
    // R_end_inv_R_h * m  （单次 3x3×3x1）
    const M3& R = ic.R_end_inv_R_h;
    const float rx = R(0,0)*mx + R(0,1)*my + R(0,2)*mz;
    const float ry = R(1,0)*mx + R(1,1)*my + R(1,2)*mz;
    const float rz = R(2,0)*mx + R(2,1)*my + R(2,2)*mz;
    // + t_base + v_base*tau + acc_base*tau²
    const float tau2_f = tau_f * tau_f;
    pt_full.x = rx + ic.t_base[0] + ic.v_base[0]*tau_f + ic.acc_base[0]*tau2_f;
    pt_full.y = ry + ic.t_base[1] + ic.v_base[1]*tau_f + ic.acc_base[1]*tau2_f;
    pt_full.z = rz + ic.t_base[2] + ic.v_base[2]*tau_f + ic.acc_base[2]*tau2_f;
  }
}


/// Apply ESKF correction delta to deskewed clouds.
/// After Observe() corrects the pose, the deskewed cloud (which used
/// IMU-predicted trajectory) is slightly misaligned with the corrected body frame.
/// ΔT = T_corrected^{-1} * T_predicted  transforms deskewed points into the
/// corrected body frame, making them self-consistent with the corrected pose.
/// Also updates points_body_v3_ so UpdateMap() uses the corrected points.
void SuperLIO::ApplyDeltaCorrection() {
  const SE3 T_corrected = kf_->GetSE3();
  const SE3 T_pred = T_predicted_end_;

  // ΔR = R_corrected^T * R_predicted
  // Δp = R_corrected^T * (p_predicted - p_corrected)
  const M3  dR = T_corrected.R_.transpose() * T_pred.R_;
  const V3  dp = T_corrected.R_.transpose() * (T_pred.t_ - T_corrected.t_);

  // Apply to full-resolution cloud
  if (scan_undistort_full_ && !scan_undistort_full_->empty()) {
    for (auto& pt : scan_undistort_full_->points) {
      const float px = pt.x, py = pt.y, pz = pt.z;
      pt.x = dR(0,0)*px + dR(0,1)*py + dR(0,2)*pz + dp[0];
      pt.y = dR(1,0)*px + dR(1,1)*py + dR(1,2)*pz + dp[1];
      pt.z = dR(2,0)*px + dR(2,1)*py + dR(2,2)*pz + dp[2];
    }
  }

  // Apply to downsampled cloud (subset of full cloud, corrected independently)
  if (ds_undistort_ && !ds_undistort_->empty()) {
    for (auto& pt : ds_undistort_->points) {
      const float px = pt.x, py = pt.y, pz = pt.z;
      pt.x = dR(0,0)*px + dR(0,1)*py + dR(0,2)*pz + dp[0];
      pt.y = dR(1,0)*px + dR(1,1)*py + dR(1,2)*pz + dp[1];
      pt.z = dR(2,0)*px + dR(2,1)*py + dR(2,2)*pz + dp[2];
    }
  }

  // Re-sync points_body_v3_ for UpdateMap() consistency
  const size_t ptsize = ds_undistort_->size();
  points_body_v3_.resize(ptsize);
  for (size_t i = 0; i < ptsize; ++i) {
    const auto& pt = ds_undistort_->points[i];
    points_body_v3_[i] = V3(pt.x, pt.y, pt.z);
  }
}

/// Queue body-frame cloud for threaded publish, immediately after deskew (pre-ESKF).
/// Pushed to output_queue_ so OutputThread handles it without blocking stateProcess().
void SuperLIO::PublishBodyCloud() {
  if (!g_visual_map_body) return;

  static int count_body = -1;
  count_body++;
  if (count_body % g_pub_step != 0) return;
  count_body = 0;

  OutputData output_data;
  output_data.body_pc.reset(new PointCloudType());
  *output_data.body_pc = *scan_undistort_full_;
  output_data.has_body_pc = true;
  output_data.state.timestamp = measures_.lidar.end_time;

  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    if (output_queue_.size() > 5) output_queue_.pop();
    output_queue_.push(std::move(output_data));
  }
  output_cv_.notify_one();
}

void SuperLIO::DownSample(){
  voxel_grid_fliter_.setInputCloud(scan_undistort_full_);
  voxel_grid_fliter_.filter(ds_undistort_);
}


void SuperLIO::DownSampleOnly(){
  // Directly process raw point cloud without IMU propagation.
  // NOTE: filter_rate/intensity_filter are already applied by ROSWrapper
  // at the point cloud input stage, so measures_.lidar.pc is pre-filtered.
  // Here we only apply range filter and voxel grid filter.
  
  scan_undistort_full_->clear();
  ds_undistort_->clear();
  
  auto& raw_pc = measures_.lidar.pc;
  std::size_t ptsize = raw_pc->size();

  for (size_t i = 0; i < ptsize; ++i) {
    const auto& pt = raw_pc->points[i];
    double dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
    if (dis > g_blind2 && dis < g_maxrange2) {
      pcl::PointXYZI pt_out;
      pt_out.x = pt.x;
      pt_out.y = pt.y;
      pt_out.z = pt.z;
      pt_out.intensity = pt.intensity;
      scan_undistort_full_->push_back(pt_out);
    }
  }
  
  // Apply voxel grid filter if enabled
  if(g_enable_downsample && !scan_undistort_full_->empty()){
    voxel_grid_fliter_.setInputCloud(scan_undistort_full_);
    voxel_grid_fliter_.filter(ds_undistort_);
  }else{
    *ds_undistort_ = *scan_undistort_full_;
  }
}

/// ESKF 观测更新：对每个点查询 IVox 邻域、拟合平面、构建雅可比，
/// 迭代更新状态；首次迭代直接顺序访问，后续迭代使用紧凑索引
void SuperLIO::Observe(){
  size_t ptsize = ds_undistort_->size();
  
  static std::vector<float> _lengths;
  points_body_v3_.resize(ptsize);
  _lengths.resize(ptsize);

  effect_knn_num_ = ptsize;
  // Don't iota effect_knn_idxs_ yet — first iteration uses direct indexing

  for (size_t i = 0; i < ptsize; ++i) {
    const auto& point_body_pcl = ds_undistort_->points[i];
    points_body_v3_[i] = V3(point_body_pcl.x, point_body_pcl.y, point_body_pcl.z);
    _lengths[i] = points_body_v3_[i].norm();
  }

  ivox_->reset_max_group();
  int iter_num = 0;

  kf_->UpdateObserve([&, this](const ESKF::KFState &kf_state, M6 &HTVH, V6 &HTVr) {
    const SE3 pose = kf_state.pose;
    const bool need_converge = kf_state.need_converge;
    const M3d R_transpose = (pose.R_.transpose()).cast<double>();

    M6d sum_HTVH = M6d::Zero();
    V6d sum_HTVr = V6d::Zero();
    KNNHeapType top_K;

    if (iter_num == 0) {
      // First iteration: direct sequential access, no indirect indexing
      for (size_t idx = 0; idx < ptsize; ++idx) {
        V3& point_body = points_body_v3_[idx];
        V3 point_world = pose * point_body;

        top_K.reset();
        ivox_->getTopK(point_world, top_K);
        if(top_K.count < 4){
          effect_mask_[idx] = false;
          effect_knn_mask_[idx] = false;
          continue;
        }
        effect_knn_mask_[idx] = true;
        effect_mask_[idx] = calc_plane_coeff(top_K.count, top_K.points_, abcd_vec_[idx]);
        if(!effect_mask_[idx]) continue;

        auto& abcd = abcd_vec_[idx];
        scalar error;
        effect_mask_[idx] = compute_error(abcd, point_world, _lengths[idx], error);
        if(!effect_mask_[idx]) continue;

        V3d normvec(abcd[0], abcd[1], abcd[2]);
        V3d nb = R_transpose * normvec;
        V3d point_body_d = point_body.cast<double>();
        V6d J;
        J.head<3>() = point_body_d.cross(nb);
        J.tail<3>() = normvec;

        // 距离自适应 Huber 核：残差超过阈值时线性降权，防止静止震荡
        const float huber_delta = g_huber_delta_base + g_huber_delta_scale * _lengths[idx];
        const float abs_error = std::abs(error);
        const float w = (abs_error <= huber_delta) ? g_obs_weight
                                                    : g_obs_weight * huber_delta / abs_error;
        sum_HTVH += J * w * J.transpose();
        sum_HTVr -= J * w * error;
      }

      // Build compact index array for subsequent iterations
      int _effect_knn_num = 0;
      for (size_t i = 0; i < ptsize; ++i) {
        if(!effect_knn_mask_[i]) continue;
        effect_knn_idxs_[_effect_knn_num] = i;
        _effect_knn_num++;
      }
      effect_knn_num_ = _effect_knn_num;
    } else {
      // Subsequent iterations: use compacted indirect index
      for (size_t r_s = 0; r_s < effect_knn_num_; ++r_s) {
        int idx = effect_knn_idxs_[r_s];
        V3& point_body = points_body_v3_[idx];
        V3 point_world = pose * point_body;

        if(!need_converge){
          top_K.reset();
          ivox_->getTopK(point_world, top_K);
          if(top_K.count < 4){
            effect_mask_[idx] = false;
            effect_knn_mask_[idx] = false;
            continue;
          }
          effect_knn_mask_[idx] = true;
          effect_mask_[idx] = calc_plane_coeff(top_K.count, top_K.points_, abcd_vec_[idx]);
        }

        if(!effect_mask_[idx]) continue;

        auto& abcd = abcd_vec_[idx];
        scalar error;
        effect_mask_[idx] = compute_error(abcd, point_world, _lengths[idx], error);
        if(!effect_mask_[idx]) continue;

        V3d normvec(abcd[0], abcd[1], abcd[2]);
        V3d nb = R_transpose * normvec;
        V3d point_body_d = point_body.cast<double>();
        V6d J;
        J.head<3>() = point_body_d.cross(nb);
        J.tail<3>() = normvec;

        // 距离自适应 Huber 核：残差超过阈值时线性降权，防止静止震荡
        const float huber_delta = g_huber_delta_base + g_huber_delta_scale * _lengths[idx];
        const float abs_error = std::abs(error);
        const float w = (abs_error <= huber_delta) ? g_obs_weight
                                                    : g_obs_weight * huber_delta / abs_error;
        sum_HTVH += J * w * J.transpose();
        sum_HTVr -= J * w * error;
      }

      if(!need_converge) {
        int _effect_knn_num = 0;
        for(size_t i = 0; i < effect_knn_num_; ++i){
          int idx = effect_knn_idxs_[i];
          if(!effect_knn_mask_[idx]) continue;
          effect_knn_idxs_[_effect_knn_num] = idx;
          _effect_knn_num++;
        }
        effect_knn_num_ = _effect_knn_num;
      }
    }

    HTVH = sum_HTVH.cast<scalar>();
    HTVr = sum_HTVr.cast<scalar>();

    iter_num++;
  });

  frame_num_++;
}


/// 更新 IVox 地图：将去畸变后的点云添加到地图中
void SuperLIO::UpdateMap() {
  const size_t ptsize = ds_undistort_->size();
  if (ptsize == 0) return;
  
  last_pose_ = kf_->GetSE3();
  points_world_v3_.resize(ptsize);
  
  const auto R = last_pose_.R_;
  const auto t = last_pose_.t_;
  
  for (size_t i = 0; i < ptsize; ++i) {
    const auto& pt = points_body_v3_[i];
    points_world_v3_[i] = R * pt + t;
  }
  
  ivox_->insert(points_world_v3_);

}


/// 输出当前帧结果：世界系点云（body 系点云已在 PublishBodyCloud 立即发出）
void SuperLIO::Output(){
  auto state = kf_->GetNavState();
  
  OutputData output_data;
  output_data.state = state;
  output_data.body_omega = kf_->GetDynamicState().w;
  output_data.lidar_receive_time = measures_.lidar.receive_time;
  output_data.is_undistort_only = g_lio_only_undistort || g_downsample_only;
  output_data.lidar_frame = current_lidar_frame_;

  // downsample_only mode: output without transformation
  if(g_downsample_only){
    output_data.state.timestamp = measures_.lidar.start_time;
    if(g_visual_map){
      static int count = -1;
      count++;
      if(count % g_pub_step == 0){
        count = 0;
        output_data.world_pc.reset(new PointCloudType());
        if(g_visual_dense){
          *output_data.world_pc = *scan_undistort_full_;
        }else{
          *output_data.world_pc = *ds_undistort_;
        }
        output_data.has_world_pc = true;
      }
    }
  }
  else if(g_lio_only_undistort){
    if(g_visual_map){
      static int count = -1;
      count++;
      if(count % g_pub_step == 0){
        count = 0;
        output_data.world_pc.reset(new PointCloudType());
        if(g_visual_dense){
          *output_data.world_pc = *scan_undistort_full_;
        }else{
          *output_data.world_pc = *ds_undistort_;
        }
        output_data.has_world_pc = true;
      }
    }
  }else{
    const Eigen::Matrix3f Rf = state.R.R_.cast<float>();
    const Eigen::Vector3f tf = state.p.cast<float>();

    if(g_visual_map){
      static int count = -1;
      count++;
      if(count % g_pub_step == 0){
        count = 0;
        output_data.world_pc.reset(new PointCloudType());
        if(g_visual_dense){
          transformPointCloudRt(*scan_undistort_full_, *output_data.world_pc, Rf, tf);
        }else{
          transformPointCloudRt(*ds_undistort_, *output_data.world_pc, Rf, tf);
        }
        output_data.has_world_pc = true;
        // Cache transformed cloud for caceData reuse
        last_transformed_world_pc_ = output_data.world_pc;
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    if(output_queue_.size() > 5){
      output_queue_.pop();
    }
    output_queue_.push(std::move(output_data));
  }
  output_cv_.notify_one();
}

void SuperLIO::OutputThread(){
  while(output_running_){
    OutputData data;
    {
      std::unique_lock<std::mutex> lock(output_mutex_);
      output_cv_.wait(lock, [this]{
        return !output_queue_.empty() || !output_running_;
      });
      
      if(!output_running_ && output_queue_.empty()){
        break;
      }
      
      if(output_queue_.empty()){
        continue;
      }
      
      data = std::move(output_queue_.front());
      output_queue_.pop();
    }

    const bool body_only = data.has_body_pc && !data.has_world_pc;

    if (!body_only && !data.is_undistort_only) {
      data_wrapper_->pub_odom(data.state, data.body_omega);
    }

    if (data.has_body_pc && data.body_pc) {
      data_wrapper_->pub_cloud_body(data.body_pc, data.state.timestamp);
    }

    if(data.has_world_pc && data.world_pc){
      if(data.is_undistort_only){
        data_wrapper_->pub_cloud_world_undistort_only(data.world_pc, data.state.timestamp, data.lidar_frame);
      }else{
        data_wrapper_->pub_cloud_world(data.world_pc, data.state.timestamp);
      }
      
      // Record end-to-end latency: lidar receive → cloud_world publish
      if(data.lidar_receive_time > 0.0){
        auto now_s = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        double lat_ms = (now_s - data.lidar_receive_time) * 1000.0;
        data_wrapper_->recordLatency("[Lidar->CloudWorld]", lat_ms);
      }
    }
  }
}

void SuperLIO::printTimeRecord(){
  if(!g_time_eva) return;
  time_record_.PrintAll();
  data_wrapper_->printLatencies();
}


void SuperLIO::pauseProcessing(){
  paused_.store(true);
  LOG(INFO) << YELLOW << " ---> [SuperLIO]: Processing paused" << RESET;
}


void SuperLIO::resumeProcessing(){
  paused_.store(false);
  LOG(INFO) << GREEN << " ---> [SuperLIO]: Processing resumed" << RESET;
}


bool SuperLIO::isPaused() const {
  return paused_.load();
}


void SuperLIO::resetIMUIntegration(){
  if(kf_){
    kf_->ResetIMUIntegration();
    LOG(INFO) << GREEN << " ---> [SuperLIO]: IMU pre-integration reset" << RESET;
  }
}

} // namespace END.