
#include "lio/super_lio.h"

#include <sys/resource.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>

#include <Eigen/Eigenvalues>


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

/// 平面拟合：用 N 个点拟合平面 ax+by+cz+1=0，返回法向量系数 abcd
/// 若给出 max_abs_dist，则输出参与拟合点中的最大 |dist|（作为平面质量指标）
inline bool calc_plane_coeff(const int N, const std::array<V3, 5>& points, std::array<double, 4>& abcd,
                             double* max_abs_dist = nullptr)
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
    if (max_abs_dist) *max_abs_dist = std::max(*max_abs_dist, std::abs(dist));
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


/// 观测信息矩阵（HTVH）的退化签名：条件数 + 退化轴数。
/// Hessian 顺序为 [rotation(3), translation(3)]，退化方向 = 特征值远小于最大特征值的方向。
struct DegeneracySig {
  bool   ok   = false;
  double cond = 0.0;
  int    axes = 0;
  V6d    ev   = V6d::Zero();
  M6d    eV   = M6d::Zero();
  /// 健康子空间投影矩阵（1.1：判据三种模式统一产出，截断直接用它）
  M6d    P    = M6d::Identity();
};

/// 退化判据模式（g_degen_trans_only）：
///   0 = 全 6 维直接分解（原行为）
///   1 = 只对平移 3x3 块分解（TRaIL-Odom：旋转块与平移块量纲不同，混排会造
///       成假退化；旋转轴退化不应吃掉平移约束）
///   2 = 先对角预条件 D^-1 H D^-1（D=sqrt(diag H)）再全 6 维分解
static DegeneracySig computeDegeneracy(const M6d& H, double threshold, int mode) {
  DegeneracySig s;
  if (!H.allFinite()) return s;
  if (mode == 1) {
    // 只判平移：旋转子空间保留全部约束，平移子空间按健康特征向量投影
    const Eigen::Matrix3d T = H.block<3, 3>(3, 3);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(T);
    if (es.info() != Eigen::Success) return s;
    const Eigen::Vector3d ev = es.eigenvalues();
    s.ev.setZero(); s.ev.tail<3>() = ev;
    s.eV = M6d::Identity();
    s.eV.block<3, 3>(3, 3) = es.eigenvectors();
    const double min_eig = ev(0), max_eig = ev(2);
    s.cond = (min_eig > 1e-12) ? max_eig / min_eig : 1e12;
    s.axes = 0;
    Eigen::Matrix3d Pt = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i) {
      if (ev(i) < max_eig / threshold) { ++s.axes; continue; }
      Pt += es.eigenvectors().col(i) * es.eigenvectors().col(i).transpose();
    }
    s.P.setZero();
    s.P.block<3, 3>(0, 0).setIdentity();
    s.P.block<3, 3>(3, 3) = Pt;
    s.ok = true;
    return s;
  }

  M6d Hn = H;
  Eigen::Matrix<double, 6, 1> d = Eigen::Matrix<double, 6, 1>::Ones();
  if (mode == 2) {
    // 对角预条件：把六个方向压到同一量纲后再比较特征值
    for (int i = 0; i < 6; ++i) d(i) = std::sqrt(std::max(H(i, i), 1e-12));
    const Eigen::Matrix<double, 6, 1> dinv = d.cwiseInverse();
    Hn = dinv.asDiagonal() * H * dinv.asDiagonal();
  }
  Eigen::SelfAdjointEigenSolver<M6d> es(Hn);
  if (es.info() != Eigen::Success) return s;
  s.ok = true;
  s.ev = es.eigenvalues();
  s.eV = es.eigenvectors();
  const double min_eig = s.ev(0);
  const double max_eig = s.ev(5);
  s.cond = (min_eig > 1e-12) ? max_eig / min_eig : 1e12;
  s.axes = 0;
  M6d U = M6d::Zero();
  int n_healthy = 0;
  for (int i = 0; i < 6; ++i) {
    if (s.ev(i) < max_eig / threshold) { ++s.axes; continue; }
    // 预条件空间的向量回到原空间：u = D^-1 v
    U.col(n_healthy++) = (mode == 2)
        ? V6d(d.cwiseInverse().asDiagonal() * es.eigenvectors().col(i))
        : V6d(es.eigenvectors().col(i));
  }
  if (n_healthy > 0) {
    // 先落到动态尺寸：n_healthy<6 时 6x3 的 block 直接构造 M6d 会触发 Eigen resize 断言
    const Eigen::MatrixXd Uh = U.leftCols(n_healthy);
    const Eigen::MatrixXd Ph = Uh * (Uh.transpose() * Uh).inverse() * Uh.transpose();
    s.P = M6d(Ph);
  } else {
    s.P.setZero();
  }
  return s;
}

/// 退化方向截断（修自 ros2 分支）：把退化轴（特征值 < max_eig/threshold）的
/// LiDAR 约束清零，交 IMU 预积分接管；再叠加 Tikhonov 地板
/// （λ = tikhonov_lambda * mean_eig）保证信息矩阵可逆。
///
/// 与 ros2 原实现的关键区别：**梯度 HTVr 必须一起投影**。
/// 原版只截断 HTVH 不动 HTVr，信息矩阵在退化轴只剩 Tikhonov 地板（量级 ~1e-3），
/// 而梯度在该方向仍有满量程分量 ⇒ Qk·b 在退化轴放大 1e3 倍，位姿直接被推飞
/// （实测 deg_trunc 发散到 7.9e5 m、地图 4969 万点）。
/// 正确做法是把 H 与 r 一起投影到"健康子空间": (H', r') = (P H P, P r)，P = Σ_healthy v vᵀ。
static void truncateDegenerate(const M6d& H, const V6d& r, const DegeneracySig& s,
                               double tikhonov_lambda, M6d& H_out, V6d& r_out) {
  const M6d& P = s.P;   // 健康子空间投影（判据模式已统一算好）
  const double tikhonov = tikhonov_lambda * H.trace() / 6.0;
  H_out = P * H * P.transpose() + tikhonov * M6d::Identity();
  r_out = P * r;   // 梯度同投影，保证 H 与 r 在同一子空间内一致
}


/// 初始化：创建 IVox 地图、ESKF 滤波器、加载参数
void SuperLIO::init(){
  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
  kf_.reset(new ESKF());
  data_wrapper_->setESKF(kf_);

  // 参数自适应：缓存观测静态基线（配置已由 ROSWrapper 读入），
  // 供"退化档 ↔ 正常档"运行时切换后恢复用。
  base_normal_weight_en_ = g_normal_weight_en;
  base_normal_sigma_      = g_normal_sigma;
  base_plane_fit_         = g_plane_fit_threshold;
  base_huber_             = g_huber_delta_base;

  scan_undistort_full_.reset(new PointCloudType());
  ds_undistort_.reset(new PointCloudType());

  if(g_save_map){
    point_map_.reset(new PointCloudType());
  }
  
  points_world_v3_.reserve(21000);
  abcd_vec_.resize(kMaxEffectPoints);
  effect_knn_idxs_.resize(kMaxEffectPoints);
  voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

  state_fn_ = &SuperLIO::stateWaitKFInit;

  output_running_ = true;
  output_thread_ = std::thread(&SuperLIO::OutputThread, this);

  // 全量点异步去畸变/变换线程（dense 话题 + pcd），总是启动
  dense_running_ = true;
  dense_thread_ = std::thread(&SuperLIO::DenseThread, this);

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

  dense_running_ = false;
  dense_cv_.notify_all();
  if(dense_thread_.joinable()){
    dense_thread_.join();
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
  // 实验 C：用固定先验地图代替首帧建图，配合 g_map_freeze 隔离地图反馈
  if(!g_map_load_path.empty()){
    if (loadFixedMap()) {
      kf_->init_ = true;
      state_fn_ = &SuperLIO::stateProcess;
      LOG(INFO) << GREEN << " ---> [SuperLIO]: Fixed-map mode (freeze="
                << g_map_freeze << "), map init done" << RESET;
    }
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
  // M.31 场景自动档：窗口未满时走楼梯档（A 线），因此初值也取楼梯档 na，
  // 保证第 1 帧起就是 A 线行为（冷启动用 B 线会在楼梯上先灌入不可恢复的漂移）。
  options.acce_var_ = g_auto_scene_en ? g_auto_scene_stair_na : g_imu_na;
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
  // SetInitialConditions 会把 P_ 重置为先验（ESKF.cpp:58-59）并把世界原点设在当前
  // imu/lidar 位置 —— 对下游而言 odom 世界系刚刚被重建，必须声明重置
  if(data_wrapper_){ data_wrapper_->notifyOdomReset("kf_init: filter re-initialized"); }
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


/// 实验 C：载入固定先验地图，替代"首帧点云建图"。
/// 地图与该 run 的 odom 世界系一致（同一 bag 的 map.pcd 即可直接用），
/// 起始位姿由 lio.map.init_pose = [x,y,z,roll,pitch,yaw(deg)] 覆盖，缺省为单位位姿。
bool SuperLIO::loadFixedMap(){
  static bool loaded = false;
  if(loaded) return true;

  auto cloud = std::make_shared<PointCloudType>();
  if(pcl::io::loadPCDFile<PointType>(g_map_load_path, *cloud) == -1){
    LOG(ERROR) << RED << " ---> Load fixed map failed: " << g_map_load_path << RESET;
    return false;
  }
  std::vector<int> useless_indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, useless_indices);

  VV3 pts;
  pts.reserve(cloud->size());
  for(const auto& p : *cloud){
    pts.emplace_back(p.x, p.y, p.z);
  }
  ivox_->insert(pts);

  if(g_map_load_use_pose){
    const double d2r = M_PI / 180.0;
    const V3 rpy(g_map_load_pose[3] * d2r, g_map_load_pose[4] * d2r,
                 g_map_load_pose[5] * d2r);
    const M3 R = (Eigen::AngleAxis<scalar>(rpy[2], V3::UnitZ())
                * Eigen::AngleAxis<scalar>(rpy[1], V3::UnitY())
                * Eigen::AngleAxis<scalar>(rpy[0], V3::UnitX())).toRotationMatrix();
    auto state = kf_->GetSysState();
    state.R = SO3(R);
    state.p = V3(g_map_load_pose[0], g_map_load_pose[1], g_map_load_pose[2]);
    kf_->SetX(state);
    sys_init_pose_ = kf_->GetSE3();
  }
  kf_->SetLastObsTime(measures_.lidar.end_time);

  LOG(INFO) << GREEN << " ---> Load fixed map: " << g_map_load_path
            << "  points: " << cloud->size()
            << "  init_pose: " << (g_map_load_use_pose ? "set" : "identity") << RESET;
  loaded = true;
  return true;
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
  }else{
    Propagation_Undistort();
    PublishBodyCloud();   // immediate, pre-ESKF (minimum latency)
    DownSample();
  }

  // ===== 坏帧护栏 =====
  // 丢帧/IMU 欠载的帧，其去畸变点云是坏的（实测一帧 100 ms 只有 55 ms IMU 覆盖，
  // 配准一次跳变 1~3.7 m 且不恢复）。这类帧不参与配准与建图，只让 IMU 预测推进。
  // 安全阀：连续跳过 g_guard_max_consecutive 帧后强制放行一帧，避免雷达频率低于
  // 阈值时永久跳过、退化成纯 IMU 积分。
  bool frame_trusted = true;
  if(g_frame_gap_guard_en){
    const double gap = (last_lidar_end_time_ < 0.0)
        ? 0.0 : (measures_.lidar.end_time - last_lidar_end_time_);
    const double imu_head_gap = measures_.imu.empty()
        ? 1e9 : (measures_.imu.front().secs - measures_.lidar.start_time);
    if(gap > g_frame_gap_max || imu_head_gap > g_imu_cover_max){
      frame_trusted = false;
      ++guard_consec_skip_;
      ++guard_skip_total_;
      if(g_guard_max_consecutive > 0 && guard_consec_skip_ >= g_guard_max_consecutive){
        // 安全阀触发：强制放行，并清零计数（下一次仍从 0 开始累计）
        frame_trusted = true;
        guard_consec_skip_ = 0;
        LOG(WARNING) << "[Guard] safety valve: forced pass after "
                     << g_guard_max_consecutive << " consecutive skips, total_skipped="
                     << guard_skip_total_;
      }
      if(frame_num_ - guard_last_log_frame_ > 20){
        LOG(WARNING) << "[Guard] skip untrusted frame: lidar_gap=" << gap * 1000.0
                     << " ms, imu_head_gap=" << imu_head_gap * 1000.0
                     << " ms, consec=" << guard_consec_skip_
                     << ", total_skipped=" << guard_skip_total_;
        guard_last_log_frame_ = frame_num_;
      }
    }else{
      guard_consec_skip_ = 0;
    }
  }
  last_lidar_end_time_ = measures_.lidar.end_time;

  if(frame_trusted){
    if(g_time_eva){
      time_record_.Evaluate([this]() { Observe(); },             "[Observe]");
      time_record_.Evaluate([this]() { ApplyDeltaCorrection(); },"[DeltaCorrect]");
      time_record_.Evaluate([this]() { UpdateMap(); },           "[UpdateMap]");
    }else{
      Observe();
      ApplyDeltaCorrection();
      UpdateMap();
    }
    Output();                // world cloud via queue (high precision)
    enqueueDense();          // 全量点异步去畸变/变换（错峰，dense 话题 + pcd 来源）
  }
  caceSCPGOData();
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


/// 把当前帧的全量点 + IMU 轨迹 + 配准位姿投递到异步 dense 队列（配准后调用）。
/// 全量去畸变/变换在 DenseThread 错峰执行，不阻塞实时配准链路。
void SuperLIO::enqueueDense(){
  if(!data_wrapper_) return;
  if(!measures_.lidar.pc_full || measures_.lidar.pc_full->empty()) return;
  if(propagate_states_.size() < 2) return;   // 无 IMU 区间，去畸变无意义

  DenseData data;
  data.pc_full = measures_.lidar.pc_full;    // shared_ptr 共享，避免 2 万点深拷贝
  data.propagate_states = propagate_states_; // 值拷贝（主线程下一帧会覆盖）
  data.imu_gyr.reserve(measures_.imu.size());
  // 与 ESKF 传播一致：减掉陀螺零偏后再去畸变（同 Propagation_Undistort）
  const V3 bg = kf_->GetSysState().bg;
  for (auto &imu : measures_.imu) {
    data.imu_gyr.emplace_back(imu.gyr - bg);
  }
  data.start_time = measures_.lidar.start_time;
  data.timestamp = kf_->GetNavState().timestamp;
  data.T_pred = T_predicted_end_;
  data.T_corrected = kf_->GetSE3();          // 配准后位姿

  {
    std::lock_guard<std::mutex> lock(dense_mutex_);
    if(dense_queue_.size() > 3) dense_queue_.pop();   // 有界，丢最旧
    dense_queue_.push(std::move(data));
  }
  dense_cv_.notify_one();
}


/// 异步全量点处理线程：去畸变 -> delta 修正 -> 发布 body_dense -> world 变换仅供 pcd 累积。
void SuperLIO::DenseThread(){
  int scan_wait_num = 0;
  bool rm_PCD_dir = false;
  while(dense_running_){
    DenseData data;
    {
      std::unique_lock<std::mutex> lock(dense_mutex_);
      dense_cv_.wait(lock, [this]{
        return (!dense_queue_.empty() && !dense_paused_) || !dense_running_;
      });
      if(!dense_running_) break;                  // 退出：无论队列是否残留
      if(dense_queue_.empty() || dense_paused_) continue;
      data = std::move(dense_queue_.front());
      dense_queue_.pop();
      dense_busy_ = true;                         // 帧处理中（供存图前静默屏障判定）
    }

    if(!data.pc_full || data.pc_full->empty()){
      {
        std::lock_guard<std::mutex> lock(dense_mutex_);
        dense_busy_ = false;
      }
      dense_cv_.notify_all();
      continue;
    }

    // 1. 全量去畸变（用预测位姿，异步执行，不阻塞实时链路）
    CloudPtr dense_body(new PointCloudType());
    undistortCloud(data.pc_full, data.propagate_states, data.imu_gyr,
                   data.start_time, data.T_pred, dense_body);

    // 2. delta correction：T_corrected vs T_pred 的修正应用到全量 body 点
    const M3 dR = data.T_corrected.R_.transpose() * data.T_pred.R_;
    const V3 dp = data.T_corrected.R_.transpose() * (data.T_pred.t_ - data.T_corrected.t_);
    for (auto& pt : dense_body->points) {
      const float px = pt.x, py = pt.y, pz = pt.z;
      pt.x = dR(0,0)*px + dR(0,1)*py + dR(0,2)*pz + dp[0];
      pt.y = dR(1,0)*px + dR(1,1)*py + dR(1,2)*pz + dp[1];
      pt.z = dR(2,0)*px + dR(2,1)*py + dR(2,2)*pz + dp[2];
    }

    // 3. 发布 cloud_body_dense（机体系全量）
    data_wrapper_->pub_cloud_body_dense(dense_body, data.timestamp);

    // 4. 变换到 world（全量）——仅供 PCD 落盘，world 系全量不再对外发布话题
    CloudPtr dense_world(new PointCloudType());
    if(g_save_map){
      transformPointCloudRt(*dense_body, *dense_world, data.T_corrected.R_, data.T_corrected.t_);
    }

    // 5. pcd 累积 + 保存（world 系全量点作为 pcd 来源）
    if(g_save_map && !dense_world->empty()){
      if(!rm_PCD_dir){
        rm_PCD_dir = true;
        std::string save_map_dir = g_save_map_dir;
        if(!save_map_dir.empty() && save_map_dir[0] != '/') {
          save_map_dir = g_root_dir + save_map_dir;
        }
        std::string cmd = "rm -rf " + save_map_dir + "/PCD";
        [[maybe_unused]] int res = system(cmd.c_str());
        cmd = "mkdir -p " + save_map_dir + "/PCD";
        res = system(cmd.c_str());
      }
      // point_map_ / pcd_index_ 与 saveMap() 并发访问，统一由 map_mutex_ 保护
      SaveData save_data;
      bool do_save = false;
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        *point_map_ += *dense_world;
        scan_wait_num++;
        if(g_pcd_save_interval < 0){
          scan_wait_num = 0;
        } else if(point_map_->size() > 0 && scan_wait_num >= g_pcd_save_interval){
          pcd_index_++;
          save_data.cloud_to_save.reset(new PointCloudType(*point_map_));
          save_data.pcd_index = pcd_index_;
          save_data.timestamp = data.timestamp;
          save_data.position = data.T_corrected.t_;
          save_data.orientation = BASIC::Quat(data.T_corrected.R_);
          point_map_->clear();
          scan_wait_num = 0;
          do_save = true;
        }
      }
      if(do_save){
        {
          std::lock_guard<std::mutex> lock(save_mutex_);
          if(save_queue_.size() > 3){
            save_queue_.pop();
          }
          save_queue_.push(std::move(save_data));
        }
        save_cv_.notify_one();
      }
    }

    // 本帧处理完毕：清除 busy 并唤醒可能在等待静默的 pauseProcessing()
    {
      std::lock_guard<std::mutex> lock(dense_mutex_);
      dense_busy_ = false;
    }
    dense_cv_.notify_all();
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
  // Prefix match, not substring: "scans_" must not pick up "filtered_scans_*",
  // otherwise the unfiltered backup would contain every fragment twice.
  std::vector<std::string> pcd_files;
  for (const auto& entry : fs::directory_iterator(pcd_folder)) {
    if (entry.path().extension() == ".pcd" &&
        entry.path().filename().string().find(scan_prefix) == 0) {
      pcd_files.push_back(entry.path().string());
    }
  }
  // Merge is order-independent, so plain string order is fine here.
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
    SaveData save_data;
    bool do_save = false;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (point_map_->size() > 0) {
        pcd_index_++;
        
        auto state = kf_->GetNavState();
        save_data.cloud_to_save.reset(new PointCloudType(*point_map_));
        save_data.pcd_index = pcd_index_;
        save_data.timestamp = state.timestamp;
        save_data.position = state.p.cast<float>();
        save_data.orientation = BASIC::Quat(state.R.R_.cast<float>());
        
        point_map_->clear();
        do_save = true;
      }
    }
    if(do_save){
      {
        std::lock_guard<std::mutex> lock(save_mutex_);
        save_queue_.push(std::move(save_data));
      }
      save_cv_.notify_one();
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
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
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
  // 构建每个区间的原始陀螺仪序列（去畸变 omega_body 用，异步 dense 也复用）
  // 必须与 ESKF 传播用同一套角速度：Predict() 里是 (gyr - bg_)，
  // 若这里直接用原始 gyr，帧内每个点会被多转 bg*tau（bg_x≈3.9 deg/s 时
  // 100 ms 帧尾可达 0.39 deg 的常值 roll 偏斜，方向恒定，属于系统性偏置）。
  const V3 bg = kf_->GetSysState().bg;
  std::vector<V3> imu_gyr;
  imu_gyr.reserve(measures_.imu.size());
  for (auto &imu : measures_.imu) {
    imu_gyr.emplace_back(imu.gyr - bg);
  }

  // 本帧平均角速度模（2.D 阈值随运动幅度自适应用）
  double omega_sum = 0.0;
  for (const auto& w : imu_gyr) omega_sum += w.norm();
  frame_omega_ = imu_gyr.empty() ? 0.0 : omega_sum / static_cast<double>(imu_gyr.size());

  undistortCloud(measures_.lidar.pc, propagate_states_, imu_gyr,
                 measures_.lidar.start_time, T_end, scan_undistort_full_,
                 g_delta_interp_en ? &scan_offsets_full_ : nullptr);
}

/// 通用去畸变：对 raw_pc 按 IMU 传播轨迹(propagate_states)去畸变到 out_pc。
/// 实时抽稀点与异步全量点共用此函数；imu_gyr 为每个区间原始陀螺仪(长度=区间数)。
void SuperLIO::undistortCloud(const pcl::PointCloud<PointXTZIT>::Ptr& raw_pc,
                              const std::vector<DynamicState>& propagate_states,
                              const std::vector<V3>& imu_gyr,
                              double start_time,
                              const SE3& T_end,
                              CloudPtr& out_pc,
                              std::vector<float>* out_offsets){
  const M3  R_inv = T_end.R_.transpose();
  const V3  T_end_t = T_end.t_;

  std::size_t ptsize = raw_pc->points.size();
  out_pc->resize(ptsize);

  const size_t M = propagate_states.size();
  if (M < 2) {
    // No IMU intervals to interpolate between; copy raw points directly
    for (size_t i = 0; i < ptsize; ++i) {
      const auto& pt = raw_pc->points[i];
      auto& pt_full = out_pc->points[i];
      pt_full.x = pt.x;
      pt_full.y = pt.y;
      pt_full.z = pt.z;
      pt_full.intensity = pt.intensity;
    }
    return;
  }

  // Pre-compute per-interval constants to avoid redundant work.
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
    const auto& s0 = propagate_states[j];
    const auto& s1 = propagate_states[j + 1];
    auto& ic = interval_cache[j];
    ic.R_end_inv_R_h = R_inv * s0.R;
    ic.t_base = R_inv * (s0.p - T_end_t);
    ic.v_base = R_inv * s0.v;
    ic.acc_base = 0.5 * R_inv * s1.a;
    // Use raw gyro from the IMU that drove this interval, not state-derived w.
    // State-derived w lags by one Predict call and causes skew during
    // direction reversals (state.w still reflects old rotation direction).
    ic.omega_body = (j < imu_gyr.size()) ? imu_gyr[j] : s0.w;
    ic.dt_inv = 1.0 / (s1.time - s0.time);
    ic.time_start = s0.time;
    ic.time_end = s1.time;
  }

  if (out_offsets) out_offsets->clear();
  // 预提取 offset_time 到连续数组，改善 cache 局部性
  const double back_time = propagate_states.back().time;
  std::vector<double> offset_times(ptsize);
  for (size_t i = 0; i < ptsize; ++i) {
    offset_times[i] = start_time + raw_pc->points[i].offset_time;
  }

  size_t j = 0;             // cached interval index
  size_t write_idx = 0;     // 压缩写入位置：尾部不可去畸变的点被剔除

  for (size_t idx = 0; idx < ptsize; ++idx) {
    const auto& pt = raw_pc->points[idx];

    double query_time = offset_times[idx];
    if (query_time > back_time) {
      // 超出 IMU 覆盖范围，无法去畸变：剔除而不是原样透传，
      // 否则这些点会以未经补偿的坐标混入观测与地图。
      continue;
    }

    auto& pt_full = out_pc->points[write_idx++];
    pt_full.intensity = pt.intensity;
    // 2.E：点时刻对齐输出（点类型无 offset_time 字段，用并列数组承载）
    if (out_offsets) out_offsets->push_back(static_cast<float>(pt.offset_time));

    // Advance j while next interval starts before query_time
    while (j + 1 < M - 1 && interval_cache[j + 1].time_start < query_time) ++j;

    const auto& ic = interval_cache[j];
    const double tau = query_time - ic.time_start;

    // 2nd-order Exp(omega*tau) approximation.
    const float px = pt.x, py = pt.y, pz = pt.z;
    const float ox = ic.omega_body[0], oy = ic.omega_body[1], oz = ic.omega_body[2];
    const float tau_f = tau;
    const float htau2 = 0.5f * tau_f * tau_f;
    const float cx = oy*pz - oz*py;
    const float cy = oz*px - ox*pz;
    const float cz = ox*py - oy*px;
    const float c2x = oy*cz - oz*cy;
    const float c2y = oz*cx - ox*cz;
    const float c2z = ox*cy - oy*cx;
    const float mx = px + cx*tau_f + c2x*htau2;
    const float my = py + cy*tau_f + c2y*htau2;
    const float mz = pz + cz*tau_f + c2z*htau2;
    const M3& R = ic.R_end_inv_R_h;
    const float rx = R(0,0)*mx + R(0,1)*my + R(0,2)*mz;
    const float ry = R(1,0)*mx + R(1,1)*my + R(1,2)*mz;
    const float rz = R(2,0)*mx + R(2,1)*my + R(2,2)*mz;
    const float tau2_f = tau_f * tau_f;
    pt_full.x = rx + ic.t_base[0] + ic.v_base[0]*tau_f + ic.acc_base[0]*tau2_f;
    pt_full.y = ry + ic.t_base[1] + ic.v_base[1]*tau_f + ic.acc_base[1]*tau2_f;
    pt_full.z = rz + ic.t_base[2] + ic.v_base[2]*tau_f + ic.acc_base[2]*tau2_f;
  }
  out_pc->resize(write_idx);
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

  // 2.E：按点时刻分配 ΔT。整帧刚性修正的残差 ∝ 帧内位置×修正量，
  // 与 M.11 的"非速率型俯仰偏置"形态吻合，因此按点时刻做 3 段线性插值：
  // 段内修正量 = s·(ΔR, Δp)，s ∈ {1/3, 2/3, 1}；s=1 段与刚性修正完全一致。
  M3 dR_seg[3] = {dR, dR, dR};
  V3 dp_seg[3] = {dp, dp, dp};
  // offsets 是相对 start_time 的秒偏移，故只用帧时长做归一化
  double t_span = 1.0;
  if (g_delta_interp_en && !scan_undistort_full_->empty()) {
    const Eigen::Matrix3d dRd = dR.cast<double>();
    const Eigen::AngleAxisd aa(dRd);
    for (int k = 0; k < 3; ++k) {
      const double sk = static_cast<double>(k + 1) / 3.0;
      dR_seg[k] = Eigen::AngleAxisd(aa.angle() * sk, aa.axis()).toRotationMatrix().cast<float>();
      dp_seg[k] = dp * sk;
    }
    t_span = std::max(1e-3, measures_.lidar.end_time - measures_.lidar.start_time);
  }

  auto apply_delta = [&](BASIC::CloudPtr& cloud, const std::vector<float>* offsets) {
    if (!cloud || cloud->empty()) return;
    const bool use_off = g_delta_interp_en && offsets && offsets->size() == cloud->size();
    for (size_t pi = 0; pi < cloud->points.size(); ++pi) {
      auto& pt = cloud->points[pi];
      int k = 2;
      if (use_off) {
        double frac = (*offsets)[pi] / t_span;
        frac = std::min(1.0, std::max(0.0, frac));
        k = std::min(2, static_cast<int>(frac * 3.0));
      }
      const M3& R = dR_seg[k];
      const V3& T = dp_seg[k];
      const float px = pt.x, py = pt.y, pz = pt.z;
      pt.x = R(0,0)*px + R(0,1)*py + R(0,2)*pz + T[0];
      pt.y = R(1,0)*px + R(1,1)*py + R(1,2)*pz + T[1];
      pt.z = R(2,0)*px + R(2,1)*py + R(2,2)*pz + T[2];
    }
  };

  // Apply to full-resolution cloud
  apply_delta(scan_undistort_full_, &scan_offsets_full_);
  // Apply to downsampled cloud (subset of full cloud, corrected independently)
  apply_delta(ds_undistort_, &scan_offsets_ds_);

  // Re-sync points_body_v3_ for UpdateMap() consistency
  const size_t ptsize = ds_undistort_->size();
  points_body_v3_.resize(ptsize);
  for (size_t i = 0; i < ptsize; ++i) {
    const auto& pt = ds_undistort_->points[i];
    points_body_v3_[i] = V3(pt.x, pt.y, pt.z);
  }
}

/// Body cloud is now published from Output() with state.timestamp (post-ESKF),
/// ensuring timestamp sync with lio/odom for downstream PGO consumption.
/// PublishBodyCloud is kept as no-op to preserve call sites; low-latency
/// visualization now uses the post-ESKF cloud (latency cost: ~5-20ms).
void SuperLIO::PublishBodyCloud() {
  // Body cloud publishing moved to Output() for timestamp consistency with odom.
  return;
}

void SuperLIO::DownSample(){
  voxel_grid_fliter_.setInputCloud(scan_undistort_full_);
  std::vector<int> src_idx;
  voxel_grid_fliter_.filter(ds_undistort_, &src_idx);
  // 2.E：把全量点的时刻映射到降采样点（保持逐点 ΔT 分段可用）
  scan_offsets_ds_.clear();
  if (g_delta_interp_en && src_idx.size() == ds_undistort_->size()) {
    scan_offsets_ds_.resize(src_idx.size());
    for (size_t i = 0; i < src_idx.size(); ++i) {
      const int si = src_idx[i];
      scan_offsets_ds_[i] = (si >= 0 && si < static_cast<int>(scan_offsets_full_.size()))
          ? scan_offsets_full_[si] : 0.0f;
    }
  }
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

  // P0-1：effect_mask_ / abcd_vec_ 等为定长数组，超限会越界；宁少不越界
  if (ptsize > kMaxEffectPoints) {
    LOG(WARNING) << "[Observe] ptsize " << ptsize << " exceeds " << kMaxEffectPoints
                 << ", truncated (tune lio.sensor.filter_rate / enable_downsample)";
    ptsize = kMaxEffectPoints;
  }

  const bool debug = g_debug_level > 0;
  kf_->SetDebug(debug);

  // 参数自适应：用上一帧退化签名切换观测参数档（退化档 ↔ 静态基线）。
  // 观测侧参数只影响配准更新、不写地图（UpdateMap 全量插入，与这些参数无关），
  // 运行时切换安全；这些全局仅在本线程的 Observe 路径被读取。
  // M.31 场景自动档优先于退化档：场景判定（慢时间尺度）一旦给出结果，
  // 观测参数就固定在该档，不再被帧级退化签名来回改（避免逐帧抖动）。
  if (g_auto_scene_en) {
    const bool stair = (!scene_decided_) || (!scene_corridor_);
    g_normal_weight_en = stair ? g_auto_scene_stair_nw : g_auto_scene_corridor_nw;
    if (stair) g_normal_sigma = g_auto_scene_stair_nw_sigma;
  } else if (g_adaptive_param_en) {
    const bool degen = degeneracy_active_;
    g_normal_weight_en    = degen ? g_adaptive_normal_weight_en : base_normal_weight_en_;
    g_normal_sigma        = degen ? g_adaptive_normal_sigma    : base_normal_sigma_;
    g_plane_fit_threshold = (degen && g_adaptive_plane_fit > 0.0) ? g_adaptive_plane_fit : base_plane_fit_;
    g_huber_delta_base    = (degen && g_adaptive_huber > 0.0)     ? g_adaptive_huber     : base_huber_;
  }

  static std::vector<float> _lengths;
  static std::vector<float> _w_quality;
  points_body_v3_.resize(ptsize);
  _lengths.resize(ptsize);
  _w_quality.resize(ptsize, 1.0f);
  // resize 在 size 不变时不清空旧值；normal_weight 运行时切换会复用上帧陈旧权重。
  // 每帧强制回填为 1 消除隐患（只多一次 O(ptsize) 的 float 写，成本可忽略）。
  std::fill(_w_quality.begin(), _w_quality.end(), 1.0f);
  if (debug) fit_res_vec_.clear();

  // 细通道：清空本帧统计与候选（候选在 UpdateObserve 首轮收集，见平面拟合失败分支）
  fine_cands_.clear();
  fine_n_used_  = 0;
  fine_gain_    = 0.0;
  fine_applied_ = false;

  effect_knn_num_ = ptsize;
  // Don't iota effect_knn_idxs_ yet — first iteration uses direct indexing

  for (size_t i = 0; i < ptsize; ++i) {
    const auto& point_body_pcl = ds_undistort_->points[i];
    points_body_v3_[i] = V3(point_body_pcl.x, point_body_pcl.y, point_body_pcl.z);
    _lengths[i] = points_body_v3_[i].norm();
  }

  // 2.A：世界系 up（ESKF 的重力反向），用于地面/天花板点的对齐软约束
  V3d up_world = V3d::Zero();
  if (g_ground_weight > 0.0) {
    const V3 g_vec = kf_->GetGravity();
    if (g_vec.norm() > 1e-6) up_world = (-g_vec.normalized()).cast<double>();
  }

  ivox_->reset_max_group();
  int iter_num = 0;
  // 首轮统计（仅埋点用）
  int n_knn_fail = 0, n_plane_fail = 0, n_res_fail = 0, n_effect = 0;
  M6d last_HTVH = M6d::Zero();

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
          ++n_knn_fail;
          continue;
        }
        effect_knn_mask_[idx] = true;
        double fit_res = 0.0;
        effect_mask_[idx] = calc_plane_coeff(top_K.count, top_K.points_, abcd_vec_[idx], &fit_res);
        if(!effect_mask_[idx]) {
          ++n_plane_fail;
          // 细通道候选：邻域非平面（墙角/扶手/窗框等粗糙区）。取 KNN 中最近的
          // 地图原始点做 point-to-point 关联，关联只在首轮做，后续轮次沿用。
          if (g_fine_channel_en &&
              static_cast<int>(fine_cands_.size()) < g_fine_channel_max_points) {
            int best = 0;
            for (int k = 1; k < top_K.count; ++k)
              if (top_K.dist2_[k] < top_K.dist2_[best]) best = k;
            if (static_cast<double>(top_K.dist2_[best]) <=
                g_fine_max_dist * g_fine_max_dist) {
              FineCand c;
              c.p_body = point_body;
              c.p_map  = top_K.points_[best].cast<double>();
              fine_cands_.push_back(c);
            }
          }
          continue;
        }

        if (debug) fit_res_vec_.push_back(static_cast<scalar>(fit_res));
        // 法向质量加权：拟合残差越大（平面越"脏"），该点观测权重越低
        if (g_normal_weight_en) {
          const float q = static_cast<float>(fit_res) / static_cast<float>(std::max(1e-3, g_normal_sigma));
          _w_quality[idx] = 1.0f / (1.0f + q * q);
        }

        auto& abcd = abcd_vec_[idx];
        scalar error;
        effect_mask_[idx] = compute_error(abcd, point_world, _lengths[idx], error);
        if(!effect_mask_[idx]) { ++n_res_fail; continue; }
        ++n_effect;

        V3d normvec(abcd[0], abcd[1], abcd[2]);
        V3d nb = R_transpose * normvec;
        V3d point_body_d = point_body.cast<double>();
        V6d J;
        J.head<3>() = point_body_d.cross(nb);
        J.tail<3>() = normvec;

        // 距离自适应 Huber 核：残差超过阈值时线性降权，防止静止震荡
        float huber_delta = g_huber_delta_base + g_huber_delta_scale * _lengths[idx];
        // 2.D：高速旋转段放宽阈值，避免运动畸变残差被误降权
        if (g_thresh_motion_k > 0.0) {
          huber_delta *= static_cast<float>(1.0 + g_thresh_motion_k * frame_omega_);
        }
        const float abs_error = std::abs(error);
        float w = (abs_error <= huber_delta) ? g_obs_weight
                                             : g_obs_weight * huber_delta / abs_error;
        if (g_normal_weight_en) w *= _w_quality[idx];
        // 2.B：观测噪声按距离建模 w /= (1 + (k*r)^2)
        if (g_obs_dist_k > 0.0) {
          const double kr = g_obs_dist_k * _lengths[idx];
          w *= static_cast<float>(1.0 / (1.0 + kr * kr));
        }
        sum_HTVH += J * w * J.transpose();
        sum_HTVr -= J * w * error;
        // 2.A：地面/天花板点追加"法向对齐重力"的旋转残差（独立权重）
        if (g_ground_weight > 0.0) {
          const double s_up = (normvec.dot(up_world) >= 0.0) ? 1.0 : -1.0;
          const double cosang = s_up * normvec.dot(up_world);
          if (cosang > 0.95) {
            const V3d Jg = s_up * normvec.cross(up_world);
            const double eg = cosang - 1.0;
            sum_HTVH.block<3, 3>(0, 0) += g_ground_weight * Jg * Jg.transpose();
            sum_HTVr.head<3>() -= g_ground_weight * Jg * eg;
          }
        }
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
          double fit_res = 0.0;  // 后续轮次不更新质量权重，只用首轮结果
          effect_mask_[idx] = calc_plane_coeff(top_K.count, top_K.points_, abcd_vec_[idx], &fit_res);
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
        float huber_delta = g_huber_delta_base + g_huber_delta_scale * _lengths[idx];
        // 2.D：高速旋转段放宽阈值，避免运动畸变残差被误降权
        if (g_thresh_motion_k > 0.0) {
          huber_delta *= static_cast<float>(1.0 + g_thresh_motion_k * frame_omega_);
        }
        const float abs_error = std::abs(error);
        float w = (abs_error <= huber_delta) ? g_obs_weight
                                             : g_obs_weight * huber_delta / abs_error;
        if (g_normal_weight_en) w *= _w_quality[idx];
        // 2.B：观测噪声按距离建模 w /= (1 + (k*r)^2)
        if (g_obs_dist_k > 0.0) {
          const double kr = g_obs_dist_k * _lengths[idx];
          w *= static_cast<float>(1.0 / (1.0 + kr * kr));
        }
        sum_HTVH += J * w * J.transpose();
        sum_HTVr -= J * w * error;
        // 2.A：地面/天花板点追加"法向对齐重力"的旋转残差（独立权重）
        if (g_ground_weight > 0.0) {
          const double s_up = (normvec.dot(up_world) >= 0.0) ? 1.0 : -1.0;
          const double cosang = s_up * normvec.dot(up_world);
          if (cosang > 0.95) {
            const V3d Jg = s_up * normvec.cross(up_world);
            const double eg = cosang - 1.0;
            sum_HTVH.block<3, 3>(0, 0) += g_ground_weight * Jg * Jg.transpose();
            sum_HTVr.head<3>() -= g_ground_weight * Jg * eg;
          }
        }
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

    last_HTVH = sum_HTVH;   // 原始 Hessian（供埋点/退化检测，未经截断）

    // ===== 细通道（M.27）：退化帧把"平面拟合拒绝点"以 point-to-point 并入 =====
    // 触发：上一帧去抖后的 degeneracy_active_（与截断同源，避免本帧 cond 抖动开关）。
    // 残差：把最近地图原始点 p_map 视作"过该点、法向 u=连线方向"的伪平面，
    //       与平面残差同构（e = u·(p_world−p_map) = −dist），复用既有 J/Huber 约定。
    // 门控：λ_min 增益——细通道必须实际改善最弱约束方向，否则整帧弃用
    //       （区分"有扶手可救"与"真空走廊"，后者细通道只添噪）。
    // 通道隔离：last_HTVH 已在上面取"纯平面 H"，退化检测/去抖不包含细通道贡献，
    //           防止"细通道改善 cond → 退出退化 → 细通道关闭"的自激振荡。
    if (g_fine_channel_en && degeneracy_active_ && !fine_cands_.empty()) {
      M6d H_f = M6d::Zero();
      V6d r_f = V6d::Zero();
      int n_used = 0;
      for (const auto& c : fine_cands_) {
        const V3d p_world = (pose * c.p_body).cast<double>();
        const V3d d_vec = c.p_map - p_world;
        const double dist = d_vec.norm();
        if (dist > g_fine_max_dist || dist < 1e-3) continue;  // 迭代后走远/重合则弃
        const V3d u = d_vec / dist;
        const V3d nb = R_transpose * u;
        const V3d p_body_d = c.p_body.cast<double>();
        V6d J;
        J.head<3>() = p_body_d.cross(nb);
        J.tail<3>() = u;
        const double e = -dist;                       // 与平面残差同符号约定
        double w = g_fine_channel_weight;             // 独立权重：细点噪声大
        const double huber_delta = g_huber_delta_base +
                                   g_huber_delta_scale * p_body_d.norm();
        if (dist > huber_delta) w *= huber_delta / dist;
        H_f += J * w * J.transpose();
        r_f -= J * w * e;
        ++n_used;
      }
      if (n_used >= 5) {
        Eigen::SelfAdjointEigenSolver<M6d> es0(sum_HTVH);
        Eigen::SelfAdjointEigenSolver<M6d> es1(sum_HTVH + H_f);
        if (es0.info() == Eigen::Success && es1.info() == Eigen::Success) {
          const double lmin0 = std::max(es0.eigenvalues().minCoeff(), 1e-12);
          fine_gain_ = es1.eigenvalues().minCoeff() / lmin0;
          if (fine_gain_ >= g_fine_channel_min_gain) {
            sum_HTVH += H_f;
            sum_HTVr += r_f;
            fine_applied_ = true;
            fine_n_used_  = n_used;
          }
        }
      }
    }

    // 退化"响应"之一：截断。用**上一帧去抖后**的退化状态驱动，不用本帧原始 cond，
    // 否则帧级抖动会让约束每两帧开一次关一次（实测 116 次翻转）。
    if (g_degeneracy_truncate_en && degeneracy_active_) {
      DegeneracySig d = computeDegeneracy(sum_HTVH, g_degeneracy_threshold,
                                          g_degen_trans_only);
      if (d.ok) {
        truncateDegenerate(sum_HTVH, sum_HTVr, d,
                           g_tikhonov_lambda, sum_HTVH, sum_HTVr);
      }
    }

    HTVH = sum_HTVH.cast<scalar>();
    HTVr = sum_HTVr.cast<scalar>();

    iter_num++;
  });

  // ===== 退化检测 + 去抖状态机（每帧一次，用最后迭代的原始 Hessian）=====
  // 输出的 degeneracy_active_ 供下一帧的截断 / 参数自适应使用。
  // 滞回（进 cond>thr，出 cond<thr·hysteresis）+ 最短持续帧数，避免帧级抖动。
  if (debug || g_degeneracy_detect_en || g_adaptive_param_en || g_degeneracy_truncate_en ||
      g_fine_channel_en || g_auto_scene_en) {
    DegeneracySig deg = computeDegeneracy(last_HTVH, g_degeneracy_threshold,
                                         g_degen_trans_only);
    degeneracy_cond_ = deg.ok ? deg.cond : 0.0;
    degeneracy_axes_ = deg.ok ? deg.axes : 0;

    const double hi = g_degeneracy_threshold;
    const double lo = g_degeneracy_threshold * std::max(0.05, g_degeneracy_hysteresis);
    if (!degeneracy_active_) {
      if (deg.ok && deg.cond > hi) {
        if (++degen_state_cnt_ >= std::max(1, g_degeneracy_enter_frames)) {
          degeneracy_active_ = true;
          degen_state_cnt_ = 0;
        }
      } else {
        degen_state_cnt_ = 0;
      }
    } else {
      if (deg.ok && deg.cond < lo) {
        if (++degen_state_cnt_ >= std::max(1, g_degeneracy_exit_frames)) {
          degeneracy_active_ = false;
          degen_state_cnt_ = 0;
        }
      } else {
        degen_state_cnt_ = 0;
      }
    }

    // ===== M.31 场景自动档：启动窗口内累计高退化帧占比，满窗判定一次 =====
    // 依据 M.30 §5：楼梯的 cond 是 ~40 s 周期振荡且全程不超过 223，1f 走廊
    // 持续高位 ⇒ "窗口内高退化帧占比"能把"偶发退化"与"常态退化"分开，
    // 实测 W=120 s/thr=150/gate=0.5 时楼梯误判 0、1f 覆盖 94.9%。
    // 判定只做一次并固定到全程：A 线是刀刃平衡，逐帧抖动风险远大于收益。
    if (g_auto_scene_en && !scene_decided_) {
      const double t = kf_->GetSysState().timestamp;
      if (scene_t0_ < 0.0) scene_t0_ = t;
      ++scene_n_;
      if (degeneracy_cond_ > g_auto_scene_cond_thr) ++scene_n_hi_;
      if (scene_n_ > 0 && (t - scene_t0_) >= g_auto_scene_window) {
        const double hi_frac = double(scene_n_hi_) / double(scene_n_);
        scene_corridor_ = hi_frac > g_auto_scene_gate;
        scene_decided_  = true;
        kf_->SetAcceVar(scene_corridor_ ? g_auto_scene_corridor_na
                                       : g_auto_scene_stair_na);
        std::cout << "[AutoScene] decided=" << (scene_corridor_ ? "corridor(B)" : "stair(A)")
                  << " hi_frac=" << hi_frac
                  << " (thr=" << g_auto_scene_cond_thr
                  << " gate=" << g_auto_scene_gate
                  << " win=" << g_auto_scene_window << "s"
                  << " frames=" << scene_n_ << ")"
                  << " -> na=" << (scene_corridor_ ? g_auto_scene_corridor_na
                                                   : g_auto_scene_stair_na)
                  << " nw=" << (scene_corridor_ ? g_auto_scene_corridor_nw
                                                : g_auto_scene_stair_nw)
                  << std::endl;
      }
    }
  }

  if (g_degeneracy_detect_en) {
    static auto last_degen_log = std::chrono::steady_clock::now();
    auto now_degen = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now_degen - last_degen_log).count() > 2.0) {
      LOG(WARNING) << "[Degeneracy] cond=" << degeneracy_cond_
                   << " axes=" << degeneracy_axes_
                   << " active=" << (degeneracy_active_ ? 1 : 0)
                   << " effect_pts=" << n_effect;
      last_degen_log = now_degen;
    }
  }

  // 细通道运行状态（2 s 限频）：cands=本帧候选数 used=计入 H_f 的点数
  // gain=λ_min 增益（<min_gain 时 applied=0 即整帧弃用）
  if (g_fine_channel_en) {
    static auto last_fine_log = std::chrono::steady_clock::now();
    auto now_fine = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now_fine - last_fine_log).count() > 2.0) {
      LOG(WARNING) << "[FineCh] cands=" << fine_cands_.size()
                   << " used=" << fine_n_used_
                   << " gain=" << fine_gain_
                   << " applied=" << (fine_applied_ ? 1 : 0)
                   << " degen=" << (degeneracy_active_ ? 1 : 0)
                   << " cond=" << degeneracy_cond_;
      last_fine_log = now_fine;
    }
  }

  // ===== 埋点 CSV（lio.debug.level > 0）=====
  if (debug) {
    if (!debug_csv_.is_open()) {
      namespace fs = std::filesystem;
      const std::string dir = g_save_map_dir.empty() ? std::string(".") : g_save_map_dir;
      std::error_code ec;
      fs::create_directories(dir, ec);
      debug_csv_.open(dir + "/debug.csv", std::ios::out | std::ios::trunc);
      // 默认 6 位有效数字会把 1.79e9 量级的绝对时间戳量化到 1e4 s，
      // 逐帧 dt 完全丢失；这里提高到 12 位（时间戳分辨率 ~1e-3 s）。
      debug_csv_ << std::setprecision(12);
      debug_csv_ << "t,ptsize,n_knn_fail,n_plane_fail,n_res_fail,n_effect,"
                    "fit_res_mean,fit_res_p95,iters,dx_inf,cond_yk,qk_finite,"
                    "htvh_ev_min,htvh_ev_max,px,py,pz,vx,vy,vz,rvx,rvy,rvz,imu_scale,"
                    "rvx_pred,rvy_pred,rvz_pred,bgx,bgy,bgz,bax,bay,baz,gx,gy,gz,dt_imu_end,frame_dt,deg_cond,deg_axes,"
                    // M.30 场景判别：平移块最弱方向的三项对齐度（见下方计算处注释）
                    "deg_cond_t,deg_evt_min,deg_dir_v,deg_dir_g,deg_dir_z\n";
    }
    double fit_mean = 0.0, fit_p95 = 0.0;
    if (!fit_res_vec_.empty()) {
      double s = 0.0;
      for (scalar v : fit_res_vec_) s += v;
      fit_mean = s / static_cast<double>(fit_res_vec_.size());
      if (g_debug_level >= 2) {
        std::vector<scalar> tmp(fit_res_vec_);
        std::sort(tmp.begin(), tmp.end());
        fit_p95 = tmp[static_cast<size_t>(0.95 * (tmp.size() - 1))];
      }
    }
    double ev_min = 0.0, ev_max = 0.0;
    if (g_debug_level >= 2) {
      Eigen::SelfAdjointEigenSolver<M6d> es(last_HTVH);
      if (es.info() == Eigen::Success) {
        ev_min = es.eigenvalues().minCoeff();
        ev_max = es.eigenvalues().maxCoeff();
      }
    }
    const auto state = kf_->GetNavState();
    const V3 rv = state.R.log_vee();
    // 更新前的预测姿态（去畸变基准），与后验一起给出每帧观测修正量
    const V3 rv_pred = SO3(T_predicted_end_.R_).log_vee();
    const auto sys_state = kf_->GetSysState();

    // ===== M.30 场景判别埋点：雷达 Hessian 平移块的最弱方向 =====
    // 动机：deg_cond / deg_axes 只回答"退化有多严重"，不回答"退化是否落在正在
    // 运动的轴上"。后者才是 na（IMU 与雷达的相对权重）该不该降的直接依据：
    //   走廊：退化轴 = 沿走廊水平轴，飞机正沿它飞 → |u_min·v̂| → 1（应低 na）
    //   楼梯：退化轴随螺旋旋转、多为竖直 → |u_min·v̂| 低（可高 na）
    // 平移块 H_tt 位于世界系（J 的平移行即世界系法向 u），故 u_min 可直接与
    // 世界系速度、重力方向比较。这三项均为纯雷达量、不含先验，无反馈回路。
    double deg_cond_t = 0.0, deg_evt_min = 0.0;
    double deg_dir_v = -1.0, deg_dir_g = -1.0, deg_dir_z = -1.0;
    {
      const Eigen::Matrix3d Htt = last_HTVH.block<3, 3>(3, 3);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> est(Htt);
      if (est.info() == Eigen::Success) {
        const Eigen::Vector3d evt = est.eigenvalues();
        deg_evt_min = evt(0);
        deg_cond_t = (evt(0) > 1e-12) ? evt(2) / evt(0) : 1e12;
        const Eigen::Vector3d umin = est.eigenvectors().col(0);
        const Eigen::Vector3d vel = state.v.cast<double>();
        const double vn = vel.norm();
        // 低速时速度方向无意义，记 -1，由分析侧剔除
        if (vn > 0.1) deg_dir_v = std::abs(umin.dot(vel / vn));
        const Eigen::Vector3d gv = kf_->GetGravity().cast<double>();
        if (gv.norm() > 1e-3) {
          deg_dir_g = std::abs(umin.dot(gv / gv.norm()));
          deg_dir_z = std::abs(umin.z());
        }
      }
    }

    debug_csv_ << state.timestamp << ',' << ptsize << ','
               << n_knn_fail << ',' << n_plane_fail << ',' << n_res_fail << ',' << n_effect << ','
               << fit_mean << ',' << fit_p95 << ','
               << kf_->GetLastIters() << ',' << kf_->GetLastDxInf() << ','
               << kf_->GetLastCondYk() << ',' << (kf_->GetLastQkFinite() ? 1 : 0) << ','
               << ev_min << ',' << ev_max << ','
               << state.p.x() << ',' << state.p.y() << ',' << state.p.z() << ','
               << state.v.x() << ',' << state.v.y() << ',' << state.v.z() << ','
               << rv.x() << ',' << rv.y() << ',' << rv.z() << ','
               << kf_->GetImuScale() << ','
               << rv_pred.x() << ',' << rv_pred.y() << ',' << rv_pred.z() << ','
               << sys_state.bg.x() << ',' << sys_state.bg.y() << ',' << sys_state.bg.z() << ','
               // 加计零偏：若姿态倾斜被 ba 吸收，这里会看到同形的地平分量
               << sys_state.ba.x() << ',' << sys_state.ba.y() << ',' << sys_state.ba.z() << ','
               // 重力状态：世界系固定于首帧，g 若转动即"地图竖直"在歪
               << kf_->GetGravity().x() << ',' << kf_->GetGravity().y() << ','
               << kf_->GetGravity().z() << ','
               // 预测姿态只推进到帧内最后一个 IMU 采样，与 end_time 的差就是
               // "观测修正 ∝ 角速度" 的候选来源，逐帧记下来可直接回归验证
               << (measures_.imu.empty() ? 0.0
                     : measures_.lidar.end_time - measures_.imu.back().secs) << ','
               << (measures_.lidar.end_time - measures_.lidar.start_time) << ','
               << degeneracy_cond_ << ',' << degeneracy_axes_ << ','
               << deg_cond_t << ',' << deg_evt_min << ','
               << deg_dir_v << ',' << deg_dir_g << ',' << deg_dir_z << '\n';
    debug_csv_.flush();
  }

  frame_num_++;
}


/// 更新 IVox 地图：将去畸变后的点云添加到地图中
/// 分段计时：g_time_eva 时分别统计世界系变换 [MapTransform] 与地图插入 [MapInsert]。
/// 实测 [UpdateMap] 峰值 ~17.8 ms，是整条链路最大的单帧尖峰，需要定位是变换还是插入。
void SuperLIO::UpdateMap() {
  // 实验 C：冻结地图更新，配准只用载入的先验地图（用于隔离地图反馈）
  if (g_map_freeze) return;

  const size_t ptsize = ds_undistort_->size();
  if (ptsize == 0) return;
  
  last_pose_ = kf_->GetSE3();
  points_world_v3_.resize(ptsize);
  
  const auto R = last_pose_.R_;
  const auto t = last_pose_.t_;

  auto transform_body_to_world = [&]() {
    for (size_t i = 0; i < ptsize; ++i) {
      const auto& pt = points_body_v3_[i];
      points_world_v3_[i] = R * pt + t;
    }
  };
  auto insert_into_map = [this]() { ivox_->insert(points_world_v3_); };

  if (g_time_eva) {
    time_record_.Evaluate(transform_body_to_world, "[MapTransform]");
    time_record_.Evaluate(insert_into_map,       "[MapInsert]");
  } else {
    transform_body_to_world();
    insert_into_map();
  }
}


/// 输出当前帧结果：body 系点云 + 世界系点云 + odom（统一使用 state.timestamp）
void SuperLIO::Output(){
  auto state = kf_->GetNavState();

  OutputData output_data;
  output_data.state = state;
  output_data.body_omega = kf_->GetDynamicState().w;
  output_data.lidar_receive_time = measures_.lidar.receive_time;
  output_data.is_undistort_only = g_lio_only_undistort || g_downsample_only;
  output_data.lidar_frame = current_lidar_frame_;

  // body cloud: published with state.timestamp (same as odom) for PGO sync
  if (g_visual_map_body) {
    static int count_body = -1;
    count_body++;
    if (count_body % g_pub_step == 0) {
      count_body = 0;
      output_data.body_pc.reset(new PointCloudType());
      *output_data.body_pc = *scan_undistort_full_;
      output_data.has_body_pc = true;
    }
  }

  // downsample_only mode: output without transformation
  if(g_downsample_only){
    output_data.state.timestamp = measures_.lidar.start_time;
    if(g_visual_map){
      static int count = -1;
      count++;
      if(count % g_pub_step == 0){
        count = 0;
        output_data.world_pc.reset(new PointCloudType());
        *output_data.world_pc = *scan_undistort_full_;
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
        *output_data.world_pc = *scan_undistort_full_;
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
        transformPointCloudRt(*scan_undistort_full_, *output_data.world_pc, Rf, tf);
        output_data.has_world_pc = true;
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

    // odom is always published with state.timestamp (same as body/world cloud)
    if (!data.is_undistort_only) {
      data_wrapper_->pub_odom(data.state, data.body_omega);
      // 端到端延迟：雷达接收 → odom 落盘。odom 始终发布，因此该指标在任何配置下可比；
      // 点云话题默认关闭（lio.output.map=false）时，用它作为管线延迟的等价口径。
      if (data.lidar_receive_time > 0.0) {
        auto now_s = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        data_wrapper_->recordLatency("[Lidar->Odom]", (now_s - data.lidar_receive_time) * 1000.0);
      }
    }

    if (data.has_body_pc && data.body_pc) {
      data_wrapper_->pub_cloud_body(data.body_pc, data.state.timestamp);
      if(data.lidar_receive_time > 0.0){
        auto now_s = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        double lat_ms = (now_s - data.lidar_receive_time) * 1000.0;
        data_wrapper_->recordLatency("[Lidar->BodyCloud]", lat_ms);
      }
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


void SuperLIO::periodicTimeReport(){
  if(!g_time_eva) return;
  time_record_.PrintAndClear();
  if (ivox_) {
    // M.29：把插入统计与表规模打到同一个计时窗口，便于判断尖峰是否与扩容/淘汰同窗
    auto s = ivox_->takeStats();
    std::cout << " > [ MapStats ] size=" << ivox_->mapSize()
              << " buckets=" << ivox_->bucketCount()
              << " new_voxel=" << s.n_new
              << " evict=" << s.n_evict << std::endl;
  }
  data_wrapper_->printLatencies();
  data_wrapper_->clearLatencies();
}


void SuperLIO::pauseProcessing(){
  paused_.store(true);

  // 停住 dense 链路（静默屏障）：等待队列排空且当前帧处理完毕，此后不再有后台线程写
  // point_map_，使随后的 saveMap() 可安全读/清地图点。等待期间不置 dense_paused_，
  // 以便 DenseThread 继续消费直至排空；主流程已由 paused_ 停住，不会再有新帧入队。
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  {
    std::unique_lock<std::mutex> lock(dense_mutex_);
    while((!dense_queue_.empty() || dense_busy_) &&
          std::chrono::steady_clock::now() < deadline){
      dense_cv_.wait_for(lock, std::chrono::milliseconds(5));
    }
    dense_paused_ = true;   // 排空后置停：暂停期间 DenseThread 不再消费
  }
  dense_cv_.notify_all();

  LOG(INFO) << YELLOW << " ---> [SuperLIO]: Processing paused" << RESET;
}


void SuperLIO::resumeProcessing(){
  {
    std::lock_guard<std::mutex> lock(dense_mutex_);
    dense_paused_ = false;
  }
  dense_cv_.notify_all();
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
    // 单一收敛点：所有走到这里的重置都会让 odom 序列出现不连续，
    // 必须让下游（PX4 EKF2）知道，否则它只能靠创新门限反复拒绝 -> reset 风暴
    if(data_wrapper_){ data_wrapper_->notifyOdomReset("IMU pre-integration reset"); }
  }
}

} // namespace END.