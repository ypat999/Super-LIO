
#include "lio/super_lio.h"

#include <sys/resource.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>

#include <Eigen/Eigenvalues>
#include <omp.h>
// GICP 采用用户仓库 ypat999/ndt_omp_ros2 的 pclomp 实现（header-only，已 vendor 到
// include/pclomp/）。相比 PCL 自带 GICP：内部固定点迭代 + BFGS，且显式实例化了
// PointXYZI，正好用于当前帧去畸变云对冻结 Anchor 的配准。
#include "pclomp/gicp_omp.h"
#include "pclomp/gicp_omp_impl.hpp"


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

/// 目标点云某点的法向（邻域 PCA 最小特征向量）。
/// 只对 GICP 内点按需计算，用于构建点到面 Hessian 检查六自由度可观测性。
inline bool estimatePointNormal(const BASIC::CloudPtr& cloud,
                                pcl::KdTreeFLANN<BASIC::PointType>& kdtree,
                                int idx, int k, BASIC::V3& n) {
  std::vector<int> ids;
  std::vector<float> d2;
  if (idx < 0 || idx >= static_cast<int>(cloud->size())) return false;
  if (kdtree.nearestKSearch(idx, k, ids, d2) < 3) return false;
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (int id : ids) mean += cloud->points[id].getVector3fMap().cast<double>();
  mean /= static_cast<double>(ids.size());
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (int id : ids) {
    const Eigen::Vector3d d = cloud->points[id].getVector3fMap().cast<double>() - mean;
    cov += d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
  n = es.eigenvectors().col(0).cast<BASIC::scalar>();
  return true;
}

/// 实验：观测噪声模型 σ²(r)=dept²+(beam·r)² 的标定距离（m）。
/// 在该距离上有效噪声等于 g_obs_weight 对应的基准噪声，即权重在此处不放大不衰减。
constexpr double kBeamRefDist = 5.0;

/// 清图重锚后地图新点比例门的豁免时长（s）：此时地图为空，指标恒接近 1。
constexpr double kNoveltyGraceS = 10.0;

/// 新点比例抑制被放弃后的冷却时长（s）：避免"放弃 → 下一帧立刻再抑制"的抖动。
constexpr double kNoveltyCooldownS = 5.0;

/// "保图放行"的短时重复判据：window 内达到 max 次即判为持续发散，升级硬重锚。
/// 正常新覆盖只会偶发放行一次；错误匹配则会按"放行-冷却-再命中"的节奏反复出现。
constexpr double kNoveltyReleaseWinS = 20.0;
constexpr int    kNoveltyReleaseMax  = 2;

/// 清图重锚允许使用种子位姿的最大龄期（s）。接管/交还后种子可能已陈旧几十秒，
/// 此时以旧种子重锚等于把位姿弹回几十秒前的位置（gicp_ad 实测 21:28:48 用
/// 79 s 前的种子，重锚后 1.5 s 速度即被泵到 5 m/s，最终漂到 53 m）。
/// 超过该龄期改用冻结时刻的可信位姿。
constexpr double kNoveltySeedMaxAgeS = 5.0;

/// 交还 Voxmap 时的地图覆盖门（新点比例上限）。
/// 新点比例是"当前扫描有多少点落进空白粗体素"的直接度量，0.15 以内说明扫描
/// 确实落在已建图区域。gicp_ac 19:53:11 与 gicp_ag 00:34:47 两次错误交还的
/// novelty 分别为 0.17~0.21 与 0.35~0.40，交还后 2~3 s 位姿即被拖走 8~12 m
/// （cond_yk 1e3~1e19：观测"自信但错误"），是两次发散的共同起点。
constexpr double kHandbackNoveltyMax = 0.15;

/// 交还 Voxmap 后的观察期时长（s）。交还门只看"交还瞬间自洽"，挡不住
/// "地图段被早先漂移污染、但局部自洽"的情形：gicp_ah 47:35 交还后 2 s 状态速度
/// 即 5.1 m/s，4 s 沿 -y 拖走 9 m。观察期内一旦速度与偏离同时抬头就撤销交还，
/// 把偏差限制在 GICP 仍能重新锁定的量级（实测 >10 m 后 inlier 掉到 0.34）。
constexpr double kHandbackProbationS = 4.0;

/// 观察期内撤销交还的速度门（m/s）：本 bag 实飞 ≤1.3 m/s，取 2.0 留余量。
constexpr double kHandbackProbationVelMax = 2.0;

/// 观察期内撤销交还的最小偏离（m）：滤掉单帧速度尖峰，避免把正常机动误判为拖拽。
constexpr double kHandbackProbationDevMin = 0.30;

/// 新点比例判据成立时，若地图仍有足够有效点（说明位姿在"偏离已有地图"，
/// 而不是单纯飞进未建图区域），直接以当前 IMU 位姿为基准清图重建的最小
/// 有效点数门槛。低于该值视为正常的新覆盖区域，不触发重置。
constexpr int kNoveltyResetMinEffectPts = 100;

/// 新点比例抑制期的"绝对一致性"放行门：用冻结 Anchor 的 GICP 绝对位姿与
/// Voxmap 结果比较。两者只差几厘米/几度 ⇒ Voxmap 并没有把位姿拖走，
/// 新点比例高只是因为飞进了尚未建图的区域（home3 115 s 实测 new=0.41、
/// GICP 修正仅 0.085 m/1.18°，旧逻辑因 inlier 0.227<0.5 一律判拒，
/// 3 s 后清图重锚，连续重锚 48 次）。
/// 与"保图放行"不同：该路径的 GICP 是独立绝对观测（目标图来自冻结 Anchor），
/// 残差随累计拖拽线性增长，因此能同时挡住"局部自洽的错误匹配"。
constexpr double kNoveltyVerifyMaxDpos = 0.30;    // m
constexpr double kNoveltyVerifyMaxDrot = 5.0;     // deg
constexpr int    kNoveltyVerifyMinInliers = 50;   // 低于此数不采信 GICP 绝对位姿

/// 平移锯齿早检：单帧位移隐含速度门（m/s）。本条 bag 实飞 ≤1.3 m/s，
/// 取 2.0 留 1.5 倍余量；正常转弯/起降的单帧尖峰偶尔会越门，靠"计数"滤掉。
constexpr double kSawImpliedSpeed = 2.0;

/// 最近 SuperLIO::kSawWin 帧里至少这么多帧超过速度门才判"平移锯齿"成立。
constexpr int kSawMinHits = 3;

/// 稠密云体素降采样 + 点数封顶。
/// 全量去畸变云（数万点）直接喂 GICP 会退化成秒级；这里按 leaf0 起滤，点数超过
/// max_pts 就逐步放大体素重滤，既保留比抽稀云稠密得多的几何，又守住单帧耗时。
inline void voxelDownsampleCapped(const BASIC::CloudPtr& in, BASIC::CloudPtr& out,
                                  double leaf0, int max_pts) {
  if (!out) out.reset(new PointCloudType());
  out->clear();
  if (!in || in->empty()) return;
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  double leaf = std::max(1e-3, leaf0);
  const int max_iter = 8;
  for (int k = 0; k < max_iter; ++k) {
    vg.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf),
                   static_cast<float>(leaf));
    vg.setInputCloud(in);
    out->clear();
    vg.filter(*out);
    if (max_pts <= 0 || static_cast<int>(out->size()) <= max_pts) return;
    leaf *= 1.25;
  }
}

} // anonymous namespace

namespace LI2Sup{

/// 平面拟合：用 N 个点拟合平面 ax+by+cz+1=0，返回法向量系数 abcd
/// 若给出 max_abs_dist，则输出参与拟合点中的最大 |dist|（作为平面质量指标）
/// 若给出 resid_var，则输出平面残差方差 Σd²/N（m²），供观测权重做软降权
inline bool calc_plane_coeff(const int N, const std::array<V3, 8>& points, std::array<double, 4>& abcd,
                             double* max_abs_dist = nullptr, double* resid_var = nullptr)
{
  // 固定 8×3 缓冲 + topLeftCorner(N)，避免按 N 分两套矩阵（K=5→8 后 N 取值 4..8）。
  Eigen::Matrix<double, 8, 3> A = Eigen::Matrix<double, 8, 3>::Zero();
  Eigen::Matrix<double, 8, 1> b = Eigen::Matrix<double, 8, 1>::Zero();
  for (int j = 0; j < N; j++) {
    A.row(j) = points[j].cast<double>();
    b(j) = -1.0;
  }
  Eigen::Vector3d normvec = A.topLeftCorner(N, 3).colPivHouseholderQr().solve(b.head(N));

  double n = normvec.norm();
  if (n < 1e-6f) return false;

  abcd[3] = 1.0 / n;
  normvec *= abcd[3];
  abcd[0] = normvec[0];
  abcd[1] = normvec[1];
  abcd[2] = normvec[2];

  double sum2 = 0.0;
  for (int i = 0; i < N; ++i) {
    const V3& p = points[i];
    auto dist = abcd[0] * p(0) + abcd[1] * p(1) + abcd[2] * p(2) + abcd[3];
    if (max_abs_dist) *max_abs_dist = std::max(*max_abs_dist, std::abs(dist));
    sum2 += static_cast<double>(dist) * static_cast<double>(dist);
    if (std::abs(dist) > g_plane_fit_threshold) return false;
  }
  if (resid_var) *resid_var = (N > 0) ? sum2 / static_cast<double>(N) : 0.0;
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
  resid_var_vec_.resize(kMaxEffectPoints);
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

  // dynamic_start=true：认定开头这段窗口不静止，跳过静态零偏标定 —— 不把
  // mean_gyro 当 bg0（取 0，陀螺零偏交给 ESKF 在线估）。
  // mean_acce 仍用于重力方向对齐与 imu_scale 归一：没有更好的来源，且去掉
  // imu_scale 会让 g 单位的比力直接进积分，后果比零偏污染更严重。
  const V3 bg0 = g_kf_dynamic_start ? V3::Zero() : mean_gyro;
  LOG(INFO) << GREEN << "      Dynamic Start: "
            << (g_kf_dynamic_start ? "true" : "false")
            << "  -> bg0 = [" << bg0.transpose() << "] rad/s" << RESET;

  kf_->SetInitialConditions(options, bg0, V3::Zero(), imu_scale, ref_gravity);
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
    last_ok_frame_wall_ = std::chrono::steady_clock::now();   // 看门狗心跳
    return;
  }

  // 本帧与上一帧的雷达 end_time 间隔：护栏、断档告警、信任域共用。
  // 必须在 Propagation_Undistort 之前算（它依赖上一帧的 end_time）。
  cur_lidar_gap_ = (last_lidar_end_time_ < 0.0)
      ? 0.0 : (measures_.lidar.end_time - last_lidar_end_time_);

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
    const double gap = cur_lidar_gap_;
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

  // 真实链路中断的指纹（正常丢一帧不会超过阈值），限频 5s 一条
  if(cur_lidar_gap_ > g_frame_gap_max){
    const auto now = std::chrono::steady_clock::now();
    if(std::chrono::duration<double>(now - last_gap_log_wall_).count() > 5.0){
      last_gap_log_wall_ = now;
      LOG(WARNING) << "[Gap] 雷达帧间隔 " << cur_lidar_gap_ * 1000.0 << " ms（> "
                   << g_frame_gap_max * 1000.0 << " ms），链路疑似中断，断档段不做积分";
    }
  }

  // bag 相对时间基准：无条件在首帧建立，供 B（SUSPECT 武装时间）与冻结实验共用。
  // 注意不能只在 g_map_freeze_after_s>0 时初始化 —— 否则 B 的 bag_rel_t 恒为 0，
  // 武装门永远打不开（gicp_f 实测：净速率超阈值但一直打印"bag 相对时间 0 s"）。
  if(first_lidar_end_time_ < 0.0) first_lidar_end_time_ = measures_.lidar.end_time;

  // 建图写图门：LOST 期间不写（除非 g_lost_map_en）；SUSPECT 帧不写；
  // 冻结实验：bag 相对时间超过 g_map_freeze_after_s 后只读旧地图、停止写图。
  // 必须等 checkPoseTrust 之后再计算 —— 它可能刚刚退出 LOST（超时放行/重锚），
  // 否则放行帧仍不写图、Anchor 又被冻结，新地图段起不来。
  map_pause_frame_ = false;   // 本帧默认写图，checkPoseTrust 可置位
  bool map_write = false;

  if(frame_trusted){
    // 保存 IMU 先验：信任域否决时据此回滚，撤销本次激光修正
    const SysState      sys_pred = kf_->GetSysState();
    const ESKF::COV     cov_pred = kf_->GetCov();

    if(g_time_eva){
      time_record_.Evaluate([this]() { Observe(); },             "[Observe]");
    }else{
      Observe();
    }

    checkPoseTrust(sys_pred, cov_pred);

    // SUSPECT 帧的决定在本帧内生效：checkPoseTrust 里已置 map_pause_frame_
    map_write = (!lost_ || g_lost_map_en) && !map_pause_frame_;
    if(map_write && g_map_freeze_after_s > 0.0){
      map_write = (measures_.lidar.end_time - first_lidar_end_time_) <= g_map_freeze_after_s;
    }

    // 可信 Anchor：一旦出现异常帧，立即作废未提交窗口并冻结，保证 Anchor 干净。
    // GICP 接管期间除外：接管帧失败只是"这一帧没配准上"，属正常抖动，
    // 若按 SUSPECT 冻结 Anchor，则接管期第一次失败就把唯一的绝对参考永久
    // 锁死（gicp_v 实测 275.5 s 首次接受后 0.08 s 的一帧失败即冻结，之后
    // committed 不再增长、目标图停止刷新，20 s 后 inlier 衰减退出接管，
    // 再过 40 s committed 被窗口裁空，彻底失去重定位能力）。
    if((map_pause_frame_ || lost_) && !gicp_active_){
      freezeTrustedAnchor(map_pause_frame_ ? "suspect" : "lost");
    }

    // 外部传感器约束注入（默认空实现，未 override 时零影响）
    ExternalObserve();

    if(g_time_eva){
      time_record_.Evaluate([this]() { ApplyDeltaCorrection(); },"[DeltaCorrect]");
      if(map_write){
        time_record_.Evaluate([this]() { UpdateMap(true); },         "[UpdateMap]");
      }else{
        UpdateMap(false);   // 仍统计地图新点比例（漂移早期指标），只是不写图
      }
    }else{
      ApplyDeltaCorrection();
      UpdateMap(map_write);
    }
    updateTrustedAnchor(map_write);
    Output();                // world cloud via queue (high precision)
    enqueueDense();          // 全量点异步去畸变/变换（错峰，dense 话题 + pcd 来源）
    last_ok_frame_wall_ = std::chrono::steady_clock::now();   // 看门狗心跳
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
  data.lidar_end_time = measures_.lidar.end_time;
  data.receive_time = measures_.lidar.receive_time;
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

    // 1b. 顺带产出可信 Anchor 的稠密机体系帧（与旧 gicp_scan_dense_ 语义一致：
    //     预测位姿去畸变、未做 delta 修正），交给主线程在安全点收编。
    //     这样主线程不必每帧再对 pc_full 重复做一次全量去畸变。
    if(g_anchor_delay_commit_s > 0.0 && g_anchor_gicp_en && !dense_body->empty()){
      auto ta = std::chrono::high_resolution_clock::now();
      AnchorFrame af;
      af.t = data.lidar_end_time;
      af.pose = data.T_corrected;
      voxelDownsampleCapped(dense_body, af.body, g_gicp_target_voxel,
                            g_gicp_anchor_body_max_pts);
      std::lock_guard<std::mutex> lock(anchor_inbox_mutex_);
      anchor_dense_inbox_.push_back(std::move(af));   // 主线程按帧序消费
      if(anchor_dense_inbox_.size() > 8) anchor_dense_inbox_.pop_front();
      if(data_wrapper_){
        const double ms = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - ta).count();
        data_wrapper_->recordLatency("[Dense]AnchorBody", ms);
      }
    }

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
    // 单体化钩子：在话题发布前把同一份全量点云推给派生类（默认空实现，不影响原行为）
    onDenseCloud(dense_body, data.timestamp, data.receive_time);
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
      // 有界等待：SaveThread 若卡在磁盘/IO 上，这里绝不能无限等 —— 调用方
      // (/map_save) 持有 paused_=true，无限等会直接锁死整条实时链路。
      std::unique_lock<std::mutex> lock(save_mutex_);
      const bool drained = save_cv_.wait_for(lock, std::chrono::seconds(60), [this]{
        return save_queue_.empty();
      });
      if(!drained){
        LOG(WARNING) << YELLOW << " ---> [map_save] 等待 PCD 落盘超时 60s，剩余 "
                     << save_queue_.size() << " 个片段（继续后续流程）" << RESET;
      }
    }
    
    LOG(INFO) << GREEN << " ---> Save last cace success. " << RESET;

    // Step 1: Run dynamic point removal on scans_*.pcd -> filtered_scans_*.pcd
    bool dyn_ok = true;
    if(g_dynamic_removal_enable){
      LOG(INFO) << YELLOW << " ---> Running dynamic point removal (per-frame mode) ... " << RESET;
      std::string save_map_dir = g_save_map_dir;
      if (!save_map_dir.empty() && save_map_dir[0] != '/') {
        save_map_dir = g_root_dir + save_map_dir;
      }
      std::string pcd_folder = save_map_dir + "/PCD";
      
      std::stringstream cmd;
      // timeout 兜底：dynamic_remove_node 是独立进程，异常时可能永不退出；
      // 而它是在 /map_save 回调里同步执行的（此时 paused_=true），卡住就等于
      // 整条实时链路被永久冻结，因此必须给进程级超时。
      cmd << "timeout 600 taskset -c 0,1,2,3,4,5,6 ros2 run super_lio dynamic_remove_node"
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
        dyn_ok = false;
        LOG(WARNING) << RED << " ---> Dynamic point removal failed with code: " << ret
                     << "（超时 600s 被杀时退出码为 124），回退为直接合并原始扫描" << RESET;
      }
    }

    // Step 2: Merge scans into final map
    if(g_dynamic_removal_enable && dyn_ok){
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
      // 动态剔除关闭或失败：直接合并原始 scans_*.pcd -> test.pcd
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
    // 断档冻结：dt 超过上限只对齐时间基准、不积分，避免把断档期的冲击加速度外推成真实运动
    kf_->Predict(imu, g_max_imu_dt);
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

  frame_imu_gyr_ = imu_gyr;

  // 稠密去畸变云不再在主线程每帧预生成：DenseThread 本来就要对同一份 pc_full
  // 做全量去畸变，Anchor 帧直接复用其结果（见 DenseThread → anchor_dense_inbox_）；
  // GICP 真正要配准时再用 ensureGicpDense() 现场补算（仅异常/接管帧，开销可忽略）。
  gicp_dense_valid_ = false;
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

/// 按需生成当前帧的全量稠密去畸变云（GICP 源云用）。
/// 只在确实要跑 GICP 配准（异常判定/接管期）时调用，主线程不再每帧预生成。
void SuperLIO::ensureGicpDense(){
  if(gicp_dense_valid_) return;
  if(!g_anchor_gicp_en || !g_gicp_dense_en) return;
  if(!measures_.lidar.pc_full || measures_.lidar.pc_full->empty()) return;
  if(propagate_states_.size() < 2) return;
  if(!gicp_scan_dense_) gicp_scan_dense_.reset(new PointCloudType());
  undistortCloud(measures_.lidar.pc_full, propagate_states_, frame_imu_gyr_,
                 measures_.lidar.start_time, T_predicted_end_, gicp_scan_dense_, nullptr);
  gicp_dense_valid_ = !gicp_scan_dense_->empty();
}


/// 位姿信任域：Observe() 之后的裁决。
/// 判据是「相对最近可信位姿的单帧跳变」——正常帧只有帧间运动（10Hz、2m/s 约 0.2m），
/// 一旦跳变超阈值，说明本次配准要么被断档后的坏先验带偏，要么撞上了错误极小值。
/// 处置分两种：
///  - 正常态跳变超阈值：撤销本次激光修正（回滚到 IMU 先验），进入 LOST；
///  - LOST：继续影子配准，每帧把位姿往配准结果限速拉一小步（slew），
///    直到修正量连续 N 帧都很小才重新接受观测。
/// LOST 期间只暂停 IVox 建图，odom/TF/点云仍按当前位姿全量输出（空中以飞行为先）。
void SuperLIO::checkPoseTrust(const SysState& sys_pred, const ESKF::COV& cov_pred){
  if(!g_trust_guard_en){ return; }

  constexpr double kRad2Deg = 57.29577951308232;
  BASIC::SE3 T_post = kf_->GetSE3();
  const BASIC::SE3& T_pred = T_predicted_end_;   // 本帧 IMU 先验

  double corr_p = (T_post.t_ - T_pred.t_).norm();
  double corr_r = BASIC::SO3(T_post.R_.transpose() * T_pred.R_).log_vee().norm() * kRad2Deg;

  // 拒绝激光修正时同时抑制速度。发散段的 IMU 速度是被错误姿态外推出来的
  // （姿态拖拽 → 重力错位 → ~2 m/s² 虚假水平加速度），实测 272 s 后升到
  // 19.6 m/s。若原样回滚，下一帧 Propagation_Undistort 会按这个速度把整帧
  // 点云沿运动方向剪切 2 m（帧内 skew），GICP 对应关系彻底失效，形成
  // "速度飞→点云变形→GICP 配不准→速度更飞"的正反馈。此处阻尼并封顶。
  constexpr double kRejectVelDamp = 0.5;
  constexpr double kRejectVelMax  = 3.0;   // m/s：drone 真实速度 < 1.5 m/s
  auto rollback_damped = [&](const SysState& base, const ESKF::COV& cov){
    SysState s = base;
    s.v *= kRejectVelDamp;
    const double vn = s.v.norm();
    if (vn > kRejectVelMax) s.v *= kRejectVelMax / vn;
    kf_->RollbackToState(s, cov);
  };
  // 清图重锚时用：位姿保留 IMU 先验，速度直接清零。实测（gicp_s）只做
  // ×0.5 + 3 m/s 封顶时，错误姿态造成的 ~2 m/s² 虚假加速度会在 1 s 内
  // 把速度重新泵到 5.2 m/s，随即撞速度硬门进 LOST。清零后由 IMU 重新
  // 观测真实运动（<1.5 m/s），不会再被旧速度外推拖出帧内剪切。
  auto rollback_zero_vel = [&](const SysState& base, const ESKF::COV& cov){
    SysState s = base;
    s.v.setZero();
    kf_->RollbackToState(s, cov);
  };

  // ===== 交还 Voxmap 后的观察期 =====
  // 交还门（corr≈0、与上次可信 GICP 一致、novelty≤0.15）只证明"交还瞬间两者自洽"，
  // 不证明地图段没被更早的漂移污染：污染段是局部自洽的，交还后观测会把位姿沿
  // 固定方向持续拖拽（gicp_ah 47:35 交还时 corr_p=0.027 m 全过门，2 s 后状态速度
  // 5.1 m/s，4 s 沿 -y 拖走 9 m、novelty 0.78；等 GICP 再看到时 dpos 已 11.4 m、
  // inlier 0.34，回正解全部被拒）。因此在观察期内一旦"速度抬头 + 已有偏离"，立即
  // 撤销交还：位姿回滚到交还瞬间，速度清零，重新交给 GICP 接管。
  if (handback_probation_until_t_ > 0.0) {
    if (measures_.lidar.end_time > handback_probation_until_t_ || !handback_ref_valid_) {
      handback_probation_until_t_ = -1.0;
      handback_ref_valid_ = false;
    } else {
      const double dev_p = (T_post.t_ - handback_ref_pose_.t_).norm();
      const double vn = sys_pred.v.norm();
      if (vn > kHandbackProbationVelMax && dev_p > kHandbackProbationDevMin) {
        SysState s_ref = kf_->GetSysState();
        s_ref.R = handback_ref_pose_.R_;
        s_ref.p = handback_ref_pose_.t_;
        s_ref.v.setZero();
        kf_->RollbackToState(s_ref, cov_pred);
        gicp_active_ = true;
        gicp_ok_streak_ = 0;
        gicp_fail_streak_ = 0;
        gicp_handback_streak_ = 0;
        gicp_frame_counter_ = 0;
        gicp_pos_hist_.clear();
        gicp_last_pose_valid_ = false;
        gicp_last_raw_valid_ = false;
        gicp_hold_cycles_ = 0;
        gicp_target_ready_ = false;   // 用最新 Anchor 重建目标图
        gicp_target_.reset();
        gicp_target_kdtree_.reset();
        gicp_seed_pose_ = handback_ref_pose_;
        gicp_seed_t_ = measures_.lidar.end_time;
        gicp_seed_state_ = s_ref;
        gicp_seed_valid_ = true;
        novelty_hold_ = false;
        lost_ = false;
        stable_pose_ = kf_->GetSE3();
        stable_pose_valid_ = true;
        map_pause_frame_ = true;       // 接管期暂停写图，避免污染 Anchor/地图
        delta_passthrough_frame_ = false;
        handback_probation_until_t_ = -1.0;
        handback_ref_valid_ = false;
        LOG(WARNING) << "[GICP] 交还观察期撤销交还：偏离交还位姿 " << dev_p
                     << " m、状态速度 " << vn << " m/s，回滚位姿并重新接管";
        return;
      }
    }
  }

  // ===== 超时兜底放行前的残差门 =====
  // 观测与当前位姿差过大时不能"强制接受"：实测（gicp_f 376 s）把 24.4 m 的
  // 单帧跳变直接吃进 ESKF 后速度升到 125 m/s、轨迹彻底发散。大残差说明 Voxmap
  // 已属于另一条轨迹（地图段失效），此时应放弃旧图、以当前 IMU 位姿重锚新建
  // 地图段，而不是接受一个错误观测。
  auto force_accept_or_reanchor = [&](const std::string& why, const std::string& detail){
    const double res_p = (T_post.t_ - sys_pred.p).norm();
    const double res_r = BASIC::SO3(T_post.R_.transpose() * sys_pred.R.R()).log_vee().norm() * kRad2Deg;
    // 观测缺失时 T_post 与 IMU 先验逐位相同，残差恒为 0，残差门会把"无观测"
    // 误判成"零残差可信"而空放行（gicp_g 6 次 LOST 超时全是这种现象）。
    // 必须要求本帧确有足够有效平面点，否则一律走清图重锚。
    const bool obs_valid = last_n_effect_ >= g_lost_min_effect_pts;
    last_trust_log_wall_ = std::chrono::steady_clock::now();
    if(obs_valid && res_p <= g_trust_force_accept_max_p && res_r <= g_trust_force_accept_max_r){
      lost_ = false;
      lost_recover_streak_ = 0;
      lost_run_dt_ = 0.0;
      gicp_lost_src_.reset();
      gicp_lost_src_valid_ = false;
      rot_corr_hist_.clear();
      suspect_hold_dt_ = 0.0;
      suspect_run_dt_ = 0.0;
      map_novelty_streak_ = 0;
      map_novelty_run_dt_ = 0.0;
      novelty_hold_ = false;
      stable_pose_ = T_post;
      stable_pose_valid_ = true;
      map_pause_frame_ = false;
      LOG(WARNING) << "[Trust] " << why << "，强制接受 Voxmap 观测并恢复建图"
                   << "（残差 " << res_p << " m / " << res_r << " deg, effect_pts="
                   << last_n_effect_ << "，" << detail
                   << "，累计否决 " << trust_reject_total_ << " 次）";
      return;
    }
    // 大残差：回滚到带阻尼的 IMU 先验，清空旧地图段并以它为基准重锚。
    rollback_damped(sys_pred, cov_pred);
    const BASIC::SE3 T_keep = kf_->GetSE3();
    if(ivox_) ivox_->clear();
    anchor_committed_.clear();
    anchor_pending_.clear();
    {
      std::lock_guard<std::mutex> lock(anchor_inbox_mutex_);
      anchor_dense_inbox_.clear();
    }
    anchor_frozen_ = false;
    anchor_last_log_t_ = -1.0;
    gicp_active_ = false;
    gicp_ok_streak_ = 0;
    gicp_fail_streak_ = 0;
    gicp_handback_streak_ = 0;
    gicp_frame_counter_ = 0;
    gicp_pos_hist_.clear();
    gicp_last_pose_valid_ = false;
    gicp_last_raw_valid_ = false;
    gicp_target_ready_ = false;
    gicp_target_.reset();
    gicp_target_kdtree_.reset();
    lost_ = false;
    lost_recover_streak_ = 0;
    lost_run_dt_ = 0.0;
    gicp_lost_src_.reset();
    gicp_lost_src_valid_ = false;
    rot_corr_hist_.clear();
    suspect_hold_dt_ = 0.0;
    suspect_run_dt_ = 0.0;
    // 清图后整帧点云都落在空白区域，novelty 必然接近 1：先豁免一段时间，
    // 等地图重新积累出覆盖后再继续用该指标判罚。
    map_novelty_streak_ = 0;
    map_novelty_run_dt_ = 0.0;
    novelty_hold_ = false;
    map_novelty_skip_until_t_ = measures_.lidar.end_time + kNoveltyGraceS;
    stable_pose_ = T_keep;
    stable_pose_valid_ = true;
    map_pause_frame_ = false;
    ++reanchor_total_;
    LOG(WARNING) << "[Trust] " << why << "：观测残差 " << res_p << " m / " << res_r
                 << " deg (effect_pts=" << last_n_effect_ << ") 超过门限或观测缺失，"
                 << "拒绝接受并清空地图段重锚"
                 << "（" << detail << "，累计重锚 " << reanchor_total_ << " 次）";
  };

  // ===== 平移锯齿（位置震荡）早检 =====
  // 只依赖位姿自身：逐帧记录接受后的位置增量，换算隐含速度 |Δp|/dt。
  // 平移震荡时相邻帧会被观测来回拽（一帧推开、下一帧拽回），隐含速度成倍
  // 超过平台真实速度；正常飞行/转弯不会连续多帧越门。LOST/接管期状态不是
  // 正常观测链，直接作废历史，避免释放后误触发。
  bool saw_hit = false;
  {
    const double dp_dt = (cur_lidar_gap_ > 1e-3) ? cur_lidar_gap_ : 0.1;
    if(lost_ || gicp_active_ || !saw_last_valid_){
      saw_fill_ = 0;
      saw_idx_ = 0;
    }else{
      const double dx = T_post.t_(0) - saw_last_p_[0];
      const double dy = T_post.t_(1) - saw_last_p_[1];
      const double dz = T_post.t_(2) - saw_last_p_[2];
      saw_vimp_[saw_idx_] = std::sqrt(dx * dx + dy * dy + dz * dz) / dp_dt;
      saw_idx_ = (saw_idx_ + 1) % kSawWin;
      if(saw_fill_ < kSawWin) ++saw_fill_;
      int hits = 0;
      for(int i = 0; i < saw_fill_; ++i){
        if(saw_vimp_[i] > kSawImpliedSpeed) ++hits;
      }
      saw_hit = (hits >= kSawMinHits);
    }
    saw_last_p_[0] = T_post.t_(0);
    saw_last_p_[1] = T_post.t_(1);
    saw_last_p_[2] = T_post.t_(2);
    saw_last_valid_ = true;
  }

  // ===== 地图新点比例门（错误匹配的最早信号，先于所有分支评估）=====
  // 位姿发散往往先表现为小幅震荡，随后错误匹配把整帧点云塞进"尚无粗体素"的
  // 空白区域——本指标就是该比例。正常回访时地图已覆盖、比例很低
  // （gicp_j/gicp_k 实测 0-290 s：mean 1-5%，p99 ≤20%，max ≤29.7%）。
  // 判据用"自 streak 起点以来的累计位移/转角"而非单帧修正量：错误匹配的拖拽
  // 每帧往往只有几厘米（gicp_k 实测 0.065 m/帧），单帧门永不成立，而累计 0.3 s
  // 就到 0.4 m（gicp_k 实测 streak 涨到 66 帧却一次没拦住）。
  // 成立时只做 SUSPECT：撤销本帧观测 + 暂停建图，下一帧自动重评。
  // LOST 期不评估：那一阶段地图本就不完整，另有重定位逻辑。
  if(g_map_novelty_max > 0.0 && !lost_ && !gicp_active_){
    const double nov_t = measures_.lidar.end_time;
    const double frame_dt = (cur_lidar_gap_ > 1e-3) ? cur_lidar_gap_ : 0.1;

    // —— 已进入持续抑制：每帧拒绝 Voxmap 观测、暂停写图，并重试冻结 Anchor 的 GICP ——
    // 只抑制"判据命中帧"不够：中间帧照常接受观测会被错误匹配继续拖走
    // （gicp_t 实测 275.4 s 首次命中后 3 s 内被拖到 3.9 m/s，GICP 修正量
    //  38.7 deg 超出单帧阈值，之后即使再跑也拉不回来）。
    if(novelty_hold_){
      if(g_anchor_gicp_en && !anchor_committed_.empty()){
        bool gicp_ran = false;
        if(tryGicpRelocalize(sys_pred, cov_pred, corr_p, corr_r, "novelty_hold", &gicp_ran)){
          novelty_hold_ = false;
          map_pause_frame_ = false;
          // 接管期保持 Anchor 冻结：冻结的已提交帧是"漂移发生前"的干净绝对参考，
          // 解冻后只有通过质量门的接受帧才提交新帧，而 updateTrustedAnchor 的窗口
          // 裁剪以 t_now 前进为准 —— 接受帧一稀疏，旧干净帧就被成批裁掉
          // （gicp_am 实测 committed 350→242→50→37→0，目标图固定 8 帧，inlier
          // 0.90→0.30，最后目标图无法构建、位姿永久钉死）。冻结后不提交也不裁剪，
          // 目标图始终是触发时刻前 10 s 的干净基准。
          anchor_pending_.clear();
          map_novelty_streak_ = 0;
          map_novelty_run_dt_ = 0.0;
          novelty_ref_valid_ = false;
          novelty_release_cnt_ = 0;
          novelty_release_last_t_ = -1.0;
          LOG(WARNING) << "[Trust] 持续抑制期 GICP 重定位成功（new="
                       << map_novelty_ratio_ << ", effect_pts=" << last_n_effect_
                       << "）：位姿已拉回可信地图并解冻 Anchor，恢复建图";
          return;
        }
        // 新覆盖放行：GICP（目标图来自触发前 10 s 的干净 Anchor）给出的绝对位姿
        // 与 Voxmap 结果只差几厘米/几度 ⇒ 位姿并没有被拖走，新点比例高只是因为
        // 飞进了尚未建图的区域。这是 inlier 门（≥0.5）挡不住的合法情形：场景
        // 新覆盖时源云与旧目标图的共同可见部分本来就少。
        // home3 实测 115 s 首次命中即属此类（new=0.41、GICP 修正 0.085 m/1.18°、
        // inlier 0.227、rmse 0.166 m、cond 113），旧逻辑一律判拒 → 3 s 后清图重锚，
        // 之后 novelty 恒 0.88+（建图已被暂停，指标本身失去意义）→ 连续重锚 48 次，
        // 地图在 8k~38k 点之间震荡，最终纯 IMU 漂到 -1327 m。
        // 与"保图放行"的区别：这里本帧观测是可信的，直接接受、不回滚不清速度。
        if(gicp_ran && gicp_check_valid_ && gicp_check_converged_ &&
           gicp_check_n_in_ >= kNoveltyVerifyMinInliers &&
           gicp_check_rmse_ <= g_gicp_max_rmse){
          const double dp_post = (gicp_check_pose_.t_ - T_post.t_).norm();
          const double dr_post =
              BASIC::SO3(gicp_check_pose_.R_.transpose() * T_post.R_).log_vee().norm() *
              kRad2Deg;
          if(dp_post <= kNoveltyVerifyMaxDpos && dr_post <= kNoveltyVerifyMaxDrot){
            novelty_hold_ = false;
            map_pause_frame_ = false;
            map_novelty_streak_ = 0;
            map_novelty_run_dt_ = 0.0;
            novelty_ref_valid_ = false;
            map_novelty_skip_until_t_ = nov_t + kNoveltyCooldownS;
            // 必须同时解冻 Anchor 并作废旧目标图：放行只说明"本帧位姿与冻结
            // Anchor 一致"，若继续冻结，committed 会永远停在触发时刻。home3_try2
            // 实测 115 s 放行后 Anchor 未解冻，252 s 再次触发时目标图仍是 115 s
            // 的旧区域，而飞行器已飞到 y≈70 m 的新区域 → GICP conv=0、
            // inlier=0/1115 → 误判发散，随后清图重锚级联（z 漂到 -578 m）。
            // 冻结期到达的异步稠密帧携带的是被钉住的位姿，一并作废。
            anchor_pending_.clear();
            {
              std::lock_guard<std::mutex> lock(anchor_inbox_mutex_);
              anchor_dense_inbox_.clear();
            }
            anchor_frozen_ = false;
            gicp_target_ready_ = false;
            gicp_target_.reset();
            gicp_target_kdtree_.reset();
            ++novelty_verified_release_cnt_;
            // 限频：新区域可能连续跨越几十秒，逐次打印会淹没日志
            if(novelty_verified_release_cnt_ <= 10 || novelty_verified_release_cnt_ % 10 == 0){
              LOG(WARNING) << "[Trust] 新覆盖放行（new=" << map_novelty_ratio_
                           << ", GICP-Voxmap dp=" << dp_post << " m/" << dr_post
                           << " deg, inlier=" << gicp_check_n_in_ << "/" << gicp_source_->size()
                           << ", rmse=" << gicp_check_rmse_
                           << " m）：位姿与冻结 Anchor 一致，保留地图继续建图"
                           << "（累计 " << novelty_verified_release_cnt_ << " 次）";
            }
            return;
          }
        }
      }
      // 抑制期位姿必须钉死在判据成立时刻：回滚到冻结状态，而不是"保留 IMU
      // 外推再把速度清零"。后者每帧仍按残余速度推进位置（实测约 1 m/s），
      // 10 s 就把位姿推出地图覆盖（gicp_w 378 s 段 effect_pts 303→9），GICP
      // 初值随之出界、inlier 恒为 0，最终只能在错误位姿清图重锚。
      // home3 复测（取消钉死后）：抑制 2.4 s 内 |v| 从 0.81→3.91 m/s、
      // effect_pts 369→14，方向随姿态误差泄漏的重力加速度单调累积；
      // 钉死是必要的，恢复可恢复性交给下面的"出覆盖放行"。
      kf_->RollbackToState(novelty_hold_state_, novelty_hold_cov_);
      stable_pose_ = kf_->GetSE3();
      stable_pose_valid_ = true;
      map_pause_frame_ = true;
      ++trust_reject_total_;
      map_novelty_run_dt_ += frame_dt;
      // 兜底：持续抑制 g_trust_suspect_max_s 仍无法重定位 ⇒ 清图重锚。
      // 清图时速度必须清零：错误姿态造成的假加速度会在 1 s 内把封顶后的
      // 速度重新泵到 5 m/s（gicp_s 实测），随即撞速度硬门进 LOST。
      // 这里不能再要求 effect_pts 足够：位姿一旦出界，Voxmap 在当前位姿下
      // 可能一个有效点都没有（effect_pts=0），若仍以 effect>=100 为前置条件，
      // 兜底永远进不去，持续抑制变成死锁（gicp_u 实测 300→420 s 纯 IMU 漂到
      // 119 m）。出界本身就说明必须重锚，与有效点数无关。
      if(g_trust_suspect_max_s > 0.0 && map_novelty_run_dt_ >= g_trust_suspect_max_s){
        // 保图放行：地图仍覆盖当前区域（本轮抑制期间 effect_pts 仍有 100+）说明
        // 位姿并没有离开已建图区域，novelty 高更可能是短时的错配/新覆盖；此时
        // 清图会把此前几十秒的准确地图一起毁掉，重锚位姿还带着抑制期累积误差。
        // 先以 IMU 位姿（速度清零）放行、保留地图，下一帧照常接受 Voxmap 观测。
        // 移动中 / 已出覆盖：此时硬重锚会把位姿钉在数秒前的位置，而飞机仍在
        // 前飞，随后只能由 IMU 外推补偿，等于人为制造漂移（home3 0.25 实测
        // 557 s 重锚后 25 s 内速度持续 5~6 m/s 被钳、z 单调下沉）。
        // GICP 判据：内点极少或残差极大 ⇒ 失败原因是"扫描已跑出冻结图覆盖"，
        // 而不是位姿被拖走，重锚没有意义。
        const bool coverage_limited = gicp_check_valid_ &&
            (gicp_check_n_in_ < 10 || gicp_check_rmse_ > 1.0);
        const bool moving_release = (sys_pred.v.norm() > 1.5) || coverage_limited;
        if(last_n_effect_ >= kNoveltyResetMinEffectPts || moving_release){
          // 短时内反复放行 = 持续发散，不是偶发的新覆盖：正常飞行只会遇到一次，
          // 错误匹配却会按"放行-冷却-再命中"的节奏每 9 s 左右复发一次。
          // gicp_ac 实测：19:53:19 起连续 5 次放行，位姿被每次 5 s 的冷却窗口
          // 推走 1-2 m，25 m 后彻底拉不回。故第 2 次直接跳过放行、走硬重锚。
          // 移动中/出覆盖的放行例外：重锚代价更大，允许连续 4 次。
          const bool rel_recent = (novelty_release_last_t_ > 0.0) &&
                                  (nov_t - novelty_release_last_t_ <= kNoveltyReleaseWinS);
          const int  rel_cnt = rel_recent ? (novelty_release_cnt_ + 1) : 1;
          const int  rel_max = moving_release ? 4 : kNoveltyReleaseMax;
          if(rel_cnt < rel_max){
            // 放行位姿取冻结时刻的可信状态，零偏恢复种子值：抑制期若沿用跑飞的
            // ba/bg 继续外推，放行后速度会被假加速度持续泵高（gicp_ac 实测
            // 19:53:16 触发瞬间 |v|=2.18 m/s，而真实位移只有厘米级）。
            if(moving_release){
              // 移动中放行：保留当前预测位姿（仅清速度），不弹回冻结时刻。
              rollback_zero_vel(sys_pred, cov_pred);
            }else{
              SysState s_rel = novelty_hold_state_;
              if(gicp_seed_valid_){
                s_rel.bg = gicp_seed_state_.bg;
                s_rel.ba = gicp_seed_state_.ba;
              }
              s_rel.v.setZero();
              kf_->RollbackToState(s_rel, novelty_hold_cov_);
            }
            novelty_release_cnt_ = rel_cnt;
            novelty_release_last_t_ = nov_t;
            stable_pose_ = kf_->GetSE3();
            stable_pose_valid_ = true;
            map_pause_frame_ = false;
            map_novelty_streak_ = 0;
            map_novelty_run_dt_ = 0.0;
            novelty_ref_valid_ = false;
            novelty_hold_ = false;
            map_novelty_skip_until_t_ = nov_t + kNoveltyCooldownS;
            const auto now = std::chrono::steady_clock::now();
            last_trust_log_wall_ = now;
            LOG(WARNING) << "[Trust] 持续抑制 " << g_trust_suspect_max_s
                         << " s 未重定位"
                         << (moving_release
                                 ? (coverage_limited ? "（扫描已出冻结图覆盖）" : "（仍在移动）")
                                 : "")
                         << "，effect_pts=" << last_n_effect_
                         << "（new=" << map_novelty_ratio_
                         << "）：保留地图、恢复 Voxmap 观测（冷却 "
                         << kNoveltyCooldownS << " s，第 " << rel_cnt << " 次放行）";
            return;
          }
          LOG(WARNING) << "[Trust] " << kNoveltyReleaseWinS << " s 内第 " << rel_cnt
                       << " 次保图放行（间隔 "
                       << (nov_t - novelty_release_last_t_) << " s）：判为持续发散，改为硬重锚";
        }
        // 重锚位姿：优先用最后一次可信 GICP 位姿，且连同零偏/重力一起恢复。
        // 抑制期内 IMU 一直在错误零偏下外推，若以当前（已漂移的）IMU 位姿清图，
        // 新地图整体偏移、后续再无绝对参考（gicp_ab 19:36:52 即此：位姿从
        // (-1.7,-2.0,4.2) 被抬到 (4.6,5.9,0) 方向，随后单调漂到 15 m 外）。
        // 种子位姿必须"新鲜"才能用来重锚：接管/交还后种子可能已陈旧几十秒，
        // 以旧种子清图会把位姿直接弹回几十秒前的位置（gicp_ad 实测 21:28:48
        // 用 79 s 前的种子重锚，1.5 s 后速度被泵到 5 m/s，最终漂到 53 m）。
        const bool seed_fresh = gicp_seed_valid_ && gicp_seed_t_ > 0.0 &&
            (measures_.lidar.end_time - gicp_seed_t_) <= kNoveltySeedMaxAgeS;
        if(seed_fresh){
          SysState s_keep = gicp_seed_state_;
          s_keep.R = gicp_seed_pose_.R_;
          s_keep.p = gicp_seed_pose_.t_;
          s_keep.v.setZero();
          kf_->RollbackToState(s_keep, cov_pred);
        }else{
          rollback_zero_vel(sys_pred, cov_pred);
        }
        const BASIC::SE3 T_keep = kf_->GetSE3();
        if(ivox_) ivox_->clear();
        anchor_committed_.clear();
        anchor_pending_.clear();
        {
          std::lock_guard<std::mutex> lock(anchor_inbox_mutex_);
          anchor_dense_inbox_.clear();
        }
        anchor_frozen_ = false;
        anchor_last_log_t_ = -1.0;
        gicp_active_ = false;
        gicp_ok_streak_ = 0;
        gicp_fail_streak_ = 0;
        gicp_handback_streak_ = 0;
        gicp_frame_counter_ = 0;
        gicp_pos_hist_.clear();
        gicp_last_pose_valid_ = false;
        gicp_last_raw_valid_ = false;
        gicp_seed_valid_ = false;
        gicp_hold_cycles_ = 0;
        gicp_target_ready_ = false;
        gicp_target_.reset();
        gicp_target_kdtree_.reset();
        gicp_lost_src_.reset();
        gicp_lost_src_valid_ = false;
        rot_corr_hist_.clear();
        suspect_hold_dt_ = 0.0;
        suspect_run_dt_ = 0.0;
        map_novelty_streak_ = 0;
        map_novelty_run_dt_ = 0.0;
        novelty_ref_valid_ = false;
        novelty_hold_ = false;
        novelty_release_cnt_ = 0;
        novelty_release_last_t_ = -1.0;
        map_novelty_skip_until_t_ = nov_t + kNoveltyGraceS + kNoveltyCooldownS;
        stable_pose_ = T_keep;
        stable_pose_valid_ = true;
        map_pause_frame_ = false;
        ++reanchor_total_;
        LOG(WARNING) << "[Trust] 持续抑制 " << g_trust_suspect_max_s
                     << " s 仍未能重定位（new=" << map_novelty_ratio_
                     << "）：清空地图段并以"
                     << (seed_fresh ? "新鲜种子位姿" : "冻结时刻可信位姿")
                     << "继续建图（速度清零，豁免 "
                     << kNoveltyGraceS << " s，累计重锚 " << reanchor_total_ << " 次）";
        return;
      }
      const auto now = std::chrono::steady_clock::now();
      if(std::chrono::duration<double>(now - last_trust_log_wall_).count() > 1.0){
        last_trust_log_wall_ = now;
        LOG(WARNING) << "[Trust] 持续抑制中 " << map_novelty_run_dt_ << " s（new="
                     << map_novelty_ratio_ << ", |v|=" << sys_pred.v.norm()
                     << " m/s, effect_pts=" << last_n_effect_
                     << "）：继续拒绝 Voxmap 观测并重试 GICP";
      }
      return;
    }

    // —— 未进入抑制：逐帧评估判据 ——
    const bool nov_grace = (nov_t < map_novelty_skip_until_t_);
    // 早期漂移前兆：新点比例与 IMU 预测速度同时抬头（任一阈值 <=0 则关闭）。
    // gicp_o 逐帧统计：健康段两者从不同时成立（40-60 s 新点比例 >0.20 连续
    // 13 帧，但速度低；60-180 s 速度峰值 1.2 m/s，但新点比例低）；275.3 s
    // 发散前 0.207 / 1.79 m/s 首次同时成立，比 0.25+3 帧判据早约 1 s。
    const bool early_on = (g_map_novelty_early > 0.0 && g_trust_vel_early > 0.0);
    const double vn_pred = sys_pred.v.norm();
    const bool early_hit = early_on && map_novelty_ratio_ > g_map_novelty_early &&
                           vn_pred > g_trust_vel_early;
    // 开阔区门控：本帧扫描尺度超门（无结构的大空间）时，新点比例与平移锯齿
    // 判据不评估。此时地图上没有对应结构，novelty 天然偏高；冻结 Anchor 的
    // 目标图与新区域零重叠，GICP 无法做绝对校验，判据一旦成立只能硬重锚，
    // 而重锚会把位姿钉在数秒前、随后由 IMU 外推补偿，反而制造漂移
    // （home3 0.25 实测：430 s 触发 → 476 s |v|>2 → 497 s 下沉）。
    const bool open_space =
        (g_open_guard_max_range > 0.0 && scan_max_range_ > g_open_guard_max_range) ||
        (g_open_guard_z_span > 0.0 && scan_z_span_ > g_open_guard_z_span);
    if(open_space && !nov_grace &&
       (map_novelty_ratio_ > g_map_novelty_max || early_hit || saw_hit)){
      const auto now = std::chrono::steady_clock::now();
      if(std::chrono::duration<double>(now - last_trust_log_wall_).count() > 2.0){
        last_trust_log_wall_ = now;
        LOG(WARNING) << "[Trust] 开阔区门控：跳过新点比例/平移锯齿判据（new="
                     << map_novelty_ratio_ << ", rmax=" << scan_max_range_
                     << " m, z_span=" << scan_z_span_ << " m）";
      }
      map_novelty_streak_ = 0;
      map_novelty_run_dt_ = 0.0;
      novelty_ref_valid_ = false;
    }else if(nov_grace || (map_novelty_ratio_ <= g_map_novelty_max && !early_hit && !saw_hit)){
      map_novelty_streak_ = 0;
      map_novelty_run_dt_ = 0.0;
      novelty_ref_valid_ = false;
    }else{
      if(!novelty_ref_valid_){
        novelty_ref_pose_ = T_post;
        novelty_ref_valid_ = true;
      }
      ++map_novelty_streak_;
      const double cum_p = (T_post.t_ - novelty_ref_pose_.t_).norm();
      const double cum_r = BASIC::SO3(novelty_ref_pose_.R_.transpose() * T_post.R_)
                               .log_vee().norm() * kRad2Deg;
      const bool late_hit = map_novelty_streak_ >= g_map_novelty_streak &&
          (cum_p > g_map_novelty_jump_min || cum_r > g_map_novelty_jump_min_rot);
      const bool early_trigger = early_on && early_hit &&
          map_novelty_streak_ >= std::max(1, g_novelty_early_streak) &&
          last_n_effect_ >= kNoveltyResetMinEffectPts;
      const bool saw_trigger = saw_hit && last_n_effect_ >= kNoveltyResetMinEffectPts;
      if(late_hit || early_trigger || saw_trigger){
        const int streak_now = map_novelty_streak_;
        const char* gate_name = saw_trigger ? "平移锯齿(位置震荡)"
                              : (early_trigger ? "早期前兆(新点比例+速度)" : "地图新点比例");
        if(last_n_effect_ < kNoveltyResetMinEffectPts){
          // 地图有效点很少 ⇒ 更可能是"飞进未建图区域"而不是错误匹配：
          // 轻量抑制，超时后放弃并接受观测（清图重锚反而会把正常的新覆盖毁掉）。
          map_novelty_run_dt_ += frame_dt;
          if(g_trust_suspect_max_s > 0.0 && map_novelty_run_dt_ >= g_trust_suspect_max_s){
            map_novelty_streak_ = 0;
            map_novelty_run_dt_ = 0.0;
            novelty_ref_valid_ = false;
            map_novelty_skip_until_t_ = nov_t + kNoveltyCooldownS;
            LOG(WARNING) << "[Trust] " << gate_name << " 抑制已持续 "
                         << g_trust_suspect_max_s << " s，但地图有效点仅 "
                         << last_n_effect_ << " 个（疑似新覆盖区域）：放弃抑制、"
                         << "接受本帧观测（冷却 " << kNoveltyCooldownS << " s）";
            return;
          }
          rollback_damped(sys_pred, cov_pred);
          stable_pose_ = kf_->GetSE3();
          stable_pose_valid_ = true;
          map_pause_frame_ = true;
          ++trust_reject_total_;
          const auto now = std::chrono::steady_clock::now();
          if(std::chrono::duration<double>(now - last_trust_log_wall_).count() > 2.0){
            last_trust_log_wall_ = now;
            LOG(WARNING) << "[Trust] " << gate_name << " 判据成立（new="
                         << map_novelty_ratio_ << ", 连续 " << streak_now
                         << " 帧），但地图有效点仅 " << last_n_effect_
                         << " 个（疑似新覆盖区域）：撤销本帧观测、暂停建图";
          }
          return;
        }
        // 地图仍有足够有效点 ⇒ 位姿在偏离已有地图，正在形成错误匹配。
        // 本帧先撤销观测，随后进入持续抑制：后续帧一律拒绝 Voxmap 观测，
        // 优先用冻结 Anchor 的 GICP 把位姿拉回可信 world 坐标；
        // 无 Anchor 或持续 g_trust_suspect_max_s 未收敛才清图重锚。
        rollback_damped(sys_pred, cov_pred);
        stable_pose_ = kf_->GetSE3();
        stable_pose_valid_ = true;
        map_pause_frame_ = true;
        ++trust_reject_total_;
        novelty_hold_ = true;
        // 冻结位姿：抑制期每帧回滚到这里，GICP 的初值始终是判据成立时刻的位姿。
        novelty_hold_state_ = kf_->GetSysState();
        novelty_hold_cov_   = kf_->GetCov();
        map_novelty_run_dt_ += frame_dt;
        LOG(WARNING) << "[Trust] " << gate_name << " 判据成立（new="
                     << map_novelty_ratio_ << ", |v|=" << vn_pred
                     << " m/s, 连续 " << streak_now << " 帧, 累计修正 "
                     << cum_p << " m / " << cum_r << " deg, effect_pts="
                     << last_n_effect_ << "）：撤销本帧观测并进入持续抑制，"
                     << "逐帧重试冻结 Anchor 的 GICP 绝对重定位";
        return;
      }
    }
  }

  // ===== GICP 接管：用冻结可信 Anchor 产生第二套 LiDAR 位姿观测 =====
  // 已接管（gicp_active_）时 GICP 是唯一可信观测：失败就退回 IMU 预测并暂停建图，
  // 绝不能让可疑的 Voxmap 观测把位姿再次拖走（否则会像实验里一样再也拉不回来）。
  // 已 LOST 时每帧尝试，成功即恢复；首次 SUSPECT 在速率门内尝试。
  if(gicp_active_){
    // 降频：单帧 GICP 实测 200~800 ms，每帧都跑会让节点越落越远（实测末尾滞后
    // 约 60 s），且 GICP 的慢变误差在帧间由 IMU 传播已能覆盖。接管期每
    // g_gicp_frame_interval 帧做一次绝对观测，其余帧按拟合速度限幅外推，
    // 且这些外推帧不写图、不提交 Anchor（见下方 map_pause_frame_）。
    ++gicp_frame_counter_;
    const int interval = std::max(1, g_gicp_frame_interval);
    const bool run_gicp = (interval <= 1) || ((gicp_frame_counter_ - 1) % interval == 0);
    if(run_gicp && tryGicpRelocalize(sys_pred, cov_pred, corr_p, corr_r, "active")){
      map_pause_frame_ = false;
      return;
    }
    if(gicp_active_){   // 未退出接管：本帧不用 Voxmap 修正
      // 累积本帧 IMU 旋转增量（相对被钉住的上一可信姿态），供接受帧的 bg 估计。
      if(gicp_last_pose_valid_){
        gicp_imu_rot_acc_ +=
            BASIC::SO3(gicp_last_pose_.R_.transpose() * sys_pred.R.R()).log_vee();
      } else {
        gicp_imu_rot_acc_.setZero();
      }
      if(run_gicp){
        // GICP 失败帧：位姿钉在上一帧"已接受的 GICP 解"上、速度清零（hold）。
        // 不能用带速度的 IMU 外推：GICP 降频后两帧之间隔 N×0.1 s，外推误差
        // 逐帧累积，一旦初值离 target 超过 0.5 m 粗对应半径就再也找不到对应，
        // GICP 会永久返回 conv=0/dpos=0（实测 11:23:3x 起的死锁）。
        SysState s = sys_pred;
        if(gicp_last_pose_valid_){
          s.R = gicp_last_pose_.R_;
          s.p = gicp_last_pose_.t_;
        }
        s.v.setZero();
        kf_->RollbackToState(s, cov_pred);
        map_pause_frame_ = true;
      }else{
        // 降频帧：**不能按 IMU 外推**。实测（gicp_h 270~320 s）每次 GICP 接受后
        // 拟合速度只有 0.06 m/s，随后 6 帧纯 IMU 传播就把速度泵到 3.4 m/s
        // （≈5.5 m/s² 的虚假加速度，来自姿态/零偏误差的重力泄漏）：位姿每 0.6 s
        // 冲出 1.5~2 m，GICP 对应关系随之劣化，一旦连续失败退出接管，位姿再也
        // 拉不回来（最终 |p| 达 3.9 km）。这里改为以"上一帧已接受的 GICP 位姿"
        // 为基准、用拟合速度做限幅外推：真实低频运动保留，IMU 速度自积分被切断。
        SysState s = sys_pred;
        V3 v_fit = fitGicpVelocity();
        const double vn_fit = v_fit.norm();
        if (vn_fit > kRejectVelMax) v_fit *= kRejectVelMax / vn_fit;
        if(gicp_last_pose_valid_){
          const double t_lidar_end = measures_.lidar.end_time;
          double dt_g = (t_lidar_end > gicp_last_t_) ? (t_lidar_end - gicp_last_t_) : 0.0;
          if (dt_g > 0.5) dt_g = 0.5;
          V3 dp = v_fit * dt_g;
          const double dpn = dp.norm();
          constexpr double kGicpHoldMaxStep = 0.30;   // m/帧，超限说明速度不可信
          if (dpn > kGicpHoldMaxStep) dp *= kGicpHoldMaxStep / dpn;
          s.R = gicp_last_pose_.R_;
          s.p = gicp_last_pose_.t_ + dp;
        }
        s.v = v_fit;
        kf_->RollbackToState(s, cov_pred);
        // 降频帧不写图、不进 Anchor。外推位姿只有拟合速度支撑，一旦累积偏差，
        // 这些帧会以漂移位姿写 IVox 并提交 Anchor，随后 updateTrustedAnchor 用
        // t_now 把窗口内干净的旧帧裁掉（gicp_al 实测 committed 136→122→…→10，
        // 目标图固定 3 帧 / 5093 点，inlier 0.60→0.11）。接管期只有已通过质量门
        // 的 GICP 接受帧才允许写图与提交 Anchor。
        map_pause_frame_ = true;
      }
      return;
    }
  }
  if(lost_ && tryGicpRelocalize(sys_pred, cov_pred, corr_p, corr_r, "lost")){
    map_pause_frame_ = false;
    return;
  }

  if(!lost_){
    // ===== 单帧姿态修正限幅（抗"渐进拖歪"）=====
    // 实测：健康段每帧 LiDAR 姿态修正 0.2~0.4 deg；发散前 272~280 s 升到 1.0~3.0 deg，
    // 且修正方向与速度同向（deg_dir_v≈0.97）。单帧跳变门（默认 25 deg）永远拦不住，
    // 但 10~20 deg/s 的持续拖拽会把姿态与重力错开，产生 ~2 m/s² 的虚假水平加速度，
    // 速度自积分到数百 m/s。此处把每帧姿态修正量限幅，平移修正照常接受。
    if(g_trust_rot_clamp_deg > 0.0 && corr_r > g_trust_rot_clamp_deg && corr_r > 1e-9){
      V3 dr = BASIC::SO3(T_pred.R_.transpose() * T_post.R_).log_vee();
      dr *= (g_trust_rot_clamp_deg / corr_r);
      SysState sys_fix = sys_pred;
      sys_fix.p = T_post.t_;                        // 平移修正保持
      sys_fix.R = sys_pred.R * BASIC::SO3::Exp(dr); // 姿态修正限幅
      kf_->RollbackToState(sys_fix, cov_pred);
      T_post = kf_->GetSE3();
      corr_p = (T_post.t_ - T_pred.t_).norm();
      corr_r = BASIC::SO3(T_post.R_.transpose() * T_pred.R_).log_vee().norm() * kRad2Deg;
      ++rot_clamp_total_;
      const auto now = std::chrono::steady_clock::now();
      if(std::chrono::duration<double>(now - last_trust_log_wall_).count() > 5.0){
        last_trust_log_wall_ = now;
        LOG(WARNING) << "[Trust] 姿态修正限幅: 本帧 " << corr_r << " deg 被压到 "
                     << g_trust_rot_clamp_deg << " deg（累计 " << rot_clamp_total_ << " 次）";
      }
    }

    // ===== 武装门（B）=====
    // bag 相对时间未到之前，速率门与跳变门都只记录、不改状态机。
    // 依据：起飞/手持上机段存在合法大修正（gicp_e 41 s 3.15 deg/s、gicp_f 2.4 s
    // 10.6 deg/s + 1.03 m 跳变），它们会把信任状态机点着并永久冻结空 Anchor。
    const double bag_rel_t = (first_lidar_end_time_ > 0.0)
        ? (measures_.lidar.end_time - first_lidar_end_time_) : 0.0;
    const bool suspect_armed =
        (g_trust_suspect_after_s <= 0.0) || (bag_rel_t >= g_trust_suspect_after_s);

    // ===== 姿态修正净速率门（SUSPECT / LOST 分级）=====
    // 判据用"净速率" |Σdr|/Σdt（不是 |dr| 之和）：健康段修正方向来回摆动、净速率
    // 小（实测 2 s 窗 mean 0.7~1.1 deg/s，p90 1.9）；拖歪时每帧修正方向一致
    // （deg_dir_v≈0.97），净速率直接到 5~10 deg/s，分离度约 4 倍。
    // 再叠加方向一致性 C_θ = |Σdr| / (Σ|dr|+ε)：只有"同向持续拖拽"才触发，
    // 排除正常转弯时来回摆动造成的偏高净速率。
    if(g_trust_corr_rate_max > 0.0){
      const double dt_f = (cur_lidar_gap_ > 1e-3) ? cur_lidar_gap_ : 0.1;
      const V3 dr_vec = BASIC::SO3(T_pred.R_.transpose() * T_post.R_).log_vee();
      rot_corr_hist_.emplace_back(dt_f, dr_vec);
      const int win = std::max(4, g_trust_corr_rate_window);
      while(static_cast<int>(rot_corr_hist_.size()) > win) rot_corr_hist_.pop_front();
      double sum_dt = 0.0, sum_abs = 0.0;
      V3 sum_dr = V3::Zero();
      for(const auto& e : rot_corr_hist_){
        sum_dt += e.first;
        sum_dr += e.second;
        sum_abs += e.second.norm();
      }
      const double rate = (sum_dt > 1e-6) ? (sum_dr.norm() * kRad2Deg / sum_dt) : 0.0;
      const double coh  = (sum_abs > 1e-9) ? (sum_dr.norm() / sum_abs) : 0.0;
      // B：实验门 —— 只在该 bag 相对时间之后才允许判 SUSPECT。
      // 起飞/手持上机段的合法大修正不应把状态机点着（gicp_e 41 s 假触发即此类）。
      const bool over_rate = rate > g_trust_corr_rate_max && coh >= g_trust_corr_coh_min &&
          static_cast<int>(rot_corr_hist_.size()) >= std::max(4, win / 2);

      if(over_rate && !suspect_armed){
        const auto now = std::chrono::steady_clock::now();
        if(std::chrono::duration<double>(now - last_trust_log_wall_).count() > 5.0){
          last_trust_log_wall_ = now;
          LOG(WARNING) << "[Trust] 姿态修正净速率 " << rate << " deg/s (C=" << coh
                       << ") 超阈值，但 bag 相对时间 " << bag_rel_t << " s < "
                       << g_trust_suspect_after_s
                       << " s，暂不判 SUSPECT（保留 Voxmap 观测）";
        }
      }

      if(over_rate && suspect_armed){
        // 先尝试用冻结可信 Anchor 的 GICP 位姿替代这次可疑的 Voxmap 观测。
        // 与"撤销修正"不同：这是换成另一套独立配准观测，滤波器不会失去 LiDAR 约束。
        bool gicp_ran = false;
        if(tryGicpRelocalize(sys_pred, cov_pred, corr_p, corr_r, "rate", &gicp_ran)){
          map_pause_frame_ = false;
          suspect_run_dt_ = 0.0;
          return;
        }
        // C：GICP 被全局节流（本帧没跑）时无法独立复核，不单凭速率门判 SUSPECT，
        // 保留本帧 Voxmap 观测继续建图；等下一个真正的复核帧再判。
        const bool can_judge = gicp_ran || !g_anchor_gicp_en;
        // SUSPECT 连续时长按帧间隔累计（含被节流帧），D 的超时才是真实墙钟时长。
        suspect_run_dt_ += dt_f;
        // D：SUSPECT 连续超时兜底 —— 长时间无法恢复时强制接受 Voxmap 观测并恢复建图，
        // 避免像 gicp_e 那样 3.5 min 纯 IMU 裸飞、Anchor 覆盖彻底丢失。
        if(g_trust_suspect_max_s > 0.0 && suspect_run_dt_ >= g_trust_suspect_max_s){
          const std::string detail = "净速率 " + std::to_string(rate) + " deg/s, C=" +
                                     std::to_string(coh);
          force_accept_or_reanchor("SUSPECT 已持续超过 " +
                                   std::to_string(g_trust_suspect_max_s) + " s", detail);
          return;
        }
        if(!can_judge){
          const auto now = std::chrono::steady_clock::now();
          if(std::chrono::duration<double>(now - last_trust_log_wall_).count() > 5.0){
            last_trust_log_wall_ = now;
            LOG(WARNING) << "[Trust] 净速率 " << rate << " deg/s (C=" << coh
                         << ") 超阈值，但本帧 GICP 被节流未复核，保留 Voxmap 观测"
                         << "（SUSPECT 累计 " << suspect_run_dt_ << " s）";
          }
        }else{
          // 本帧激光修正被判不可信：撤销，保留 IMU 传播（不进 LOST 也能自恢复）
          rollback_damped(sys_pred, cov_pred);
          T_post = kf_->GetSE3();
          corr_p = (T_post.t_ - T_pred.t_).norm();
          corr_r = BASIC::SO3(T_post.R_.transpose() * T_pred.R_).log_vee().norm() * kRad2Deg;
          ++trust_reject_total_;
          // 拒绝后当前状态即 IMU 推断，是现有最佳估计，刷新可信位姿锁存，
          // 避免长时间 SUSPECT 后 IMU 慢漂把跳变门误触成 LOST。
          stable_pose_ = T_post;
          stable_pose_valid_ = true;
          map_pause_frame_ = true;   // 本帧不写图（防止把拖歪的点云写进配准地图）

          // 分级：只有"更快 + 持续"才升级 LOST，其余仅 SUSPECT（下一帧自动重评）
          bool escalate = false;
          if(g_trust_corr_rate_lost > 0.0 && rate > g_trust_corr_rate_lost){
            suspect_hold_dt_ += dt_f;
            escalate = suspect_hold_dt_ >= g_trust_corr_lost_hold_s;
          }else{
            suspect_hold_dt_ = 0.0;
          }

          const auto now = std::chrono::steady_clock::now();
          if(std::chrono::duration<double>(now - last_trust_log_wall_).count() > 5.0){
            last_trust_log_wall_ = now;
            LOG(WARNING) << "[Trust] 姿态修正净速率 " << rate << " deg/s (C=" << coh
                         << ") 超阈值(" << g_trust_corr_rate_max << " deg/s, 窗 " << win
                         << " 帧)，撤销本帧激光修正、暂停建图"
                         << (escalate ? "，升级 LOST" : "（SUSPECT，不进 LOST）")
                         << "（累计否决 " << trust_reject_total_ << " 次）";
          }
          if(!escalate){
            return;   // SUSPECT：保持正常态，下一帧继续评估
          }
          lost_ = true;
          lost_recover_streak_ = 0;
          lost_run_dt_ = 0.0;
          gicp_lost_src_.reset();
          gicp_lost_src_valid_ = false;
          rot_corr_hist_.clear();
          suspect_hold_dt_ = 0.0;
          suspect_run_dt_ = 0.0;
          return;
        }
      }
    }else{
      suspect_hold_dt_ = 0.0;   // 本帧正常：清空 SUSPECT 持续计时
      suspect_run_dt_ = 0.0;
    }

    if(!stable_pose_valid_){
      stable_pose_ = T_post;
      stable_pose_valid_ = true;
      return;
    }
    const double jump_p = (T_post.t_ - stable_pose_.t_).norm();
    const double jump_r = BASIC::SO3(T_post.R_.transpose() * stable_pose_.R_).log_vee().norm() * kRad2Deg;

    // 阈值按帧间隔缩放：掉帧/断档时 cur_lidar_gap_ 明显大于标称 0.1 s，
    // 单帧修正量本来就更大（IMU 先验覆盖更长时段）。不缩放会把"因掉帧变大
    // 的正常修正"误判为跳变并回滚 —— 实测一次 0.5 s 掉帧 + 快速自转产生
    // 31.6° 修正被回滚，位姿直接丢 30°，随后同向恢复修正又把 SUSPECT 门
    // 永久点着（exp_susp 在 39 s 就是这样崩的）。
    const double gap_scale = std::max(1.0, cur_lidar_gap_ / 0.1);
    const double jump_p_max = g_trust_jump_max * gap_scale;
    const double jump_r_max = g_trust_jump_rot_max * gap_scale;

    // ===== 状态速度硬门 =====
    // 单帧跳变门只看"单帧位移"，挡不住"每帧 <1 m 的持续拖拽"：gicp_i 298 s 起
    // 状态速度 1.1 → 10 m/s（≈6 m/s²，姿态/零偏误差的重力泄漏叠加被放行的错误
    // 观测），py 在 1.5 s 内从 1.4 m 冲到 10 m，而每帧位移都低于 1 m 门限，
    // 于是一路放行，最终飞到 327 m、pz −246 m。drone 本 bag 实飞速度 ≤1.3 m/s。
    // 速度超门时把状态速度钳回"真实运动量级"；但不再整帧否决/进 LOST ——
    // 那样会丢掉唯一能修正姿态的观测（见下方注释与 gicp_w 末段实测）。
    const double vn_pred = sys_pred.v.norm();
    if(g_trust_vel_max > 0.0 && suspect_armed && vn_pred > g_trust_vel_max){
      // 超速几乎总是姿态/零偏误差造成的重力泄漏（本 bag 实飞 ≤1.3 m/s）。
      // 原策略"撤销本帧激光修正 + 进 LOST"会同时停掉建图与全部观测：LOST 期
      // lost_map_en=false，纯 IMU 在 4~5 m/s 状态下继续积分，实测 gicp_w 末段
      // 30 s 冲出 46 m，z 掉到 −23 m，且 LOST↔退出 LOST 反复振荡。
      // 改为：保留本帧观测（它同时修正姿态与速度），仅把状态速度钳到
      // lost_vel_max；位姿仍有 Voxmap 约束，地图继续累积。
      SysState s_vel = kf_->GetSysState();
      const double vn_post = s_vel.v.norm();
      if(vn_post > g_lost_vel_max && vn_post > 1e-9){
        s_vel.v *= (g_lost_vel_max / vn_post);
        kf_->RollbackToState(s_vel, cov_pred);
      }
      const auto now = std::chrono::steady_clock::now();
      if(std::chrono::duration<double>(now - last_trust_log_wall_).count() > 1.0){
        last_trust_log_wall_ = now;
        LOG(WARNING) << "[Trust] 状态速度 " << vn_pred << " m/s 超门限("
                     << g_trust_vel_max << " m/s)：保留本帧观测、姿态参与修正，"
                     << "状态速度钳到 " << g_lost_vel_max << " m/s（不进 LOST）";
      }
      return;
    }

    if((jump_p > jump_p_max || jump_r > jump_r_max) && !suspect_armed){
      // B：未武装段的合法大修正（起飞/上机）只记录，不升级 LOST
      const auto now = std::chrono::steady_clock::now();
      if(std::chrono::duration<double>(now - last_trust_log_wall_).count() > 5.0){
        last_trust_log_wall_ = now;
        LOG(WARNING) << "[Trust] 单帧跳变 " << jump_p << " m / " << jump_r
                     << " deg 超阈值，但 bag 相对时间 " << bag_rel_t << " s < "
                     << g_trust_suspect_after_s << " s，暂不升级（保留 Voxmap 观测）";
      }
    }
    if((jump_p > jump_p_max || jump_r > jump_r_max) && suspect_armed){
      // 单帧跳变也先尝试 GICP 独立观测，避免直接进入 LOST
      bool jump_gicp_ran = false;
      if(tryGicpRelocalize(sys_pred, cov_pred, corr_p, corr_r, "jump", &jump_gicp_ran)){
        map_pause_frame_ = false;
        return;
      }
      // C：本帧被全局节流时没有独立复核，不升级 LOST——只撤销本帧观测并保持
      // 非 LOST 态，等下一个复核帧（≤1.5 s）再判定。否则 1.06 m 的正常大修正
      // 会抢在速率门/GICP 之前把状态机打进 LOST（gicp_f 225 s 即如此）。
      if(!jump_gicp_ran && g_anchor_gicp_en){
        rollback_damped(sys_pred, cov_pred);
        stable_pose_ = kf_->GetSE3();
        stable_pose_valid_ = true;
        map_pause_frame_ = true;
        const auto now = std::chrono::steady_clock::now();
        if(std::chrono::duration<double>(now - last_trust_log_wall_).count() > 5.0){
          last_trust_log_wall_ = now;
          LOG(WARNING) << "[Trust] 单帧跳变 " << jump_p << " m / " << jump_r
                       << " deg 超阈值，但本帧 GICP 被节流未复核："
                       << "仅撤销本帧观测、暂不升级 LOST（下一复核帧再判）";
        }
        return;
      }
      rollback_damped(sys_pred, cov_pred);
      lost_ = true;
      lost_recover_streak_ = 0;
      lost_run_dt_ = 0.0;
      gicp_lost_src_.reset();
      gicp_lost_src_valid_ = false;
      rot_corr_hist_.clear();
      suspect_hold_dt_ = 0.0;
      ++trust_reject_total_;
      last_trust_log_wall_ = std::chrono::steady_clock::now();
      LOG(WARNING) << "[Trust] 单帧跳变 " << jump_p << " m / " << jump_r
                   << " deg 超阈值(" << jump_p_max << " m / "
                   << jump_r_max << " deg @" << cur_lidar_gap_ << " s gap)，"
                   << "撤销本次激光修正、进入 LOST："
                   << "暂停建图，位姿/TF/点云继续全量输出（累计否决 "
                   << trust_reject_total_ << " 次）";
      return;
    }
    stable_pose_ = T_post;   // 正常帧：刷新可信位姿锁存
    return;
  }

  // ===== LOST：影子配准 + 限速再锚定 =====
  // 断档（如雷达掉线 2s 后接回）后，IMU 先验与真实位姿可能差出米级：
  //   - 一次性接受配准结果 → 位姿瞬间跳变，下游（PX4/ego planner）直接甩飞；
  //   - 完全否决配准结果 → 位姿永久停在断档前的位置，永远回不到真实位姿。
  // 折中策略：把「先验 → 配准结果」的修正量按每帧最大平移/旋转切成小步，
  // 逐帧把状态拉回配准结果（配准结果与冻结前的局部地图是一致的，因此是可信锚点）。
  // n_effect==0（无任何有效平面残差）时 corr_p 必然为 0，但这不是"收敛"而是
  // "无信息"：drone bag 275 s 后正是靠这一点每 5 帧假收敛退出 LOST，随后
  // 纯 IMU 自由积分把速度推到 700 m/s。此处要求本帧确实有足够观测。
  const bool obs_valid = last_n_effect_ >= g_lost_min_effect_pts;
  const bool converged = obs_valid &&
      (corr_p <= g_trust_corr_max && corr_r <= g_trust_corr_rot_max);
  lost_recover_streak_ = converged ? (lost_recover_streak_ + 1) : 0;

  if(converged && lost_recover_streak_ >= g_lost_recover_frames){
    lost_ = false;
    lost_recover_streak_ = 0;
    lost_run_dt_ = 0.0;
    gicp_lost_src_.reset();
    gicp_lost_src_valid_ = false;
    rot_corr_hist_.clear();
    suspect_hold_dt_ = 0.0;
    stable_pose_ = T_post;
    stable_pose_valid_ = true;
    LOG(WARNING) << "[Trust] 连续 " << g_lost_recover_frames
                 << " 帧修正量收敛(" << corr_p << " m / " << corr_r
                 << " deg)，退出 LOST，恢复建图";
    return;
  }

  // ===== LOST 超时兜底 =====
  // 与 SUSPECT 的 D 对应：GICP 长期无法恢复（conv=0 / 源云与目标无重叠）时，
  // 不能让系统永久挂在 LOST 不建图。超时后强制接受本帧 Voxmap 观测并恢复建图。
  lost_run_dt_ += (cur_lidar_gap_ > 1e-3) ? cur_lidar_gap_ : 0.1;
  if(g_trust_lost_max_s > 0.0 && lost_run_dt_ >= g_trust_lost_max_s){
    const std::string detail = "LOST 保持 " + std::to_string(lost_run_dt_) + " s";
    force_accept_or_reanchor("LOST 已持续超过 " +
                             std::to_string(g_trust_lost_max_s) + " s", detail);
    return;
  }

  // 限速接近：p / R 各走一步，剩余残差留到下一帧继续收敛。
  // 但只有观测本身有效（有效点数足够）时才允许朝配准结果靠拢；配准无有效点时
  // 返回的位姿基本等于 IMU 先验，此时按 0.1 m/帧 slew 反而会持续"漏"位姿
  // （实测 LOST 期间位置以 ~0.5~1 m/s 匀速漂走，正是该机制）。
  SysState sys_fix = sys_pred;
  if(obs_valid){
    V3 dp = T_post.t_ - sys_pred.p;
    const double dp_n = dp.norm();
    if(dp_n > g_lost_slew_p && dp_n > 1e-9){
      dp *= (g_lost_slew_p / dp_n);
    }
    sys_fix.p = sys_pred.p + dp;

    V3 dr = BASIC::SO3(sys_pred.R.R().transpose() * T_post.R_).log_vee();
    const double dr_d = dr.norm() * kRad2Deg;
    if(dr_d > g_lost_slew_r && dr_d > 1e-9){
      dr *= (g_lost_slew_r / dr_d);
    }
    sys_fix.R = sys_pred.R * BASIC::SO3::Exp(dr);
  }
  // LOST 期间观测不可信，先验速度不应无条件继续积分（1.0 = 不衰减）。
  // 阻尼 + 硬封顶同时生效：姿态误差造成的重力泄漏约 5 m/s²，仅按 0.5 衰减
  // 仍会在每帧 IMU 传播后反弹（gicp_h 实测 6 帧从 0.06 泵到 3.4 m/s），
  // 因此再用 g_lost_vel_max 把 LOST 期速度钉在"drone 真实速度量级"内。
  if(g_lost_vel_damp < 1.0) sys_fix.v = sys_pred.v * g_lost_vel_damp;
  if(g_lost_vel_max > 0.0){
    const double vn_lost = sys_fix.v.norm();
    if(vn_lost > g_lost_vel_max) sys_fix.v *= (g_lost_vel_max / vn_lost);
  }
  kf_->RollbackToState(sys_fix, cov_pred);
  ++trust_reject_total_;

  const auto now = std::chrono::steady_clock::now();
  if(std::chrono::duration<double>(now - last_trust_log_wall_).count() > 5.0){
    last_trust_log_wall_ = now;
    LOG(WARNING) << "[Trust] LOST 保持: corr=" << corr_p << " m / " << corr_r
                 << " deg, streak=" << lost_recover_streak_ << "/"
                 << g_lost_recover_frames << ", 已持续 " << lost_run_dt_ << " s"
                 << "，本帧限速修正 "
                 << g_lost_slew_p << " m / " << g_lost_slew_r
                 << " deg，建图暂停，位姿输出不中断";
  }
}

/// Apply ESKF correction delta to deskewed clouds.
/// After Observe() corrects the pose, the deskewed cloud (which used
/// IMU-predicted trajectory) is slightly misaligned with the corrected body frame.
/// ΔT = T_corrected^{-1} * T_predicted  transforms deskewed points into the
/// corrected body frame, making them self-consistent with the corrected pose.
/// Also updates points_body_v3_ so UpdateMap() uses the corrected points.
void SuperLIO::ApplyDeltaCorrection() {
  // GICP 接管帧：点云仍是真实机体系，按修正后的位姿直接投放 world 才是正确几何。
  // 若再做 delta 修正，世界系位置反而会被"抵消"回 IMU 预测轨迹（见 T_pred=const 推导）。
  if (delta_passthrough_frame_) {
    delta_passthrough_frame_ = false;   // 只影响本帧
    return;
  }
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

  // 越界防护：Observe() 用点序号直接索引 effect_mask_[20000] / effect_knn_mask_[20000]，
  // 并且以同一序号访问 abcd_vec_ / effect_knn_idxs_（均按 20000 预分配）。
  // 点数一旦超限就是堆越界写，行为不可预测（静默错乱或卡死），因此在降采样出口
  // 统一封顶，让 Observe / UpdateMap 看到一致的点数。
  constexpr std::size_t kMaxObservePoints = 20000;
  if(ds_undistort_->size() > kMaxObservePoints){
    static long warn_cnt = 0;
    if(++warn_cnt % 50 == 1){
      LOG(WARNING) << "[DownSample] 单帧点数 " << ds_undistort_->size()
                   << " 超过配准上限 " << kMaxObservePoints << "，已截断（累计 "
                   << warn_cnt << " 帧）";
    }
    ds_undistort_->resize(kMaxObservePoints);
    ds_undistort_->width = static_cast<uint32_t>(kMaxObservePoints);
    ds_undistort_->height = 1;
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

  // M.32 时延侧车：本帧累加器清零。只在首轮累加（多轮迭代位姿不同，重复计入
  // 会把同一批点算成多份独立观测，人为放大信息量）。
  td_scc_frame_ = 0.0;
  td_scr_frame_ = 0.0;
  td_c2_raw_    = 0.0;
  td_n_raw_     = 0;
  // 时延雅可比 c = nb^T·(ω×p + v_body) 所需的运动量（体系角速度 + 体系速度）
  V3d td_omega = V3d::Zero();
  V3d td_v_body = V3d::Zero();
  if (g_calib_td_en) {
    const DynamicState ds = kf_->GetDynamicState();
    td_omega = ds.w.cast<double>();
    td_v_body = (ds.R.transpose() * ds.v).cast<double>();
  }

  effect_knn_num_ = ptsize;
  // Don't iota effect_knn_idxs_ yet — first iteration uses direct indexing

  for (size_t i = 0; i < ptsize; ++i) {
    const auto& point_body_pcl = ds_undistort_->points[i];
    points_body_v3_[i] = V3(point_body_pcl.x, point_body_pcl.y, point_body_pcl.z);
    _lengths[i] = points_body_v3_[i].norm();
  }
  // 开阔区门控所需的扫描尺度（复用上面的循环结果，O(n) 一次极值统计）
  {
    double rmax = 0.0, zmin = 1e9, zmax = -1e9;
    for (size_t i = 0; i < ptsize; ++i) {
      rmax = std::max(rmax, static_cast<double>(_lengths[i]));
      zmin = std::min(zmin, static_cast<double>(points_body_v3_[i].z()));
      zmax = std::max(zmax, static_cast<double>(points_body_v3_[i].z()));
    }
    scan_max_range_ = rmax;
    scan_z_span_ = (ptsize > 0) ? (zmax - zmin) : 0.0;
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
  double last_sum_w = 0.0;   // 最后一轮的平面点权重和（仅 P0.1 埋点，不参与估计）

  kf_->UpdateObserve([&, this](const ESKF::KFState &kf_state, M6 &HTVH, V6 &HTVr) {
    const SE3 pose = kf_state.pose;
    const bool need_converge = kf_state.need_converge;
    const M3d R_transpose = (pose.R_.transpose()).cast<double>();

    M6d sum_HTVH = M6d::Zero();
    V6d sum_HTVr = V6d::Zero();
    double sum_obs_w = 0.0;   // 本轮平面点 Huber 后权重和（仅埋点）
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
        effect_mask_[idx] = calc_plane_coeff(top_K.count, top_K.points_, abcd_vec_[idx], &fit_res,
                                             &resid_var_vec_[idx]);
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
        // 实验（对齐官方 dept_err/beam_err + plane_var_ 传播）：
        // σ²(r) = dept² + (beam·r)²，以 r = kBeamRefDist 标定到 g_obs_weight；
        // 再把拟合残差方差并入有效噪声 → 脏平面软降权，而不是二值丢弃。
        // 与上面 2.B 的 g_obs_dist_k 是同一族噪声模型，两者不应同时开。
        // g_beam_err 默认 0，此时整段跳过，行为与 2.B 实现完全一致。
        if (g_beam_err > 0.0) {
          const double r = static_cast<double>(_lengths[idx]);
          const double s2 = g_dept_err * g_dept_err + g_beam_err * g_beam_err * r * r;
          const double s2_ref = g_dept_err * g_dept_err +
                                g_beam_err * g_beam_err * kBeamRefDist * kBeamRefDist;
          w *= static_cast<float>(s2_ref / (s2 + resid_var_vec_[idx]));
        }
        sum_HTVH += J * w * J.transpose();
        sum_HTVr -= J * w * error;
        sum_obs_w += static_cast<double>(w);
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

        // M.32 时延侧车（首轮累加，用与主链路相同的权重 w）：
        //   c = ∂r/∂δ = nb^T·(ω×p + v_body)
        //   r(δ) ≈ r + c·δ  ⇒  δ_est = -Σc·r / Σc²
        // 只收 |r| 在阈值内的点：动态物体/错关联的残差与时延无因果关系，
        // 混进来会把估计拉向随机值。
        if (g_calib_td_en && std::abs(error) <= g_calib_td_outlier) {
          const double c_td = nb.dot(td_omega.cross(point_body_d) + td_v_body);
          td_scc_frame_ += w * c_td * c_td;
          td_scr_frame_ += w * c_td * static_cast<double>(error);
          // 未加权量只用于门控：加权和里的 w 含 g_obs_weight（默认 1000），
          // 拿它和绝对阈值比较会让门控形同虚设（任何帧都"有信息"）。
          td_c2_raw_ += c_td * c_td;
          ++td_n_raw_;
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
          effect_mask_[idx] = calc_plane_coeff(top_K.count, top_K.points_, abcd_vec_[idx], &fit_res,
                                                &resid_var_vec_[idx]);
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
        // 实验（对齐官方 dept_err/beam_err + plane_var_ 传播）：
        // σ²(r) = dept² + (beam·r)²，以 r = kBeamRefDist 标定到 g_obs_weight；
        // 再把拟合残差方差并入有效噪声 → 脏平面软降权，而不是二值丢弃。
        // 与上面 2.B 的 g_obs_dist_k 是同一族噪声模型，两者不应同时开。
        // g_beam_err 默认 0，此时整段跳过，行为与 2.B 实现完全一致。
        if (g_beam_err > 0.0) {
          const double r = static_cast<double>(_lengths[idx]);
          const double s2 = g_dept_err * g_dept_err + g_beam_err * g_beam_err * r * r;
          const double s2_ref = g_dept_err * g_dept_err +
                                g_beam_err * g_beam_err * kBeamRefDist * kBeamRefDist;
          w *= static_cast<float>(s2_ref / (s2 + resid_var_vec_[idx]));
        }
        sum_HTVH += J * w * J.transpose();
        sum_HTVr -= J * w * error;
        sum_obs_w += static_cast<double>(w);
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
    last_sum_w = sum_obs_w;

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

    // M.33 退化"响应"之三：沿**运动方向**截断。
    // 动机（drone bag 275 s）：漂移期间 deg_dir_v 0.90~1.00、平移块 cond_t 只有 3~6，
    // 6 维 cond 仅 10~45，既有 threshold=100 判据全程不触发 ⇒ 观测在"走的那个方向"
    // 上其实已经不可信，却仍以满权重参与更新，把位姿推着跑。
    // 判据：λ_v / λ_max(H_tt) < ratio（λ_v = v̂ᵀH_tt v̂，v̂ 为世界系速度方向）。
    if (g_degen_motion_dir_en) {
      const auto mds = kf_->GetDynamicState();
      V3d u = mds.v.cast<double>();
      const double vnorm = u.norm();
      if (vnorm > g_degen_motion_dir_vmin) {
        u /= vnorm;
        const Eigen::Matrix3d Htt = sum_HTVH.block<3, 3>(3, 3);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es_tt(Htt);
        if (es_tt.info() == Eigen::Success) {
          const double lmax = std::max(es_tt.eigenvalues()(2), 1e-12);
          const double ratio = u.dot(Htt * u) / lmax;
          if (ratio < g_degen_motion_dir_ratio) {
            M6d Pm = M6d::Identity();
            Pm.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() - u * u.transpose();
            const double tik = g_tikhonov_lambda * sum_HTVH.trace() / 6.0;
            sum_HTVH = Pm * sum_HTVH * Pm.transpose() + tik * M6d::Identity();
            sum_HTVr = Pm * sum_HTVr;
            static auto last_mt_log = std::chrono::steady_clock::now();
            const auto now_mt = std::chrono::steady_clock::now();
            if (debug && std::chrono::duration<double>(now_mt - last_mt_log).count() > 2.0) {
              LOG(WARNING) << "[MotionTrunc] ratio=" << std::fixed << std::setprecision(3)
                           << ratio << " |v|=" << vnorm;
              last_mt_log = now_mt;
            }
          }
        }
      }
    }

    // 旋转通道放权：只缩放 Hessian 的旋转块（等价于把旋转观测的 σ 缩小
    // sqrt(g_obs_rot_weight) 倍），平移通道不变。H 与 r 同步缩放，保证解不变号。
    if(g_obs_rot_weight > 0.0 && std::abs(g_obs_rot_weight - 1.0) > 1e-6){
      const double s = std::sqrt(g_obs_rot_weight);
      V6d S = V6d::Ones();
      S(0) = S(1) = S(2) = s;
      sum_HTVH = S.asDiagonal() * sum_HTVH * S.asDiagonal();
      sum_HTVr = S.cwiseProduct(sum_HTVr);
    }

    HTVH = sum_HTVH.cast<scalar>();
    HTVr = sum_HTVr.cast<scalar>();

    iter_num++;
  });

  // M.32 时延侧车结算：本帧观测已收敛，用首轮累加的 (Σc², Σc·r) 更新估计。
  // 放在退化检测之前不影响任何既有逻辑（只读本帧累加量，不写滤波状态）。
  if (g_calib_td_en) finalizeTimeDelayCalib(frame_omega_);

  // 进程内快照：本帧原始观测信息矩阵（与退化检测同源，未经截断），
  // 供单体内视觉支路（方案 B 的 lidarUpdate）直接读取，无需新增话题。
  {
    std::lock_guard<std::mutex> lk(snapshot_mutex_);
    snapshot_.has_info = true;
    snapshot_.info = last_HTVH;
  }

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

    // 进程内快照：退化签名（cond/axes/active/最弱方向），供单体视觉支路直读。
    {
      std::lock_guard<std::mutex> lk(snapshot_mutex_);
      snapshot_.ok = deg.ok;
      snapshot_.cond = deg.ok ? deg.cond : 0.0;
      snapshot_.axes = deg.ok ? deg.axes : 0;
      snapshot_.active = degeneracy_active_;
      snapshot_.dir_w = deg.ok ? BASIC::V3d(deg.eV.block<3, 1>(3, 0)) : BASIC::V3d::Zero();
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

  // 供 Trust 判据使用：本帧有效平面残差点数。n_effect==0 时"修正量为 0"只是
  // 因为没有任何约束参与，不能当成收敛（drone bag 275 s 后即因此假收敛退出 LOST）。
  last_n_effect_ = n_effect;

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
                    "deg_cond_t,deg_evt_min,deg_dir_v,deg_dir_g,deg_dir_z,"
                    // 地图新点比例：错误匹配的早期指标（上一帧 UpdateMap 统计）
                    "map_novelty_ratio,map_novelty_streak\n";
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

    // ===== P0.1 可观测性诊断（level>=2）：归一化信息矩阵全谱，仅记录不参与估计 =====
    // 布局 [rot(0-2), pos(3-5)]；S=diag(L,L,L,1,1,1) 把旋转列换算到米制，
    // 使 λ 与阈值具备统一物理量纲。H 为纯平面粗通道（不含细通道/截断）。
    if (g_debug_level >= 2) {
      if (!obs_csv_.is_open()) {
        namespace fs = std::filesystem;
        const std::string dir = g_save_map_dir.empty() ? std::string(".") : g_save_map_dir;
        std::error_code ec;
        fs::create_directories(dir, ec);
        obs_csv_.open(dir + "/observability.csv", std::ios::out | std::ios::trunc);
        obs_csv_ << std::setprecision(12);
        obs_csv_ << "t,feature_len,n_effect,sum_w,h_trace,h_mean_trace,"
                    "lam1,lam2,lam3,lam4,lam5,lam6";
        for (int c = 0; c < 6; ++c)
          for (int r = 0; r < 6; ++r) obs_csv_ << ",v" << (c + 1) << "_" << r;
        obs_csv_ << ",htt_e1,htt_e2,htt_e3,aabb_dx,aabb_dy,aabb_dz\n";
      }
      const double Lf = (g_debug_feature_length > 1e-6) ? g_debug_feature_length : 1.0;
      V6d sc;
      sc << Lf, Lf, Lf, 1.0, 1.0, 1.0;
      const M6d Hn = sc.asDiagonal() * last_HTVH * sc.asDiagonal();
      Eigen::SelfAdjointEigenSolver<M6d> eso(Hn);
      const bool ok6 = (eso.info() == Eigen::Success);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> est3(last_HTVH.block<3, 3>(3, 3));
      const bool ok3 = (est3.info() == Eigen::Success);
      // 有效点世界系 AABB 跨度（空间分布）；仅遍历本帧有效点
      double mn[3] = {1e18, 1e18, 1e18}, mx[3] = {-1e18, -1e18, -1e18};
      const size_t n_pts = std::min<size_t>(
          {static_cast<size_t>(ptsize), points_body_v3_.size(), 20000});
      for (size_t i = 0; i < n_pts; ++i) {
        if (!effect_mask_[i]) continue;
        const V3d pw = (state.R * points_body_v3_[i]).cast<double>() + state.p.cast<double>();
        for (int k = 0; k < 3; ++k) {
          mn[k] = std::min(mn[k], pw(k));
          mx[k] = std::max(mx[k], pw(k));
        }
      }
      obs_csv_ << state.timestamp << ',' << Lf << ',' << n_effect << ',' << last_sum_w << ','
               << last_HTVH.trace() << ','
               << (last_sum_w > 1e-9 ? last_HTVH.trace() / last_sum_w : 0.0);
      for (int i = 0; i < 6; ++i) obs_csv_ << ',' << (ok6 ? eso.eigenvalues()(i) : 0.0);
      for (int c = 0; c < 6; ++c)
        for (int r = 0; r < 6; ++r) obs_csv_ << ',' << (ok6 ? eso.eigenvectors()(r, c) : 0.0);
      for (int i = 0; i < 3; ++i) obs_csv_ << ',' << (ok3 ? est3.eigenvalues()(i) : 0.0);
      if (mx[0] > mn[0])
        obs_csv_ << ',' << (mx[0] - mn[0]) << ',' << (mx[1] - mn[1]) << ',' << (mx[2] - mn[2]) << '\n';
      else
        obs_csv_ << ",0,0,0\n";
      obs_csv_.flush();
    }

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
               << deg_dir_v << ',' << deg_dir_g << ',' << deg_dir_z << ','
               // 地图新点比例（上一帧 UpdateMap 统计）：错误匹配的早期指标。
               // 注意 debug 行在本帧 UpdateMap 之前写出，故这里是上一帧的值。
               << map_novelty_ratio_ << ',' << map_novelty_streak_ << '\n';
    debug_csv_.flush();
  }

  frame_num_++;
}


/// ===== M.32 在线时延估计（侧车）=====
/// 观测方程：世界系点 P = R(p_body + δ·(ω×p_body + v_body)) + t，
///           平面残差 r = n^T P + d，故 ∂r/∂δ = nb^T(ω×p + v) ≡ c。
/// 渐消最小二乘：S_cc ← λ·S_cc + Σ(w·c²)，S_cr ← λ·S_cr + Σ(w·c·r)，
///               δ_est = −S_cr/S_cc（w 与主链路观测同权，Huber 已压过外点）。
/// 注意 Σ(w·c·r) 的期望：若真实时延为 δ*，则 r_obs ≈ −c·δ*，
/// 故 Σ(w·c·r) ≈ −δ*·Σ(w·c²)，解得 δ_est ≈ δ*，符号自洽。
/// 门控与锁定：
///   1) 本帧 RMS(c) < sqrt(cc_gate) ⇒ 静止/低速帧，c≈0，硬解只有噪声，整帧不并入
///      （只做遗忘衰减）。这也是"必须运动才能可观"的实现层表达。
///      门控用未加权 Σc²/n（单位 m/s），不能用加权和：w 含 g_obs_weight=1000。
///   2) 信息帧数不足 20 前不出估计值，避免前几帧抖出离谱数字。
///   3) 最近 lock_win 个信息帧的估计极差 < lock_tol ⇒ LOCKED，此后冻结。
///   4) LOCKED 后仅在 apply_en 为真时写入 g_lidar_time_offset_extra；
///      默认只打印，人工确认后再生效（借鉴 Ultra-Fusion 的
///      apply_locked_result_to_slam=false 设计）。
void SuperLIO::finalizeTimeDelayCalib(double frame_omega_norm) {
  // 1) 遗忘衰减（无论本帧是否有信息都衰减，保证估计跟随最近的运动激励）
  td_scc_ *= g_calib_td_lambda;
  td_scr_ *= g_calib_td_lambda;

  const double c_rms2 = (td_n_raw_ > 0) ? (td_c2_raw_ / td_n_raw_) : 0.0;
  const bool informative = (td_n_raw_ > 0) && (c_rms2 >= g_calib_td_cc_gate);
  if (informative) {
    td_scc_ += td_scc_frame_;
    td_scr_ += td_scr_frame_;
    ++td_info_frames_;
  }

  // 2) 信息帧不足时不给估计（保持上一次的值）
  const int min_frames = 20;
  if (td_info_frames_ < min_frames || td_scc_ <= 0.0) {
    if (g_debug_level > 0) {
      LOG(INFO) << "[TdCalib] warmup info_frames=" << td_info_frames_ << "/"
                << min_frames << " c_rms=" << std::sqrt(c_rms2)
                << " frame_omega=" << frame_omega_norm;
    }
    return;
  }

  const double est = -td_scr_ / td_scc_;
  td_est_ = est;

  if (!td_locked_) {
    td_hist_.push_back(est);
    while (static_cast<int>(td_hist_.size()) > std::max(2, g_calib_td_lock_win)) {
      td_hist_.pop_front();
    }
    if (static_cast<int>(td_hist_.size()) >= std::max(2, g_calib_td_lock_win)) {
      const auto mm = std::minmax_element(td_hist_.begin(), td_hist_.end());
      const double range = *mm.second - *mm.first;
      if (range < g_calib_td_lock_tol) {
        td_locked_ = true;
        LOG(INFO) << "[TdCalib] LOCKED td=" << est * 1e3 << " ms (range="
                  << range * 1e3 << " ms over " << td_hist_.size()
                  << " frames, info_frames=" << td_info_frames_ << ")";
      }
    }
    if (g_debug_level > 0 && (td_info_frames_ % 50) == 0) {
      LOG(INFO) << "[TdCalib] est=" << est * 1e3 << " ms scc=" << td_scc_
                << " hist=" << td_hist_.size() << "/" << g_calib_td_lock_win
                << " info_frames=" << td_info_frames_
                << " frame_omega=" << frame_omega_norm;
    }
  }

  // 3) 锁定后按需应用到点时间戳（单位延迟反馈：本帧估出的值下一帧才用）
  if (td_locked_ && g_calib_td_apply_en && !td_applied_) {
    g_lidar_time_offset_extra.store(est);
    td_applied_ = true;
    LOG(INFO) << "[TdCalib] APPLIED td=" << est * 1e3
              << " ms → lidar_time_offset"
              << " (base=" << g_lidar_time_offset * 1e3 << " ms, effective="
              << (g_lidar_time_offset + est) * 1e3 << " ms, next frame onward)";
  }
}

/// 更新 IVox 地图：将去畸变后的点云添加到地图中
/// 分段计时：g_time_eva 时分别统计世界系变换 [MapTransform] 与地图插入 [MapInsert]。
/// 实测 [UpdateMap] 峰值 ~17.8 ms，是整条链路最大的单帧尖峰，需要定位是变换还是插入。
void SuperLIO::UpdateMap(bool write) {
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
  // 本帧点云按当前位姿投影后，落在"尚无粗体素"的点占比。
  // 这是错误匹配的早期指标：正常回访时地图已覆盖（比例低），
  // 而错误匹配会把整帧塞进空白区域（比例飙升）。写入暂停的帧也照常统计，
  // 否则 SUSPECT 期间指标会因"没有新点"而假性归零、无法持续判据。
  auto update_novelty = [&]() {
    const std::size_t n_new = ivox_ ? ivox_->countNewCells(points_world_v3_) : 0;
    map_novelty_ratio_ = (ptsize > 0)
        ? static_cast<double>(n_new) / static_cast<double>(ptsize) : 0.0;
  };
  auto insert_into_map = [this]() { ivox_->insert(points_world_v3_); };

  if (g_time_eva) {
    time_record_.Evaluate(transform_body_to_world, "[MapTransform]");
    update_novelty();
    if (write) time_record_.Evaluate(insert_into_map, "[MapInsert]");
  } else {
    transform_body_to_world();
    update_novelty();
    if (write) insert_into_map();
  }
}

/// 可信 Anchor 延迟提交。
/// 本帧点云先进入可撤销队列，只有"距离当前时刻超过 g_anchor_delay_commit_s
/// 且期间没有触发 SUSPECT/LOST"的帧才允许提交；提交后按 g_anchor_window_s 裁剪。
/// 这样即使渐进漂移已经持续数秒才被检测器发现，漂移期的点云也不会进入 Anchor。
void SuperLIO::updateTrustedAnchor(bool map_write) {
  if (g_anchor_delay_commit_s <= 0.0) return;          // 特性关闭：零开销
  // 冻结期不提交、不裁剪：接管期由 GICP 负责绝对参考，Anchor 保持冻结时刻的状态。
  if (anchor_frozen_) return;
  if (!map_write) {
    // 接管期的"暂停帧"（GICP 单帧失败）只是本帧不入队，不是信任异常，
    // 不能据此冻结 Anchor；否则接管期一次抖动就永久锁死绝对参考。
    if (!gicp_active_) freezeTrustedAnchor("map_write=false");
    return;
  }

  const double t_now = measures_.lidar.end_time;

  // GICP 需要足够的几何约束：沿 ds_undistort_（约 300 点）无法锁住平移，
  // 因此 Anchor 存稠密去畸变云按 g_gicp_target_voxel 降采样后的副本。
  if (g_anchor_gicp_en) {
    auto t0 = std::chrono::high_resolution_clock::now();
    // 稠密帧由 DenseThread 顺带产出（同一份 pc_full 的全量去畸变）后异步汇入，
    // 主线程不再每帧重复算一遍；仅当异步链路断流（拥塞/dense 暂停 >1 s）才补算。
    bool drained = false;
    {
      std::lock_guard<std::mutex> lock(anchor_inbox_mutex_);
      while (!anchor_dense_inbox_.empty()) {
        AnchorFrame fr = std::move(anchor_dense_inbox_.front());
        anchor_dense_inbox_.pop_front();
        // 活跃度判据用"到达"而非"收编"：被判重复说明异步链路是通的，不能因此补算。
        anchor_last_inbox_t_ = std::max(anchor_last_inbox_t_, fr.t);
        ++anchor_inbox_seen_;
        // 保序：补算路径可能已把同一帧入队，跳过时间戳不前进的异步帧
        if (!anchor_pending_.empty() && fr.t <= anchor_pending_.back().t) continue;
        anchor_pending_.push_back(std::move(fr));
        ++anchor_inbox_used_;
        drained = true;
      }
    }
    if (!drained) {
      // DenseThread 对同一帧的处理必然晚于本函数（enqueueDense 在本函数之后调用），
      // 所以"本帧 inbox 为空"是常态，不能据此在主线程补算：补算帧会让下一帧到达的
      // 异步帧被判为重复而丢弃，形成"永远 fallback"的自锁（实测 fallback=1166、
      // inbox=0）。只有 inbox 断流超过 1 s（DenseThread 卡住/暂停）才现场补算保底。
      ++anchor_missed_;
      const bool starved = (t_now - anchor_last_inbox_t_ > 1.0);
      if (starved) {
        ++anchor_fallback_hits_;
        ensureGicpDense();
        if (gicp_dense_valid_ && gicp_scan_dense_ && !gicp_scan_dense_->empty() &&
            (anchor_pending_.empty() || t_now > anchor_pending_.back().t)) {
          AnchorFrame f;
          f.t = t_now;
          f.pose = kf_->GetSE3();
          f.body.reset(new PointCloudType());
          voxelDownsampleCapped(gicp_scan_dense_, f.body, g_gicp_target_voxel,
                                g_gicp_anchor_body_max_pts);
          anchor_pending_.push_back(std::move(f));
        }
      }
    }
    if (data_wrapper_) {
      const double ms = std::chrono::duration<double, std::milli>(
          std::chrono::high_resolution_clock::now() - t0).count();
      data_wrapper_->recordLatency("[Anchor]Update", ms);
    }
  } else {
    if (!ds_undistort_ || ds_undistort_->empty()) return;
    AnchorFrame f;
    f.t = t_now;
    f.pose = kf_->GetSE3();
    f.body.reset(new PointCloudType(*ds_undistort_));
    anchor_pending_.push_back(std::move(f));
  }

  // 超过确认窗口 → 提交
  const double cut = t_now - g_anchor_delay_commit_s;
  while (!anchor_pending_.empty() && anchor_pending_.front().t <= cut) {
    anchor_committed_.push_back(std::move(anchor_pending_.front()));
    anchor_pending_.pop_front();
    ++anchor_commit_total_;
  }

  // 保留窗口裁剪
  if (g_anchor_window_s > 0.0) {
    while (!anchor_committed_.empty() &&
           t_now - anchor_committed_.front().t > g_anchor_window_s) {
      anchor_committed_.pop_front();
    }
  }

  if (anchor_last_log_t_ < 0.0 || t_now - anchor_last_log_t_ > 5.0) {
    anchor_last_log_t_ = t_now;
    LOG(INFO) << "[Anchor] t=" << t_now << " committed=" << anchor_committed_.size()
              << " pending=" << anchor_pending_.size()
              << " (total_commit=" << anchor_commit_total_
              << ", dropped=" << anchor_drop_total_
              << ", inbox=" << anchor_inbox_used_
              << "/" << anchor_inbox_seen_
              << ", missed=" << anchor_missed_
              << ", fallback=" << anchor_fallback_hits_ << ")";
  }
}

/// 冻结可信 Anchor：作废所有未提交帧。冻结后不再接收新帧，
/// 直到重定位/切换逻辑显式解冻（后续阶段实现）。
void SuperLIO::freezeTrustedAnchor(const char* why) {
  if (anchor_frozen_) return;
  // 已提交不足 5 帧时冻结没有意义：buildGicpTarget() 本来就会拒绝构图，
  // 而冻结后不再接收新帧，等于把异常期唯一的绝对观测通道永久废掉。
  // 实测（gicp_h 末段）重锚清空 Anchor 后立刻再入 SUSPECT，freeze 把
  // committed=0 锁死，之后每次重锚都重复同一死局、GICP 再也建不起目标图。
  // 这里只丢弃过期 pending，保持 Anchor 可继续积累。
  if (anchor_committed_.size() < 5) {
    const long n_drop = static_cast<long>(anchor_pending_.size());
    anchor_drop_total_ += n_drop;
    anchor_pending_.clear();
    {
      std::lock_guard<std::mutex> lock(anchor_inbox_mutex_);
      anchor_dense_inbox_.clear();
    }
    if (n_drop > 0) {
      LOG(WARNING) << "[Anchor] 推迟冻结 (" << why << ")：已提交仅 "
                   << anchor_committed_.size() << " 帧，丢弃 pending=" << n_drop;
    }
    return;
  }
  anchor_frozen_ = true;
  const long n_drop = static_cast<long>(anchor_pending_.size());
  anchor_drop_total_ += n_drop;
  anchor_pending_.clear();
  {   // 冻结后到达的异步稠密帧一律作废，避免解冻时混入冻结前的旧帧
    std::lock_guard<std::mutex> lock(anchor_inbox_mutex_);
    anchor_dense_inbox_.clear();
  }
  LOG(WARNING) << "[Anchor] FROZEN (" << why << ") at lidar_end="
               << measures_.lidar.end_time << " s: dropped_pending=" << n_drop
               << ", committed=" << anchor_committed_.size()
               << ", total_dropped=" << anchor_drop_total_;
}

/// 惰性构建 GICP 目标地图：把 Anchor 已提交帧按各自可信位姿投到 world，
/// 以最近已提交帧为中心裁剪半径，再体素降采样 + 建 KdTree。
/// 仅在首次异常帧调用一次；之后目标冻结不变（Anchor 冻结后也不再增长）。
bool SuperLIO::buildGicpTarget() {
  if (gicp_target_ready_) return true;
  if (!g_anchor_gicp_en) return false;
  if (anchor_committed_.size() < 5) {
    LOG(WARNING) << "[GICP] 已提交 Anchor 帧不足 (" << anchor_committed_.size()
                 << ")，暂无法构建目标地图";
    return false;
  }

  const V3 center = anchor_committed_.back().pose.t_;
  const double r2 = g_gicp_target_radius * g_gicp_target_radius;

  // 基准帧抽稀：只取最近 window_s 内的已提交帧，并按 keyframe_dt（约 1 s）
  // 取稠密帧。不抽帧时 10 Hz×40 s≈350 帧全拼，raw 点数十万、KdTree 与
  // GICP 内部协方差计算都被拖慢；1 Hz×10 s≈10 帧已足够覆盖局部几何。
  const double t_latest = anchor_committed_.back().t;
  pcl::PointCloud<pcl::PointXYZI> raw;
  raw.reserve(static_cast<size_t>(std::max(1, g_gicp_target_max_pts)));
  int n_sel = 0;
  double last_sel_t = 1e18;
  for (auto it = anchor_committed_.rbegin(); it != anchor_committed_.rend(); ++it) {
    const AnchorFrame& f = *it;
    if (g_gicp_target_window_s > 0.0 && t_latest - f.t > g_gicp_target_window_s) break;
    if (!f.body || f.body->empty()) continue;
    if (g_gicp_target_keyframe_dt > 0.0 && last_sel_t - f.t < g_gicp_target_keyframe_dt) continue;
    last_sel_t = f.t;
    ++n_sel;
    const M3 R = f.pose.R_.matrix();
    const V3 t = f.pose.t_;
    for (const auto& p : f.body->points) {
      const V3 q = R * V3(p.x, p.y, p.z) + t;
      if ((q - center).squaredNorm() > r2) continue;
      pcl::PointXYZI o;
      o.x = q.x(); o.y = q.y(); o.z = q.z(); o.intensity = p.intensity;
      raw.push_back(o);
    }
  }
  if (n_sel <= 0 || raw.size() < 1000) {
    LOG(WARNING) << "[GICP] Anchor 目标点不足 (frames=" << n_sel
                 << " raw=" << raw.size() << ")，放弃";
    return false;
  }

  gicp_target_.reset(new PointCloudType());
  // 拼接后点数按 g_gicp_target_max_pts 封顶（超限时自动放大体素），
  // 控制 KdTree 规模与单帧查询开销（目标只建一次，但仍要保证 GICP 单帧可实时）。
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_ptr = raw.makeShared();
  voxelDownsampleCapped(raw_ptr, gicp_target_, g_gicp_target_voxel,
                        g_gicp_target_max_pts);

  gicp_target_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  gicp_target_kdtree_->setInputCloud(gicp_target_);
  gicp_target_ready_ = true;
  gicp_target_build_t_ = measures_.lidar.end_time;
  LOG(INFO) << "[GICP] 可信 Anchor 目标地图建立: committed=" << anchor_committed_.size()
            << " selected=" << n_sel << " raw=" << raw.size()
            << " target=" << gicp_target_->size()
            << " (voxel=" << g_gicp_target_voxel << " m, radius="
            << g_gicp_target_radius << " m, window=" << g_gicp_target_window_s
            << " s/" << g_gicp_target_keyframe_dt << " s dt, center="
            << center.transpose() << ")";
  return true;
}

/// 最近 3~5 帧 GICP 位置对时间做加权最小二乘拟合（等权）：
///   v = Σ (t_i-t̄)(p_i-p̄) / Σ (t_i-t̄)²
/// 不足 3 帧时退回零速（配合协方差膨胀，让后续观测快速纠正）。
BASIC::V3 SuperLIO::fitGicpVelocity() const {
  const int n = static_cast<int>(gicp_pos_hist_.size());
  if (n < 3) return V3::Zero();
  double t_mean = 0.0;
  V3 p_mean = V3::Zero();
  for (const auto& e : gicp_pos_hist_) { t_mean += e.first; p_mean += e.second; }
  t_mean /= n;
  p_mean /= n;
  V3 num = V3::Zero();
  double den = 0.0;
  for (const auto& e : gicp_pos_hist_) {
    const double dt = e.first - t_mean;
    num += dt * (e.second - p_mean);
    den += dt * dt;
  }
  if (den < 1e-6) return V3::Zero();
  return num / den;
}

/// 接管期 → 交还 Voxmap。
/// Observe() 已算完本帧 Voxmap 修正（ESKF 当前状态就是 T_post），这里用三条
/// 独立证据判断"退化已结束"：
///   ① 观测有效：有效平面残差点数足够（n_effect==0 时 corr 恒 0，会伪装成健康）
///   ② 修正量小：Voxmap 相对 IMU 先验的修正 < 阈值（拖歪期实测 1~2 m / 5~15 deg）
///   ③ 与 GICP 一致：本帧 GICP 原始解与 Voxmap 解位姿差 < 阈值（两套独立观测交叉校验）
/// ①②③ 同时成立才计数，连续 g_gicp_handback_frames 次即交还：清空接管状态、
/// 解冻 Anchor、恢复 delta 修正与正常建图。只靠"连续失败才退出"会把 GICP
/// 一直挂着跑，既费时又得不到 Voxmap 的全局约束。
bool SuperLIO::tryHandbackToVoxmap(double corr_p, double corr_r) {
  if (!gicp_active_ || g_gicp_handback_frames <= 0) return false;
  if (!gicp_last_raw_valid_) return false;

  const BASIC::SE3 T_vox = kf_->GetSE3();
  const bool obs_ok   = last_n_effect_ >= g_lost_min_effect_pts;
  const bool corr_ok  = corr_p <= g_gicp_handback_corr_p &&
                        corr_r <= g_gicp_handback_corr_r;
  const double agree_p = (gicp_last_raw_pose_.t_ - T_vox.t_).norm();
  const double agree_r = BASIC::SO3(
      gicp_last_raw_pose_.R_.transpose() * T_vox.R_).log_vee().norm() * 57.29577951308232;
  const bool agree_ok = agree_p <= g_gicp_handback_agree_p &&
                        agree_r <= g_gicp_handback_agree_r;
  // ④ 地图覆盖：新点比例低说明当前扫描确实落在已建图区域（见 kHandbackNoveltyMax）。
  const bool nov_ok = map_novelty_ratio_ <= kHandbackNoveltyMax;

  if (obs_ok && corr_ok && agree_ok && nov_ok) {
    ++gicp_handback_streak_;
  } else {
    gicp_handback_streak_ = 0;
  }
  if (gicp_handback_streak_ < g_gicp_handback_frames) return false;

  // ===== 交还：本帧按 Voxmap 结果继续（ESKF 已处于 T_post，无需回滚）=====
  gicp_active_ = false;
  gicp_ok_streak_ = 0;
  gicp_fail_streak_ = 0;
  gicp_handback_streak_ = 0;
  gicp_frame_counter_ = 0;
  gicp_pos_hist_.clear();
  gicp_last_pose_valid_ = false;
  gicp_last_raw_valid_ = false;
  gicp_hold_cycles_ = 0;
  // 刚完成一次健康交还，重新起算"保图放行"计数：下一次偶发放行不应被旧的
  // 放行记录误升级为硬重锚。
  novelty_release_cnt_ = 0;
  novelty_release_last_t_ = -1.0;
  // 种子保留：下一次异常冻结 Anchor 时仍可用它做初值重试
  // 旧目标仍被复用的 GICP 对象持有（shared_ptr 保活），新分配地址必然不同，
  // 不会触发"指针相同→跳过 setInputTarget"的误判。
  gicp_target_ready_ = false;
  gicp_target_.reset();
  gicp_target_kdtree_.reset();
  // Anchor 恢复流动：已提交帧都是冻结前的可信帧，保留；从本帧起重新积累。
  anchor_pending_.clear();
  {
    std::lock_guard<std::mutex> lock(anchor_inbox_mutex_);
    anchor_dense_inbox_.clear();
  }
  anchor_frozen_ = false;
  // 本帧按 Voxmap 结果写图（delta 修正照常执行）
  delta_passthrough_frame_ = false;
  stable_pose_ = T_vox;
  stable_pose_valid_ = true;
  rot_corr_hist_.clear();
  suspect_hold_dt_ = 0.0;
  lost_ = false;
  // 起算交还观察期（见 handback_probation_until_t_ 注释）
  handback_probation_until_t_ = measures_.lidar.end_time + kHandbackProbationS;
  handback_ref_pose_ = T_vox;
  handback_ref_valid_ = true;

  LOG(WARNING) << "[GICP] 交还 Voxmap：连续 " << g_gicp_handback_frames
               << " 次观测健康且与 GICP 一致 (corr_p=" << corr_p
               << " m corr_r=" << corr_r << " deg agree_p=" << agree_p
               << " m agree_r=" << agree_r << " deg effect=" << last_n_effect_
               << ")，恢复 Voxmap 观测与建图";
  return true;
}

/// 用冻结的可信 Anchor 做 GICP，得到独立的 world<-body 位姿观测并覆盖 ESKF 状态。
/// 与"拒绝 LiDAR 修正"不同：这里是把不再可信的 Voxmap 观测换成另一套配准观测。
/// 通过质量门才接受；否则返回 false，由调用方走原有回滚/SUSPECT 路径。
bool SuperLIO::tryGicpRelocalize(const SysState& sys_pred, const ESKF::COV& cov_pred,
                                 double vox_corr_p, double vox_corr_r,
                                 const char* why, bool* ran) {
  if (ran) *ran = false;
  gicp_check_valid_ = false;   // 本次复核未走到质量评估前，快照一律作废
  if (!g_anchor_gicp_en) return false;
  if (!anchor_frozen_ && anchor_committed_.empty()) {
    // 接管期 Anchor 被掏空 ⇒ 目标图再也建不出来，GICP 永远不会再成功。继续留在
    // 接管里只会把位姿永久钉死（gicp_am 实测 314 s 起冻结 130 s：位姿完全不动、
    // 新点比例 0.29→0.80、cond_yk 1e9、降频帧与拒绝帧都不写图）。这里退出接管、
    // 交回 Voxmap，至少保留一条观测通路。
    if (gicp_active_) {
      LOG(WARNING) << "[GICP] 接管期 Anchor 已空，无法构建目标图：退出接管，交回 Voxmap";
      gicp_active_ = false;
      gicp_pos_hist_.clear();
      gicp_last_pose_valid_ = false;
      gicp_last_raw_valid_ = false;
      gicp_fail_streak_ = 0;
      gicp_ok_streak_ = 0;
      gicp_hold_cycles_ = 0;
      gicp_target_ready_ = false;
      gicp_target_.reset();
      gicp_target_kdtree_.reset();
      anchor_pending_.clear();
      map_pause_frame_ = false;
    }
    return false;
  }

  // ===== 全局节流（rate / jump / lost / active 四条路径共用）=====
  // 降本后单帧 GICP 实测 66~99 ms（源云 ≤800、目标 ≤40k，BFGS 内层 3 次），
  // 已可在 10 Hz 下每 2 帧跑一次。此前 1.5 s 的墙钟下限是 0.3~0.7 s 时代的
  // 遗留值：它把接管期的绝对观测稀到每 1.5 s 一次，帧间只能靠 IMU 外推，
  // 而 IMU 速度在姿态/零偏误差下会自积分发散（gicp_h 实测 6 帧泵到 3.4 m/s）。
  // 被节流跳过的帧只返回 false，**不累计 fail_streak**。
  constexpr double kGicpMinWallIntervalS = 0.2;
  {
    const int min_frames = std::max(1, g_gicp_frame_interval);
    if (frame_num_ - gicp_last_run_frame_ < min_frames) return false;
    const auto t_wall_gate = std::chrono::steady_clock::now();
    if (gicp_last_run_wall_valid_ &&
        std::chrono::duration<double>(t_wall_gate - gicp_last_run_wall_).count() <
            kGicpMinWallIntervalS) {
      return false;
    }
    gicp_last_run_frame_ = frame_num_;
    gicp_last_run_wall_ = t_wall_gate;
    gicp_last_run_wall_valid_ = true;
    if (ran) *ran = true;   // 本帧确实做了（或即将做）一次独立配准复核
  }

  if (!scan_undistort_full_ || scan_undistort_full_->empty()) return false;
  if (!buildGicpTarget()) return false;

  // 配准源用稠密全量去畸变云：沿 ds_undistort_（约 300 点）在弱几何场景里
  // 对应关系不足以约束平移，GICP 解会被 IMU 预测位姿拖着走（实测 274~278 s
  // 平移被放大 6~20 倍仍未纠正）。gicp_scan_dense_ 是本帧 pc_full 的去畸变结果
  // （数万点），按 g_gicp_target_voxel 降采样并封顶 g_gicp_source_max_pts。
  // LOST 期间冻结源云：此时速度被错误外推，每帧去畸变都把点云沿运动方向剪切成
  // 非刚体，体素重滤后点数持续膨胀（实测 1270→4621），对应关系越来越差，
  // 最终 conv=0 死循环。这里固定用进入 LOST 时的首帧快照配准，退出 LOST 时释放。
  if (lost_ && gicp_lost_src_valid_ && gicp_lost_src_ && !gicp_lost_src_->empty()) {
    gicp_source_ = gicp_lost_src_;
  } else {
    ensureGicpDense();   // 主线程不再每帧预生成全量去畸变云，这里按需补算
    BASIC::CloudPtr src_raw = (g_gicp_dense_en && gicp_dense_valid_)
        ? gicp_scan_dense_ : scan_undistort_full_;
    if (!src_raw || src_raw->empty()) return false;
    if (!gicp_source_) gicp_source_.reset(new PointCloudType());
    voxelDownsampleCapped(src_raw, gicp_source_, g_gicp_target_voxel,
                          g_gicp_source_max_pts);
    if (gicp_source_->size() < 50) return false;
    if (lost_) {
      gicp_lost_src_.reset(new PointCloudType(*gicp_source_));
      gicp_lost_src_valid_ = true;
    }
  }

  const double t_now = measures_.lidar.end_time;
  const auto t_wall = std::chrono::steady_clock::now();
  const bool do_log =
      std::chrono::duration<double>(t_wall - last_gicp_log_wall_).count() > 1.0;

  // 初值 = IMU 预测位姿。此刻 ds_undistort_ 仍在预测机体系（delta 修正尚未执行），
  // 因此 GICP 输出即 world<-body，可直接注入滤波器。
  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  guess.block<3, 3>(0, 0) = T_predicted_end_.R_.matrix().cast<float>();
  guess.block<3, 1>(0, 3) = T_predicted_end_.t_.cast<float>();

  // 退出接管后的重试：此时 IMU 位姿可能已被错误 Voxmap 拖离 Anchor 数米，
  // 直接当 GICP 初值会让 1.5 m 粗配半径内一个对应都找不到（conv=0、1~5 ms）。
  // Anchor 仍冻结说明绝对参考没有移动，用最后一次可信 GICP 位姿作初值，
  // 才能把位姿拉回 Anchor 所在位置（"回正常位姿继续建图"）。
  constexpr double kGicpSeedMaxAgeS = 30.0;
  bool seed_guess_used = false;
  if (anchor_frozen_ && gicp_seed_valid_ && !gicp_active_ &&
      (t_now - gicp_seed_t_) <= kGicpSeedMaxAgeS) {
    const double dp_seed = (gicp_seed_pose_.t_ - T_predicted_end_.t_).norm();
    const double dr_seed =
        BASIC::SO3(gicp_seed_pose_.R_.transpose() * T_predicted_end_.R_)
            .log_vee().norm() * 57.29577951308232;
    if (dp_seed > 0.30 || dr_seed > 5.0) {
      guess.block<3, 3>(0, 0) = gicp_seed_pose_.R_.matrix().cast<float>();
      guess.block<3, 1>(0, 3) = gicp_seed_pose_.t_.cast<float>();
      seed_guess_used = true;
      LOG(WARNING) << "[GICP] 初值改用上次可信位姿（种子）：与 IMU 预测差 "
                   << dp_seed << " m / " << dr_seed << " deg（锚龄 "
                   << (t_now - gicp_seed_t_) << " s）";
    }
  }

  // 复用一个 GICP 对象：target 协方差只算一次；setInputSource 每帧都会重置
  // source 协方差并重算，不能省略。
  static pclomp::GeneralizedIterativeClosestPoint<BASIC::PointType, BASIC::PointType> gicp;
  static const void* gicp_target_set = nullptr;
  if (gicp_target_set != static_cast<const void*>(gicp_target_.get())) {
    gicp.setInputTarget(gicp_target_);
    gicp_target_set = static_cast<const void*>(gicp_target_.get());
  }
  gicp.setInputSource(gicp_source_);
  // 初值来自 IMU 预测（误差通常 < 0.3 m / 5 deg），不需要 PCL 默认的 14×20
  // 次迭代；实测 14×20 单帧 200~1800 ms，节点滞后 30~60 s。目标图改为
  // 10 s/1 Hz 抽帧后进一步降到 4×3，配合源云 ≤800 点把单次压到 300 ms 内。
  gicp.setMaximumIterations(4);
  gicp.setMaximumOptimizerIterations(3);
  gicp.setCorrespondenceRandomness(20);

  // 两级对应距离：1.5 m 粗配 → 0.2 m 精配。
  // 粗配半径必须覆盖"进入本通道时的先验误差"：速率门/跳变门触发的那一刻，
  // IMU 先验与真值的差已经接近门限（跳变门 1.0 m、实测 1.06 m），若粗配半径
  // 只有 0.5 m，则内部一个对应都找不到 → final_transformation 保持初值 →
  // 日志呈现 conv=0 inlier=0/1312 dpos=0（gicp_f 从 225 s 起 50 次全如此）。
  const auto t_align0 = std::chrono::steady_clock::now();
  bool converged = false;
  Eigen::Matrix4f T = guess;
  pcl::PointCloud<BASIC::PointType> aligned;
  const float kStageCorr[2] = {1.5f, 0.2f};
  for (int stage = 0; stage < 2; ++stage) {
    gicp.setMaxCorrespondenceDistance(kStageCorr[stage]);
    aligned.clear();
    gicp.align(aligned, T);
    converged = gicp.hasConverged();
    T = gicp.getFinalTransformation();
    if (!converged) break;
  }
  const double align_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_align0)
          .count();

  // ===== 一次性诊断：GICP 首次 conv=0 时定位根因 =====
  // 用 guess 变换源点后直接查目标 KdTree（不经过 GICP 内部协方差/BFGS）：
  //   - 最近邻分布整体 ≫ 粗配半径 ⇒ 源/目标不在同一坐标系或初值错位；
  //   - 距离很小却 conv=0     ⇒ 内部协方差/优化器数值失效（NaN/退化）。
  {
    static bool gicp_diag_done = false;
    if (!gicp_diag_done && !converged) {
      gicp_diag_done = true;
      auto finite_bbox = [](const BASIC::CloudPtr& c, int& nf,
                            Eigen::Vector3d& lo, Eigen::Vector3d& hi) {
        nf = 0;
        lo.setConstant(1e9);
        hi.setConstant(-1e9);
        for (const auto& p : c->points) {
          if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
          ++nf;
          lo = lo.cwiseMin(p.getVector3fMap().cast<double>());
          hi = hi.cwiseMax(p.getVector3fMap().cast<double>());
        }
      };
      int nfs = 0, nft = 0;
      Eigen::Vector3d slo, shi, tlo, thi;
      finite_bbox(gicp_source_, nfs, slo, shi);
      finite_bbox(gicp_target_, nft, tlo, thi);
      std::vector<float> nd;
      nd.reserve(gicp_source_->size());
      int n15 = 0, n03 = 0, n01 = 0;
      Eigen::Vector3d wlo = Eigen::Vector3d::Constant(1e9);
      Eigen::Vector3d whi = Eigen::Vector3d::Constant(-1e9);
      std::vector<int> id(1);
      std::vector<float> d2(1);
      for (const auto& p : gicp_source_->points) {
        const Eigen::Vector4f q = guess * Eigen::Vector4f(p.x, p.y, p.z, 1.0f);
        BASIC::PointType qp;
        qp.x = q.x(); qp.y = q.y(); qp.z = q.z();
        wlo = wlo.cwiseMin(Eigen::Vector3d(q.x(), q.y(), q.z()));
        whi = whi.cwiseMax(Eigen::Vector3d(q.x(), q.y(), q.z()));
        if (gicp_target_kdtree_->nearestKSearch(qp, 1, id, d2) < 1) continue;
        const float d = std::sqrt(d2[0]);
        nd.push_back(d);
        if (d < 1.5f) ++n15;
        if (d < 0.3f) ++n03;
        if (d < 0.1f) ++n01;
      }
      std::sort(nd.begin(), nd.end());
      auto pct = [&](double q) {
        return nd.empty() ? -1.0
                          : static_cast<double>(nd[std::min(nd.size() - 1,
                                                            static_cast<size_t>(q * (nd.size() - 1)))]);
      };
      LOG(WARNING) << "[GICP][DIAG] 首次 conv=0: src=" << gicp_source_->size()
                   << " finite=" << nfs << " bbox=[" << slo.transpose() << "]..["
                   << shi.transpose() << "] | tgt=" << gicp_target_->size()
                   << " finite=" << nft << " bbox=[" << tlo.transpose() << "]..["
                   << thi.transpose() << "] | guess_t=" << T_predicted_end_.t_.transpose()
                   << " | src_w(guess) bbox=[" << wlo.transpose() << "]..[" << whi.transpose()
                   << "] | NN(guess源→目标) n=" << nd.size() << " p50=" << pct(0.5)
                   << " p95=" << pct(0.95) << " max=" << pct(1.0) << " <1.5m=" << n15
                   << " <0.3m=" << n03 << " <0.1m=" << n01
                   << " | dense_valid=" << gicp_dense_valid_ << " dense="
                   << (gicp_scan_dense_ ? gicp_scan_dense_->size() : 0UL)
                   << " undistort_full="
                   << (scan_undistort_full_ ? scan_undistort_full_->size() : 0UL);
    }
  }

  // ===== 质量指标：内点比例 / 米制 RMSE / 点到面 Hessian 特征值 =====
  const double inlier_d2 = 0.3 * 0.3;
  std::vector<int> nn_idx(1);
  std::vector<float> nn_d2(1);
  int n_in = 0;
  double sum_d2 = 0.0;
  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
  for (const auto& p : aligned.points) {
    if (gicp_target_kdtree_->nearestKSearch(p, 1, nn_idx, nn_d2) < 1) continue;
    if (nn_d2[0] > inlier_d2) continue;
    ++n_in;
    sum_d2 += nn_d2[0];
    BASIC::V3 nrm;
    if (!estimatePointNormal(gicp_target_, *gicp_target_kdtree_, nn_idx[0], 10, nrm)) continue;
    // 点到面残差 e = nᵀ(T p - q) 的雅可比 [ (p_w × n)ᵀ, nᵀ ]；
    // 旋转列按 5 m 尺度归一化，使 cond 只反映几何退化而非单位差异。
    const Eigen::Vector3d pw(p.x, p.y, p.z);
    const Eigen::Vector3d n = nrm.cast<double>();
    Eigen::Matrix<double, 1, 6> J;
    J.head<3>() = (pw.cross(n) / 5.0).transpose();
    J.tail<3>() = n.transpose();
    H += J.transpose() * J;
  }
  double eig_min = 0.0, cond = 0.0;
  if (n_in >= 6) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(H);
    eig_min = es.eigenvalues()(0);
    const double eig_max = es.eigenvalues()(5);
    cond = eig_max / std::max(eig_min, 1e-9);
  }

  // ===== 与 IMU 预测的修正量 =====
  const Eigen::Matrix3f Rg = T.block<3, 3>(0, 0);
  const Eigen::Vector3f tg = T.block<3, 1>(0, 3);
  const double dpos = (tg - T_predicted_end_.t_.cast<float>()).norm();
  const Eigen::Matrix3f dR = Rg.transpose() * T_predicted_end_.R_.matrix().cast<float>();
  const double drot_deg =
      std::abs(Eigen::AngleAxisf(dR).angle()) * 180.0 / static_cast<double>(M_PI);

  // ===== 连续帧一致性：相邻两次接受的 GICP 位姿变化不能超过物理可能 =====
  double inst_speed = 0.0, inst_rate = 0.0;
  if (gicp_last_pose_valid_) {
    const double dt = std::max(1e-3, t_now - gicp_last_t_);
    inst_speed = (tg - gicp_last_pose_.t_.cast<float>()).norm() / dt;
    const Eigen::Matrix3f dRl =
        Rg.transpose() * gicp_last_pose_.R_.matrix().cast<float>();
    inst_rate = std::abs(Eigen::AngleAxisf(dRl).angle()) * 180.0 /
                static_cast<double>(M_PI) / dt;
  }

  const double inlier_ratio =
      static_cast<double>(n_in) / static_cast<double>(gicp_source_->size());
  const double rmse = (n_in > 0) ? std::sqrt(sum_d2 / n_in) : 1e9;

  bool dpos_ok = dpos <= g_gicp_max_dpos;
  bool drot_ok = drot_deg <= g_gicp_max_drot;
  if (seed_guess_used) {
    // 用种子作初值 ⇒ IMU 预测本身已被错误 Voxmap 拖离 Anchor（gicp_ac 8.75 m、
    // gicp_ag 12.19 m），此时"与预测差"大正是"把位姿拉回可信地图"的证据，
    // 不能当拒绝理由（否则每次重定位都被否掉，最终落到清图重锚死循环）。
    // 真正该守的是：解不能离种子太远，否则说明匹配到了别的地方。
    const double dp_res = (tg - gicp_seed_pose_.t_.cast<float>()).norm();
    const Eigen::Matrix3f dRr = Rg.transpose() * gicp_seed_pose_.R_.matrix().cast<float>();
    const double dr_res =
        std::abs(Eigen::AngleAxisf(dRr).angle()) * 180.0 / static_cast<double>(M_PI);
    dpos_ok = dpos_ok || dp_res <= 2.0;
    drot_ok = drot_ok || dr_res <= 20.0;
  }

  bool ok = converged && n_in >= 200 &&
            inlier_ratio >= g_gicp_min_inlier_ratio &&
            rmse <= g_gicp_max_rmse &&
            dpos_ok && drot_ok;
  if (gicp_last_pose_valid_) {
    ok = ok && inst_speed <= 10.0 && inst_rate <= 180.0;
  }

  // 本次复核的质量快照：即使最终判拒，也把"配准解本身"留给信任守护判断
  // "位姿是否真的被 Voxmap 拖走"（inlier 低可能只是场景新覆盖，见 novelty_hold）。
  gicp_check_valid_ = true;
  gicp_check_converged_ = converged;
  gicp_check_n_in_ = n_in;
  gicp_check_rmse_ = rmse;
  gicp_check_dpos_ = dpos;
  gicp_check_drot_ = drot_deg;
  gicp_check_pose_ = BASIC::SE3(BASIC::SO3(Rg.cast<BASIC::scalar>()),
                                V3(tg.x(), tg.y(), tg.z()));

  if (!ok) {
    ++gicp_fail_streak_;
    gicp_ok_streak_ = 0;
    // 接管期的拒绝帧既不写实时地图、也不进可信 Anchor：本帧位姿只是 IMU 外推/
    // 回滚值（没有可信观测背书），按它写图会污染两套地图；更致命的是
    // updateTrustedAnchor 的窗口裁剪以 t_now 前进为准，拒绝帧一路把"旧的干净帧"
    // 裁掉、却不产生新的可信帧 —— gicp_aj 实测接管 20 s 内 committed 181→90、
    // 目标图只剩 8 帧 5093 点，inlier 从 0.45 单调掉到 0.12，此后 GICP 永远接不
    // 上，位姿退化为纯 IMU（0.25 m/s 平滑漂移，末端 25 m）。
    if (gicp_active_) map_pause_frame_ = true;
    if (do_log || (gicp_active_ && gicp_fail_streak_ >= g_gicp_fail_max)) {
      last_gicp_log_wall_ = t_wall;
      LOG(WARNING) << "[GICP] 拒绝(" << why << "): conv=" << converged
                   << " inlier=" << n_in << "/" << gicp_source_->size()
                   << " (" << inlier_ratio << ") rmse=" << rmse
                   << " m dpos=" << dpos << " m drot=" << drot_deg
                   << " deg eig_min=" << eig_min << " cond=" << cond
                   << " inst_v=" << inst_speed << " m/s fail_streak="
                   << gicp_fail_streak_ << " ms=" << align_ms;
    }
    if (gicp_active_ && gicp_fail_streak_ >= g_gicp_fail_max) {
      const int n_fail = gicp_fail_streak_;
      // 是否交还 Voxmap：effect_pts 只说明"在当前地图里找得到平面"，地图自身
      // 被错误位姿污染时它仍然很大（自洽但错误）。必须再叠加"Voxmap 相对 IMU
      // 先验修正小"与"与上一次可信 GICP 位姿一致"两条。gicp_z 19:02:24 正是
      // 仅凭 effect_pts=304 交还，随后 5 s 被拖走 4 m，Anchor 一并被污染，
      // 之后每次 GICP 都 conv=0，彻底失去恢复能力。
      const BASIC::SE3 T_vox_exit = kf_->GetSE3();
      const double agree_p_exit = gicp_seed_valid_
          ? (gicp_seed_pose_.t_ - T_vox_exit.t_).norm() : 1e9;
      const bool voxmap_trusted = last_n_effect_ >= g_lost_min_effect_pts &&
                                  vox_corr_p <= 0.10 && vox_corr_r <= 1.0 &&
                                  agree_p_exit <= 0.20 &&
                                  map_novelty_ratio_ <= kHandbackNoveltyMax;
      // 保持接管的轮数上限。原实现 3 轮后一律"清图重锚"放行，但重锚会丢掉
      // 唯一可用的绝对参考（Anchor），新图段从"钉住时的状态"重建：gicp_ai 实测
      // 01:06:28 第 3 轮触发重锚，5 s 后状态速度即 5.1 m/s、末段漂到 28 m。
      // 而 gicp_af（接管永不退出、从不重锚）同段只有 2.09 m。故这里把上限抬到
      // 实际不可达：只要 Voxmap 一直不可信就持续钉住并重试 GICP，绝不重锚。
      constexpr int kGicpHoldMaxCycles = 1000000;
      if (voxmap_trusted || gicp_hold_cycles_ >= kGicpHoldMaxCycles) {
        const bool hard_reanchor = !voxmap_trusted;
        gicp_active_ = false;
        gicp_pos_hist_.clear();
        gicp_last_pose_valid_ = false;
        // 目标图是基于"冻结时的 Anchor"构建的，接管期间 Anchor 若已解冻并积累
        // 新帧，旧目标会逐渐落后于当前区域（gicp_u 275→297 s 正是这样失败）。
        // 退出接管时作废，下次异常按最新 Anchor 帧重建。
        gicp_target_ready_ = false;
        gicp_target_.reset();
        gicp_target_kdtree_.reset();
        gicp_hold_cycles_ = 0;
        anchor_pending_.clear();
        {
          std::lock_guard<std::mutex> lock(anchor_inbox_mutex_);
          anchor_dense_inbox_.clear();
        }
        anchor_frozen_ = false;
        novelty_hold_ = false;
        map_pause_frame_ = false;
        map_novelty_streak_ = 0;
        map_novelty_run_dt_ = 0.0;
        novelty_ref_valid_ = false;
        novelty_release_cnt_ = 0;
        novelty_release_last_t_ = -1.0;
        if (hard_reanchor) {
          // Voxmap 不可信：以最后一次可信 GICP 位姿清图重锚，绝不接受当前
          // （已被拖离的）IMU 位姿，否则新地图整体偏移、后续再无绝对参考。
          // 种子必须新鲜（≤kNoveltySeedMaxAgeS）：陈旧种子会把位姿弹回几十秒前。
          const bool seed_fresh = gicp_seed_valid_ && gicp_seed_t_ > 0.0 &&
              (measures_.lidar.end_time - gicp_seed_t_) <= kNoveltySeedMaxAgeS;
          SysState s_anchor = seed_fresh ? gicp_seed_state_ : kf_->GetSysState();
          if (seed_fresh) {
            s_anchor.R = gicp_seed_pose_.R_;
            s_anchor.p = gicp_seed_pose_.t_;
          }
          s_anchor.v.setZero();
          kf_->RollbackToState(s_anchor, cov_pred);
          if (ivox_) ivox_->clear();
          anchor_committed_.clear();
          anchor_last_log_t_ = -1.0;
          gicp_seed_valid_ = false;
          gicp_handback_streak_ = 0;
          gicp_ok_streak_ = 0;
          gicp_fail_streak_ = 0;
          gicp_frame_counter_ = 0;
          gicp_last_raw_valid_ = false;
          gicp_lost_src_.reset();
          gicp_lost_src_valid_ = false;
          rot_corr_hist_.clear();
          suspect_hold_dt_ = 0.0;
          suspect_run_dt_ = 0.0;
          map_novelty_skip_until_t_ =
              measures_.lidar.end_time + kNoveltyGraceS + kNoveltyCooldownS;
          stable_pose_ = kf_->GetSE3();
          stable_pose_valid_ = true;
          ++reanchor_total_;
          LOG(WARNING) << "[GICP] 连续 " << n_fail
                       << " 帧失败且 Voxmap 不可信（effect_pts=" << last_n_effect_
                       << ", corr_p=" << vox_corr_p << " m, corr_r=" << vox_corr_r
                       << " deg, 与上次可信位姿差 " << agree_p_exit
                       << " m）：以最后一次可信 GICP 位姿清图重锚（累计 "
                       << reanchor_total_ << " 次）";
        } else {
          // 交还 Voxmap
          map_novelty_skip_until_t_ = measures_.lidar.end_time + kNoveltyCooldownS;
          // 种子保留（30 s 龄期内）：若后续新点比例/锯齿再次触发并冻结 Anchor，
          // 仍可从最后一次可信 GICP 位姿重试，而不是拿已被拖偏的 IMU 位姿当初值。
          // 交还瞬间必须清速度：接管期位姿由 GICP 直接写状态，速度来自拟合外推，
          // 与 Voxmap 观测的差值会被 ESKF 更新放大（gicp_v 实测交还后 2 s
          // |v|=5.37 m/s 撞速度硬门进 LOST，随后 Anchor 被冻结、committed 被
          // 窗口裁空，彻底失去恢复能力）。保留位姿、速度归零，运动由 IMU 重建。
          SysState s_hand = kf_->GetSysState();
          s_hand.v.setZero();
          kf_->RollbackToState(s_hand, cov_pred);
          stable_pose_ = kf_->GetSE3();
          stable_pose_valid_ = true;
          // 同 tryHandbackToVoxmap：起算交还观察期。注意本路径是"GICP 连续失败
          // 后被动交还"，独立性比 10 帧健康交还更弱（gicp_ah 47:35 即走此路径
          // 交还后立刻被拖走 9 m），观察期同样必须生效。
          handback_probation_until_t_ = measures_.lidar.end_time + kHandbackProbationS;
          handback_ref_pose_ = kf_->GetSE3();
          handback_ref_valid_ = true;
          LOG(WARNING) << "[GICP] 连续 " << n_fail
                       << " 帧失败，退出接管：Voxmap 观测可信（effect_pts="
                       << last_n_effect_ << ", corr_p=" << vox_corr_p
                       << " m, corr_r=" << vox_corr_r << " deg），解冻 Anchor "
                       << "并继续按 Voxmap 建图";
        }
      } else {
        // 保持接管：GICP 失败只是"源云暂时对不上 Anchor"，而 Voxmap 已被
        // 拖离（修正大/与最后一次可信位姿不一致）。此时把位姿钉在上一次可信
        // GICP 位姿、速度清零，并冻结 Anchor 防止错误位姿继续污染它；
        // 下一帧用该位姿作初值重试。
        gicp_fail_streak_ = 0;
        ++gicp_hold_cycles_;
        // 不冻结 Anchor：接管期写图已被 map_pause_frame_ 暂停，Anchor 不会被
        // 本次错误位姿污染；而冻结的代价是目标图停止 1.5 s 周期重建（重建只在
        // GICP 成功帧里触发），过期目标继续失败，形成
        // "冻结 → 目标过期 → 持续失败 → committed 老化清空" 死局：
        // gicp_ac 实测 19:51:57 冻结后 committed 350→0，19:53:11 交还时
        // Anchor 已空，随后被 Voxmap 沿退化方向拖走 25 m。
        // 这里只作废未提交窗口，保持 Anchor 继续积累，并强制下次重建目标图。
        const long n_drop = static_cast<long>(anchor_pending_.size());
        anchor_drop_total_ += n_drop;
        anchor_pending_.clear();
        gicp_target_ready_ = false;
        // 完整恢复种子状态（含零偏/重力）：只钉 R/p 而沿用当前跑飞的零偏时，
        // IMU 预积分仍会持续给出错误加速度（gicp_ab 实测 ba_z→0.73 m/s²，
        // 90 s 后 z 掉到 -78 m；ba_y→0.4）。
        SysState s_hold = gicp_seed_valid_ ? gicp_seed_state_ : kf_->GetSysState();
        if (gicp_seed_valid_) {
          s_hold.R = gicp_seed_pose_.R_;
          s_hold.p = gicp_seed_pose_.t_;
        }
        s_hold.v.setZero();
        kf_->RollbackToState(s_hold, cov_pred);
        stable_pose_ = kf_->GetSE3();
        stable_pose_valid_ = true;
        map_pause_frame_ = true;
        LOG(WARNING) << "[GICP] 连续 " << n_fail
                     << " 帧失败但 Voxmap 不可信（effect_pts=" << last_n_effect_
                     << ", corr_p=" << vox_corr_p << " m, corr_r=" << vox_corr_r
                     << " deg, 与上次可信位姿差 " << agree_p_exit
                     << " m）：保持接管、钉住位姿、作废未提交 Anchor 帧（丢弃 "
                     << n_drop << " 帧，第 " << gicp_hold_cycles_ << " 轮）";
      }
    }
    return false;
  }

  // ===== 接受：覆盖 ESKF 的 R/p/v =====
  // 抖动抑制：GICP 单帧输出实测有 0.1~0.4 m / 5~15 deg 抖动。直接把它注入
  // ESKF，下一帧就会按带抖动的速度去畸变，帧内点云被剪切成非刚体，反过来
  // 破坏 GICP 对应关系（正反馈）。这里只对"相对上一帧已接受位姿"的增量做
  // 限幅：真实低频运动保留，高频抖动压掉。降频后两次接受之间隔了 N 帧
  // （约 N×0.1 s），限幅量按帧间隔同比放大，否则会把真实运动当抖动削掉。
  constexpr double kGicpStepPos = 0.15;                        // m/(N 帧)
  // 姿态限幅 2° → 0.75°（interval=2 时实际 1.5°/帧）。实测接管期被接受的 GICP
  // 解 drot 达 3~15°/帧，按 2°/帧×2 放行后等效 20°/s 持续注入，ESKF 只能把它
  // 解释成陀螺零偏（gicp_am/an 的 bg 均为 0.09 rad/s ≈5°/s），姿态被虚假旋转
  // 拖走 → drot 越来越大，正反馈。真实低频运动（≤1.3 m/s、≤30°/s）仍能跟住。
  constexpr double kGicpStepRot = 0.75 * M_PI / 180.0;          // rad/(N 帧)
  // 限幅按帧间隔放大，但封顶 3 帧：interval 拉到 10 时若同比放大到 1.5 m，
  // 单次误配就能直接跨过质量门（max_dpos 2.0 m）把位姿拽走。
  const double step_scale = static_cast<double>(std::min(std::max(1, g_gicp_frame_interval), 3));
  const BASIC::SE3 T_gicp(BASIC::SO3(Rg.cast<BASIC::scalar>()), V3(tg.x(), tg.y(), tg.z()));
  gicp_last_raw_pose_ = T_gicp;
  gicp_last_raw_valid_ = true;

  // ===== 退化结束 → 主动交还 Voxmap =====
  // 三条证据见 tryHandbackToVoxmap 注释。满足即不注入 GICP，ESKF 保留本帧
  // Voxmap 结果（当前状态就是 T_post），delta 修正与建图照常进行。
  if (gicp_active_ && tryHandbackToVoxmap(vox_corr_p, vox_corr_r)) {
    return true;
  }

  // 种子初值下的"回正"解：位姿已被错误 Voxmap 拖走 8~12 m，这里要一次性把
  // 位姿拉回可信地图，不能按抖动限幅一刀刀切（0.15 m/帧 ⇒ 十几米要 8 s，
  // 期间位姿停在半路，反而是最危险的状态）。
  const bool snap_back = seed_guess_used && dpos > g_gicp_max_dpos;
  BASIC::SE3 T_apply = T_gicp;
  if (gicp_last_pose_valid_ && !snap_back) {
    V3 dp = T_gicp.t_ - gicp_last_pose_.t_;
    const double dpn = dp.norm();
    if (dpn > kGicpStepPos * step_scale) dp *= (kGicpStepPos * step_scale) / dpn;
    V3 dr = BASIC::SO3(gicp_last_pose_.R_.transpose() * T_gicp.R_).log_vee();
    const double drn = dr.norm();
    if (drn > kGicpStepRot * step_scale) dr *= (kGicpStepRot * step_scale) / drn;
    T_apply = BASIC::SE3(
        BASIC::SO3(gicp_last_pose_.R_.matrix() * BASIC::SO3::Exp(dr).matrix()),
        gicp_last_pose_.t_ + dp);
  }

  // 速度用"限幅后"的位置序列拟合（9 帧 ≈ 0.9 s），抖动已被压掉，
  // 避免 5 帧窗口里 0.3 m 抖动直接变成 3 m/s 的速度噪声。
  gicp_pos_hist_.emplace_back(t_now, T_apply.t_);
  while (gicp_pos_hist_.size() > 9) gicp_pos_hist_.pop_front();
  V3 v_fit = fitGicpVelocity();
  const double v_sp = v_fit.norm();
  if (v_sp > 2.0) v_fit *= 2.0 / v_sp;

  SysState sys = sys_pred;
  sys.R = T_apply.R_;
  sys.p = T_apply.t_;
  sys.v = v_fit;
  // 接管期 Voxmap 观测暂停 ⇒ bg 没有任何观测源。漂移段被污染的 bg（gicp_ap
  // debug：正常段 bgx≈0.065/bgy≈-0.022，接管入口 0.072/-0.059）让 IMU 预测姿态
  // 持续空转，drot 随钉住时长线性增长，越过 10° 门后整段被拒（gicp_ao 300~320 s
  // 拒绝帧 drot=15~37°）。GICP 是绝对姿态观测：自上次接受以来
  //   r_obs(GICP 实测旋转) − imu_acc(IMU 积分旋转) ≈ (bg_est − bg_true)·dt，
  // 可直接估计零偏误差。注意接管期每个失败/降频帧都把姿态钉回 gicp_last_pose_，
  // sys_pred 只前进一帧，必须用逐帧累积的 gicp_imu_rot_acc_ 而非单帧预测
  // （gicp_ap 第一版用单帧预测，把 −0.5·ω_true 积进 bg：bgx 0.065→0.127 发散）。
  if (gicp_last_pose_valid_) {
    const double dt_acc = t_now - gicp_last_t_;
    if (dt_acc > 0.5 && dt_acc < 30.0) {
      const V3 r_obs = BASIC::SO3(gicp_last_pose_.R_.transpose() * T_gicp.R_).log_vee();
      const V3 imu_acc =
          gicp_imu_rot_acc_ +
          BASIC::SO3(gicp_last_pose_.R_.transpose() * sys_pred.R.R()).log_vee();
      const V3 bg_err = (r_obs - imu_acc) / dt_acc;
      sys.bg -= 0.5 * bg_err.cwiseMin(0.02).cwiseMax(-0.02);
    }
  }
  gicp_imu_rot_acc_.setZero();
  kf_->SetMainState(sys);

  // 旧协方差是围绕 IMU 预测建立的，GICP 是独立绝对观测：重开 p/v/bg
  // （零偏不视为可信，交给后续观测纠正）。
  ESKF::COV cov = cov_pred;
  for (int i = 3; i < 6; ++i) cov(i, i) += 0.25;        // 0.5 m
  for (int i = 6; i < 9; ++i) cov(i, i) += 1.0;         // 1.0 m/s
  for (int i = 9; i < 12; ++i) cov(i, i) += 1e-4;       // 0.01 rad/s
  kf_->SetCov(cov);

  const bool was_active = gicp_active_;
  delta_passthrough_frame_ = true;   // 本帧点云按修正后位姿投放，不做 delta 修正
  stable_pose_ = kf_->GetSE3();
  stable_pose_valid_ = true;
  gicp_active_ = true;
  gicp_ok_streak_++;
  gicp_fail_streak_ = 0;
  gicp_last_pose_ = T_apply;   // 记录"已应用"位姿，下一帧限幅以它为基准
  gicp_last_t_ = t_now;
  gicp_last_pose_valid_ = true;
  // 本次解已通过质量门，作为退出接管后重试的初值种子保留
  gicp_seed_pose_ = T_apply;
  gicp_seed_t_ = t_now;
  gicp_seed_state_ = kf_->GetSysState();
  gicp_seed_valid_ = true;
  gicp_hold_cycles_ = 0;
  const bool recover = lost_;
  lost_ = false;
  lost_recover_streak_ = 0;
  lost_run_dt_ = 0.0;
  gicp_lost_src_.reset();
  gicp_lost_src_valid_ = false;
  suspect_hold_dt_ = 0.0;
  rot_corr_hist_.clear();
  // 只在"首次进入接管"时冻结 Anchor。接管期间的每一帧 GICP 成功都重新冻结，
  // 会让 Anchor 永远停在触发时刻、目标图不随飞行更新，十几秒后源云出界，
  // 内点比例自然跌到门限以下（gicp_u 实测 275→297 s 后 0.70→0.40 退出接管）。
  if (!was_active && !anchor_frozen_) freezeTrustedAnchor("gicp_takeover");

  last_gicp_log_wall_ = t_wall;
  LOG(WARNING) << "[GICP] 接管" << (recover ? "(恢复 LOST)" : "") << "(" << why
               << "): inlier=" << n_in << "/" << gicp_source_->size()
               << " (" << inlier_ratio << ") rmse=" << rmse
               << " m dpos=" << dpos << " m drot=" << drot_deg
               << " deg eig_min=" << eig_min << " cond=" << cond
               << " | t_raw=" << T_gicp.t_.transpose() << " t=" << T_apply.t_.transpose()
               << " v=" << v_fit.transpose()
               << " |v|=" << v_fit.norm() << " m/s bg=" << sys.bg.transpose()
               << " ms=" << align_ms;
  return true;
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

    // 单体化钩子：所有官方输出发布完毕后，把同一帧数据推给派生类（默认空实现）
    onLioOutput(data);
  }
}

SuperLIO::DegeneracySnapshot SuperLIO::degeneracySnapshot() const {
  std::lock_guard<std::mutex> lk(snapshot_mutex_);
  return snapshot_;
}

void SuperLIO::printTimeRecord(){
  if(!g_time_eva) return;
  time_record_.PrintAll();
  data_wrapper_->printLatencies();
}


/// 周期任务：由 1Hz 定时器驱动。看门狗每拍都检查；性能统计仍按 60s 窗口打印。
void SuperLIO::periodicTimeReport(){
  watchdogCheck();

  // M.32 时延估计状态（开启时每分钟一行，不依赖 lio.eva.timer；关闭时无输出）
  if (g_calib_td_en) {
    std::cout << " > [ TdCalib ] est=" << td_est_ * 1e3 << " ms"
              << " scc=" << td_scc_
              << " info_frames=" << td_info_frames_
              << " locked=" << (td_locked_ ? 1 : 0)
              << " applied=" << (td_applied_ ? 1 : 0)
              << " effective_toff=" << (g_lidar_time_offset +
                                         g_lidar_time_offset_extra.load()) * 1e3
              << " ms" << std::endl;
  }
  if(!g_time_eva) return;
  if(++report_tick_count_ < 60) return;
  report_tick_count_ = 0;

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


/// 实时链路看门狗：心跳只在一帧完整走完（配准+建图+输出）后刷新，
/// 因此任何静默 return / 持锁卡死都会如实体现在 idle 时间上。
/// 本函数运行在独立回调组的定时器线程上，即使 process 线程卡在 ESKF 的持锁区里，
/// 也仍能照常执行 —— 这正是原实现缺失的可观测性。
void SuperLIO::watchdogCheck(){
  const auto now = std::chrono::steady_clock::now();
  if(!watchdog_armed_){
    watchdog_armed_ = true;
    last_ok_frame_wall_ = now;
    last_watchdog_log_wall_ = now;
    return;
  }

  const double idle_s = std::chrono::duration<double>(now - last_ok_frame_wall_).count();
  if(idle_s < 3.0){ return; }
  if(std::chrono::duration<double>(now - last_watchdog_log_wall_).count() < 10.0){ return; }
  last_watchdog_log_wall_ = now;

  // 逐段探测：try_lock 拿不到即说明有线程正卡在该段的持锁区里
  size_t dense_n = 0, save_n = 0;
  bool dense_locked = true, save_locked = true, dense_busy = false;
  {
    std::unique_lock<std::mutex> lk(dense_mutex_, std::try_to_lock);
    if(lk.owns_lock()){ dense_locked = false; dense_n = dense_queue_.size(); dense_busy = dense_busy_; }
  }
  {
    std::unique_lock<std::mutex> lk(save_mutex_, std::try_to_lock);
    if(lk.owns_lock()){ save_locked = false; save_n = save_queue_.size(); }
  }

  size_t lidar_n = 0, imu_n = 0;
  double last_imu = -1.0, last_lidar = -1.0;
  const bool got_buf = (data_wrapper_ != nullptr) &&
      data_wrapper_->getBufferStats(lidar_n, imu_n, last_imu, last_lidar);

  LOG(WARNING) << "[Watchdog] 已 " << idle_s << " s 未产出有效帧"
               << " | paused=" << (paused_.load() ? 1 : 0)
               << " frame_num=" << frame_num_
               << " | lidar_buf=" << (got_buf ? std::to_string(lidar_n) : std::string("lock"))
               << " imu_buf="   << (got_buf ? std::to_string(imu_n) : std::string("lock"))
               << " last_imu=" << (got_buf ? std::to_string(last_imu) : std::string("lock"))
               << " last_lidar=" << (got_buf ? std::to_string(last_lidar) : std::string("lock"))
               << " | dense_q=" << (dense_locked ? std::string("lock") : std::to_string(dense_n))
               << " dense_busy=" << (dense_busy ? 1 : 0)
               << " save_q=" << (save_locked ? std::string("lock") : std::to_string(save_n));
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