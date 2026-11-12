

#ifndef SUPER_LIO_H_
#define SUPER_LIO_H_

#include <queue>
#include <deque>
#include <vector>
#include <iostream>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>

#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "basic/alias.h"
#include "common/ds.h"
#include "common/timer.h"
#include "params.h"
#include "ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"
#include "OctVoxMap/VoxelGridFilter.h"

#include <opencv2/core.hpp>
#include "visual/visual_rotation.hpp"
#include "ros/ROSWrapper.h"

namespace LI2Sup{

class SuperLIO{
public:
  SuperLIO(){};
  ~SuperLIO();

  void setROSWrapper(const ROSWrapper::Ptr& wrapper){
    data_wrapper_ = wrapper;
  }
  virtual void init();
  void process();
  void saveMap();
  void printTimeRecord();
  /// 周期性性能报告：打印本窗口的统计后清空，供 lio.eva.timer 开启时每分钟调用
  void periodicTimeReport();

  void pauseProcessing();
  void resumeProcessing();
  bool isPaused() const;
  void resetIMUIntegration();

  /// 退化签名（供视觉侧做"按方向分配权限"，只读最近一帧结果）
  double degeneracyCond() const { return degeneracy_cond_; }
  int degeneracyAxes() const { return degeneracy_axes_; }
  bool degeneracyActive() const { return degeneracy_active_; }
  /// 平移块（世界系）最弱方向的条件数 / 最小特征值
  double degeneracyCondTrans() const { return degeneracy_cond_t_; }
  double degeneracyEvtMin() const { return degeneracy_evt_min_; }
  /// 平移块最弱方向（世界系单位向量）。H_tt 的平移行本就是世界系法向，
  /// 故该向量可直接与该帧的世界系运动方向比较，无需任何坐标变换。
  const BASIC::V3d& degeneracyDirWorld() const { return degeneracy_dir_w_; }

  /// 看门狗：由独立回调组的定时器 1Hz 调用，检测整条实时链路是否停摆。
  /// 与 process 线程解耦，因此即使 process 卡在持锁区里也能照常输出现场状态。
  void watchdogCheck();

  /// B2：图像输入。由 ROS 图像回调线程调用（独立回调组），内部持锁，
  /// KLT + 两帧几何在前端线程完成，不给 LIO 主线程增加耗时。
  void onVisualImage(const cv::Mat& image, double stamp);

protected:
  struct OutputData {
    NavState state;
    BASIC::V3 body_omega = BASIC::V3::Zero();  // body-frame angular velocity for twist
    BASIC::CloudPtr world_pc;
    BASIC::CloudPtr body_pc;
    bool has_world_pc = false;
    bool has_body_pc = false;
    bool is_undistort_only = false;
    double lidar_receive_time = 0.0;
    std::string lidar_frame;
  };

  struct SaveData {
    BASIC::CloudPtr cloud_to_save;
    int pcd_index;
    double timestamp;
    BASIC::V3 position;
    BASIC::Quat orientation;
  };

  // 全量点异步去畸变/变换的投递数据（错峰：不阻塞实时配准链路）
  struct DenseData {
    pcl::PointCloud<PointXTZIT>::Ptr pc_full;     // 全量原始点（含 offset_time，shared_ptr 共享）
    std::vector<DynamicState> propagate_states;   // IMU 传播轨迹（值拷贝，主线程下一帧会覆盖）
    std::vector<BASIC::V3> imu_gyr;               // 每个区间原始陀螺仪（去畸变 omega_body 用）
    double start_time = 0.0;                      // 去畸变起始时间（s）
    double timestamp = 0.0;                       // 状态时间戳（发布 dense 话题用）
    BASIC::SE3 T_pred;                            // 预测位姿（去畸变基准）
    BASIC::SE3 T_corrected;                       // 配准修正位姿（delta 修正用）
  };

  void stateWaitKFInit();
  void stateWaitMapInit();
  void stateProcess();
  virtual bool kf_init();
  virtual bool map_init();
  /// 实验 C：载入固定初始地图并跳过 map_init（g_map_load_path 非空时启用）
  bool loadFixedMap();
  void Propagation_Undistort();
  void undistortCloud(const pcl::PointCloud<PointXTZIT>::Ptr& raw_pc,
                      const std::vector<DynamicState>& propagate_states,
                      const std::vector<BASIC::V3>& imu_gyr,
                      double start_time,
                      const BASIC::SE3& T_end,
                      BASIC::CloudPtr& out_pc,
                      std::vector<float>* out_offsets = nullptr);
  void ApplyDeltaCorrection();
  void PublishBodyCloud();
  void DownSample();
  void DownSampleOnly();
  void Observe();
  /// B2：按参数构建视觉相对旋转前端（enable=false 时不创建对象，零开销）
  void buildVisualRotation();
  /// B2：把跨越本 lidar 帧窗口的相机相对旋转作为一路独立观测注入 ESKF
  void VisualObserve();
  /// B2：从后验位姿历史里取 t 时刻的机体姿态（线性插值/最近端点），供固定参考节点用
  bool anchorRotation(double t, double tol, BASIC::M3& R_out, double& t_used) const;
  virtual void UpdateMap();
  virtual void Output();
  void OutputThread();
  void SaveThread();
  void DenseThread();
  void enqueueDense();
  void caceSCPGOData();
  void ProcessCaceMap(const std::string& output_name, bool filtered);

  using StateFn = void (SuperLIO::*)();
  using OctVoxMapType = OctVoxMap<BASIC::V3, BASIC::scalar>;
  using KNNHeapType = KNNHeap<8, BASIC::V3>;
  StateFn state_fn_;
  ESKF::Ptr kf_;

  OctVoxMapType::Ptr ivox_;
  VoxelGridClosest<BASIC::PointType> voxel_grid_fliter_;
  ROSWrapper::Ptr data_wrapper_;
  MeasureGroup measures_;
  std::string current_lidar_frame_;
  
  bool flg_init_ = false;
  bool flg_first_scan_ = true;
  std::vector<DynamicState> propagate_states_;
  BASIC::SE3 T_predicted_end_;
  BASIC::CloudPtr scan_undistort_full_;
  BASIC::CloudPtr ds_undistort_;
  BASIC::CloudPtr point_map_;
  std::mutex map_mutex_;  // 保护 point_map_ / pcd_index_（DenseThread 与 saveMap 并发访问）
  int frame_num_ = 0;
  BASIC::SE3 sys_init_pose_;
  BASIC::SE3 last_pose_;

  std::size_t effect_knn_num_ = 0;
  BASIC::VV3 points_world_v3_, points_body_v3_;
  /// effect_mask_ / effect_knn_mask_ / abcd_vec_ 等定长数组的上限，
  /// 超过后 Observe 会截断 ptsize（见 P0-1）
  static constexpr std::size_t kMaxEffectPoints = 20000;
  alignas(64) bool effect_mask_[kMaxEffectPoints] = {false};
  alignas(64) bool effect_knn_mask_[kMaxEffectPoints] = {false};
  std::vector<int> effect_knn_idxs_;
  std::vector<std::pair<BASIC::M6, BASIC::V6>> H_R_;
  std::vector<std::array<double, 4>> abcd_vec_;
  /// 实验：平面拟合残差方差 Σd²/N（m²），与 abcd_vec_ 同长度、同步写入。
  /// 用于观测权重 w *= s2_ref/(s2 + resid_var)，抑制"脏平面"逐帧参考面漂移。
  std::vector<double> resid_var_vec_;
  int pcd_index_ = -1;

  // ===== 实验埋点（lio.debug.level > 0 时生效，不参与滤波）=====
  std::vector<BASIC::scalar> fit_res_vec_;
  std::ofstream debug_csv_;
  /// 本帧平均角速度模（rad/s），2.D 阈值随运动幅度自适应用
  double frame_omega_ = 0.0;
  /// 2.E：实时点云的逐点时刻（offset_time，s），与 scan_undistort_full_/ds_undistort_ 对齐
  std::vector<float> scan_offsets_full_;
  std::vector<float> scan_offsets_ds_;

  // ===== 退化检测 + 参数自适应 =====
  // 上一帧观测信息矩阵的退化签名（cond / 退化轴数 / 是否触发），供本帧参数自适应使用
  double degeneracy_cond_   = 0.0;
  int    degeneracy_axes_   = 0;
  bool   degeneracy_active_ = false;   // 去抖后的退化状态（响应机制都用它）
  int    degen_state_cnt_   = 0;       // 状态机连续帧计数
  // 平移块（世界系）签名：最弱方向 u_min 及条件数，供视觉侧按方向分配权限
  double degeneracy_cond_t_  = 0.0;
  double degeneracy_evt_min_ = 0.0;
  BASIC::V3d degeneracy_dir_w_ = BASIC::V3d::Zero();
  // 观测静态基线（init 时从配置缓存），供"退化档 ↔ 正常档"切换时恢复
  bool   base_normal_weight_en_ = false;
  double base_normal_sigma_      = 0.05;
  double base_plane_fit_         = 0.15;
  double base_huber_             = 0.05;
  // ===== 场景自动档（M.31）：启动窗口内累计高退化帧，满窗判定一次，全程固定 =====
  // 只在 Observe 线程访问，无需加锁。
  bool   scene_decided_   = false;   // 是否已完成判定
  bool   scene_corridor_  = false;   // 判定结果：true=走廊（B 线）/ false=楼梯（A 线）
  double scene_t0_        = -1.0;    // 窗口起点（首个参与累计帧的时间戳）
  std::size_t scene_n_    = 0;       // 窗口内参与累计的帧数
  std::size_t scene_n_hi_ = 0;       // 其中 deg_cond > 阈值的高退化帧数
  // ===== 细通道（M.27）：平面拟合拒绝点的 point-to-point 救援观测 =====
  // p_body：体轴系点（float，与 points_body_v3_ 同源）；p_map：世界系最近地图原始点。
  // 仅 UpdateObserve 首轮收集，后续轮次沿用关联、用当前位姿重算残差。
  struct FineCand { BASIC::V3 p_body; BASIC::V3d p_map; };
  std::vector<FineCand> fine_cands_;
  // 本帧细通道统计（最后一次 KF 迭代的结果，供 [FineCh] 日志）
  int    fine_n_used_   = 0;    // 通过关联距离筛选并计入 H_f 的点数
  double fine_gain_      = 0.0;  // λ_min(H+H_f)/λ_min(H)，<1.5 未并入
  bool   fine_applied_   = false;
  // ===== M.32 在线时延估计（侧车渐消最小二乘；默认关）=====
  // 残差对 δ 的近似为 r(δ) ≈ r_obs - c·δ，故 S_cr/S_cc 的比即 δ 的 LS 解。
  // λ 为遗忘因子，逐帧把累计量乘以 λ 实现指数加权（等价长度 ~1/(1-λ) 帧）。
  double td_scc_        = 0.0;   // Σλ^k·Σ(w·c²)（加权，用于 LS 求解）
  double td_scr_        = 0.0;   // Σλ^k·Σ(w·c·r)
  double td_scc_frame_  = 0.0;   // 本帧 Σ(w·c²)
  double td_scr_frame_  = 0.0;   // 本帧 Σ(w·c·r)
  double td_c2_raw_     = 0.0;   // 本帧 Σc²（未加权，只用于信息量门控）
  int    td_n_raw_      = 0;     // 本帧参与累加的点数
  double td_est_        = 0.0;   // 当前估计（秒）
  bool   td_locked_     = false;
  bool   td_applied_    = false;
  std::deque<double> td_hist_;   // 最近信息帧的 δ_est 序列，用于锁定判据
  long   td_info_frames_ = 0;    // 累计并入的信息帧数
  /// 每帧结束时结算：遗忘衰减 → 信息量门控 → 求 δ_est → 锁定判定 →（可选）应用
  void finalizeTimeDelayCalib(double frame_omega_norm);

  // ===== 坏帧护栏 =====
  double last_lidar_end_time_  = -1.0;   // 上一帧雷达 end_time
  int    guard_consec_skip_    = 0;      // 连续跳过帧数（安全阀计数）
  long   guard_skip_total_     = 0;      // 累计跳过帧数
  long   guard_last_log_frame_ = -10000; // 限频日志用

  // ===== 位姿信任域 / LOST =====
  /// 在 Observe() 之后调用：比较本次激光修正与最近可信位姿，
  /// 跳变超阈值则回滚到 IMU 先验并进入 LOST（输出不中断、暂停建图）。
  void checkPoseTrust(const SysState& sys_pred, const ESKF::COV& cov_pred);
  double cur_lidar_gap_ = 0.0;                     // 本帧与上一帧雷达 end_time 间隔 (s)
  BASIC::SE3 stable_pose_;                         // 最近一次可信位姿
  bool   stable_pose_valid_ = false;
  bool   lost_ = false;                            // LOST：不信任激光修正，只做影子配准
  int    lost_recover_streak_ = 0;
  long   trust_reject_total_ = 0;                  // 累计否决次数
  std::chrono::steady_clock::time_point last_trust_log_wall_{};
  std::chrono::steady_clock::time_point last_gap_log_wall_{};

  // ===== 看门狗 =====
  // 心跳只在「一帧完整走完配准+建图+输出」后刷新；任何静默 return 都不会刷新，
  // 因此停摆会被如实检出。日志 5s 限频，避免刷屏。
  std::chrono::steady_clock::time_point last_ok_frame_wall_{};
  std::chrono::steady_clock::time_point last_watchdog_log_wall_{};
  bool watchdog_armed_ = false;
  int  report_tick_count_ = 0;           // 1Hz 定时器 → 60s 打印窗口

  Timer time_record_;

  std::thread output_thread_;
  std::mutex output_mutex_;
  std::condition_variable output_cv_;
  std::queue<OutputData> output_queue_;
  std::atomic<bool> output_running_{false};

  std::thread save_thread_;
  std::mutex save_mutex_;
  std::condition_variable save_cv_;
  std::queue<SaveData> save_queue_;
  std::atomic<bool> save_running_{false};

  // 全量点异步去畸变/变换线程（错峰：实时链路不被全量处理阻塞）
  std::thread dense_thread_;
  std::mutex dense_mutex_;
  std::condition_variable dense_cv_;
  std::queue<DenseData> dense_queue_;
  std::atomic<bool> dense_running_{false};
  bool dense_paused_ = false;  // 暂停期间 DenseThread 不再消费队列（存图前静默屏障）
  bool dense_busy_ = false;    // DenseThread 正在处理一帧（dense_mutex_ 保护）

  // SC-PGO offline output
  int sc_pgo_index_ = 0;
  BASIC::SE3 sc_pgo_pose_prev_;
  float sc_pgo_trans_accum_ = 0.0f;
  float sc_pgo_rot_accum_ = 0.0f;
  bool sc_pgo_first_ = true;
  std::ofstream sc_pgo_odom_file_;

  // ===== B2：视觉相对旋转 =====
  // 仅在 g_visual_enable=true 时创建；关闭时指针为空，链路上零开销。
  std::unique_ptr<vis::VisualRotation> visual_;
  BASIC::M3 vis_R_cl_ = BASIC::M3::Identity();  ///< T_cam_lidar 的旋转部分
  struct VisPoseNode { double t = 0.0; BASIC::M3 R = BASIC::M3::Identity(); };
  std::deque<VisPoseNode> vis_pose_hist_;       ///< 每 lidar 帧后验姿态，作固定参考节点池
  uint64_t vis_applied_ = 0;                    ///< 实际注入次数
  uint64_t vis_frames_ = 0;                     ///< 拿到测量但因故未注入的帧数
  double   vis_dx_r_sum_ = 0.0;
  double   vis_dx_r_max_ = 0.0;
  // 残差偏置探针：e 定义在 prev 机体系（Log(R_meas_relᵀ·R_pred_rel)），
  // e_w = R_p·e 映到世界系。世界系均值非零 <=> 存在可学的系统性 yaw 偏置；
  // 机体系均值非零 <=> 偏置跟着机体走（陀螺标度/外参类）。
  Eigen::Vector3d vis_e_sum_w_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d vis_e_sum_b_ = Eigen::Vector3d::Zero();
  uint64_t        vis_e_cnt_ = 0;
  // ---- B2 χ² 门控 + 安全性统计（阈值与 B3 的 b3.chi2_thresh 完全独立）----
  // 门控只看，不改变 B3 的任何计数；<=0 表示门控关闭（复现无门控的旧行为）。
  uint64_t vis_rejected_ = 0;      ///< B2 被门控丢弃的候选注入数
  double   vis_chi2_sum_ = 0.0;    ///< 所有被评估候选的 chi2 之和
  double   vis_chi2_max_ = 0.0;    ///< chi2 最大值（含被拒者）
  double   vis_dp_max_ = 0.0;      ///< 单次注入造成的最大位置跳变 (m)
  double   vis_dp_sum_ = 0.0;
  uint64_t vis_dp_over_ = 0;       ///< 单次注入 |Δp| > 0.05 m 的次数
  double   vis_dyaw_max_ = 0.0;    ///< 单次注入造成的最大世界系 yaw 跳变 (rad)
  uint64_t vis_dyaw_over_ = 0;     ///< 单次注入 |Δyaw| > 20 mrad 的次数
  std::chrono::steady_clock::time_point vis_last_log_wall_{};
  std::chrono::steady_clock::time_point vis_last_stat_wall_{};  ///< 无测量时的周期状态行

  // ================= B3：视觉观测陀螺零偏 =================
  /// Φ(t) = Σ ω_meas·dt（未补偿累计积分），只在主线程写（Propagation_Undistort）
  /// 与读（VisualObserveBias），故不加锁。样本按 stamp 排序后累加：本项目
  /// 出现过外部节点污染回放导致时间戳乱序 ±3s 的事故，乱序直接积分会把 Φ 打乱。
  struct PhiSample {
    double t = 0.0;
    Eigen::Vector3d phi = Eigen::Vector3d::Zero();
  };
  std::deque<PhiSample> vis_phi_hist_;
  double          vis_phi_last_t_ = -1.0;
  Eigen::Vector3d vis_phi_last_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d vis_phi_last_gyr_ = Eigen::Vector3d::Zero();
  bool            vis_phi_init_ = false;
  uint64_t        vis_phi_unordered_ = 0;  ///< stamp 乱序/重复而被跳过的样本数
  uint64_t        vis_phi_gap_ = 0;        ///< dt 超上限、按断档处理不积分的次数
  void  AccumulateGyroPhase();
  bool  GyroPhaseAt(double t, Eigen::Vector3d &out) const;

  void VisualObserveBias();
  uint64_t vis_b3_frames_ = 0;
  uint64_t vis_b3_applied_ = 0;
  uint64_t vis_b3_rejected_ = 0;   ///< χ² 门控拒绝数
  uint64_t vis_b3_no_meas_ = 0;
  uint64_t vis_b3_no_phase_ = 0;
  double   vis_b3_chi2_sum_ = 0.0;
  double   vis_b3_chi2_max_ = 0.0;
  double   vis_b3_dxbg_sum_ = 0.0;
  double   vis_b3_dxbg_max_ = 0.0;
  double   vis_b3_resid_sum_ = 0.0;
  double   vis_b3_fd_max_ = 0.0;   ///< 解析雅可比 vs 中心差分的最大相对偏差
  uint64_t vis_b3_fd_n_ = 0;
  std::chrono::steady_clock::time_point vis_b3_last_log_{};

  std::atomic<bool> paused_{false};
};

} // namespace END.

#endif


