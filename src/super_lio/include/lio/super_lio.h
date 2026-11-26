

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
#include <pcl/kdtree/kdtree_flann.h>

#include "basic/alias.h"
#include "common/ds.h"
#include "common/timer.h"
#include "params.h"
#include "ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"
#include "OctVoxMap/VoxelGridFilter.h"
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

  /// 看门狗：由独立回调组的定时器 1Hz 调用，检测整条实时链路是否停摆。
  /// 与 process 线程解耦，因此即使 process 卡在持锁区里也能照常输出现场状态。
  void watchdogCheck();

  /// 外部传感器观测注入钩子（视觉等）：默认空实现。
  /// 派生类 override 后，在每帧雷达观测（含信任域裁决）之后、建图与输出之前
  /// 调用 eskf()->UpdateObserveFull(...) 写回约束。
  /// 未 override 时整条链路与纯 LIO 逐位一致。
  virtual void ExternalObserve() {}

  /// 供派生类/外部调用者访问滤波器；线程安全由 ESKF 内部递归锁保证。
  ESKF::Ptr eskf() const { return kf_; }

  /// 本帧观测信息矩阵的退化签名快照，供进程内派生类直接读取。
  /// 用途：视觉支路（方案 A/B）在单体内无需订阅 `/lio/degeneracy` 话题，
  /// 也不需要在 LIO 侧新增话题发布，直接从内存取同源数据。
  /// info 为原始位姿观测 Hessian（`HTVH`，布局 [rot(0-2), pos(3-5)]，未经截断），
  /// 与退化检测同源；`has_info=false` 表示本帧尚未产出（首帧前）。
  struct DegeneracySnapshot {
    bool  ok     = false;
    double cond  = 0.0;
    int    axes  = 0;
    bool  active = false;
    BASIC::V3d dir_w = BASIC::V3d::Zero();  ///< 最弱方向（世界系，零向量=无方向）
    bool  has_info = false;
    BASIC::M6d info = BASIC::M6d::Zero();
  };

  /// 线程安全地拷贝一份退化签名快照（写方在 Observe 线程，读方可跨线程）。
  DegeneracySnapshot degeneracySnapshot() const;

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
    double lidar_end_time = 0.0;                  // 该帧雷达 end_time（Anchor 排队/裁剪用）
    double receive_time = 0.0;                    // 该帧在宿主侧的接收时刻（墙钟，s）
    BASIC::SE3 T_pred;                            // 预测位姿（去畸变基准）
    BASIC::SE3 T_corrected;                       // 配准修正位姿（delta 修正用）
  };

  void stateWaitKFInit();
  void stateWaitMapInit();
  void stateProcess();
  virtual bool kf_init();

  // ===== 进程内输出钩子（纯加法，默认空实现；未覆写时纯 LIO 行为逐位不变）=====
  // 供单体化视觉支路（方案 A/B）在同进程直接接收 LIO 输出，取代 DDS 话题回环。
  /// 每帧输出线程取到 OutputData 后调用（位姿 + body/world 点云）。
  virtual void onLioOutput(const OutputData& data) { (void)data; }
  /// DenseThread 产出全量机体系点云后调用（在 dense 话题发布前）。
  /// receive_time：该帧在宿主侧的接收时刻（墙钟秒），供视觉支路在线估计
  /// 雷达时钟相对宿主时钟的偏移（两时钟独立且会漂移）。
  virtual void onDenseCloud(const BASIC::CloudPtr& cloud, double stamp, double receive_time) {
    (void)cloud;
    (void)stamp;
    (void)receive_time;
  }

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
  virtual void UpdateMap(bool write = true);
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
  /// 地图新点比例（novelty）：本帧点云按当前位姿投到 world 后，落在"尚无粗体素"
  /// 的点占比。正常回访时低（已有地图覆盖），错误匹配时整帧被塞进空白区域而飙升，
  /// 是"位姿开始发散"的最早可测信号。写入暂停的帧也照常统计。
  double map_novelty_ratio_ = 0.0;
  /// 本帧扫描尺度（机体系）：最远点距离与 z 跨度。用于开阔区门控——
  /// 无结构的大空间里新点比例/平移锯齿判据无意义（指标噪声大且 GICP 无重叠可校验）。
  double scan_max_range_ = 0.0;
  double scan_z_span_ = 0.0;
  int    map_novelty_streak_ = 0;    // 连续超阈值帧数
  /// 新点比例门的连续抑制时长（s）：超过 g_trust_suspect_max_s 走强制接受/重锚兜底，
  /// 避免"地图被清空后 novelty 恒为 1"把建图永久冻死。
  double map_novelty_run_dt_ = 0.0;
  /// 重锚清图后的豁免截止时刻（lidar end_time）：此时整帧都是新点，novelty 无意义。
  double map_novelty_skip_until_t_ = -1.0;
  /// 新点比例连续超阈值段的起点位姿：判据用它算累计位移/转角。
  /// 不能用"单帧修正量"——错误匹配的拖拽每帧只有几厘米，单帧门永不成立。
  BASIC::SE3 novelty_ref_pose_;
  bool       novelty_ref_valid_ = false;
  /// 新点比例前兆触发后的持续抑制状态：置位后每帧都拒绝 Voxmap 观测、暂停写图，
  /// 并逐帧重试冻结 Anchor 的 GICP；超过 g_trust_suspect_max_s 仍未收敛才清图重锚。
  /// 不能只在"命中帧"抑制——未命中帧照常接受观测会让位姿继续漂移（gicp_t 实测
  /// 275.4 s 首次命中后，中间帧仍被拖到 3.9 m/s，GICP 再也追不上）。
  bool       novelty_hold_ = false;
  /// 持续抑制期的冻结位姿：判据成立时刻的 ESKF 主状态与协方差，抑制期每帧
  /// 回滚到它。不能沿用"保留 IMU 外推 + 速度清零"——速度是传播之后才清零的，
  /// 位置仍按该帧残余速度推进（gicp_w 实测约 1 m/s），10 s 就能把位姿推出
  /// 地图/Anchor 覆盖范围（effect_pts 303→9），GICP 初值出界后 inlier 恒为 0，
  /// 只能清图重锚在错误位姿上。冻结后 GICP 每帧的初值都停留在判据成立时刻
  /// （此时位姿仍准确），重定位成功率最高。
  SysState   novelty_hold_state_;
  ESKF::COV  novelty_hold_cov_ = ESKF::COV::Identity();
  /// "保图放行"计数：短时内反复放行说明不是偶发新覆盖，而是错误匹配在持续
  /// 拖拽位姿（gicp_ac 实测 19:53:19-19:53:55 每 9 s 放行一次，位姿被推走 25 m）。
  /// 同一窗口内第 2 次即升级为硬重锚，不再反复放行。
  int        novelty_release_cnt_ = 0;
  double     novelty_release_last_t_ = -1.0;
  /// "新覆盖放行"计数：GICP 独立绝对位姿与 Voxmap 结果一致时的放行。它不代表
  /// 发散（恰恰相反），因此不参与 kNoveltyReleaseMax 的升级判据，只用于限频日志。
  int        novelty_verified_release_cnt_ = 0;

  /// 平移锯齿（位置震荡）早检：最近若干帧的逐帧位移隐含速度 |Δp|/dt。
  /// 漂移早期常以平移震荡为主、旋转误差很小（用户实测），表现为相邻帧位移
  /// 方向来回翻转且单帧隐含速度远超平台真实速度；该指标只用位姿自身，
  /// 不依赖地图统计，可与新点比例判据互补。
  static constexpr int kSawWin = 6;
  double     saw_vimp_[kSawWin] = {0.0};
  int        saw_idx_ = 0;
  int        saw_fill_ = 0;
  double     saw_last_p_[3] = {0.0, 0.0, 0.0};
  bool       saw_last_valid_ = false;
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
  /// P0.1 可观测性诊断：每帧归一化信息矩阵全谱（lio.debug.level >= 2 时输出）
  std::ofstream obs_csv_;
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

  // 进程内退化签名快照（供 degeneracySnapshot() 跨线程安全读取）
  mutable std::mutex snapshot_mutex_;
  DegeneracySnapshot snapshot_;
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
  int    last_n_effect_ = 0;                       // 本帧有效平面残差点数（首轮统计）
  long   trust_reject_total_ = 0;                  // 累计否决次数
  long   reanchor_total_ = 0;                      // 累计清图重锚次数（超时兜底残差过大）
  std::deque<std::pair<double, BASIC::V3>> rot_corr_hist_;  // (dt, 姿态修正旋转矢量) 速率窗
  long   rot_clamp_total_ = 0;                     // 累计姿态限幅次数
  bool   map_pause_frame_ = false;                 // 本帧 SUSPECT：跳过 IVox 写图（不影响输出）
  double suspect_hold_dt_ = 0.0;                   // 连续 SUSPECT 的累计时长 (s)，用于升级 LOST
  double suspect_run_dt_ = 0.0;                    // SUSPECT 连续累计时长 (s)，用于超时强制放行
  double lost_run_dt_ = 0.0;                       // LOST 连续累计时长 (s)，用于超时强制放行
  double first_lidar_end_time_ = -1.0;             // 首帧雷达 end_time（冻结实验的相对时间基准）
  std::chrono::steady_clock::time_point last_trust_log_wall_{};
  std::chrono::steady_clock::time_point last_gap_log_wall_{};

  // ===== 可信 Anchor（独立于实时 IVox，延迟提交）=====
  // 与实时建图地图完全分离：实时 IVox 仍逐帧写入，Anchor 只接收"确认窗口内
  // 未触发 SUSPECT/LOST"的帧，因此可用于异常后的第二套位姿观测（GICP 目标）。
  // 关闭时（g_anchor_delay_commit_s <= 0）零开销、零行为变化。
  struct AnchorFrame {
    double        t = 0.0;    // 帧 end_time（s，bag 时基）
    BASIC::SE3    pose;       // 该帧提交时的位姿（世界系，来自 kf_->GetSE3()）
    BASIC::CloudPtr body;     // body 系去畸变降采样点云（ds_undistort_ 的副本）
  };
  void updateTrustedAnchor(bool map_write);
  void freezeTrustedAnchor(const char* why);
  std::deque<AnchorFrame> anchor_pending_;    // 可撤销：尚未过确认窗口
  std::deque<AnchorFrame> anchor_committed_;  // 已确认可信
  /// DenseThread 产出的稠密 Anchor 帧（异步，通常滞后 1~2 帧），
  /// 由 updateTrustedAnchor 在安全点收编进 anchor_pending_（见 DenseThread）。
  std::deque<AnchorFrame> anchor_dense_inbox_;
  std::mutex anchor_inbox_mutex_;             // 保护 anchor_dense_inbox_
  long   anchor_inbox_used_ = 0;              // 被收编的 DenseThread 帧数
  long   anchor_inbox_seen_ = 0;              // inbox 实际到达帧数（含被判重复者）
  long   anchor_missed_ = 0;                  // 本帧 inbox 为空（DenseThread 正常滞后）次数
  long   anchor_fallback_hits_ = 0;           // 断流 >1 s 后主线程补算次数（诊断用）
  double anchor_last_inbox_t_ = -1.0;         // inbox 最近到达帧的 lidar 时间（活跃度判据）
  bool   anchor_frozen_ = false;              // 触发异常后永久冻结（恢复由重定位阶段接管）
  long   anchor_drop_total_ = 0;              // 累计因异常丢弃的未提交帧数
  long   anchor_commit_total_ = 0;            // 累计提交帧数
  double anchor_last_log_t_ = -1.0;           // 限频日志

  // ===== 可信 Anchor GICP 重定位（第二套 LiDAR 位姿观测）=====
  // 触发时机：信任域检测判 SUSPECT / LOST。Source = 当前帧去畸变机体系点云
  // （尚未乘任何世界位姿，故与漂移无关）；Target = anchor_committed_ 按各自
  // 可信位姿投到 world 后拼成的点云（冻结，不参与实时 IVox 清理）。
  // 通过质量门 → 直接覆盖 ESKF 的 R/p/v（不是撤销修正），并跳过本帧 delta 修正，
  // 让点云按修正后的位姿写入地图。连续失败 g_gicp_fail_max 帧后退出接管。
  bool buildGicpTarget();                     // 惰性构建目标地图 + KdTree（仅一次）
  bool tryGicpRelocalize(const SysState& sys_pred, const ESKF::COV& cov_pred,
                         double vox_corr_p, double vox_corr_r, const char* why,
                         bool* ran = nullptr);
  BASIC::V3 fitGicpVelocity() const;          // 最近 3~5 帧位置对时间加权拟合速度
  BASIC::CloudPtr gicp_target_;
  BASIC::CloudPtr gicp_source_;   ///< 当前帧全量去畸变云按 g_gicp_target_voxel 降采样（配准源）
  /// LOST 期间冻结的源云：进入 LOST 时的首帧稠密云快照。LOST 中 IMU 速度外推会把
  /// 每帧去畸变云剪切成非刚体，源点数持续膨胀、对应关系崩坏（conv=0 死循环），
  /// 因此 LOST 期间固定用该快照配准，退出 LOST 时释放。
  BASIC::CloudPtr gicp_lost_src_;
  bool   gicp_lost_src_valid_ = false;
  pcl::KdTreeFLANN<BASIC::PointType>::Ptr gicp_target_kdtree_;
  bool   gicp_target_ready_ = false;
  double gicp_target_build_t_ = -1.0;         // 目标图构建时刻，用于按周期重建
  bool   gicp_active_ = false;                // 已由 GICP 接管 LiDAR 位姿观测
  int    gicp_ok_streak_ = 0;
  int    gicp_fail_streak_ = 0;
  std::deque<std::pair<double, BASIC::V3>> gicp_pos_hist_;  // (t, p) 最近接受的 GICP 位置
  BASIC::SE3 gicp_last_pose_;                 // 上一帧接受的 GICP 位姿（连续性检查）
  double gicp_last_t_ = -1.0;
  bool   gicp_last_pose_valid_ = false;
  /// 接管期逐帧累积的 IMU 旋转增量（机体系 log，相对上次接受的 GICP 姿态）。
  /// 接管期每个失败/降频帧都把状态姿态钉回 gicp_last_pose_，因此 sys_pred 只
  /// 前进一帧；bg 估计必须用"整段累积"与 GICP 绝对旋转差比较（gicp_ap 第一版
  /// 直接拿单帧预测比整段观测，把 −0.5·ω_true 积进了 bg：bgx 0.065→0.127）。
  BASIC::V3 gicp_imu_rot_acc_ = BASIC::V3::Zero();
  /// 退出接管后仍保留的"最后一次可信 GICP 位姿"，只用作下一次尝试的配准初值。
  /// 退出接管时若连它一起清掉，重试只能拿已被 Voxmap 拖离数米的 IMU 位姿当
  /// 初值，粗配半径 1.5 m 内一个对应都找不到 ⇒ conv=0 立即失败（gicp_z
  /// 19:02:24 退出后 19:02:29 起每次 1~5 ms 失败），再也回不到 Anchor。
  BASIC::SE3 gicp_seed_pose_;
  double gicp_seed_t_ = -1.0;
  bool   gicp_seed_valid_ = false;
  /// 与种子位姿同时保留的完整滤波状态（含零偏/重力）。
  /// 长时间拒绝观测后 ESRF 会把不一致吸收进 ba/bg（gicp_ab 实测 baz 从 0
  /// 涨到 0.73 m/s²，清图后纯 IMU 外推 90 s 直接下坠 −78 m）。重锚/钉住时
  /// 连同零偏一起恢复，运动学才与"回到可信位姿"自洽。
  SysState gicp_seed_state_;
  /// "连续失败但 Voxmap 不可信"时保持接管的轮数，超过后以种子位姿清图重锚。
  int    gicp_hold_cycles_ = 0;
  /// 最近一次 GICP 复核（tryGicpRelocalize）的结果快照：供信任守护在"拒绝"后
  /// 区分「位姿确实被 Voxmap 拖走」与「只是飞进了尚未建图的区域」。
  /// 仅在本帧确实执行了配准并走到质量评估时有效（未运行/提前 return 时置 false）。
  bool   gicp_check_valid_ = false;
  bool   gicp_check_converged_ = false;
  int    gicp_check_n_in_ = 0;
  double gicp_check_rmse_ = 1e9;
  double gicp_check_dpos_ = 1e9;              // 相对 IMU 先验
  double gicp_check_drot_ = 1e9;              // deg，相对 IMU 先验
  BASIC::SE3 gicp_check_pose_;                // GICP 绝对位姿（world<-body）
  bool   delta_passthrough_frame_ = false;    // 本帧跳过 delta 修正（GICP 接管帧）
  std::chrono::steady_clock::time_point last_gicp_log_wall_{};
  /// GICP 全局节流：rate / jump / lost / active 四条调用路径共用同一道闸门。
  /// 单帧 0.3~0.7 s，若逐帧调用会把 process 线程拖到几十秒滞后。
  int    gicp_last_run_frame_ = -100000;      // 上次真正跑 GICP 的雷达帧号
  std::chrono::steady_clock::time_point gicp_last_run_wall_{};
  bool   gicp_last_run_wall_valid_ = false;
  /// 本帧全量去畸变云（机体系，未做 delta 修正）：GICP 源云与 Anchor body 共用。
  /// 常规流程由 DenseThread 顺带产出（Anchor 走 anchor_dense_inbox_），主线程只在
  /// 真要跑 GICP 配准时用 ensureGicpDense() 按需补算，避免每帧重复去畸变全量点。
  BASIC::CloudPtr gicp_scan_dense_;
  bool   gicp_dense_valid_ = false;
  void ensureGicpDense();
  /// 本帧 IMU 去畸变用的角速度序列（帧内区间），供异常帧现场重建稠密源云
  std::vector<BASIC::V3> frame_imu_gyr_;
  int    gicp_frame_counter_ = 0;   // 接管期雷达帧计数（降频用）
  int    gicp_handback_streak_ = 0; // 连续满足"Voxmap 已恢复且与 GICP 一致"的次数
  /// 接管期 → 交还 Voxmap：调用点在 checkPoseTrust（Observe 之后、delta 修正之前）
  bool tryHandbackToVoxmap(double corr_p, double corr_r);
  /// 本帧稠密源云结果（world 系 GICP 解）供交还判定交叉校验
  BASIC::SE3 gicp_last_raw_pose_;
  bool   gicp_last_raw_valid_ = false;

  // ===== 交还 Voxmap 后的观察期 =====
  // 交还瞬间"Voxmap 修正≈0 + 与上次可信 GICP 一致"只说明两者当前自洽，不说明
  // 地图段没被早先的漂移污染（gicp_ah 47:35 交还时 corr_p=0.027 m、novelty≤0.15
  // 全过门，但 2 s 后状态速度被泵到 5.1 m/s，4 s 位姿沿 -y 被拖走 9 m、novelty
  // 升到 0.78，随后 GICP 回正解 inlier 只剩 0.34 彻底拉不回来）。
  // 观察期内偏离交还位姿且速度抬头即撤销交还、回到 GICP 接管，把偏差压在
  // GICP 还能重新锁定的量级内。
  double handback_probation_until_t_ = -1.0;   ///< 观察期截止（lidar end_time）
  BASIC::SE3 handback_ref_pose_;               ///< 交还瞬间的位姿（撤销交还时回滚用）
  bool   handback_ref_valid_ = false;

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

  std::atomic<bool> paused_{false};
};

} // namespace END.

#endif


