

#ifndef SUPER_LIO_H_
#define SUPER_LIO_H_

#include <queue>
#include <vector>
#include <iostream>
#include <cassert>
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
                      BASIC::CloudPtr& out_pc);
  void ApplyDeltaCorrection();
  void PublishBodyCloud();
  void DownSample();
  void DownSampleOnly();
  void Observe();
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
  using KNNHeapType = KNNHeap<5, BASIC::V3>;
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
  int pcd_index_ = -1;

  // ===== 实验埋点（lio.debug.level > 0 时生效，不参与滤波）=====
  std::vector<BASIC::scalar> fit_res_vec_;
  std::ofstream debug_csv_;

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


