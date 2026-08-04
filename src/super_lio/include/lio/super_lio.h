

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

  void stateWaitKFInit();
  void stateWaitMapInit();
  void stateProcess();
  virtual bool kf_init();
  virtual bool map_init();
  void Propagation_Undistort();
  void ApplyDeltaCorrection();
  void PublishBodyCloud();
  void DownSample();
  void DownSampleOnly();
  void Observe();
  virtual void UpdateMap();
  virtual void Output();
  void OutputThread();
  void SaveThread();
  void caceData();
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
  BASIC::CloudPtr point_map_, world_pc_, ds_world_;
  int frame_num_ = 0;
  BASIC::SE3 sys_init_pose_;
  BASIC::SE3 last_pose_;

  std::size_t effect_knn_num_ = 0;
  BASIC::VV3 points_world_v3_, points_body_v3_;
  alignas(64) bool effect_mask_[20000] = {false};
  alignas(64) bool effect_knn_mask_[20000] = {false};
  std::vector<int> effect_knn_idxs_;
  std::vector<std::pair<BASIC::M6, BASIC::V6>> H_R_;
  std::vector<std::array<double, 4>> abcd_vec_;
  int pcd_index_ = -1;

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

  // SC-PGO offline output
  int sc_pgo_index_ = 0;
  BASIC::SE3 sc_pgo_pose_prev_;
  float sc_pgo_trans_accum_ = 0.0f;
  float sc_pgo_rot_accum_ = 0.0f;
  bool sc_pgo_first_ = true;
  std::ofstream sc_pgo_odom_file_;

  std::atomic<bool> paused_{false};

  // Cache for caceData reuse (avoid duplicate transformPointCloud)
  BASIC::CloudPtr last_transformed_world_pc_;
};

} // namespace END.

#endif


