/// B2 方案：相邻图像帧光流两帧几何 -> 相机系相对旋转 -> 供 ESKF 旋转注入。
///
/// 只产出相对旋转，不做逐点 LiDAR 深度关联/重投影（那条路已验证无效并删除）。
/// 前端 KLT（FAST 角点 + 金字塔 LK + 前后向校验），几何 cv::findEssentialMat +
/// cv::recoverPose，平移与尺度全部丢弃。
#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace LI2Sup {
namespace vis {

struct VisualRotationConfig {
  bool enable = false;

  /// 图像 header 时刻 + 此偏移 = 归到 lidar 帧头的采集时刻
  double img_time_offset = 0.0;
  /// 匹配窗口松弛量（由调用方在 pick 时传入，这里只存一份方便日志）
  double frame_slack = 0.05;

  // 内参（去畸变在模块内部用 remap 完成，之后一律按理想针孔 + K 处理）
  cv::Matx33d K = cv::Matx33d::eye();
  cv::Vec4d dist = cv::Vec4d(0.0, 0.0, 0.0, 0.0);
  int width = 0;
  int height = 0;

  /// T_cam_lidar 的旋转部分（行主序 3x3）。body 帧即 IMU 帧，用 Rcl 近似 R_cam_body。
  Eigen::Matrix3d R_cl = Eigen::Matrix3d::Identity();

  // KLT
  int max_features = 150;
  double min_distance = 10.0;
  double fb_error = 1.0;
  int pyramid_levels = 3;
  bool use_fast = true;
  int fast_threshold = 20;
  int lk_window = 21;

  // 两帧几何
  double essential_threshold = 1.0;  ///< RANSAC 像素阈值
  double essential_prob = 0.999;
  int min_inliers = 30;

  /// 旋转测量噪声标准差(rad)，由 SuperLIO 侧读取用于构造信息阵
  double sigma_rot = 0.02;

  // ---- B3：长基线图像对（观测陀螺零偏用）----
  /// 关闭时下面全部逻辑不启用，B2 行为与内存占用逐位不变
  bool long_enable = false;
  /// 参考节点与当前帧的最小时间间隔(s)。相邻帧 Δt≈0.1s 时等效零偏精度
  /// σ/Δt 毫无约束力，必须拉长基线。
  double baseline_dt = 1.0;
  /// 长基线位移大，金字塔层数单独给（相邻帧仍用 pyramid_levels）
  int long_pyramid_levels = 4;
  /// 节点缓存保留时长(s)，需 > baseline_dt + 消费松弛量
  double long_node_keep = 5.0;
};

/// 一对相邻图像帧的相对旋转测量：相机系 cur <- prev
struct RotMeasure {
  double t_prev = 0.0;
  double t_cur = 0.0;
  Eigen::Matrix3d R_rel_c = Eigen::Matrix3d::Identity();
  int matched = 0;   ///< KLT 通过前后向校验的点数
  int inliers = 0;   ///< recoverPose 最终内点数
};

class VisualRotation {
 public:
  explicit VisualRotation(const VisualRotationConfig &cfg) : cfg_(cfg) {}

  /// 由 ROS 图像回调线程调用。内部完成去畸变 + KLT + 两帧几何，产出的测量进 pending_。
  /// 调用方线程独立于 LIO 主线程，故这里做完整计算以避免给 process 增加耗时。
  void pushImage(const cv::Mat &image, double stamp);

  /// 取时间上最贴近 t1 且满足 |t_cur - t1| <= slack 的最新一对测量。
  /// 返回 false 表示窗口内没有可用测量（不消费队列）。
  bool pickMeasure(double t0, double t1, double slack, RotMeasure &out);

  /// B3：同上，但取长基线对（t_cur - t_prev >= cfg_.baseline_dt）
  bool pickLongMeasure(double t0, double t1, double slack, RotMeasure &out);

  bool enabled() const { return cfg_.enable; }
  double sigma_rot() const { return cfg_.sigma_rot; }
  const Eigen::Matrix3d &R_cl() const { return cfg_.R_cl; }

  /// 诊断统计（累计）
  uint64_t pairsTotal() const { return n_pairs_; }
  uint64_t pairsOk() const { return n_ok_; }
  uint64_t imagesRecv() const { return n_img_; }
  double meanInliers() const { return n_pairs_ > 0 ? double(sum_inl_) / double(n_pairs_) : 0.0; }
  /// 诊断：pending_ 中与 t_ref 最近的时间残差(s)；队列为空返回 <0
  double bestGap(double t_ref) const {
    std::lock_guard<std::mutex> lock(mtx_);
    double g = -1.0;
    for (const auto &m : pending_) {
      const double e = std::fabs(m.t_cur - t_ref);
      if (g < 0.0 || e < g) g = e;
    }
    return g;
  }
  size_t pendingSize() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return pending_.size();
  }
  std::string lastReason() const;

  // ---- B3 长基线诊断 ----
  uint64_t longPairsTotal() const { return n_pairs_long_; }
  uint64_t longPairsOk() const { return n_ok_long_; }
  double longMeanInliers() const {
    return n_pairs_long_ > 0 ? double(sum_inl_long_) / double(n_pairs_long_) : 0.0;
  }
  std::string lastLongReason() const;

 private:
  void ensureMaps(const cv::Size &size);
  void detectCorners();
  void trackAndSolve(const cv::Mat &cur, double t_cur);

  /// B3：参考节点（去畸变图 + 该图上检测的角点 + 时刻）
  struct Node {
    cv::Mat gray;
    std::vector<cv::Point2f> pts;
    double t = 0.0;
  };
  void pushNode(const cv::Mat &cur, double t);
  /// 用「距当前 >= baseline_dt 的最近可用节点」做长基线跟踪
  void trackLongBaseline(const cv::Mat &cur, double t_cur);

  VisualRotationConfig cfg_;

  mutable std::mutex mtx_;
  std::string reason_;

  // 前端状态（mtx_ 保护）
  bool has_prev_ = false;
  cv::Mat prev_gray_;
  std::vector<cv::Point2f> prev_pts_;
  double prev_t_ = 0.0;
  bool maps_ready_ = false;
  cv::Mat map1_, map2_;

  std::deque<RotMeasure> pending_;

  // B3 长基线状态
  std::deque<Node> nodes_;
  std::deque<RotMeasure> pending_long_;
  std::string reason_long_;

  uint64_t n_img_ = 0;
  uint64_t n_pairs_ = 0;
  uint64_t n_ok_ = 0;
  uint64_t sum_inl_ = 0;

  uint64_t n_pairs_long_ = 0;
  uint64_t n_ok_long_ = 0;
  uint64_t sum_inl_long_ = 0;
};

}  // namespace vis
}  // namespace LI2Sup
