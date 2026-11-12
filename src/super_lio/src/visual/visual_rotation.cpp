#include "visual/visual_rotation.hpp"

#include <glog/logging.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <cmath>

namespace LI2Sup {
namespace vis {

namespace {

constexpr int kMaxPending = 32;
constexpr double kMinBaselineRotation = 1e-4;  ///< 旋转太小 <=> 基本纯平移，E 矩阵病态

/// 行主序 cv::Mat(CV_64F, 3x3) -> Eigen
Eigen::Matrix3d toEigen(const cv::Mat &m) {
  Eigen::Matrix3d r;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) r(i, j) = m.at<double>(i, j);
  return r;
}

}  // namespace

std::string VisualRotation::lastReason() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return reason_;
}

void VisualRotation::ensureMaps(const cv::Size &size) {
  if (maps_ready_) return;
  const cv::Size sz(cfg_.width > 0 ? cfg_.width : size.width,
                    cfg_.height > 0 ? cfg_.height : size.height);
  // 新内参矩阵沿用同一个 K：主点不动，只校正径向/切向畸变
  cv::initUndistortRectifyMap(cfg_.K, cv::Mat(cfg_.dist), cv::Mat(), cfg_.K, sz, CV_16SC2, map1_,
                              map2_);
  maps_ready_ = true;
}

void VisualRotation::detectCorners() {
  prev_pts_.clear();
  if (prev_gray_.empty()) return;

  const int border = std::max(4, cfg_.lk_window / 2 + 1);
  cv::Mat mask = cv::Mat::ones(prev_gray_.size(), CV_8UC1) * 255;
  cv::Rect inner(border, border, prev_gray_.cols - 2 * border, prev_gray_.rows - 2 * border);
  if (inner.width <= 0 || inner.height <= 0) return;
  cv::rectangle(mask, cv::Rect(0, 0, prev_gray_.cols, prev_gray_.rows), cv::Scalar(0),
                cv::FILLED);
  cv::rectangle(mask, inner, cv::Scalar(255), cv::FILLED);

  if (cfg_.use_fast) {
    auto det = cv::FastFeatureDetector::create(cfg_.fast_threshold, true);
    std::vector<cv::KeyPoint> kp;
    det->detect(prev_gray_, kp, mask);
    std::sort(kp.begin(), kp.end(),
              [](const cv::KeyPoint &a, const cv::KeyPoint &b) { return a.response > b.response; });
    // min_distance 空间抑制（goodFeaturesToTrack 自带，Fast 需自己做）
    const double min_d2 = cfg_.min_distance * cfg_.min_distance;
    for (const auto &k : kp) {
      if ((int)prev_pts_.size() >= cfg_.max_features) break;
      bool ok = true;
      for (const auto &q : prev_pts_) {
        const double dx = q.x - k.pt.x, dy = q.y - k.pt.y;
        if (dx * dx + dy * dy < min_d2) { ok = false; break; }
      }
      if (ok) prev_pts_.emplace_back(k.pt);
    }
  } else {
    cv::goodFeaturesToTrack(prev_gray_, prev_pts_, cfg_.max_features, 0.01, cfg_.min_distance,
                            cv::noArray(), 3, false, 0.04);
  }
}

void VisualRotation::pushImage(const cv::Mat &image, double stamp) {
  if (!cfg_.enable || image.empty()) return;

  cv::Mat gray;
  if (image.channels() == 1) {
    gray = image;
  } else if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    return;
  }

  const double t = stamp + cfg_.img_time_offset;

  std::lock_guard<std::mutex> lock(mtx_);
  ++n_img_;
  ensureMaps(gray.size());

  cv::Mat und;
  cv::remap(gray, und, map1_, map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

  if (!has_prev_) {
    prev_gray_ = und;
    prev_t_ = t;
    has_prev_ = true;
    detectCorners();
    reason_ = "init";
    if (cfg_.long_enable) {
      pushNode(und, t);
      reason_long_ = "init";
    }
    return;
  }
  if (t <= prev_t_) {
    reason_ = "timestamp-backward";
    return;
  }

  trackAndSolve(und, t);

  prev_gray_ = und;
  prev_t_ = t;
  detectCorners();

  // B3：节点角点用的是刚检测出来的 prev_pts_（即本帧图上的角点）
  if (cfg_.long_enable) {
    pushNode(und, t);
    trackLongBaseline(und, t);
  }
}

void VisualRotation::pushNode(const cv::Mat &cur, double t) {
  Node nd;
  nd.t = t;
  nd.gray = cur;  // cv::Mat 引用计数，cur 是 remap 的新缓冲，安全
  nd.pts = prev_pts_;
  nodes_.push_back(std::move(nd));
  while (!nodes_.empty() && (t - nodes_.front().t > cfg_.long_node_keep || nodes_.size() > 128))
    nodes_.pop_front();
}

void VisualRotation::trackLongBaseline(const cv::Mat &cur, double t_cur) {
  ++n_pairs_long_;
  // 参考节点 = 距当前 >= baseline_dt 的节点里最靠后的那个（基线刚好过阈值）
  int idx = -1;
  for (int i = (int)nodes_.size() - 1; i >= 0; --i) {
    if (t_cur - nodes_[i].t >= cfg_.baseline_dt) {
      idx = i;
      break;
    }
  }
  if (idx < 0) {
    reason_long_ = "no-ref-node";
    return;
  }
  const Node &ref = nodes_[idx];
  const int n_in = (int)ref.pts.size();
  if (n_in < std::max(cfg_.min_inliers, 20)) {
    reason_long_ = "too-few-corners";
    return;
  }

  const cv::Size win(cfg_.lk_window, cfg_.lk_window);
  const cv::TermCriteria tc(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 21, 0.01);
  const int levels = std::max(1, cfg_.long_pyramid_levels);

  std::vector<cv::Point2f> cur_pts, back_pts;
  std::vector<uchar> st_fwd, st_bwd;
  std::vector<float> err_fwd, err_bwd;
  cv::calcOpticalFlowPyrLK(ref.gray, cur, ref.pts, cur_pts, st_fwd, err_fwd, win, levels, tc);
  cv::calcOpticalFlowPyrLK(cur, ref.gray, cur_pts, back_pts, st_bwd, err_bwd, win, levels, tc);

  std::vector<cv::Point2f> p1, p2;
  p1.reserve(n_in);
  p2.reserve(n_in);
  const double fb2 = cfg_.fb_error * cfg_.fb_error;
  for (int i = 0; i < n_in; ++i) {
    if (!st_fwd[i] || !st_bwd[i]) continue;
    const double dx = back_pts[i].x - ref.pts[i].x;
    const double dy = back_pts[i].y - ref.pts[i].y;
    if (dx * dx + dy * dy > fb2) continue;
    if (!std::isfinite(cur_pts[i].x) || !std::isfinite(cur_pts[i].y)) continue;
    p1.emplace_back(ref.pts[i]);
    p2.emplace_back(cur_pts[i]);
  }

  const int matched = (int)p1.size();
  if (matched < cfg_.min_inliers) {
    reason_long_ = "too-few-tracked(" + std::to_string(matched) + "/" + std::to_string(n_in) + ")";
    return;
  }

  double rot_energy = 0.0;
  for (int i = 0; i < matched; ++i)
    rot_energy += std::hypot(p2[i].x - p1[i].x, p2[i].y - p1[i].y);
  if (rot_energy / matched < kMinBaselineRotation) {
    reason_long_ = "no-parallax-motion";
    return;
  }

  cv::Mat mask;
  const cv::Mat E = cv::findEssentialMat(p1, p2, cfg_.K, cv::RANSAC, cfg_.essential_prob,
                                         cfg_.essential_threshold, mask);
  if (E.empty()) {
    reason_long_ = "essential-failed";
    return;
  }
  const int inl = cv::countNonZero(mask);
  if (inl < cfg_.min_inliers) {
    reason_long_ = "essential-low-inliers";
    return;
  }

  cv::Mat R_cv;
  cv::Mat t_cv;
  const int inl2 = cv::recoverPose(E, p1, p2, cv::Mat(cfg_.K), R_cv, t_cv, mask);
  if (inl2 < cfg_.min_inliers) {
    reason_long_ = "recoverpose-low-inliers";
    return;
  }

  RotMeasure m;
  m.t_prev = ref.t;
  m.t_cur = t_cur;
  m.R_rel_c = toEigen(R_cv);
  m.matched = matched;
  m.inliers = inl2;
  if (!m.R_rel_c.allFinite()) {
    reason_long_ = "non-finite-rotation";
    return;
  }

  pending_long_.push_back(m);
  if ((int)pending_long_.size() > kMaxPending) pending_long_.pop_front();
  ++n_ok_long_;
  sum_inl_long_ += inl2;
  reason_long_ = "ok";
}

std::string VisualRotation::lastLongReason() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return reason_long_;
}

void VisualRotation::trackAndSolve(const cv::Mat &cur, double t_cur) {
  ++n_pairs_;
  const int n_in = (int)prev_pts_.size();
  if (n_in < std::max(cfg_.min_inliers, 20)) {
    reason_ = "too-few-corners";
    return;
  }

  const cv::Size win(cfg_.lk_window, cfg_.lk_window);
  const cv::TermCriteria tc(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 21, 0.01);
  const int levels = std::max(1, cfg_.pyramid_levels);

  std::vector<cv::Point2f> cur_pts, back_pts;
  std::vector<uchar> st_fwd, st_bwd;
  std::vector<float> err_fwd, err_bwd;
  cv::calcOpticalFlowPyrLK(prev_gray_, cur, prev_pts_, cur_pts, st_fwd, err_fwd, win, levels, tc);
  cv::calcOpticalFlowPyrLK(cur, prev_gray_, cur_pts, back_pts, st_bwd, err_bwd, win, levels, tc);

  std::vector<cv::Point2f> p1, p2;
  p1.reserve(n_in);
  p2.reserve(n_in);
  const double fb2 = cfg_.fb_error * cfg_.fb_error;
  for (int i = 0; i < n_in; ++i) {
    if (!st_fwd[i] || !st_bwd[i]) continue;
    const double dx = back_pts[i].x - prev_pts_[i].x;
    const double dy = back_pts[i].y - prev_pts_[i].y;
    if (dx * dx + dy * dy > fb2) continue;
    if (!std::isfinite(cur_pts[i].x) || !std::isfinite(cur_pts[i].y)) continue;
    p1.emplace_back(prev_pts_[i]);
    p2.emplace_back(cur_pts[i]);
  }

  const int matched = (int)p1.size();
  if (matched < cfg_.min_inliers) {
    reason_ = "too-few-tracked";
    return;
  }

  // 光流场整体旋转量过小 <=> 近似纯平移，Essential 矩阵退化
  double rot_energy = 0.0;
  for (int i = 0; i < matched; ++i) {
    rot_energy += std::hypot(p2[i].x - p1[i].x, p2[i].y - p1[i].y);
  }
  if (rot_energy / matched < kMinBaselineRotation) {
    reason_ = "no-parallax-motion";
    return;
  }

  cv::Mat mask;
  const cv::Mat E = cv::findEssentialMat(p1, p2, cfg_.K, cv::RANSAC, cfg_.essential_prob,
                                         cfg_.essential_threshold, mask);
  if (E.empty()) {
    reason_ = "essential-failed";
    return;
  }
  int inl = cv::countNonZero(mask);
  if (inl < cfg_.min_inliers) {
    reason_ = "essential-low-inliers";
    return;
  }

  cv::Mat R_cv;
  cv::Mat t_cv;
  const int inl2 = cv::recoverPose(E, p1, p2, cv::Mat(cfg_.K), R_cv, t_cv, mask);
  if (inl2 < cfg_.min_inliers) {
    reason_ = "recoverpose-low-inliers";
    return;
  }

  RotMeasure m;
  m.t_prev = prev_t_;
  m.t_cur = t_cur;
  m.R_rel_c = toEigen(R_cv);
  m.matched = matched;
  m.inliers = inl2;
  if (!m.R_rel_c.allFinite()) {
    reason_ = "non-finite-rotation";
    return;
  }

  pending_.push_back(m);
  if ((int)pending_.size() > kMaxPending) pending_.pop_front();
  ++n_ok_;
  sum_inl_ += inl2;
  (void)inl;
  reason_ = "ok";
}

bool VisualRotation::pickMeasure(double t0, double t1, double slack, RotMeasure &out) {
  std::lock_guard<std::mutex> lock(mtx_);
  // 丢掉远早于本帧窗口的陈旧测量，避免积压后被当成当前测量使用
  while (!pending_.empty() && pending_.front().t_cur < t0 - 5.0 * slack) pending_.pop_front();

  int best = -1;
  double best_err = slack + 1e-9;
  for (int i = 0; i < (int)pending_.size(); ++i) {
    const double e = std::fabs(pending_[i].t_cur - t1);
    if (e <= best_err) {
      best_err = e;
      best = i;
    }
  }
  if (best < 0) return false;

  out = pending_[best];
  pending_.erase(pending_.begin(), pending_.begin() + best + 1);
  return true;
}

bool VisualRotation::pickLongMeasure(double t0, double t1, double slack, RotMeasure &out) {
  std::lock_guard<std::mutex> lock(mtx_);
  while (!pending_long_.empty() && pending_long_.front().t_cur < t0 - 5.0 * slack)
    pending_long_.pop_front();

  int best = -1;
  double best_err = slack + 1e-9;
  for (int i = 0; i < (int)pending_long_.size(); ++i) {
    const double e = std::fabs(pending_long_[i].t_cur - t1);
    if (e <= best_err) {
      best_err = e;
      best = i;
    }
  }
  if (best < 0) return false;

  out = pending_long_[best];
  pending_long_.erase(pending_long_.begin(), pending_long_.begin() + best + 1);
  return true;
}

}  // namespace vis
}  // namespace LI2Sup
