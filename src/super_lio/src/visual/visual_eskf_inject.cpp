/// B2：视觉相对旋转 -> ESKF 注入。
///
/// 帧与符号约定（与 ESKF 一致，逐条对照 src/lio/super_lio.cpp 的激光 obs 与
/// src/lio/ESKF.cpp 的 Update/UpdateObserve）：
///   * ESKF 误差状态顺序 [δθ(0..2) δp(3..5) δv δbg δba δg]，旋转误差按
///     `R_ = R_ * Exp(dxθ)` 的方式修正，即 R_true = R_nom · Exp(δθ)，δθ 在机体系。
///     （ESKF.cpp::Update 第一行；super_lio.h 状态顺序注释）
///   * obs 回调写入的是 HT_Vinv_H / HT_Vinv_r，激光路径的约定是
///     J = ∂e/∂δx（`sum_HTVH += J w Jᵀ`、`sum_HTVr -= J w e`），
///     见 super_lio.cpp::Observe 内的累加。本文件沿用同一约定。
///
/// 相机系 -> 机体系的相对旋转（recoverPose 的约定是 X_cur = R·X_prev + t，
/// 即 P2=[R|t]、P1=[I|0]，R 把 prev 相机系向量映到 cur 相机系）：
///   R_rel_c = R_c(cur) <- c(prev)
///   R_wc(t) = R_wb(t) · Rclᵀ         （Rcl: body(lidar)->cam）
///   R_rel_c = Rcl · R_wb(cur)ᵀ · R_wb(prev) · Rclᵀ
///   ⇒ R_meas_rel := R_wb(cur)ᵀ R_wb(prev) = Rclᵀ · R_rel_c · Rcl
/// 预测取同一形式，只是 cur 用当前状态、prev 用「冻结的后验参考节点」：
///   R_pred_rel = R_kᵀ · R_p
///   e = Log(R_meas_relᵀ · R_pred_rel)
///   R_k -> R_k·Exp(δθ)  ⇒  R_pred_rel -> Exp(-δθ)·R_pred_rel
///   ⇒ H = ∂e/∂δθ = -J_l(e)·R_pred_relᵀ    （小角度时 ≈ -I）
/// 参考节点 R_p 是常量，故本路测量对 δbg 没有解析依赖；陀螺零偏通过
/// P 的 (θ,bg) 交叉协方差被间接修正（K = P Hᵀ(...)⁻¹，见 ESKF::UpdateObserve）。
/// 平移/速度/加计行全部置零：只约束旋转。
#include "lio/super_lio.h"

#include <glog/logging.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace LI2Sup {

namespace {

std::once_flag g_visual_once;

Eigen::Matrix3d hat(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  // clang-format off
  m <<      0, -v(2),  v(1),
         v(2),      0, -v(0),
        -v(1),  v(0),      0;
  // clang-format on
  return m;
}

Eigen::Vector3d so3Log(const Eigen::Matrix3d &R) {
  Eigen::AngleAxisd aa(R);
  return aa.axis() * aa.angle();
}

/// 左雅可比 J_l(φ) = I + (1-cosθ)/θ² [φ]× + (θ-sinθ)/θ³ [φ]×²
Eigen::Matrix3d leftJacobian(const Eigen::Vector3d &phi) {
  const double theta = phi.norm();
  const Eigen::Matrix3d K = hat(phi);
  if (theta < 1e-8) return Eigen::Matrix3d::Identity() - 0.5 * K;
  const double a = (1.0 - std::cos(theta)) / (theta * theta);
  const double b = (theta - std::sin(theta)) / (theta * theta * theta);
  return Eigen::Matrix3d::Identity() + a * K + b * (K * K);
}

/// 右雅可比 J_r(φ) = J_l(-φ)（ESKF.cpp::Predict 里 f_x(0,9) = -dt*J_r 同一约定）
Eigen::Matrix3d rightJacobian(const Eigen::Vector3d &phi) { return leftJacobian(-phi); }

/// Exp: so(3) -> SO(3)，与 BASIC::SO3::Exp 同一 Rodrigues
Eigen::Matrix3d expSO3(const Eigen::Vector3d &phi) {
  const double th = phi.norm();
  const Eigen::Matrix3d K = hat(phi);
  if (th < 1e-8) return Eigen::Matrix3d::Identity() + K + 0.5 * (K * K);
  return Eigen::Matrix3d::Identity() + std::sin(th) / th * K +
         (1.0 - std::cos(th)) / (th * th) * (K * K);
}

/// 解析雅可比一旦与有限差分不符就永久改用差分（见 VisualObserveBias）
bool g_b3_use_fd = false;

/// 轻微正交化（recoverPose 返回值已是正交阵，这里只兜住浮点误差）
Eigen::Matrix3d orthonormal(const Eigen::Matrix3d &R) {
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d O = svd.matrixU() * svd.matrixV().transpose();
  if (O.determinant() < 0.0) {
    Eigen::Matrix3d U = svd.matrixU();
    U.col(2) *= -1.0;
    O = U * svd.matrixV().transpose();
  }
  return O;
}

}  // namespace

void SuperLIO::buildVisualRotation() {
  vis::VisualRotationConfig c;
  c.enable = g_visual_enable;
  if (!c.enable) return;

  c.img_time_offset = g_visual_img_time_offset;
  c.frame_slack = g_visual_frame_slack;
  c.K = cv::Matx33d(g_visual_cam_fx, 0.0, g_visual_cam_cx, 0.0, g_visual_cam_fy, g_visual_cam_cy,
                    0.0, 0.0, 1.0);
  for (int i = 0; i < 4; ++i) c.dist[i] = g_visual_cam_dist[i];
  c.width = g_visual_cam_width;
  c.height = g_visual_cam_height;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) c.R_cl(i, j) = g_visual_Rcl[i * 3 + j];
  c.max_features = g_visual_klt_max_features;
  c.min_distance = g_visual_klt_min_distance;
  c.fb_error = g_visual_klt_fb_error;
  c.pyramid_levels = g_visual_klt_pyramid_levels;
  c.use_fast = g_visual_klt_use_fast;
  c.fast_threshold = g_visual_klt_fast_threshold;
  c.lk_window = g_visual_klt_lk_window;
  c.essential_threshold = g_visual_essential_threshold;
  c.essential_prob = g_visual_essential_prob;
  c.min_inliers = g_visual_min_inliers;
  c.sigma_rot = g_visual_sigma_rot;

  // B3：长基线图像对（关掉时这些字段全为默认，前端行为与 B2 逐位一致）
  c.long_enable = g_visual_b3_enable;
  c.baseline_dt = g_visual_b3_baseline_dt;
  c.long_pyramid_levels = g_visual_b3_pyramid_levels;
  c.long_node_keep = std::max(5.0, 2.0 * g_visual_b3_baseline_dt + 3.0);

  vis_R_cl_ = c.R_cl.cast<float>();
  visual_ = std::make_unique<vis::VisualRotation>(c);

  LOG(INFO) << GREEN << " ---> [VisRot] 视觉相对旋转已启用: topic=" << g_visual_image_topic
            << " sigma_rot=" << g_visual_sigma_rot << " rad"
            << " min_inliers=" << g_visual_min_inliers
            << " t_offset=" << g_visual_img_time_offset << " slack=" << g_visual_frame_slack
            << RESET;
}

void SuperLIO::onVisualImage(const cv::Mat &image, double stamp) {
  if (!g_visual_enable) return;
  std::call_once(g_visual_once, [this]() { buildVisualRotation(); });
  if (!visual_) return;
  visual_->pushImage(image, stamp);
}

bool SuperLIO::anchorRotation(double t, double tol, BASIC::M3 &R_out, double &t_used) const {
  if (vis_pose_hist_.empty()) return false;
  // 最近邻：候选节点必须是「过去的后验」，取时间上最贴近 t 的一个
  size_t best = 0;
  double best_err = std::fabs(vis_pose_hist_[0].t - t);
  for (size_t i = 1; i < vis_pose_hist_.size(); ++i) {
    const double e = std::fabs(vis_pose_hist_[i].t - t);
    if (e < best_err) {
      best_err = e;
      best = i;
    }
  }
  // tol 必须小于节点间隔(=lidar 帧周期 0.1 s)的一半，否则会跨格误匹配
  if (best_err > tol) return false;
  R_out = vis_pose_hist_[best].R;
  t_used = vis_pose_hist_[best].t;
  return true;
}

void SuperLIO::VisualObserve() {
  if (!g_visual_enable) return;
  std::call_once(g_visual_once, [this]() { buildVisualRotation(); });

  const double t0 = measures_.lidar.start_time;
  const double t1 = measures_.lidar.end_time;

  // 每帧记录后验姿态，供后续帧当作冻结参考节点使用（只保留 3 s）
  {
    VisPoseNode node;
    node.t = t1;
    node.R = kf_->GetSE3().R_;
    vis_pose_hist_.push_back(node);
    while (!vis_pose_hist_.empty() && (t1 - vis_pose_hist_.front().t) > 3.0)
      vis_pose_hist_.pop_front();
  }

  // B2 注入本体开关：关掉后视觉前端与 B3 照旧，只是不再做瞬时旋转更新，
  // 这样 B2/B3 可以各自独立开关（前端总开关仍是 lio.visual.enable）。
  if (!g_visual_b2_enable || !visual_) return;

  const auto wall_now = std::chrono::steady_clock::now();
  auto fmt_mean = [this](const Eigen::Vector3d &sum) {
    const Eigen::Vector3d m = vis_e_cnt_ > 0 ? Eigen::Vector3d(sum / double(vis_e_cnt_))
                                             : Eigen::Vector3d::Zero();
    std::ostringstream os;
    os << "[" << (m * 1e3).transpose() << "]";
    return os.str();
  };
  auto log_status = [&](const char *why) {
    if (std::chrono::duration<double>(wall_now - vis_last_stat_wall_).count() < 5.0) return;
    vis_last_stat_wall_ = wall_now;
    LOG(INFO) << "[VisRot] none(" << why << "): win=" << (t1 - t0) * 1e3 << "ms gap="
              << visual_->bestGap(t1) * 1e3 << "ms pend=" << visual_->pendingSize()
              << " hist=" << vis_pose_hist_.size() << " imgs=" << visual_->imagesRecv()
              << " pairs=" << visual_->pairsTotal() << " ok=" << visual_->pairsOk()
              << " mean_inl=" << visual_->meanInliers()
              << " reason=" << visual_->lastReason() << " applied=" << vis_applied_
              << " e_mean_b=" << fmt_mean(vis_e_sum_b_)
              << " e_mean_w=" << fmt_mean(vis_e_sum_w_) << " mrad n_e=" << vis_e_cnt_;
  };

  // 本 bag 的回放延迟：lidar header ≈ 比其 recorded 时刻晚 112 ms，图像 header ≈ 晚 54 ms，
  // 所以 VisualObserve 时刻能拿到的最新图像对，其 t_cur 比当前状态时刻 t1 落后约 1.5 帧。
  // 放宽取窗，用「等量平移」方式对齐两端（见下面 delta），使预测区间与测量区间只差一个
  // 固定平移量，恒定角速度下两者扫过的旋转相同 ⇒ 残差只来自角加速度项。
  vis::RotMeasure m;
  if (!visual_->pickMeasure(t0, t1, 4.0 * g_visual_frame_slack, m)) {
    log_status("no-measure");
    return;
  }
  ++vis_frames_;

  const double delta = t1 - m.t_cur;              ///< 测量整体滞后量(s)
  const double pair_dt = m.t_cur - m.t_prev;      ///< 图像对实际间隔(s)
  BASIC::M3 R_p = BASIC::M3::Identity();
  double t_p = 0.0;
  // 参考节点目标 = t_prev + delta = t1 - pair_dt（自动随丢帧后移一格）
  if (!anchorRotation(m.t_prev + delta, 0.04, R_p, t_p)) {
    log_status("no-anchor");
    if (g_debug_level > 0) {
      LOG(INFO) << "[VisRot] skip(no-anchor): target=" << m.t_prev + delta - t1
                << " hist=" << vis_pose_hist_.size();
    }
    return;
  }

  // 相机系相对旋转 -> 机体系同一形式（curᵀ·prev）
  const Eigen::Matrix3d R_cl_d = vis_R_cl_.cast<double>();
  const Eigen::Matrix3d R_meas_rel = orthonormal(R_cl_d.transpose() * m.R_rel_c * R_cl_d);
  const double sigma = std::max(1e-4, visual_->sigma_rot());
  const double inv_s2 = 1.0 / (sigma * sigma);
  const Eigen::Matrix3f R_p_f = R_p;

  const BASIC::SE3 pose_before = kf_->GetSE3();
  // 增益探针：注入前的姿态方差与"单轴解耦"下的有效增益 P/(P+σ²)。
  // gain≈1 => 量测被完全采信，修正没留住只可能是被后续激光重新拉回；
  // gain<<1 => 是 P 本身太小（激光把 yaw 方差压死了），σ 不是瓶颈。
  const Eigen::Vector3d P_rot = kf_->GetCovDiagRot().cast<double>();
  const double gain_z = P_rot.z() / (P_rot.z() + sigma * sigma);

  // ---- χ² 门控（补 B2 缺的那道闸门；阈值 lio.visual.chi2_thresh 与 B3 独立）----
  // 在当前名义状态上算一次残差与雅可比：
  //   e  = Log(R_meas_relᵀ · R_kᵀ R_p)
  //   H  = ∂e/∂δθ = -J_l(e)·R_pred_relᵀ        （与下面 obs 回调里同一式）
  //   χ² = eᵀ(HPHᵀ + σ²I)⁻¹e   （3 自由度）
  // σ 很小时 P 里的姿态方差远小于 σ²，正常帧 χ² 也小；一旦视觉与激光后验互相
  // 矛盾（外参失配、退化场景、KLT 误匹配），χ² 会爆掉，整帧丢弃而不注入。
  double chi2_val = -1.0;
  {
    const Eigen::Matrix3d R_k0 = pose_before.R_.cast<double>();
    const Eigen::Matrix3d R_pred0 = orthonormal(R_k0.transpose() * R_p_f.cast<double>());
    const Eigen::Vector3d e0 = so3Log(orthonormal(R_meas_rel.transpose() * R_pred0));
    if (!e0.allFinite()) return;
    const Eigen::Matrix3d H0 = -leftJacobian(e0) * R_pred0.transpose();
    Eigen::Matrix<double, 3, 18> H18 = Eigen::Matrix<double, 3, 18>::Zero();
    H18.block<3, 3>(0, 0) = H0;
    const Eigen::Matrix3d Pzz = (H18 * kf_->GetCov().cast<double>() * H18.transpose()).eval();
    Eigen::Matrix3d S =
        0.5 * (Pzz + Pzz.transpose()) + sigma * sigma * Eigen::Matrix3d::Identity();
    const double chi2 = e0.transpose() * S.inverse() * e0;
    if (!std::isfinite(chi2)) return;
    chi2_val = chi2;  // 阈值 <=0 时只统计不丢弃，便于比较"门掉了什么"
    vis_chi2_sum_ += chi2;
    vis_chi2_max_ = std::max(vis_chi2_max_, chi2);
    if (g_visual_chi2_thresh > 0.0 && chi2 > g_visual_chi2_thresh) {
      ++vis_rejected_;
      if (g_debug_level > 0 || (vis_rejected_ % 50) == 1) {
        // 决定性实验埋点：拒绝帧也带 Pstd，才能做 acc(P) 自适应曲线
        LOG(WARNING) << "[VisB2] reject chi2=" << chi2 << " > " << g_visual_chi2_thresh
                     << " t=" << t1 << " sigma=" << sigma << " |e|=" << e0.norm() * 1e3
                     << "mrad inl=" << m.inliers << " rej=" << vis_rejected_
                     << " Pstd=[" << (P_rot.cwiseSqrt() * 1e3).transpose() << "]mrad";
      }
      return;
    }
  }

  double dx_r_used = 0.0;
  double e_norm_used = 0.0;
  Eigen::Vector3d e_used = Eigen::Vector3d::Zero();
  bool e_ok = false;

  kf_->UpdateObserve([&](const ESKF::KFState &kf_state, BASIC::M6 &HTVH, BASIC::V6 &HTVr) {
    HTVH.setZero();
    HTVr.setZero();

    const Eigen::Matrix3d R_k = kf_state.pose.R_.cast<double>();
    const Eigen::Matrix3d R_pred_rel = orthonormal(R_k.transpose() * R_p_f.cast<double>());
    const Eigen::Vector3d e = so3Log(orthonormal(R_meas_rel.transpose() * R_pred_rel));
    if (!e.allFinite()) return;

    const Eigen::Matrix3d H = -leftJacobian(e) * R_pred_rel.transpose();

    HTVH.block<3, 3>(0, 0) = (H.transpose() * H * inv_s2).cast<float>();
    HTVr.head<3>() = (-H.transpose() * e * inv_s2).cast<float>();
    e_used = e;
    e_ok = true;
    e_norm_used = e.norm();
  });

  // 偏置探针累加（只在成功注入后）：e 在 prev 机体系，R_p·e 即世界系
  if (e_ok) {
    vis_e_sum_b_ += e_used;
    vis_e_sum_w_ += R_p.cast<double>() * e_used;
    ++vis_e_cnt_;
  }

  const BASIC::SE3 pose_after = kf_->GetSE3();
  const Eigen::Matrix3d dR =
      (pose_before.R_.transpose() * pose_after.R_).cast<double>();
  const double tr = std::min(1.0, std::max(-1.0, (dR.trace() - 1.0) * 0.5));
  dx_r_used = std::acos(tr);
  ++vis_applied_;
  vis_dx_r_sum_ += dx_r_used;
  vis_dx_r_max_ = std::max(vis_dx_r_max_, dx_r_used);

  // ---- 安全性量化 ----
  // B2 的 H 只占旋转 3 行，但更新量 dx = P Hᵀ(...)⁻¹ e 里的位置/速度分量来自
  // P 的 (θ,p) 交叉协方差：σ 压到 1e-4 时这一通道能把位置一次拽走几十厘米，
  // 这就是之前 0.42 m 单帧位移的来源。故每次注入记录 Δp 与世界系 Δyaw。
  const double dp_now = (pose_after.t_ - pose_before.t_).cast<double>().norm();
  const Eigen::Matrix3d dRw = pose_after.R_.cast<double>() *
                              pose_before.R_.cast<double>().transpose();
  const double dyaw_now = so3Log(dRw).z();
  vis_dp_sum_ += dp_now;
  vis_dp_max_ = std::max(vis_dp_max_, dp_now);
  if (dp_now > 0.05) ++vis_dp_over_;
  vis_dyaw_max_ = std::max(vis_dyaw_max_, std::fabs(dyaw_now));
  if (std::fabs(dyaw_now) > 0.02) ++vis_dyaw_over_;

  const auto now_wall = std::chrono::steady_clock::now();
  const bool every = (vis_applied_ % 20) == 1;
  if (g_debug_level > 0 || every ||
      std::chrono::duration<double>(now_wall - vis_last_log_wall_).count() > 5.0) {
    vis_last_log_wall_ = now_wall;
    LOG(INFO) << "[VisRot] inl=" << m.inliers << "/" << m.matched
              << " delta=" << delta * 1e3 << "ms pair_dt=" << pair_dt * 1e3
              << "ms danchor=" << (m.t_prev + delta - t_p) * 1e3
              << "ms |e|=" << e_norm_used * 1e3 << "mrad e=["
              << (e_used * 1e3).transpose() << "] ew=["
              << ((R_p.cast<double>() * e_used) * 1e3).transpose()
              << "] mrad dx_r=" << dx_r_used * 1e3
              << "mrad mean_dx_r=" << (vis_dx_r_sum_ / double(vis_applied_)) * 1e3
              << " max_dx_r=" << vis_dx_r_max_ * 1e3 << "mrad sigma=" << sigma
              << " applied=" << vis_applied_ << "/" << vis_frames_
              << " pairs=" << visual_->pairsTotal() << " ok=" << visual_->pairsOk()
              << " mean_inl=" << visual_->meanInliers()
              << " e_mean_b=" << fmt_mean(vis_e_sum_b_)
              << " e_mean_w=" << fmt_mean(vis_e_sum_w_) << " mrad n_e=" << vis_e_cnt_
              << " Pstd=[" << (P_rot.cwiseSqrt() * 1e3).transpose() << "]mrad gain_z=" << gain_z
              << " dth_z=" << gain_z * e_used.z() * 1e3 << "mrad"
              << " chi2=" << chi2_val << " thresh=" << g_visual_chi2_thresh
              << " acc=" << vis_applied_ << "/" << (vis_applied_ + vis_rejected_) << "("
              << (vis_applied_ + vis_rejected_ > 0
                      ? 100.0 * double(vis_applied_) / double(vis_applied_ + vis_rejected_)
                      : 100.0)
              << "%)" << " dp=" << dp_now << "m dyaw=" << dyaw_now * 1e3 << "mrad";
    LOG(INFO) << "[VisB2gate] t=" << t1 << " chi2_max=" << vis_chi2_max_
                          << " chi2_mean="
                          << (vis_applied_ + vis_rejected_ > 0
                                  ? vis_chi2_sum_ / double(vis_applied_ + vis_rejected_)
                                  : 0.0)
                          << " rej=" << vis_rejected_ << "/"
                          << (vis_applied_ + vis_rejected_) << " dp_max=" << vis_dp_max_
                          << "m dp_mean="
                          << (vis_applied_ > 0 ? vis_dp_sum_ / double(vis_applied_) : 0.0)
                          << "m over5cm=" << vis_dp_over_ << " dyaw_max="
                          << vis_dyaw_max_ * 1e3 << "mrad over20mrad=" << vis_dyaw_over_
                          << " max_dx_r=" << vis_dx_r_max_ * 1e3 << "mrad";
  }
}

// ===========================================================================
//  B3：视觉长基线 -> 陀螺零偏观测
// ===========================================================================
//
// 与 B2 的本质区别：预测项不再用状态姿态差 R_curᵀ·R_prev(cached)（参考节点是缓存
// 常量 ⇒ ∂e/∂δbg ≡ 0），而用「基线内未补偿陀螺积分 + 当前 bg 补偿」：
//     Φ_base = Φ(t_cur) − Φ(t_prev),  Φ(t) = Σ ω_meas·dt
//     φ      = Φ_base − bg·Δt                （机体系右扰动增量 prevᵀ·cur）
//     ΔR_pred(与 R_meas_rel 同形式 curᵀ·prev) = Exp(−φ) = Exp(bg·Δt − Φ_base) = Exp(ψ)
//     e      = Log(R_meas_relᵀ · Exp(ψ))
// 姿态完全不在残差里 ⇒ ∂e/∂δθ ≡ 0。∂e/∂δbg 的推导（逐步，不抄教科书）：
//   e = Log(A·Exp(ψ))，B = Exp(e) = A·Exp(ψ)，ψ = bg·Δt − Φ_base
//   本包 leftJacobian(=I+(1-cos)/θ²[φ]×+(θ-sin)/θ³[φ]×²) 满足 δExp(φ)=(J_l(φ)δφ)^∧·Exp(φ)
//   ⇒ δB = A·(J_l(ψ)δψ)^∧·Exp(ψ) = (A·J_l(ψ)δψ)^∧·B ；又 δB=(J_l(e)δe)^∧·B
//   ⇒ δe = J_l(e)⁻¹·A·J_l(ψ)·δψ，δψ = Δt·δbg
//   ⇒ H = ∂e/∂δbg = Δt·J_l(e)⁻¹·A·J_l(ψ)   （ψ,A→0 时 ≈ +Δt·I，正号）
// 注意 A = R_meas_relᵀ 本身是基线内的相对旋转（转 10° 时 ‖ψ‖≈0.17 rad），
// 漏掉 Ad(A) 会在非对角上留下 ~10-30% 误差 —— 由中心差分抓出并改正
// （原写 J_r(e)J_r(ψ) 与 J_r(e)J_l(ψ) 都不对，见 [VisB3-FD] 打印与离线核对）。
// 符号不再靠推教科书，用中心差分对同一 residual() 复核（见 fd_check）。
void SuperLIO::AccumulateGyroPhase() {
  // 只读 measures_.imu 的副本并排序：本包已出现过外部节点污染回放导致时间戳
  // 乱序 ±3 s 的事故，乱序直接积分会让 Φ 完全失真。绝不修改原 deque
  // （Propagation_Undistort 之后还要按原顺序喂给别的逻辑）。
  std::vector<IMUData> s;
  s.reserve(measures_.imu.size());
  for (const auto &i : measures_.imu) s.push_back(i);
  std::sort(s.begin(), s.end(),
            [](const IMUData &a, const IMUData &b) { return a.secs < b.secs; });

  for (const auto &i : s) {
    const double t = i.secs;
    const Eigen::Vector3d g = i.gyr.cast<double>();
    if (!vis_phi_init_) {
      vis_phi_init_ = true;
      vis_phi_last_t_ = t;
      vis_phi_last_.setZero();
      vis_phi_last_gyr_ = g;
      vis_phi_hist_.push_back(PhiSample{t, vis_phi_last_});
      continue;
    }
    if (t <= vis_phi_last_t_ + 1e-9) {  // 重复投递或乱序尾巴：跳过，不污染积分
      ++vis_phi_unordered_;
      continue;
    }
    const double dt = t - vis_phi_last_t_;
    if (g_max_imu_dt > 0 && dt > g_max_imu_dt) {  // 断档：不积分，只重新对齐基准
      ++vis_phi_gap_;
      vis_phi_last_t_ = t;
      vis_phi_last_gyr_ = g;
      vis_phi_hist_.push_back(PhiSample{t, vis_phi_last_});
      continue;
    }
    // 与 ESKF::Predict 的 body_omega_ = 0.5*(gyr_i + gyr_{i-1}) − bg 同一梯形律
    vis_phi_last_ += 0.5 * (vis_phi_last_gyr_ + g) * dt;
    vis_phi_last_t_ = t;
    vis_phi_last_gyr_ = g;
    vis_phi_hist_.push_back(PhiSample{t, vis_phi_last_});
  }

  const double keep = std::max(6.0, 3.0 * g_visual_b3_baseline_dt + 3.0);
  while (!vis_phi_hist_.empty() && vis_phi_last_t_ - vis_phi_hist_.front().t > keep)
    vis_phi_hist_.pop_front();
}

bool SuperLIO::GyroPhaseAt(double t, Eigen::Vector3d &out) const {
  const size_t n = vis_phi_hist_.size();
  if (n < 2) return false;
  if (t < vis_phi_hist_.front().t || t > vis_phi_hist_.back().t) return false;
  size_t lo = 0, hi = n - 1;  // 二分找第一个 .t >= t 的下标
  while (lo < hi) {
    const size_t mid = (lo + hi) / 2;
    if (vis_phi_hist_[mid].t < t)
      lo = mid + 1;
    else
      hi = mid;
  }
  const size_t b = lo;
  const size_t a = b > 0 ? b - 1 : 0;
  const double span = vis_phi_hist_[b].t - vis_phi_hist_[a].t;
  if (span <= 0.0) {
    out = vis_phi_hist_[b].phi;
    return true;
  }
  const double w = (t - vis_phi_hist_[a].t) / span;
  out = (1.0 - w) * vis_phi_hist_[a].phi + w * vis_phi_hist_[b].phi;
  return true;
}

void SuperLIO::VisualObserveBias() {
  if (!g_visual_enable || !g_visual_b3_enable) return;
  std::call_once(g_visual_once, [this]() { buildVisualRotation(); });
  if (!visual_) return;

  const double t0 = measures_.lidar.start_time;
  const double t1 = measures_.lidar.end_time;
  ++vis_b3_frames_;

  vis::RotMeasure m;
  if (!visual_->pickLongMeasure(t0, t1, 4.0 * g_visual_frame_slack, m)) return;

  const double dt = m.t_cur - m.t_prev;
  if (dt < 0.5 * g_visual_b3_baseline_dt) return;  // 基线不足，等效 σ_bg 太松

  Eigen::Vector3d phi_cur, phi_prev;
  if (!GyroPhaseAt(m.t_cur, phi_cur) || !GyroPhaseAt(m.t_prev, phi_prev)) {
    ++vis_b3_no_phase_;
    return;
  }
  const Eigen::Vector3d phi_base = phi_cur - phi_prev;

  // 机体系约定与 B2 完全一致，不动：R_meas_rel := R_wb(cur)ᵀ R_wb(prev)
  const Eigen::Matrix3d R_cl_d = vis_R_cl_.cast<double>();
  const Eigen::Matrix3d R_meas_rel = orthonormal(R_cl_d.transpose() * m.R_rel_c * R_cl_d);
  const Eigen::Matrix3d A = R_meas_rel.transpose();  // 残差里的固定左乘项

  const double sigma = std::max(1e-4, g_visual_b3_sigma_rot);
  const double inv_s2 = 1.0 / (sigma * sigma);

  auto residual = [&](const Eigen::Vector3d &bg) {
    return so3Log(orthonormal(A * expSO3(bg * dt - phi_base)));
  };

  const Eigen::Vector3d bg_pre = kf_->GetSysState().bg.cast<double>();
  const Eigen::Vector3d e0 = residual(bg_pre);
  if (!e0.allFinite()) return;

  // 解析雅可比 ∂e/∂δbg = Δt·J_l(e)⁻¹·A·J_l(ψ)（ψ = bg·Δt − Φ_base，A = R_meas_relᵀ）
  const Eigen::Vector3d psi = bg_pre * dt - phi_base;
  const Eigen::Matrix3d H_an = dt * leftJacobian(e0).inverse() * A * leftJacobian(psi);
  // 中心差分复核（对同一 residual 加 ±1e-3 rad/s 扰动）
  Eigen::Matrix3d H_fd = Eigen::Matrix3d::Zero();
  if (g_visual_b3_fd_check) {
    const double h = 1e-3;
    for (int j = 0; j < 3; ++j) {
      Eigen::Vector3d d = Eigen::Vector3d::Zero();
      d(j) = h;
      H_fd.col(j) = (residual(bg_pre + d) - residual(bg_pre - d)) / (2.0 * h);
    }
    const double rel = (H_an - H_fd).norm() / std::max(1e-12, H_fd.norm());
    vis_b3_fd_max_ = std::max(vis_b3_fd_max_, rel);
    ++vis_b3_fd_n_;
    if (vis_b3_fd_n_ <= 2) {
      // 符号校核证据：∂e/∂δbg 应为 +Δt·I 量级（右扰动约定，见文件头推导）
      LOG(INFO) << "[VisB3-FD] #" << vis_b3_fd_n_ << " dt=" << dt
                << " bg=[" << (bg_pre * 1e3).transpose() << "]mrad/s"
                << " e0=[" << (e0 * 1e3).transpose() << "]mrad"
                << "\nH_an=\n" << H_an << "\nH_fd=\n" << H_fd << "rel=" << rel;
    }
    if (rel > 0.2 && !g_b3_use_fd) {
      g_b3_use_fd = true;
      LOG(WARNING) << "[VisB3] 解析雅可比与有限差分不符 rel=" << rel
                   << "，改用有限差分（H_an=\n" << H_an << "\nH_fd=\n" << H_fd << ")";
    }
  }
  const Eigen::Matrix3d &H = (g_visual_b3_fd_check && g_b3_use_fd) ? H_fd : H_an;

  // ---- χ² 门控：eᵀ(HPHᵀ+R)⁻¹e ----
  Eigen::Matrix<double, 3, 18> H18 = Eigen::Matrix<double, 3, 18>::Zero();
  H18.block<3, 3>(0, 9) = H;
  const Eigen::Matrix3d Pzz = (H18 * kf_->GetCov().cast<double>() * H18.transpose()).eval();
  Eigen::Matrix3d S = 0.5 * (Pzz + Pzz.transpose()) + sigma * sigma * Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d Sinv = S.inverse();
  const double chi2 = e0.transpose() * Sinv * e0;
  if (!std::isfinite(chi2)) return;
  if (chi2 > g_visual_b3_chi2_thresh) {
    ++vis_b3_rejected_;
    if (g_debug_level > 0 || (vis_b3_rejected_ % 50) == 1) {
      LOG(WARNING) << "[VisB3] reject chi2=" << chi2 << " > " << g_visual_b3_chi2_thresh
                   << " t=" << t1 << " dt=" << dt << " |e|=" << e0.norm() * 1e3
                   << "mrad inl=" << m.inliers << " rej=" << vis_b3_rejected_;
    }
    return;
  }

  const Eigen::Matrix3d R_before = kf_->GetSE3().R_.cast<double>();

  kf_->UpdateObserveFull([&](const ESKF::KFState &, BASIC::M18 &HTVH, BASIC::V18 &HTVr) {
    HTVH.setZero();
    HTVr.setZero();
    // 迭代中 nominal bg 已被 Update() 改掉，残差必须在当前迭代点重算
    const Eigen::Vector3d bg = kf_->GetSysState().bg.cast<double>();
    const Eigen::Vector3d e = residual(bg);
    if (!e.allFinite()) return;
    HTVH.block<3, 3>(9, 9) = (H.transpose() * H * inv_s2).cast<float>();
    HTVr.block<3, 1>(9, 0) = (-H.transpose() * e * inv_s2).cast<float>();
  });

  const Eigen::Vector3d bg_post = kf_->GetSysState().bg.cast<double>();
  const Eigen::Vector3d d_bg = bg_post - bg_pre;
  ++vis_b3_applied_;
  vis_b3_chi2_sum_ += chi2;
  vis_b3_chi2_max_ = std::max(vis_b3_chi2_max_, chi2);
  vis_b3_dxbg_sum_ += d_bg.norm();
  vis_b3_dxbg_max_ = std::max(vis_b3_dxbg_max_, d_bg.norm());
  vis_b3_resid_sum_ += e0.norm();

  const BASIC::SE3 pose_after = kf_->GetSE3();
  const Eigen::Matrix3d dR = R_before.transpose() * pose_after.R_.cast<double>();
  const double dx_r = std::acos(std::min(1.0, std::max(-1.0, (dR.trace() - 1.0) * 0.5)));

  // bg_z 时间序列：每次注入一行，判断 B3 是否真在修零偏的关键证据
  LOG(INFO) << "[VisB3bg] t=" << t1 << " bgz=" << bg_post.z() * 1e3
            << " dxbg_z=" << d_bg.z() * 1e3 << " mrad/s chi2=" << chi2;

  // 每次注入的详细诊断（含接受率、|δbg|、等效 σ_bg）
  const double acc_rate = vis_b3_applied_ + vis_b3_rejected_ > 0
                              ? 100.0 * double(vis_b3_applied_) /
                                    double(vis_b3_applied_ + vis_b3_rejected_)
                              : 100.0;
  LOG_EVERY_N(INFO, 10) << "[VisB3] t=" << t1 << " dt=" << dt
                        << " |e|=" << e0.norm() * 1e3 << "mrad e=[" << (e0 * 1e3).transpose()
                        << "] chi2=" << chi2 << " acc=" << vis_b3_applied_ << "/"
                        << (vis_b3_applied_ + vis_b3_rejected_) << "(" << acc_rate << "%)"
                        << " dxbg=[" << (d_bg * 1e3).transpose() << "]mrad/s"
                        << " bgz=" << bg_post.z() * 1e3 << "mrad/s bg=["
                        << (bg_post * 1e3).transpose() << "]mrad/s dx_r=" << dx_r * 1e3
                        << "mrad sigma_eq_bg=" << sigma / dt * 1e3 << "mrad/s"
                        << " inl=" << m.inliers << "/" << m.matched
                        << " phi=[" << (phi_base / dt * 1e3).transpose() << "]mrad/s"
                        << " fd_max_rel=" << vis_b3_fd_max_
                        << " pairs=" << visual_->longPairsTotal() << " ok=" << visual_->longPairsOk()
                        << " mean_inl=" << visual_->longMeanInliers()
                        << " reason=" << visual_->lastLongReason()
                        << " no_phase=" << vis_b3_no_phase_ << " gap=" << vis_phi_gap_
                        << " unordered=" << vis_phi_unordered_;
}

}  // namespace LI2Sup
