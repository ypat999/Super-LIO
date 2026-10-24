#!/usr/bin/env bash
# M.25 阶段 1 截断类补跑（lane F，ROS_DOMAIN_ID=47）
# 原因：m25_tr0 / m25_h20_tr2 在 /tmp/sl_install（未含 Eigen 断言修复）上直接 abort，
# 修复后统一用 /tmp/sl_install2（阶段 2 代码默认全关 = 与原行为一致）重跑。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=47
export BAG=/home/ywj/slam_data/livox_record_drone
export WS_SETUP=/tmp/sl_install2/setup.bash
H20_ENV="NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 IMU_NBG=0.003 HUBER_BASE=0.20"

# 0) 先跑一格带埋点的截断实验：确认 cond 是否真的越过阈值（deg_cond/deg_axes 落 debug.csv）
DEBUG_LEVEL=1 DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=0.001 DEGEN_TRANS_ONLY=0 \
  ./run_one.sh m25_tr0_dbg 6 0.25 0.1 0.1 0.25

# 1.1 三模式（mode 0 / 2 曾被断言打死；mode 1 已在 A 跑过，这里只补缺的）
DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=0.001 DEGEN_TRANS_ONLY=0 \
  ./run_one.sh m25_tr0 6 0.25 0.1 0.1 0.25
DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=0.001 DEGEN_TRANS_ONLY=2 \
  ./run_one.sh m25_tr2b 6 0.25 0.1 0.1 0.25

# 1.2 h20 基座 + 截断
env $H20_ENV DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=0.001 \
  ./run_one.sh m25_h20_tr2 6 0.25 0.1 0.1 0.25

# 1.3 阈值/λ 灵敏度
DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=1e-2 DEGENERACY_THRESHOLD=50.0 \
  ./run_one.sh m25_t50_l2 6 0.25 0.1 0.1 0.25
DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=1e-4 DEGENERACY_THRESHOLD=50.0 \
  ./run_one.sh m25_t50_l4 6 0.25 0.1 0.1 0.25
DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=1e-2 DEGENERACY_THRESHOLD=200.0 \
  ./run_one.sh m25_t200_l2 6 0.25 0.1 0.1 0.25
DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=1e-4 DEGENERACY_THRESHOLD=200.0 \
  ./run_one.sh m25_t200_l4 6 0.25 0.1 0.1 0.25

echo "[laneF] all done"
