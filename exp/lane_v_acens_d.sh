#!/usr/bin/env bash
# M.36 A 线重定标（lane D）：污染复测 + huber 上界
#   m36_as_na01 首轮带 1.49 s odom_gap（判"污染"），9.851 m 不可直接用 ⇒ 复测
#   huber 只扫过 0.05→0.20（M.19 的 -31% 收益方向），上界 0.5 从未有数据；
#   既然新代码把 na 的最优点推到 0.3 之上，观测鲁棒核的合理宽度大概率也变
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=88
export WS_SETUP=/tmp/sl_install/setup.bash
export BAG=/home/ywj/slam_data/livox_record_drone
export PLANE_KNN_K=8 MAP_REP_MODE=0

PLANE_KNN_K=8 IMU_NBG=0.003 HUBER_BASE=0.20 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m36_as_na01_r2 6 0.25 0.1 0.1 0.25

PLANE_KNN_K=8 IMU_NBG=0.003 HUBER_BASE=0.50 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m36_ac_h05 6 0.25 0.3 0.1 0.25

echo "[laneV-ac-huber-up] done"
