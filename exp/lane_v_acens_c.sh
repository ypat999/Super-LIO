#!/usr/bin/env bash
# M.36 A 线重定标（lane C）：na 上界探索，K=8 固定
#   敏感度已给出单调改善：na 0.01 发散 / 0.1 → 9.85 / 0.3 → 0.562
#   M.26 当年在"nw on + hub0.20"组合下只扫到 0.3（基座组合是反向的，
#   0.01→0.552、0.3→2.150），所以 0.3 以上从未有过数据 ⇒ 补 0.5 / 1.0
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=87
export WS_SETUP=/tmp/sl_install/setup.bash
export BAG=/home/ywj/slam_data/livox_record_drone
export PLANE_KNN_K=8 MAP_REP_MODE=0

PLANE_KNN_K=8 IMU_NBG=0.003 HUBER_BASE=0.20 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m36_ac_na05 6 0.25 0.5 0.1 0.25

PLANE_KNN_K=8 IMU_NBG=0.003 HUBER_BASE=0.20 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
  ./run_one.sh m36_ac_na10 6 0.25 1.0 0.1 0.25

echo "[laneV-ac-na-up] done"
