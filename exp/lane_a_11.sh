#!/usr/bin/env bash
# M.25 阶段 1：1.1 退化判据模式（lane A，ROS_DOMAIN_ID=41）
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=41
export BAG=/home/ywj/slam_data/livox_record_drone

./run_one.sh m25_base 6 0.25 0.1 0.1 0.25

DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=0.001 DEGEN_TRANS_ONLY=0 \
  ./run_one.sh m25_tr0 6 0.25 0.1 0.1 0.25

DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=0.001 DEGEN_TRANS_ONLY=1 \
  ./run_one.sh m25_tr1 6 0.25 0.1 0.1 0.25

DEGENERACY_TRUNCATE_EN=true TIKHONOV_LAMBDA=0.001 DEGEN_TRANS_ONLY=2 \
  ./run_one.sh m25_tr2b 6 0.25 0.1 0.1 0.25
echo "[laneA] all done"
