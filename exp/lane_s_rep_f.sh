#!/usr/bin/env bash
# M.34 复现格（lane F）：merge_dist 0.1->0.25（保持均值）是本轮唯一有生产价值的候选
#   1f: 3.052 -> 2.275 (-25%，竖直 -2.182 -> -1.399)
#   楼梯: 0.021 -> 1.559 (74x 恶化)
#   首轮各只跑一次；要与 M.26 扫参结论同级采信，必须复现。
#     stair -> 楼梯复现    f1 -> 1f 复现
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
WHICH=${1:?用法: lane_s_rep_f.sh <stair|f1>}
export WS_SETUP=/tmp/sl_install/setup.bash
export MAP_REP_MODE=0 MAP_MERGE_DIST=0.25

if [ "$WHICH" = "stair" ]; then
  export ROS_DOMAIN_ID=67
  export BAG=/home/ywj/slam_data/livox_record_drone
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 \
    ./run_one.sh m34_rep0m25_stair_r2 6 0.25 0.3 0.1 0.25
elif [ "$WHICH" = "f1" ]; then
  export ROS_DOMAIN_ID=68
  export BAG=/home/ywj/slam_data/livox_record_drone_1f
  HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=false \
    ./run_one.sh m34_rep0m25_1f_r2 6 0.25 0.01 0.1 0.25
else
  echo "未知参数 $WHICH" >&2; exit 2
fi
echo "[laneS-rep-f:$WHICH] done"
