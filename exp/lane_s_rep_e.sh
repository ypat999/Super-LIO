#!/usr/bin/env bash
# M.34 归因格（lane E，1f）：保持"均值"，只放宽单个地图门限
#   楼梯上已出现"保持均值只放宽 merge_dist/mp 也崩到 1.56 m"，
#   所以 1f 上 rep_mode=1 的 +23% 必须先排除"任何地图扰动都有效"这一解释。
#     m25 -> rep_mode=0 + merge_dist 0.25
#     mp  -> rep_mode=0 + max_points 254
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
WHICH=${1:?用法: lane_s_rep_e.sh <m25|mp>}
export WS_SETUP=/tmp/sl_install/setup.bash
export BAG=/home/ywj/slam_data/livox_record_drone_1f
export HUBER_BASE=0.20 IMU_NBG=0.003 NORMAL_WEIGHT_EN=false

if [ "$WHICH" = "m25" ]; then
  export ROS_DOMAIN_ID=65
  MAP_REP_MODE=0 MAP_MERGE_DIST=0.25 \
    ./run_one.sh m34_rep0m25_1f 6 0.25 0.01 0.1 0.25
elif [ "$WHICH" = "mp" ]; then
  export ROS_DOMAIN_ID=66
  MAP_REP_MODE=0 MAP_MAX_POINTS=254 \
    ./run_one.sh m34_rep0mp_1f 6 0.25 0.01 0.1 0.25
else
  echo "未知参数 $WHICH" >&2; exit 2
fi
echo "[laneS-rep-e:$WHICH] done"
