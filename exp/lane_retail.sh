#!/usr/bin/env bash
# Retail_Street（RTK/自采街景 135 s）A/B 两线复跑
# 用法: ./lane_retail.sh <A|B>
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"

LINE=${1:-A}
export BAG=${BAG:-/home/ywj/slam_data/Retail_Street_d2}
export WS_SETUP=${WS_SETUP:-/tmp/sl_install/setup.bash}
export SUB_DIV=4
export PLANE_KNN_K=8
export TRUST_JUMP_ROT_MAX=25.0
export DEGENERACY_DETECT_EN=true
export DEBUG_LEVEL=2
export HUBER_BASE=0.20
export IMU_NBG=0.003
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-85}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ "$LINE" = "A" ]; then
  export NORMAL_WEIGHT_EN=true
  export NORMAL_SIGMA=0.035
  NA=0.3
else
  export NORMAL_WEIGHT_EN=false
  export NORMAL_SIGMA=0.035
  NA=0.01
fi

echo "[lane_retail] line=$LINE bag=$BAG na=$NA"
./run_one.sh "m40_retail_$LINE" 6 0.25 "$NA" 0.1 0.25 0
