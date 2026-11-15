#!/usr/bin/env bash
# M.40 E-2：Trust 姿态阈值正向核验（1f / noshell 楼梯段）
# 目的（M.39 §8）：统计 >15° 且 <=25° 的真实修正帧数，以及这些帧对闭环的贡献。
#   同一 bag 跑两遍，只差 lio.observe.trust_jump_rot_max：
#     r15 = 旧阈值（会整帧撤销修正、进 LOST）
#     r25 = 生产定稿阈值（采纳这些修正）
#   其余参数全部锁在 A 线 + sub_div=4 + K=8 + huber0.20 + nbg0.003。
#
# 用法: ./lane_e2.sh <1f|ns> <15|25>
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"

WHICH=${1:?用法: lane_e2.sh <1f|ns> <15|25>}
ROT=${2:?用法: lane_e2.sh <1f|ns> <15|25>}
case "$WHICH" in
  1f) export BAG=/home/ywj/slam_data/livox_record_drone_1f; NAME=1f ;;
  ns) export BAG=/home/ywj/slam_data/livox_record_noshell; NAME=ns ;;
  *) echo "[lane_e2] unknown bag: $WHICH" >&2; exit 1 ;;
esac

export WS_SETUP=${WS_SETUP:-/tmp/sl_install/setup.bash}
export SUB_DIV=4
export PLANE_KNN_K=8
export TRUST_JUMP_ROT_MAX=${ROT}.0
export DEGENERACY_DETECT_EN=true
export DEBUG_LEVEL=2
export HUBER_BASE=0.20
export IMU_NBG=0.003
export NORMAL_WEIGHT_EN=true
export NORMAL_SIGMA=0.035
export START_OFFSET=${START_OFFSET:-0}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-83}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "[lane_e2] bag=$BAG rot_max=$ROT dur=${DURATION:-0}"
./run_one.sh "m40_e2_${NAME}_r${ROT}" 6 0.25 0.3 0.1 0.25 "${DURATION:-0}"
