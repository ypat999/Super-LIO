#!/usr/bin/env bash
# M.40 E-1（P-2）统一跑批入口：楼梯 bag 切窗 x A/B 线
#
# 用法: ./lane_p2.sh <tag> <start_offset_s> <duration_s> <line:A|B>
#   start/dur 单位为 bag 秒；dur=0 表示整包（无看门狗）
#
# 参数口径（M.39 生产定稿）+ 两条对照线（M.29/M.34）：
#   公共: sub_div=4  K=8  trust_jump_rot_max=25  huber=0.20  imu_nbg=0.003
#   A 线: imu_na=0.3  normal_weight=on  sigma=0.035
#   B 线: imu_na=0.01 normal_weight=off
# DEBUG_LEVEL=2 才写 debug.csv（含 deg_cond / rv / rv_pred，E-1 判 cond 分布、
# E-2 判单帧修正角都靠它）。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"

TAG=$1
START=$2
DUR=$3
LINE=${4:-A}

export WS_SETUP=${WS_SETUP:-/tmp/sl_install/setup.bash}
export BAG=${BAG:-/home/ywj/slam_data/livox_record_drone}
export SUB_DIV=4
export PLANE_KNN_K=8
export TRUST_JUMP_ROT_MAX=25.0
export DEGENERACY_DETECT_EN=true
export DEBUG_LEVEL=2
export HUBER_BASE=0.20
export IMU_NBG=0.003
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-81}
# 本机 WSL 重启后 FastDDS 跨进程发现失效（同进程回环正常、跨进程 0 消息），
# 换 CycloneDDS 传输；只影响中间件，不改算法与时间戳。
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export START_OFFSET=$START

if [ "$LINE" = "A" ]; then
  export NORMAL_WEIGHT_EN=true
  export NORMAL_SIGMA=0.035
  # P2_NA 可覆盖 A 线的 imu_na，用于"温和扰动"对照（默认 0.3 = A 线原值）
  NA=${P2_NA:-0.3}
else
  export NORMAL_WEIGHT_EN=false
  NA=0.01
fi

echo "[lane_p2] tag=$TAG start=$START dur=$DUR line=$LINE item=$BAG"
./run_one.sh "$TAG" 6 0.25 "$NA" 0.1 0.25 "$DUR"
