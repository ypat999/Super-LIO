#!/usr/bin/env bash
# M.25 阶段 2：新机制跨场景验证（lane E，园区 1f bag，ROS_DOMAIN_ID=45）
# 与 lane D 同一套开关，验证机制在另一场景是否同样有效/无害
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=45
export BAG=/home/ywj/slam_data/livox_record_drone_1f
export WS_SETUP=/tmp/sl_install2/setup.bash

                 ./run_one.sh f1_s2_base     6 0.25 0.1 0.1 0.25
GROUND_WEIGHT=1000.0 ./run_one.sh f1_s2_ground1k 6 0.25 0.1 0.1 0.25
OBS_DIST_K=0.02 ./run_one.sh f1_s2_obsd002 6 0.25 0.1 0.1 0.25
THRESH_MOTION_K=0.5 ./run_one.sh f1_s2_thm05 6 0.25 0.1 0.1 0.25
DELTA_INTERP_EN=true ./run_one.sh f1_s2_dint 6 0.25 0.1 0.1 0.25

echo "[laneE] all done"
