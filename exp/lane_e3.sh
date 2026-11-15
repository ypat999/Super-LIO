#!/usr/bin/env bash
# M.40 E-3（P-1）：人为结构矛盾是否诱导漂移。
#   同一条 seq2（转换后的 _d2）bag、同一套生产参数，唯一差别是
#   在 250~400 s 窗口把每帧点云复制一份并整体平移 1 m（机体系 +x）。
#   判据：16 个全站仪检查点的弦长/绝对偏差是否显著变坏。
#
# 前置：construction_seq2_d2 与 construction_seq2_ghost_d2 已生成
#   python3 exp/convert_livox_driver.py <src> /home/ywj/slam_data/rtk_slam/construction_seq2_d2
#   python3 exp/inject_ghost.py /home/ywj/slam_data/rtk_slam/construction_seq2_d2 \
#       /home/ywj/slam_data/rtk_slam/construction_seq2_ghost_d2 250 400 1.0 0 0
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=85
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 同 lane_e4：固定 Cyclone 网卡，避免节点与播放端选到不同接口导致收不到数据
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP4353p0s0"/></Interfaces></General></Domain></CycloneDDS>'
export SUB_DIV=4
export PLANE_KNN_K=8
export TRUST_JUMP_ROT_MAX=25.0
export DEGENERACY_DETECT_EN=true
export NORMAL_WEIGHT_EN=true
export NORMAL_SIGMA=0.035
export HUBER_BASE=0.20
export IMU_NBG=0.003
export DEBUG_LEVEL=2

BAG=/home/ywj/slam_data/rtk_slam/construction_seq2_d2 \
  ./run_one.sh m40_e3_base 6 0.25 0.3 0.1 0.25 0
BAG=/home/ywj/slam_data/rtk_slam/construction_seq2_ghost_d2 \
  ./run_one.sh m40_e3_ghost 6 0.25 0.3 0.1 0.25 0
echo "[lane_e3] all done"
