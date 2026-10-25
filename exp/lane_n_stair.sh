#!/usr/bin/env bash
# M.27-P3.2：细通道验证（楼梯 lane）
# 设计见 M.27：fine_channel_en=true 时，退化去抖状态激活的帧把 plane_fit 拒绝点
# 以 point-to-point 并入（λ_min 增益 ≥1.5 才生效）。
#
# 运行顺序（必须）：
#   1) 先跑 m30_fine_anchor（全默认，细通道关）：验收"默认关 = 原行为"，
#      md5 必须等于 m26_base/odom.csv —— 不相等则代码有副作用，先停下排查。
#   2) 再跑 m30_fine / m30_h20_fine。
# 判据：loop_error.py --bag $BAG --loops --win 12:21,392:420,440:465 --health <runs>
# 细通道日志：node.log 里 [FineCh] cands/used/gain/applied/degen/cond（2s 限频），
# 配套分析：python3 fine_diag.py --run results/m30_fine --node-log
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=63
export BAG=/home/ywj/slam_data/livox_record_drone
export WS_SETUP=/tmp/sl_install4/setup.bash
H20_CHAMP="NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.035 IMU_NBG=0.003 HUBER_BASE=0.20"

# 0) 代码回归锚点：细通道关，应与 m26_base 逐字节一致
./run_one.sh m30_fine_anchor 6 0.25 0.1 0.1 0.25
echo "== anchor md5（两行应相等）=="
md5sum results/m30_fine_anchor/odom.csv 2>/dev/null | cut -d' ' -f1
md5sum results/m26_base/odom.csv 2>/dev/null | cut -d' ' -f1

# 1) 基座 + 细通道（对照 m26_base=1.599）
FINE_CHANNEL_EN=true DEGENERACY_DETECT_EN=true \
  ./run_one.sh m30_fine 6 0.25 0.1 0.1 0.25

# 2) 楼梯冠军 + 细通道（对照 m26_h20_na03=0.024）
env $H20_CHAMP FINE_CHANNEL_EN=true DEGENERACY_DETECT_EN=true \
  ./run_one.sh m30_h20_fine 6 0.25 0.3 0.1 0.25

echo "[laneN-stair] all done"
