#!/usr/bin/env bash
# M.27-P3.2：细通道验证（1f lane，走廊是细通道的主战场）
# 1f 走廊是真退化场景（沿走廊方向无平面约束），细通道的两个预期行为都在这里检验：
#   - 有扶手/门框/消防栓的区段：gain 达标、applied=1，闭环应改善；
#   - 光墙真空区段：gain<1.5、applied=0，不添噪（门控生效的直接证据）。
set -u
cd "$(dirname "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID=64
export BAG=/home/ywj/slam_data/livox_record_drone_1f
export WS_SETUP=/tmp/sl_install4/setup.bash

# 0) 代码回归锚点：细通道关，应与 f1_s2_base 逐字节一致
./run_one.sh f1_fine_anchor 6 0.25 0.1 0.1 0.25
echo "== anchor md5（两行应相等）=="
md5sum results/f1_fine_anchor/odom.csv 2>/dev/null | cut -d' ' -f1
md5sum results/f1_s2_base/odom.csv 2>/dev/null | cut -d' ' -f1

# 1) 1f 基座 + 细通道（对照 f1_s2_base=3.519）
FINE_CHANNEL_EN=true DEGENERACY_DETECT_EN=true \
  ./run_one.sh f1_fine 6 0.25 0.1 0.1 0.25

# 2)（可选，若 1) 有正收益再跑）通用线 hub0.2+na01 + 细通道（对照 2.437）
HUBER_BASE=0.20 FINE_CHANNEL_EN=true DEGENERACY_DETECT_EN=true \
  ./run_one.sh f1_hub20_na01_fine 6 0.25 0.01 0.1 0.25

echo "[laneN-1f] all done"
