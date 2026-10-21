#!/usr/bin/env bash
# 兼容入口：只跑代码级开关对照实验（等价于 SUITE=code ./run_all.sh "$@"）。
# 全部固定 filter_rate=6, voxel=0.25, imu_na/ng=0.1, vox_res=0.25，只改代码开关。
set -u
EXP_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SUITE=code exec "$EXP_ROOT/run_all.sh" "$@"
