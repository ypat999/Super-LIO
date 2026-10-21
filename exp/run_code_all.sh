#!/usr/bin/env bash
# 代码级对照实验：法向质量加权 / 地图端代表点与门控 / HKNN 分组搜索。
# 全部固定 filter_rate=6, voxel=0.25, imu_na/ng=0.1, vox_res=0.25，只改代码开关。
#
# 用法:
#   ./run_code_all.sh                 # 跑全部（跳过已完成的）
#   ./run_code_all.sh expc2_nw005     # 只跑指定的一到多个
#   DURATION=300 ./run_code_all.sh    # 改每段时长（秒）
set -u

EXP_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DURATION=${DURATION:-300}

# name             额外环境变量（DEBUG_LEVEL=2 由 run_case 统一加）
# 说明：
#   expc1  基线（仅开埋点，应与此前 exp0_baseline 轨迹一致）
#   expc2  实验 1：按平面拟合残差 max|dist| 做质量加权，sigma=0.05
#   expc3  实验 2：地图端只存真实扫描点（离子体素中心最近者）
#   expc4  实验 5：均值更新后夹回子体素包围盒
#   expc5  实验 5：clamp + 收紧合并门限 merge_dist=0.05
#   expc6  HKNN 只搜前 3 组（半径收缩的显式替代）
#   expc7  HKNN 提前终止阈值减半（order_scale=0.5）
CASES=(
  "expc1_base|"
  "expc2_nw005|NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.05"
  "expc3_maprep|MAP_REP_MODE=1"
  "expc4_clamp|MAP_CLAMP_MU=true"
  "expc5_clamp_merge005|MAP_CLAMP_MU=true MAP_MERGE_DIST=0.05"
  "expc6_hknn_g3|HKNN_MAX_GROUP=3"
  "expc7_hknn_s05|HKNN_ORDER_SCALE=0.5"
)

selected=("$@")

for line in "${CASES[@]}"; do
  name=${line%%|*}
  extra=${line#*|}
  if [ ${#selected[@]} -gt 0 ]; then
    hit=0
    for s in "${selected[@]}"; do [ "$s" = "$name" ] && hit=1; done
    [ $hit -eq 1 ] || continue
  fi
  if grep -q "end_wall=" "$EXP_ROOT/results/$name/meta.txt" 2>/dev/null; then
    echo "[run_code_all] skip $name (done)"
    continue
  fi
  echo "==================== $name ($extra) ===================="
  # shellcheck disable=SC2086
  env DEBUG_LEVEL=2 $extra "$EXP_ROOT/run_one.sh" "$name" 6 0.25 0.1 0.1 0.25 "$DURATION"
  echo "[run_code_all] $name rc=$?"
  sleep 3
done

echo "[run_code_all] all done $(date -Is)"
