#!/usr/bin/env bash
# 依次跑全部实验点（参数级 + 代码级）。已完成的（meta.txt 含 end_wall）默认跳过，方便断点续跑。
#
# 用法:
#   ./run_all.sh                        # 全部（参数 + 代码开关）
#   SUITE=param ./run_all.sh            # 只跑参数级 exp0..exp6
#   SUITE=code  ./run_all.sh            # 只跑代码级 expc1..expc7
#   ./run_all.sh exp0_baseline expc2_nw005   # 只跑指定的一到多个
#   DURATION=0 ./run_all.sh             # 默认：整包播完（不设看门狗）
#   DURATION=300 ./run_all.sh           # 只播前 300 s（开发/冒烟）
#   JOBS=3 ./run_all.sh                 # 3 组并行（每组独立 ROS_DOMAIN_ID，必须 DURATION=0）
#
# 环境变量:
#   BAG          数据集（默认 A 楼楼梯 bag）
#   RESULT_ROOT  结果根目录（默认 exp/results_<bag 目录名>）
#   START_OFFSET 起始偏移秒（默认 0）
#   JOBS         并行组数（默认 1）
#   BASE_DOMAIN  并行时起始 ROS_DOMAIN_ID（默认 42）
set -u

EXP_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DURATION=${DURATION:-0}
JOBS=${JOBS:-1}
BASE_DOMAIN=${BASE_DOMAIN:-42}
SUITE=${SUITE:-all}
BAG=${BAG:-/home/ywj/slam_data/livox_record_A_building}
START_OFFSET=${START_OFFSET:-0}
RESULT_ROOT=${RESULT_ROOT:-$EXP_ROOT/results_$(basename "${BAG%/}")}
export BAG START_OFFSET RESULT_ROOT

if [ "$JOBS" -gt 1 ] && [ "$DURATION" -gt 0 ]; then
  echo "[run_all] JOBS>1 不支持看门狗（pkill 会杀掉其他并行回放），强制 DURATION=0"
  DURATION=0
fi

# name | filter_rate | voxel | imu_na | imu_ng | vox_res | 额外的代码级开关
#   - exp0..exp6：参数实验（与户外回环那一轮同一套组合）
#   - expc1..expc7：代码级开关对照（debug 只开埋点，不改行为）
CASES=(
  "exp0_baseline|6|0.25|0.1|0.1|0.25|"
  "exp1_fr3|3|0.25|0.1|0.1|0.25|"
  "exp2_fr1|1|0.25|0.1|0.1|0.25|"
  "exp3_vox020|6|0.20|0.1|0.1|0.20|"
  "exp4_vox015|6|0.15|0.1|0.1|0.15|"
  "exp5_imu001|6|0.25|0.01|0.01|0.25|"
  "exp6_imu0001|6|0.25|0.001|0.001|0.25|"
  "expc1_base|6|0.25|0.1|0.1|0.25|DEBUG_LEVEL=2"
  "expc2_nw005|6|0.25|0.1|0.1|0.25|DEBUG_LEVEL=2 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.05"
  "expc3_maprep|6|0.25|0.1|0.1|0.25|DEBUG_LEVEL=2 MAP_REP_MODE=1"
  "expc4_clamp|6|0.25|0.1|0.1|0.25|DEBUG_LEVEL=2 MAP_CLAMP_MU=true"
  "expc5_clamp_merge005|6|0.25|0.1|0.1|0.25|DEBUG_LEVEL=2 MAP_CLAMP_MU=true MAP_MERGE_DIST=0.05"
  "expc6_hknn_g3|6|0.25|0.1|0.1|0.25|DEBUG_LEVEL=2 HKNN_MAX_GROUP=3"
  "expc7_hknn_s05|6|0.25|0.1|0.1|0.25|DEBUG_LEVEL=2 HKNN_ORDER_SCALE=0.5"
  # A 楼复测后前三名两两/三者组合（normal_weight σ=0.05 + HKNN 组数 3 + voxel 0.15）
  "expm1_nw_g3|6|0.25|0.1|0.1|0.25|DEBUG_LEVEL=2 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.05 HKNN_MAX_GROUP=3"
  "expm2_g3_vox015|6|0.15|0.1|0.1|0.15|DEBUG_LEVEL=2 HKNN_MAX_GROUP=3"
  "expm3_nw_vox015|6|0.15|0.1|0.1|0.15|DEBUG_LEVEL=2 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.05"
  "expm4_nw_g3_vox015|6|0.15|0.1|0.1|0.15|DEBUG_LEVEL=2 NORMAL_WEIGHT_EN=true NORMAL_SIGMA=0.05 HKNN_MAX_GROUP=3"
)

selected=("$@")
ok=(); skipped=(); failed=()

# 先按 SUITE/指定名单/断点续跑筛出待跑清单
pending=()
declare -A case_line
for line in "${CASES[@]}"; do
  IFS='|' read -r name fr vox na ng vres extra <<< "$line"

  case "$SUITE" in
    param) [[ "$name" == expc* ]] && continue ;;
    code)  [[ "$name" == expc* ]] || continue ;;
  esac

  if [ ${#selected[@]} -gt 0 ]; then
    hit=0
    for s in "${selected[@]}"; do [ "$s" = "$name" ] && hit=1; done
    [ $hit -eq 1 ] || continue
  fi
  # 只有「跑完 + 存图成功 + 有 map.pcd」才算完成。
  # 只看 end_wall 会把 /map_save 超时（service_rc=124、没有 map.pcd）的组误判为已完成。
  if grep -q "end_wall=" "$RESULT_ROOT/$name/meta.txt" 2>/dev/null \
     && grep -q "service_rc=0" "$RESULT_ROOT/$name/meta.txt" 2>/dev/null \
     && [ -s "$RESULT_ROOT/$name/map.pcd" ]; then
    echo "[run_all] skip $name (done)"
    skipped+=("$name")
    continue
  fi
  if [ -e "$RESULT_ROOT/$name/meta.txt" ]; then
    echo "[run_all] rerun $name (incomplete: $(grep -E 'service_rc=|end_wall=' "$RESULT_ROOT/$name/meta.txt" | tr '\n' ' '))"
  fi
  pending+=("$name")
  case_line["$name"]="$line"
done

run_case() {
  local line=$1 domain=$2
  local name fr vox na ng vres extra
  IFS='|' read -r name fr vox na ng vres extra <<< "$line"
  echo "==================== $name (fr=$fr vox=$vox na=$na ng=$ng vres=$vres ${extra:-}) domain=$domain ===================="
  # shellcheck disable=SC2086
  env ROS_DOMAIN_ID=$domain $extra "$EXP_ROOT/run_one.sh" \
    "$name" "$fr" "$vox" "$na" "$ng" "$vres" "$DURATION"
}

i=0
while [ $i -lt ${#pending[@]} ]; do
  pids=(); names=()
  for j in $(seq 0 $((JOBS - 1))); do
    k=$((i + j))
    [ $k -ge ${#pending[@]} ] && break
    nm=${pending[$k]}
    run_case "${case_line[$nm]}" $((BASE_DOMAIN + j)) &
    pids+=($!); names+=("$nm")
  done
  for x in "${!pids[@]}"; do
    if wait "${pids[$x]}"; then
      echo "[run_all] ${names[$x]} rc=0"
      ok+=("${names[$x]}")
    else
      echo "[run_all] ${names[$x]} rc=$?"
      failed+=("${names[$x]}")
    fi
  done
  i=$((i + JOBS))
  sleep 5
done

echo "[run_all] done $(date -Is)"
echo "[run_all]   ok:      ${ok[*]:-}"
echo "[run_all]   skipped: ${skipped[*]:-}"
echo "[run_all]   failed:  ${failed[*]:-}"
[ ${#failed[@]} -eq 0 ]
