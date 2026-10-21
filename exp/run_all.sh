#!/usr/bin/env bash
# 依次跑全部实验点。已完成的（存在 meta.txt 且含 end_wall）默认跳过，方便断点续跑。
#
# 用法:
#   ./run_all.sh                # 跑全部
#   ./run_all.sh exp0_baseline  # 只跑指定的一到多个
#   DURATION=300 ./run_all.sh   # 改每段时长（秒）
set -u

EXP_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DURATION=${DURATION:-300}

# name              filter_rate  voxel  imu_na imu_ng vox_res  说明
EXPS=(
  "exp0_baseline    6   0.25  0.1    0.1    0.25"
  "exp1_fr3         3   0.25  0.1    0.1    0.25"
  "exp2_fr1         1   0.25  0.1    0.1    0.25"
  "exp3_vox020      6   0.20  0.1    0.1    0.20"
  "exp4_vox015      6   0.15  0.1    0.1    0.15"
  "exp5_imu001      6   0.25  0.01   0.01   0.25"
  "exp6_imu0001     6   0.25  0.001  0.001  0.25"
)

selected=("$@")

for line in "${EXPS[@]}"; do
  read -r name fr vox na ng vres <<< "$line"
  if [ ${#selected[@]} -gt 0 ]; then
    hit=0
    for s in "${selected[@]}"; do [ "$s" = "$name" ] && hit=1; done
    [ $hit -eq 1 ] || continue
  fi
  if grep -q "end_wall=" "$EXP_ROOT/results/$name/meta.txt" 2>/dev/null; then
    echo "[run_all] skip $name (done)"
    continue
  fi
  echo "==================== $name (fr=$fr vox=$vox na=$na ng=$ng vres=$vres) ===================="
  "$EXP_ROOT/run_one.sh" "$name" "$fr" "$vox" "$na" "$ng" "$vres" "$DURATION"
  rc=$?
  echo "[run_all] $name rc=$rc"
  sleep 3
done

echo "[run_all] all done $(date -Is)"
