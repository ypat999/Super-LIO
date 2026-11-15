#!/usr/bin/env bash
# 排在 E-2 批次之后：等 _run_e2.sh 退出，再跑 Retail_Street A/B 两线。
cd "$(dirname "${BASH_SOURCE[0]}")"
for i in $(seq 1 400); do
  pgrep -f '_run_e2\.sh' > /dev/null || break
  sleep 10
done
echo "[queue] E-2 已结束 $(date +%H:%M:%S)"
for L in A B; do
  bash ./p2_clean.sh
  bash ./lane_retail.sh "$L" > "/tmp/retail_$L.log" 2>&1
  echo "[queue] retail $L rc=$?"
done
echo "[queue] ALLDONE"
