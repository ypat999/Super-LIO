#!/usr/bin/env bash
# E-2 四次跑批：1f/ns × rot_max 15/25，串行。
cd "$(dirname "${BASH_SOURCE[0]}")"
for a in "1f 25" "1f 15" "ns 25" "ns 15"; do
  set -- $a
  bash ./p2_clean.sh
  bash ./lane_e2.sh "$1" "$2" > "/tmp/e2_$1_$2.log" 2>&1
  echo "[$1 r$2] rc=$?"
done
echo ALLDONE
