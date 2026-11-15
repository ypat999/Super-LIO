#!/usr/bin/env bash
# 等 m40_p2_full_c 结束，打印 meta 关键行。
for i in $(seq 1 110); do
  pgrep -f 'run_one.sh m40_p2_full_c' > /dev/null || break
  sleep 6
done
date +%H:%M:%S
grep -E 'play_rc|odom_rows|map_points|end_wall' /home/ywj/git/Super-LIVO/third_party/Super-LIO/exp/results/m40_p2_full_c/meta.txt 2>/dev/null
