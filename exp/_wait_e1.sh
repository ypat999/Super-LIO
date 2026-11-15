#!/usr/bin/env bash
# 等 E-1 的 3cyc A/B 两个窗口跑完，打印 meta 关键行。
for i in $(seq 1 100); do
  pgrep -f 'run_one.sh m40_p2_3cyc_' > /dev/null || break
  sleep 5
done
date +%H:%M:%S
cd /home/ywj/git/Super-LIVO/third_party/Super-LIO/exp/results
for d in m40_p2_3cyc_a m40_p2_3cyc_b; do
  echo "== $d"
  grep -E 'play_rc|odom_rows|map_points|end_wall' $d/meta.txt 2>/dev/null
done
