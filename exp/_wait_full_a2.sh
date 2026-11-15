#!/usr/bin/env bash
# 等 run_one m40_p2_full_a 完全结束（含 saveMap），打印 meta 尾部。
for i in $(seq 1 100); do
  pgrep -f 'run_one.sh m40_p2_full_a' > /dev/null || break
  sleep 6
done
date +%H:%M:%S
tail -8 /home/ywj/git/Super-LIVO/third_party/Super-LIO/exp/results/m40_p2_full_a/meta.txt
