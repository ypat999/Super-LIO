#!/usr/bin/env bash
# 等待 m40_p2_full_a 的播包结束（最多 ~9 分钟），然后打印进度。
for i in $(seq 1 110); do
  pgrep -f 'ros2 bag play /home/ywj/slam_data/livox_record_drone -r' > /dev/null || break
  sleep 5
done
date +%H:%M:%S
tail -6 /home/ywj/git/Super-LIVO/third_party/Super-LIO/exp/results/m40_p2_full_a/meta.txt
