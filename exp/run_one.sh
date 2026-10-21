#!/usr/bin/env bash
# 单个实验：生成参数文件 -> 起 LIO -> 录 odom -> 播 bag -> 存图 -> 收尾
#
# 用法:
#   ./run_one.sh <exp_name> <filter_rate> <voxel_size> <imu_na> <imu_ng> <vox_res> [duration_s]
#
# 结果目录: exp/results/<exp_name>/
#   params.yaml   本次实际使用的完整参数
#   node.log      LIO stdout/stderr（含每分钟 [Undistort]/[Observe]... 耗时块）
#   odom.csv      /lio/odom 轨迹
#   play.log      ros2 bag play 输出
#   map.pcd       saveMap() 合并后的点云
#   meta.txt      关键环境/时间信息
set -u

EXP_NAME=$1
FILTER_RATE=$2
VOXEL_SIZE=$3
IMU_NA=$4
IMU_NG=$5
VOX_RES=$6
DURATION=${7:-300}
START_OFFSET=${START_OFFSET:-30}

BAG=${BAG:-/home/ywj/slam_data/livox_record_noshell}
EXP_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
OUT=$EXP_ROOT/results/$EXP_NAME
TEMPLATE=$EXP_ROOT/config_template.yaml

WS_SETUP=/tmp/sl_install/setup.bash
LIVOX_SETUP=/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/install/setup.bash

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

# ROS 的 setup.bash 里有未定义变量（AMENT_TRACE_SETUP_FILES 等），必须先关掉 set -u
set +u
source /opt/ros/humble/setup.bash
source "$LIVOX_SETUP"
source "$WS_SETUP"
set -u

mkdir -p "$OUT"

sed -e "s|__OUTDIR__|$OUT|g" \
    -e "s|__FILTER_RATE__|$FILTER_RATE|g" \
    -e "s|__VOXEL_SIZE__|$VOXEL_SIZE|g" \
    -e "s|__IMU_NA__|$IMU_NA|g" \
    -e "s|__IMU_NG__|$IMU_NG|g" \
    -e "s|__VOX_RES__|$VOX_RES|g" \
    "$TEMPLATE" > "$OUT/params.yaml"

{
  echo "exp_name=$EXP_NAME"
  echo "filter_rate=$FILTER_RATE"
  echo "voxel_fliter_size=$VOXEL_SIZE"
  echo "imu_na=$IMU_NA"
  echo "imu_ng=$IMU_NG"
  echo "vox_resolution=$VOX_RES"
  echo "duration=$DURATION"
  echo "start_offset=$START_OFFSET"
  echo "bag=$BAG"
  echo "start_wall=$(date -Is)"
  echo "git_head=$(git -C "$EXP_ROOT/.." rev-parse --short HEAD 2>/dev/null || echo unknown)"
  echo "uname=$(uname -r)"
} > "$OUT/meta.txt"

echo "[run_one] $EXP_NAME start $(date -Is)" | tee -a "$OUT/meta.txt"

# 0) 清理残留节点：ros2 run 会再 fork 出真正的可执行文件，只杀 wrapper 杀不掉节点，
#    残留节点会继续订阅同一份 bag 数据、往 /lio/odom 发第二套轨迹，污染结果。
if pgrep -f "super_lio_node --ros-args --params-file" > /dev/null; then
  echo "[run_one] killing leftover super_lio_node" | tee -a "$OUT/meta.txt"
  pkill -TERM -f "super_lio_node --ros-args --params-file"
  sleep 5
  pkill -KILL -f "super_lio_node --ros-args --params-file" 2>/dev/null
  sleep 1
fi

# 1) LIO 节点
ros2 run super_lio super_lio_node --ros-args --params-file "$OUT/params.yaml" \
  > "$OUT/node.log" 2>&1 < /dev/null &
NODE_PID=$!

# 等节点起来 + 参数就绪
sleep 5
if ! kill -0 $NODE_PID 2>/dev/null; then
  echo "[run_one] node died at startup, see $OUT/node.log" | tee -a "$OUT/meta.txt"
  exit 2
fi

# 2) odom 记录
python3 "$EXP_ROOT/odom_recorder.py" --out "$OUT/odom.csv" \
  > "$OUT/recorder.log" 2>&1 < /dev/null &
REC_PID=$!

sleep 3

# 3) 播包（固定时长、1x 实时）。
#    不用 timeout：GNU timeout 走 CLOCK_REALTIME，WSL2 时钟跳变会提前/延后结束；
#    这里用 sleep(相对时间, 单调钟) + pkill 做看门狗。
T0=$(date +%s.%N)
setsid bash -c "sleep $DURATION; pkill -INT -f 'ros2 bag play $BAG'" &
WD_PID=$!
ros2 bag play "$BAG" -r 1.0 -s sqlite3 \
  --start-offset "$START_OFFSET" \
  --topics /livox/lidar /livox/imu --disable-keyboard-controls \
  > "$OUT/play.log" 2>&1 < /dev/null
PLAY_RC=$?
kill -TERM -"$WD_PID" 2>/dev/null
T1=$(date +%s.%N)

sleep 3

# 4) 触发存图（服务回调内部会 pause -> save -> printTimeRecord -> resume）
timeout 600 ros2 service call /map_save std_srvs/srv/Trigger "{}" \
  > "$OUT/service.log" 2>&1 < /dev/null
SVC_RC=$?

sleep 5

# 5) 停 recorder / 节点（SIGTERM 触发 printTimeRecord）
kill -TERM $REC_PID 2>/dev/null
sleep 2
kill -9 $REC_PID 2>/dev/null

kill -TERM $NODE_PID 2>/dev/null
pkill -TERM -f "super_lio_node --ros-args --params-file $OUT/params.yaml" 2>/dev/null
for i in $(seq 1 20); do
  pgrep -f "super_lio_node --ros-args --params-file $OUT/params.yaml" > /dev/null || break
  sleep 1
done
pkill -KILL -f "super_lio_node --ros-args --params-file $OUT/params.yaml" 2>/dev/null
kill -9 $NODE_PID 2>/dev/null

{
  echo "play_rc=$PLAY_RC"
  echo "service_rc=$SVC_RC"
  echo "play_wall_s=$(awk "BEGIN{printf \"%.2f\", $T1 - $T0}")"
  echo "end_wall=$(date -Is)"
  echo "odom_rows=$(( $(wc -l < "$OUT/odom.csv" 2>/dev/null || echo 1) - 1 ))"
  if [ -f "$OUT/map.pcd" ]; then
    echo "map_pcd=$(du -h "$OUT/map.pcd" | cut -f1)"
    echo "map_points=$(grep -a -m1 '^POINTS' "$OUT/map.pcd" | awk '{print $2}')"
  else
    echo "map_pcd=MISSING"
  fi
} >> "$OUT/meta.txt"

echo "[run_one] $EXP_NAME done $(date -Is)"
