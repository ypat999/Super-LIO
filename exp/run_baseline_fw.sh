#!/usr/bin/env bash
# 用 dog 侧已编译的框架（fast_lio / point_lio）在同一份 A 楼 bag 上跑一遍，
# 产出与 Super-LIO 实验同口径的 odom.csv + map.pcd，用于跨框架对比
# “逐层同向同角度”的 yaw 扭曲是否存在。
#
# 用法: ./run_baseline_fw.sh <fastlio|pointlio> <exp_name>
#
# 与 dog 现场配置的差异（**仅这两项，且是必须的**）:
#   1) mapping.extrinsic_R 由 30° yaw 改成单位阵
#      —— 那份配置里 extrinsic_R 是狗体上 MID360 的安装姿态；本 bag 的
#         Super-LIO 配置 lio.extrinsic.lidar_imu 是单位旋转，两者必须一致才能比。
#   2) point_lio 的 mapping.gravity / gravity_init 由 [4.905,0,-8.495] 改成实测值
#      [-4.708, 0.052, 8.601]（= -mean(acc)*9.80665，取 bag 起始 3 s 静态段）。
#      该值与 point_lio 自算的 state_in.gravity = -mean_acc*G/acc_norm 相等，作用只是把
#      "世界系 = 首帧 IMU 系"这件事显式化（Set_init 的 rot_init 退化为单位阵）。
#      实测：改完之后本 bag 上依然发散（姿态先崩、111 s 水平漂 3052 m），
#      说明 point_lio 的失效与重力初值无关，见 stairwell_drift_diagnosis.txt 五-H.5。
#   其余参数（point_filter_num / filter_size_* / 各类 cov / max_iteration ...）全部沿用 dog 配置。
#
# 结果目录: exp/results_livox_record_A_building/<exp_name>/
#   odom.csv  map.pcd  node.log  play.log  params_dump.txt  meta.txt
set -u

FW=$1
EXP_NAME=$2

BAG=${BAG:-/home/ywj/slam_data/livox_record_A_building}
EXP_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
OUT=${RESULT_ROOT:-$EXP_ROOT/results_livox_record_A_building}/$EXP_NAME
DOG_WS=/home/ywj/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2
FAST_CFG=$DOG_WS/src/FAST_LIO_ROS2_edit/config/mid360.yaml
PONIT_CFG=$DOG_WS/src/point_lio_ros2/config/mid360.yaml
FAST_PCD_DIR=$DOG_WS/src/FAST_LIO_ROS2_edit/PCD
POINT_PCD_DIR=$DOG_WS/src/point_lio_ros2/PCD

IDENT="1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0"
EXT_T="-0.011, -0.02329, 0.04412"

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_LOG_DIR=$OUT/roslog
mkdir -p "$OUT" "$ROS_LOG_DIR"

set +u
source /opt/ros/humble/setup.bash
source "$DOG_WS/install/setup.bash"
set -u

{
  echo "exp_name=$EXP_NAME"
  echo "framework=$FW"
  echo "bag=$BAG"
  echo "extrinsic_R=[$IDENT]   # 相对 dog 配置的覆盖（单位阵）"
  echo "extrinsic_T=[$EXT_T]"
  echo "start_wall=$(date -Is)"
} > "$OUT/meta.txt"

# 0) 清残留：同名节点会订阅同一份 bag，写出第二套轨迹
if [ "$FW" = "fastlio" ]; then
  NODE_PAT="fastlio_mapping"
else
  NODE_PAT="pointlio_mapping"
fi
if pgrep -f "$NODE_PAT" > /dev/null; then
  echo "[run] killing leftover $NODE_PAT" | tee -a "$OUT/meta.txt"
  pkill -TERM -f "$NODE_PAT"; sleep 5; pkill -KILL -f "$NODE_PAT" 2>/dev/null; sleep 1
fi

# 1) 起节点
case "$FW" in
  fastlio)
    ros2 run fast_lio fastlio_mapping --ros-args \
      --params-file "$FAST_CFG" \
      -p common.lid_topic:=/livox/lidar \
      -p common.imu_topic:=/livox/imu \
      -p mapping.extrinsic_R:="[$IDENT]" \
      -p mapping.extrinsic_T:="[$EXT_T]" \
      > "$OUT/node.log" 2>&1 < /dev/null &
    NODE_PID=$!
    sleep 6
    # FAST-LIO 是 lifecycle node：publisher 只有 activate 之后才发消息
    for i in $(seq 1 10); do
      ros2 lifecycle set /laser_mapping configure >> "$OUT/meta.txt" 2>&1 && break
      sleep 2
    done
    for i in $(seq 1 10); do
      ros2 lifecycle set /laser_mapping activate >> "$OUT/meta.txt" 2>&1 && break
      sleep 2
    done
    ODOM_TOPIC=/Odometry
    ;;
  pointlio)
    rm -rf "$POINT_PCD_DIR"
    # ROOT_DIR 宏为空串，point_lio 的 debug 日志写到相对路径 Log/ 下，故在 $OUT 里启动
    mkdir -p "$OUT/Log"
    # P_EXTRA：额外 -p 覆盖（空格分隔），用于短跑筛查不同假设
    P_EXTRA=()
    if [ -n "${EXTRA_PARAMS:-}" ]; then read -r -a P_EXTRA <<< "$EXTRA_PARAMS"; fi
    ( cd "$OUT" && exec ros2 run point_lio pointlio_mapping --ros-args \
      --params-file "$PONIT_CFG" \
      -p common.lid_topic:=/livox/lidar \
      -p common.imu_topic:=/livox/imu \
      -p mapping.extrinsic_R:="[$IDENT]" \
      -p mapping.extrinsic_T:="[$EXT_T]" \
      -p mapping.gravity:="[-4.708, 0.052, 8.601]" \
      -p mapping.gravity_init:="[-4.708, 0.052, 8.601]" \
      "${P_EXTRA[@]}" ) \
      > "$OUT/node.log" 2>&1 < /dev/null &
    NODE_PID=$!
    sleep 6
    ODOM_TOPIC=/aft_mapped_to_init
    ;;
  *)
    echo "unknown framework: $FW (fastlio|pointlio)" >&2
    exit 2
    ;;
esac

if ! kill -0 $NODE_PID 2>/dev/null; then
  echo "[run] node died at startup, see $OUT/node.log" | tee -a "$OUT/meta.txt"
  exit 2
fi
ros2 param dump /$( [ "$FW" = fastlio ] && echo laser_mapping || echo laserMapping ) \
  > "$OUT/params_dump.txt" 2>&1 || true

# 2) odom + 世界系点云记录
python3 "$EXP_ROOT/odom_recorder.py" --out "$OUT/odom.csv" --topic "$ODOM_TOPIC" \
  > "$OUT/recorder.log" 2>&1 < /dev/null &
REC_PID=$!
# 地图统一用 /cloud_registered 累积重建：FAST-LIO 的 pcl_wait_pub 只在 map_en=true 时
# 才累积（且每秒才塞一帧），point_lio 的 pcd_save 只能分块落盘；两边都不好用。
python3 "$EXP_ROOT/cloud_map_recorder.py" --out "$OUT/map.pcd" \
  --topic /cloud_registered --stride 3 --voxel 0.1 \
  > "$OUT/map_recorder.log" 2>&1 < /dev/null &
MAP_PID=$!
sleep 3

# 3) 播包（整包 1x；DUR=<秒> 时只播前 DUR 秒，用于短跑筛查）
T0=$(date +%s.%N)
PLAY_CMD=(ros2 bag play "$BAG" -r 1.0 -s sqlite3 \
  --topics /livox/lidar /livox/imu --disable-keyboard-controls)
if [ -n "${DUR:-}" ]; then
  timeout "$DUR" "${PLAY_CMD[@]}" > "$OUT/play.log" 2>&1 < /dev/null
else
  "${PLAY_CMD[@]}" > "$OUT/play.log" 2>&1 < /dev/null
fi
PLAY_RC=$?
T1=$(date +%s.%N)

sleep 3

# 4) 停 odom recorder（立刻），再停节点，最后让点云 recorder 落盘
kill -TERM $REC_PID 2>/dev/null; sleep 2; kill -9 $REC_PID 2>/dev/null

if [ "$FW" = "fastlio" ]; then
  pkill -INT -f fastlio_mapping 2>/dev/null
  sleep 3
  pkill -KILL -f fastlio_mapping 2>/dev/null
  SVC_RC=0
else
  pkill -INT -f pointlio_mapping 2>/dev/null
  for i in $(seq 1 60); do
    pgrep -f pointlio_mapping > /dev/null || break
    sleep 2
  done
  pkill -KILL -f pointlio_mapping 2>/dev/null
  SVC_RC=0
fi

# 点云累积器退出时写 map.pcd（体素降采样后约几百万点，几秒）
# 用 wait 而不是轮询 kill -0：进程退出后成为僵尸，kill -0 仍返回 0，会白等满超时
kill -TERM $MAP_PID 2>/dev/null
( sleep 180; kill -9 $MAP_PID 2>/dev/null ) &
WATCHDOG_PID=$!
wait $MAP_PID 2>/dev/null
kill $WATCHDOG_PID 2>/dev/null

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

if [ ! -s "$OUT/map.pcd" ]; then
  echo "[run] $EXP_NAME FAILED: map.pcd missing"
  exit 3
fi
echo "[run] $EXP_NAME done $(date -Is)"
