#!/usr/bin/env bash
# 单个实验：生成参数文件 -> 起 LIO -> 录 odom -> 播 bag -> 存图 -> 收尾
#
# 用法:
#   ./run_one.sh <exp_name> <filter_rate> <voxel_size> <imu_na> <imu_ng> <vox_res> [duration_s]
#
# duration_s 传 0（默认）表示整包播完，不设看门狗；>0 时只播前 duration_s 秒。
#
# 环境变量:
#   BAG          数据集目录（默认 A 楼楼梯 bag）
#   RESULT_ROOT  结果根目录（默认 exp/results）
#   START_OFFSET 起始偏移秒（默认 0）
#   DEBUG_LEVEL / NORMAL_WEIGHT_EN / ... 代码级开关，默认全部保持原行为
#
# 结果目录: $RESULT_ROOT/<exp_name>/
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
IMU_NBG=${IMU_NBG:-0.0001}
IMU_GYR_SCALE=${IMU_GYR_SCALE:-1.0}
LIDAR_TIME_OFFSET=${LIDAR_TIME_OFFSET:-0.0}
VOX_RES=$6
# 0 = 整包，不设看门狗
DURATION=${7:-${DURATION:-0}}
START_OFFSET=${START_OFFSET:-0}

# 代码级实验开关（默认全部保持原行为）
DEBUG_LEVEL=${DEBUG_LEVEL:-0}
NORMAL_WEIGHT_EN=${NORMAL_WEIGHT_EN:-false}
NORMAL_SIGMA=${NORMAL_SIGMA:-0.05}
FREEZE_GRAVITY=${FREEZE_GRAVITY:-false}
OBS_WEIGHT=${OBS_WEIGHT:-1000.0}
PLANE_FIT_THR=${PLANE_FIT_THR:-0.15}
HUBER_BASE=${HUBER_BASE:-0.05}
DEGENERACY_DETECT_EN=${DEGENERACY_DETECT_EN:-false}
DEGENERACY_THRESHOLD=${DEGENERACY_THRESHOLD:-100.0}
DEGENERACY_TRUNCATE_EN=${DEGENERACY_TRUNCATE_EN:-false}
TIKHONOV_LAMBDA=${TIKHONOV_LAMBDA:-0.001}
DEGENERACY_HYSTERESIS=${DEGENERACY_HYSTERESIS:-0.5}
DEGENERACY_ENTER_FRAMES=${DEGENERACY_ENTER_FRAMES:-3}
DEGENERACY_EXIT_FRAMES=${DEGENERACY_EXIT_FRAMES:-10}
ADAPTIVE_PARAM_EN=${ADAPTIVE_PARAM_EN:-false}
ADAPTIVE_NORMAL_WEIGHT_EN=${ADAPTIVE_NORMAL_WEIGHT_EN:-true}
ADAPTIVE_NORMAL_SIGMA=${ADAPTIVE_NORMAL_SIGMA:-0.035}
ADAPTIVE_PLANE_FIT=${ADAPTIVE_PLANE_FIT:-0.0}
ADAPTIVE_HUBER=${ADAPTIVE_HUBER:-0.0}
FRAME_GAP_GUARD_EN=${FRAME_GAP_GUARD_EN:-false}
FRAME_GAP_MAX=${FRAME_GAP_MAX:-0.2}
IMU_COVER_MAX=${IMU_COVER_MAX:-0.03}
# 点云话题输出（默认关）。测 [Lidar->CloudWorld] 端到端延迟时置 true
OUTPUT_MAP=${OUTPUT_MAP:-false}
MAP_REP_MODE=${MAP_REP_MODE:-0}
MAP_CLAMP_MU=${MAP_CLAMP_MU:-false}
MAP_MAX_POINTS=${MAP_MAX_POINTS:-20}
MAP_MERGE_DIST=${MAP_MERGE_DIST:-0.1}
HKNN_MAX_GROUP=${HKNN_MAX_GROUP:--1}
HKNN_ORDER_SCALE=${HKNN_ORDER_SCALE:-1.0}
# SC-PGO 兼容输出（只导出关键帧扫描+位姿，不改变 LIO 行为）
SC_PGO_EN=${SC_PGO_EN:-false}
SC_KF_GAP=${SC_KF_GAP:-0.5}
SC_KF_DEG=${SC_KF_DEG:-5.0}
# 实验 C：固定地图 / 冻结地图更新
MAP_FREEZE=${MAP_FREEZE:-false}
MAP_LOAD_PATH=${MAP_LOAD_PATH:-}
MAP_INIT_POSE=${MAP_INIT_POSE:-[]}
MAP_DS_SIZE=${MAP_DS_SIZE:-0.25}

BAG=${BAG:-/home/ywj/slam_data/livox_record_A_building}
EXP_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
RESULT_ROOT=${RESULT_ROOT:-$EXP_ROOT/results}
OUT=$RESULT_ROOT/$EXP_NAME
TEMPLATE=$EXP_ROOT/config_template.yaml

WS_SETUP=/tmp/sl_install/setup.bash
LIVOX_SETUP=/home/ywj/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/install/setup.bash

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
# 每个实验独立的 ROS 日志目录：并行时互不干扰，也避免 ~/.ros/log 无限膨胀
export ROS_LOG_DIR=${ROS_LOG_DIR:-$OUT/roslog}
mkdir -p "$ROS_LOG_DIR"

# ROS 的 setup.bash 里有未定义变量（AMENT_TRACE_SETUP_FILES 等），必须先关掉 set -u
set +u
source /opt/ros/humble/setup.bash
[ -f "$LIVOX_SETUP" ] && source "$LIVOX_SETUP"
if [ ! -f "$WS_SETUP" ]; then
  echo "[run_one] missing $WS_SETUP, build first: build_ws.sh" >&2
  exit 3
fi
source "$WS_SETUP"
set -u

mkdir -p "$OUT"

sed -e "s|__OUTDIR__|$OUT|g" \
    -e "s|__FILTER_RATE__|$FILTER_RATE|g" \
    -e "s|__VOXEL_SIZE__|$VOXEL_SIZE|g" \
    -e "s|__IMU_NA__|$IMU_NA|g" \
    -e "s|__IMU_NG__|$IMU_NG|g" \
    -e "s|__IMU_NBG__|$IMU_NBG|g" \
    -e "s|__IMU_GYR_SCALE__|$IMU_GYR_SCALE|g" \
    -e "s|__LIDAR_TIME_OFFSET__|$LIDAR_TIME_OFFSET|g" \
    -e "s|__VOX_RES__|$VOX_RES|g" \
    -e "s|__DEBUG_LEVEL__|$DEBUG_LEVEL|g" \
    -e "s|__NORMAL_WEIGHT_EN__|$NORMAL_WEIGHT_EN|g" \
    -e "s|__NORMAL_SIGMA__|$NORMAL_SIGMA|g" \
    -e "s|__FREEZE_GRAVITY__|$FREEZE_GRAVITY|g" \
    -e "s|__OBS_WEIGHT__|$OBS_WEIGHT|g" \
    -e "s|__PLANE_FIT_THR__|$PLANE_FIT_THR|g" \
    -e "s|__HUBER_BASE__|$HUBER_BASE|g" \
    -e "s|__DEGENERACY_DETECT_EN__|$DEGENERACY_DETECT_EN|g" \
    -e "s|__DEGENERACY_THRESHOLD__|$DEGENERACY_THRESHOLD|g" \
    -e "s|__DEGENERACY_TRUNCATE_EN__|$DEGENERACY_TRUNCATE_EN|g" \
    -e "s|__TIKHONOV_LAMBDA__|$TIKHONOV_LAMBDA|g" \
    -e "s|__DEGENERACY_HYSTERESIS__|$DEGENERACY_HYSTERESIS|g" \
    -e "s|__DEGENERACY_ENTER_FRAMES__|$DEGENERACY_ENTER_FRAMES|g" \
    -e "s|__DEGENERACY_EXIT_FRAMES__|$DEGENERACY_EXIT_FRAMES|g" \
    -e "s|__ADAPTIVE_PARAM_EN__|$ADAPTIVE_PARAM_EN|g" \
    -e "s|__ADAPTIVE_NORMAL_WEIGHT_EN__|$ADAPTIVE_NORMAL_WEIGHT_EN|g" \
    -e "s|__ADAPTIVE_NORMAL_SIGMA__|$ADAPTIVE_NORMAL_SIGMA|g" \
    -e "s|__ADAPTIVE_PLANE_FIT__|$ADAPTIVE_PLANE_FIT|g" \
    -e "s|__ADAPTIVE_HUBER__|$ADAPTIVE_HUBER|g" \
    -e "s|__FRAME_GAP_GUARD_EN__|$FRAME_GAP_GUARD_EN|g" \
    -e "s|__FRAME_GAP_MAX__|$FRAME_GAP_MAX|g" \
    -e "s|__IMU_COVER_MAX__|$IMU_COVER_MAX|g" \
    -e "s|__OUTPUT_MAP__|$OUTPUT_MAP|g" \
    -e "s|__MAP_REP_MODE__|$MAP_REP_MODE|g" \
    -e "s|__MAP_CLAMP_MU__|$MAP_CLAMP_MU|g" \
    -e "s|__MAP_MAX_POINTS__|$MAP_MAX_POINTS|g" \
    -e "s|__MAP_MERGE_DIST__|$MAP_MERGE_DIST|g" \
    -e "s|__HKNN_MAX_GROUP__|$HKNN_MAX_GROUP|g" \
    -e "s|__HKNN_ORDER_SCALE__|$HKNN_ORDER_SCALE|g" \
    -e "s|__SC_PGO_EN__|$SC_PGO_EN|g" \
    -e "s|__SC_KF_GAP__|$SC_KF_GAP|g" \
    -e "s|__SC_KF_DEG__|$SC_KF_DEG|g" \
    -e "s|__MAP_FREEZE__|$MAP_FREEZE|g" \
    -e "s|__MAP_LOAD_PATH__|$MAP_LOAD_PATH|g" \
    -e "s|__MAP_DS_SIZE__|$MAP_DS_SIZE|g" \
    "$TEMPLATE" > "$OUT/params.yaml"

# 空列表在 YAML 里是无类型序列，rclcpp 读 vector<double> 会直接抛异常，
# 所以初始位姿只在显式给出时追加。
if [ -n "$MAP_INIT_POSE" ] && [ "$MAP_INIT_POSE" != "[]" ]; then
  echo "    lio.map.init_pose: $MAP_INIT_POSE" >> "$OUT/params.yaml"
fi

{
  echo "exp_name=$EXP_NAME"
  echo "filter_rate=$FILTER_RATE"
  echo "voxel_fliter_size=$VOXEL_SIZE"
  echo "imu_na=$IMU_NA"
  echo "imu_ng=$IMU_NG"
  echo "imu_nbg=$IMU_NBG"
  echo "imu_gyr_scale=$IMU_GYR_SCALE"
  echo "lidar_time_offset=$LIDAR_TIME_OFFSET"
  echo "vox_resolution=$VOX_RES"
  echo "debug_level=$DEBUG_LEVEL"
  echo "normal_weight_en=$NORMAL_WEIGHT_EN"
  echo "normal_sigma=$NORMAL_SIGMA"
  echo "freeze_gravity=$FREEZE_GRAVITY"
  echo "obs_weight=$OBS_WEIGHT"
  echo "plane_fit_threshold=$PLANE_FIT_THR"
  echo "huber_delta_base=$HUBER_BASE"
  echo "degeneracy_detect_en=$DEGENERACY_DETECT_EN"
  echo "degeneracy_threshold=$DEGENERACY_THRESHOLD"
  echo "degeneracy_truncate_en=$DEGENERACY_TRUNCATE_EN"
  echo "tikhonov_lambda=$TIKHONOV_LAMBDA"
  echo "degeneracy_hysteresis=$DEGENERACY_HYSTERESIS"
  echo "degeneracy_enter_frames=$DEGENERACY_ENTER_FRAMES"
  echo "degeneracy_exit_frames=$DEGENERACY_EXIT_FRAMES"
  echo "adaptive_param_en=$ADAPTIVE_PARAM_EN"
  echo "adaptive_normal_weight_en=$ADAPTIVE_NORMAL_WEIGHT_EN"
  echo "adaptive_normal_sigma=$ADAPTIVE_NORMAL_SIGMA"
  echo "adaptive_plane_fit=$ADAPTIVE_PLANE_FIT"
  echo "adaptive_huber=$ADAPTIVE_HUBER"
  echo "frame_gap_guard_en=$FRAME_GAP_GUARD_EN"
  echo "frame_gap_max=$FRAME_GAP_MAX"
  echo "imu_cover_max=$IMU_COVER_MAX"
  echo "output_map=$OUTPUT_MAP"
  echo "map_rep_mode=$MAP_REP_MODE"
  echo "map_clamp_mu=$MAP_CLAMP_MU"
  echo "map_max_points=$MAP_MAX_POINTS"
  echo "map_merge_dist=$MAP_MERGE_DIST"
  echo "hknn_max_group=$HKNN_MAX_GROUP"
  echo "hknn_order_scale=$HKNN_ORDER_SCALE"
  echo "sc_pgo_en=$SC_PGO_EN"
  echo "sc_kf_gap=$SC_KF_GAP"
  echo "sc_kf_deg=$SC_KF_DEG"
  echo "map_freeze=$MAP_FREEZE"
  echo "map_load_path=$MAP_LOAD_PATH"
  echo "map_init_pose=$MAP_INIT_POSE"
  echo "map_ds_size=$MAP_DS_SIZE"
  echo "duration=$DURATION"
  echo "start_offset=$START_OFFSET"
  echo "bag=$BAG"
  echo "start_wall=$(date -Is)"
  echo "git_head=$(git -C "$EXP_ROOT/.." rev-parse --short HEAD 2>/dev/null || echo unknown)"
  echo "uname=$(uname -r)"
} > "$OUT/meta.txt"

echo "[run_one] $EXP_NAME start $(date -Is)" | tee -a "$OUT/meta.txt"

# 0) 清理同名实验的残留节点：ros2 run 会再 fork 出真正的可执行文件，只杀 wrapper 杀不掉节点，
#    残留节点会继续订阅同一份 bag 数据、往 /lio/odom 发第二套轨迹，污染结果。
#    只匹配本实验的 params 文件，避免误杀并行运行的其他实验。
if pgrep -f "super_lio_node --ros-args --params-file $OUT/params.yaml" > /dev/null; then
  echo "[run_one] killing leftover super_lio_node" | tee -a "$OUT/meta.txt"
  pkill -TERM -f "super_lio_node --ros-args --params-file $OUT/params.yaml"
  sleep 5
  pkill -KILL -f "super_lio_node --ros-args --params-file $OUT/params.yaml" 2>/dev/null
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

# 3) 播包（默认 1x 实时、整包播完）。
#    不用 timeout：GNU timeout 走 CLOCK_REALTIME，WSL2 时钟跳变会提前/延后结束；
#    这里用 sleep(相对时间, 单调钟) + pkill 做看门狗。
T0=$(date +%s.%N)
WD_PID=""
if [ "$DURATION" -gt 0 ] 2>/dev/null; then
  setsid bash -c "sleep $DURATION; pkill -INT -f 'ros2 bag play $BAG'" &
  WD_PID=$!
fi
ros2 bag play "$BAG" -r 1.0 -s sqlite3 \
  --start-offset "$START_OFFSET" \
  --topics /livox/lidar /livox/imu --disable-keyboard-controls \
  > "$OUT/play.log" 2>&1 < /dev/null
PLAY_RC=$?
if [ -n "$WD_PID" ]; then kill -TERM -"$WD_PID" 2>/dev/null; fi
T1=$(date +%s.%N)

sleep 3

# 4) 先停 recorder，再存图。
#    /map_save 回调内部会 reset IMU pre-integration 并触发 odom reset，
#    若此时 recorder 仍在运行，会把重置后的坐标系写进 odom.csv，污染终点位姿指标。
kill -TERM $REC_PID 2>/dev/null
sleep 2
kill -9 $REC_PID 2>/dev/null

# 5) 触发存图（服务回调内部会 pause -> save -> printTimeRecord -> resume）
#    整栋楼的地图合并比户外大很多，超时放宽到 20 min。
timeout 1200 ros2 service call /map_save std_srvs/srv/Trigger "{}" \
  > "$OUT/service.log" 2>&1 < /dev/null
SVC_RC=$?

sleep 5

# 6) 停节点（SIGTERM 触发 printTimeRecord）
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

if [ "$SVC_RC" -ne 0 ] || [ ! -s "$OUT/map.pcd" ]; then
  echo "[run_one] $EXP_NAME FAILED: service_rc=$SVC_RC map_pcd=$( [ -s "$OUT/map.pcd" ] && echo ok || echo missing)"
  exit 3
fi

echo "[run_one] $EXP_NAME done $(date -Is)"
