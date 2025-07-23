#!/bin/bash

# ==== 환경 설정 ====
source /opt/ros/humble/setup.bash
export PYTHONPATH=$PYTHONPATH:$(eval echo ~/shared_dir/B2D_Demo/B2D_tcp/Bench2Drive/Bench2DriveZoo)

export CKPT_PATH='./../Bench2DriveZoo/ckpts/tcp_b2d.ckpt'
export SAVE_PATH='./../../jw_ws/dual_eval'
export DEBUG_MODE=0
export IMG_INPUT='compressed'
export IMG_K=0.65
export LEADERBOARD_ROOT=leaderboard
export PLANNER_TYPE='merge_ctrl_traj'   # 필요시 'only_ctrl', 'only_traj'로 변경

# ==== 로그 디렉토리 생성 ====
mkdir -p $SAVE_PATH

# ==== 백본 노드 실행 ====
echo "[RUN] Starting TCPBackboneNode..."
/usr/bin/python3 ${LEADERBOARD_ROOT}/team_code/final_tcp_backbone_node_FPS.py \
  --ckpt-path=${CKPT_PATH} \
  --save-path=${SAVE_PATH} \
  --debug-mode=${DEBUG_MODE} \
  --img-input=${IMG_INPUT} \
  --img-k=${IMG_K} &

BACKBONE_PID=$!

# ==== 릴레이 노드 실행 ====
echo "[RUN] Starting TCPControlRelay..."
/usr/bin/python3 ${LEADERBOARD_ROOT}/team_code/final_tcp_control_relay_FPS.py &

RELAY_PID=$!

# ==== 종료 감시 ====
trap "echo 'Stopping...'; kill $BACKBONE_PID $RELAY_PID; wait" SIGINT SIGTERM

wait
