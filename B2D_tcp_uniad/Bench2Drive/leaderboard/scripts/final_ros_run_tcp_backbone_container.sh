#!/bin/bash

source /opt/ros/foxy/setup.bash
export PYTHONPATH=$PYTHONPATH:$(eval echo ~/shared_dir/BBD-AD/B2D_tcp_uniad/Bench2Drive/Bench2DriveZoo)

export CKPT_PATH='./../Bench2DriveZoo/ckpts/tcp_b2d.ckpt'
export SAVE_PATH='./../../my_ws/dual_eval'
export DEBUG_MODE=1 #0: default 1: meta_data/timing_log 2: print log 3: save rgb_front/ meta_pid
export LEADERBOARD_ROOT=leaderboard
export PLANNER_TYPE='merge_ctrl_traj' # or 'only_ctrl', 'only_traj'
export IMG_INPUT='compressed'
export IMG_K=0.65

mkdir -p $SAVE_PATH

# ==== Backbone node ====
echo "[RUN] Starting TCPBackboneNode..."
/usr/bin/python3 ${LEADERBOARD_ROOT}/team_code/final_tcp_backbone_node_container.py \
  --ckpt-path=${CKPT_PATH} \
  --save-path=${SAVE_PATH} \
  --debug-mode=${DEBUG_MODE} \
  --img-input=${IMG_INPUT} \
  --img-k=${IMG_K} &

BACKBONE_PID=$!

trap "echo 'Stopping...'; kill $BACKBONE_PID; wait" SIGINT SIGTERM

wait
