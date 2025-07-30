#!/bin/bash

source /opt/ros/foxy/setup.bash
export PYTHONPATH=$PYTHONPATH:$(eval echo ~/shared_dir/BBD-AD/B2D_tcp_uniad/Bench2Drive/Bench2DriveZoo)

export LEADERBOARD_ROOT=leaderboard
export CKPT_PATH='./../Bench2DriveZoo/ckpts/tcp_b2d.ckpt'
export SAVE_PATH='./../../my_ws/dual_eval/'
export DEBUG_MODE=0  #0: default 1: meta_data/timing_log 2: print log 3: save rgb_front/ meta_pid

export PLANNER_TYPE='merge_ctrl_traj'  # or 'only_ctrl', 'only_traj'

python3  ${LEADERBOARD_ROOT}/team_code/final_tcp_branch_server_container.py \
--ckpt-path=${CKPT_PATH} \
--debug-mode=${DEBUG_MODE}
