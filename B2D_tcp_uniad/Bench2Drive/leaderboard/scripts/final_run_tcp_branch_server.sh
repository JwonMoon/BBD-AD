#!/bin/bash

export PYTHONPATH=$PYTHONPATH:$(eval echo ~/shared_dir/BBD-AD/B2D_tcp_uniad/Bench2Drive/Bench2DriveZoo)

export LEADERBOARD_ROOT=leaderboard
export CKPT_PATH='./../Bench2DriveZoo/ckpts/tcp_b2d.ckpt'
export SAVE_PATH='./../../my_ws/dual_eval/'
export DEBUG_MODE=0  # 0: off, 1: timing/log, 2: verbose, 3: +image save
export PLANNER_TYPE='merge_ctrl_traj'  # or 'only_ctrl', 'only_traj'

python3  ${LEADERBOARD_ROOT}/team_code/final_tcp_branch_server.py \
--ckpt-path=${CKPT_PATH} \
--debug-mode=${DEBUG_MODE}
