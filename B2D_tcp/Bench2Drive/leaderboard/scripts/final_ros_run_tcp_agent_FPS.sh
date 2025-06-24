#!/bin/bash

# export PYTHONPATH=$PYTHONPATH:~/shared_dir/B2D_Demo/B2D_tcp/Bench2Drive/Bench2DriveZoo #jw) TCP import error
source /opt/ros/humble/setup.bash
export PYTHONPATH=$PYTHONPATH:$(eval echo ~/shared_dir/B2D_Demo/B2D_tcp/Bench2Drive/Bench2DriveZoo)

export CKPT_PATH='./../Bench2DriveZoo/ckpts/tcp_b2d.ckpt'
export SAVE_PATH='./../../jw_ws/single_eval/'
export DEBUG_MODE=1 #0: default 1: meta_data/timing_log 2: print log 3: save rgb_front/ meta_pid
export LEADERBOARD_ROOT=leaderboard
export PLANNER_TYPE='merge_ctrl_traj'
export IMG_INPUT='compressed'
export IMG_K=0.65
# export PLANNER_TYPE='only_traj'
# export PLANNER_TYPE='only_ctrl'

/usr/bin/python3 ${LEADERBOARD_ROOT}/team_code/final_tcp_agent_node_FPS.py \
--ckpt-path=${CKPT_PATH} \
--save-path=${SAVE_PATH} \
--debug-mode=${DEBUG_MODE} \
--img-input=${IMG_INPUT} \
--img-k=${IMG_K}