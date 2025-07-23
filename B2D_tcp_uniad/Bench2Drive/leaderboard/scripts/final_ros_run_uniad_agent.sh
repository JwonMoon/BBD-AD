#!/bin/bash

# export PYTHONPATH=$PYTHONPATH:~/shared_dir/B2D_Demo/B2D_tcp/Bench2Drive/Bench2DriveZoo #jw) TCP import error
source /opt/ros/humble/setup.bash
export PYTHONPATH=$PYTHONPATH:$(eval echo ~/shared_dir/B2D_Demo/B2D_tcp/Bench2Drive/Bench2DriveZoo)

export CONFIG_CKPT_PATH='./../Bench2DriveZoo/adzoo/uniad/configs/stage2_e2e/base_e2e_b2d.py+./../Bench2DriveZoo/ckpts/uniad_base_b2d.pth'
# export CONFIG_CKPT_PATH='./../Bench2DriveZoo/adzoo/uniad/configs/stage2_e2e/tiny_e2e_b2d.py+./../Bench2DriveZoo/ckpts/uniad_tiny_b2d.pth'
export SAVE_PATH='./../../jw_ws/single_eval/'
export DEBUG_MODE=0 #0: default 1: meta_data/timing_log 2: print log 3: save rgb_front/ meta_pid
export LEADERBOARD_ROOT=leaderboard
export PLANNER_TYPE='merge_ctrl_traj'
# export PLANNER_TYPE='only_traj'

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