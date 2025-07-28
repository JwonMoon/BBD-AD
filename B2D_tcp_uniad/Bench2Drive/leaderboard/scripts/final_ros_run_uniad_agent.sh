#!/bin/bash

source /opt/ros/foxy/setup.bash
export PYTHONPATH=$PYTHONPATH:$(eval echo ~/shared_dir/B2D_Demo/B2D_tcp_uniad/Bench2Drive/Bench2DriveZoo)

export CONFIG_CKPT_PATH='./../Bench2DriveZoo/adzoo/uniad/configs/stage2_e2e/base_e2e_b2d.py+./../Bench2DriveZoo/ckpts/uniad_base_b2d.pth'
# export CONFIG_CKPT_PATH='./../Bench2DriveZoo/adzoo/uniad/configs/stage2_e2e/tiny_e2e_b2d.py+./../Bench2DriveZoo/ckpts/uniad_tiny_b2d.pth'
export SAVE_PATH='./../../my_ws/single_eval/'
export DEBUG_MODE=2 #0: default 1: meta_data/timing_log 2: print log 3: save rgb_front/ meta_pid
export LEADERBOARD_ROOT=leaderboard
export PLANNER_TYPE='merge_ctrl_traj' # or 'only_ctrl', 'only_traj'
export IMG_INPUT='compressed'

/usr/bin/python3 ${LEADERBOARD_ROOT}/team_code/final_uniad_agent_node.py \
--config-ckpt-path=${CONFIG_CKPT_PATH} \
--save-path=${SAVE_PATH} \
--debug-mode=${DEBUG_MODE} \
--img-input=${IMG_INPUT}