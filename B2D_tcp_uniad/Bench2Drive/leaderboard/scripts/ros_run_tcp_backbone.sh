#!/bin/bash

# export PYTHONPATH=$PYTHONPATH:~/shared_dir/BBD-AD/B2D_tcp/Bench2Drive/Bench2DriveZoo #jw) TCP import error
source /opt/ros/humble/setup.bash
export PYTHONPATH=$PYTHONPATH:$(eval echo ~/shared_dir/BBD-AD/B2D_tcp/Bench2Drive/Bench2DriveZoo)

export CKPT_PATH='./../Bench2DriveZoo/ckpts/tcp_b2d.ckpt'
# export SAVE_PATH='./eval_v1/'
export SAVE_PATH='./../../jw_ws/dual_eval'
export DEBUG_MODE=2 #0: default 1: meta_data/timing_log 2: print log 3: save rgb_front/ meta_pid
export IMG_INPUT='compressed'
export IMG_K=0.65
export LEADERBOARD_ROOT=leaderboard

/usr/bin/python3 ${LEADERBOARD_ROOT}/team_code/tcp_backbone_node.py \
--ckpt-path=${CKPT_PATH} \
--save-path=${SAVE_PATH} \
--debug-mode=${DEBUG_MODE} \
--img-input=${IMG_INPUT} \
--img-k=${IMG_K}