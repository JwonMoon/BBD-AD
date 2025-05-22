#!/bin/bash

# export PYTHONPATH=$PYTHONPATH:~/shared_dir/B2D_Demo/B2D_tcp/Bench2Drive/Bench2DriveZoo #jw) TCP import error
source /opt/ros/humble/setup.bash
export PYTHONPATH=$PYTHONPATH:$(eval echo ~/shared_dir/B2D_Demo/B2D_tcp/Bench2Drive/Bench2DriveZoo)

export CKPT_PATH='./../Bench2DriveZoo/ckpts/tcp_b2d.ckpt'
export SAVE_PATH='./eval_v1/'
export DEBUG=True
export LEADERBOARD_ROOT=leaderboard
export PLANNER_TYPE='merge_ctrl_traj'
# export PLANNER_TYPE='only_traj'
# export PLANNER_TYPE='only_ctrl'

/usr/bin/python3 ${LEADERBOARD_ROOT}/team_code/tcp_agent_node.py \
--ckpt-path=${CKPT_PATH} \
--save-path=${SAVE_PATH} \
--debug=${DEBUG} \
--img-input=$1 \
--img-k=$2