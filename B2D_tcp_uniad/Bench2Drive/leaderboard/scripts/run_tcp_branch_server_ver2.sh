#!/bin/bash

# PYTHONPATH 설정 (TCP 모듈이 import될 수 있도록)
export PYTHONPATH=$PYTHONPATH:$(eval echo ~/shared_dir/B2D_Demo/B2D_tcp/Bench2Drive/Bench2DriveZoo)

# 체크포인트 및 저장 경로 설정
export LEADERBOARD_ROOT=leaderboard
export CKPT_PATH='./../Bench2DriveZoo/ckpts/tcp_b2d.ckpt'
export SAVE_PATH='./../../jw_ws/dual_eval/'
export DEBUG_MODE=1  # 0: off, 1: timing/log, 2: verbose, 3: +image save
export PLANNER_TYPE='merge_ctrl_traj'  # or 'only_ctrl', 'only_traj'

# 실행 (주의: ROS 노드가 아니므로 rclpy 없이 실행)
python3  ${LEADERBOARD_ROOT}/team_code/tcp_branch_server_ver2.py \
--ckpt-path=${CKPT_PATH} \
--debug-mode=${DEBUG_MODE}
