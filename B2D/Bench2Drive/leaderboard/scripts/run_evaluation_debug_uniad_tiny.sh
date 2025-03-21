#!/bin/bash

export PYTHONPATH=$PYTHONPATH:~/shared_dir/B2D/Bench2Drive/Bench2DriveZoo #jw) TCP import error
BASE_PORT=10000 #inwoong
# BASE_PORT=2000 #localhost
BASE_TM_PORT=50000
IS_BENCH2DRIVE=True
BASE_ROUTES=leaderboard/data/my_routes_highway_24795_50pts
TEAM_AGENT=team_code/uniad_b2d_agent.py
TEAM_CONFIG=./../Bench2DriveZoo/adzoo/uniad/configs/stage2_e2e/tiny_e2e_b2d.py+./../Bench2DriveZoo/ckpts/uniad_tiny_b2d.pth
# base_e2e_b2d.py : 모델 구조를 정의(예: UniAD의 네트워크 아키텍처).
# TEAM_CONFIG=your_team_agent_ckpt.pth   # for TCP and ADMLP
# TEAM_CONFIG=your_team_agent_config.py+your_team_agent_ckpt.pth # for UniAD and VAD
BASE_CHECKPOINT_ENDPOINT=eval
SAVE_PATH=./eval_v1/
PLANNER_TYPE=only_traj

# GPU_RANK=3
GPU_RANK=0
PORT=$BASE_PORT
TM_PORT=$BASE_TM_PORT
ROUTES="${BASE_ROUTES}.xml"
CHECKPOINT_ENDPOINT="${BASE_CHECKPOINT_ENDPOINT}.json"
HOST="192.168.0.6" #inwoong
# HOST="127.0.0.1"  #localhost
bash leaderboard/scripts/run_evaluation.sh $PORT $TM_PORT $IS_BENCH2DRIVE $ROUTES $TEAM_AGENT $TEAM_CONFIG $CHECKPOINT_ENDPOINT $SAVE_PATH $PLANNER_TYPE $GPU_RANK $HOST
