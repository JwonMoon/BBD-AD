#!/bin/bash
# BASE_PORT=30000
BASE_PORT=10000 #jiwon / inwoong
BASE_TM_PORT=50000
IS_BENCH2DRIVE=True
# BASE_ROUTES=leaderboard/data/bench2drive220
BASE_ROUTES=leaderboard/data/my_routes_highway_24795_50pts
TEAM_AGENT=leaderboard/team_code/tcp_b2d_agent.py
TEAM_CONFIG=./../Bench2DriveZoo/ckpts/tcp_b2d.ckpt
# TEAM_CONFIG=your_team_agent_ckpt.pth   # for TCP and ADMLP
# TEAM_CONFIG=your_team_agent_config.py+your_team_agent_ckpt.pth # for UniAD and VAD
BASE_CHECKPOINT_ENDPOINT=eval
SAVE_PATH=./eval_v1/
# PLANNER_TYPE=only_traj
PLANNER_TYPE=merge_ctrl_traj

# GPU_RANK=3
GPU_RANK=0
PORT=$BASE_PORT
TM_PORT=$BASE_TM_PORT
ROUTES="${BASE_ROUTES}.xml"
CHECKPOINT_ENDPOINT="${BASE_CHECKPOINT_ENDPOINT}.json"
# HOST="192.168.0.6" #inwoong
HOST="192.168.0.2" #jiwon
# HOST="127.0.0.1"  #localhost
DEBUG_CHALLENGE=0
bash leaderboard/scripts/run_evaluation.sh $PORT $TM_PORT $IS_BENCH2DRIVE $ROUTES $TEAM_AGENT $TEAM_CONFIG \
                                        $CHECKPOINT_ENDPOINT $SAVE_PATH $PLANNER_TYPE $GPU_RANK $HOST $DEBUG_CHALLENGE
