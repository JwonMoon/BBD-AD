#!/bin/bash
BASE_PORT=10000
BASE_TM_PORT=50000
IS_BENCH2DRIVE=True
BASE_ROUTES=leaderboard/data/my_routes_highway_2
TEAM_AGENT=leaderboard/team_code/driveadapter_b2d_agent.py
TEAM_CONFIG=./../Bench2DriveZoo/DriveAdapter/open_loop_training/configs/driveadapter.py+./../Bench2DriveZoo/ckpts/driveadapter_189k.pth
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
HOST="192.168.0.5" #inwoong
# HOST="127.0.0.1"
DEBUG_CHALLENGE=0
bash leaderboard/scripts/run_evaluation.sh $PORT $TM_PORT $IS_BENCH2DRIVE $ROUTES $TEAM_AGENT $TEAM_CONFIG \
                                        $CHECKPOINT_ENDPOINT $SAVE_PATH $PLANNER_TYPE $GPU_RANK $HOST $DEBUG_CHALLENGE
