#!/bin/bash

export PYTHONPATH=$PYTHONPATH:~/shared_dir/B2D_Demo/B2D_tcp_uniad/Bench2Drive/Bench2DriveZoo #jw) TCP import error
BASE_PORT=10000 #localhost
BASE_TM_PORT=50000
IS_BENCH2DRIVE=True
BASE_ROUTES=leaderboard/data/my_routes_highway_26406_40pts
# TEAM_AGENT=leaderboard/team_code/tcp_b2d_agent.py
TEAM_CONFIG=./../Bench2DriveZoo/ckpts/tcp_b2d.ckpt   # 필요 없음
TEAM_AGENT=leaderboard/leaderboard/leaderboard_evaluator.py  # EvaluatorAgent 사용(필요 없음)
BASE_CHECKPOINT_ENDPOINT=eval
SAVE_PATH=./../../my_ws/single/eval_v1/
# PLANNER_TYPE=only_traj
PLANNER_TYPE=merge_ctrl_traj
# GPU_RANK=3
GPU_RANK=0
PORT=$BASE_PORT
TM_PORT=$BASE_TM_PORT
ROUTES="${BASE_ROUTES}.xml"
CHECKPOINT_ENDPOINT="${BASE_CHECKPOINT_ENDPOINT}.json"
HOST="127.0.0.1"  #localhost
DEBUG_MODE=1 
# TICK_HZ=30
# TICK_HZ=20
TICK_HZ=10

bash leaderboard/scripts/run_evaluation_uniad_async.sh $PORT $TM_PORT $IS_BENCH2DRIVE $ROUTES $TEAM_AGENT $TEAM_CONFIG \
                                        $CHECKPOINT_ENDPOINT $SAVE_PATH $PLANNER_TYPE $GPU_RANK $HOST $DEBUG_MODE $TICK_HZ
