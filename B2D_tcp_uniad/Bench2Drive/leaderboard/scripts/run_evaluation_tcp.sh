#!/bin/bash
# Must set CARLA_ROOT
export CARLA_ROOT=YOUR_CARLA_PATH
export CARLA_SERVER=${CARLA_ROOT}/CarlaUE4.sh
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg
export PYTHONPATH=$PYTHONPATH:leaderboard
export PYTHONPATH=$PYTHONPATH:leaderboard/team_code
export PYTHONPATH=$PYTHONPATH:scenario_runner
export SCENARIO_RUNNER_ROOT=scenario_runner

export LEADERBOARD_ROOT=leaderboard
export CHALLENGE_TRACK_CODENAME=SENSORS
export PORT=${1}
export TM_PORT=${2}
export REPETITIONS=1 # multiple evaluation runs
export RESUME=True
export IS_BENCH2DRIVE=${3}
export PLANNER_TYPE=${9}
export GPU_RANK=${10}
export RECORD_PATH="" #jw) parsing error

# TCP evaluation
export ROUTES=${4}
export TEAM_AGENT=${5}
export TEAM_CONFIG=${6} #jw) 필요 없음
export CHECKPOINT_ENDPOINT=${7}
export SAVE_PATH=${8}
export HOST=${11} #jw
# export DEBUG_CHALLENGE=0
export DEBUG_MODE=${12} #jw


# echo "PORT=$PORT"
# echo "TM_PORT=$TM_PORT"
# echo "IS_BENCH2DRIVE=$IS_BENCH2DRIVE"
# echo "ROUTES=$ROUTES"
# echo "TEAM_AGENT=$TEAM_AGENT"
# echo "TEAM_CONFIG=$TEAM_CONFIG"
# echo "CHECKPOINT_ENDPOINT=$CHECKPOINT_ENDPOINT"
# echo "SAVE_PATH=$SAVE_PATH"
# echo "PLANNER_TYPE=$PLANNER_TYPE"
# echo "GPU_RANK=$GPU_RANK"
# echo "HOST=$HOST"
# echo "DEBUG_CHALLENGE=$DEBUG_CHALLENGE"

# echo ""
# echo "▶ Calling run_evaluation.sh with 12 args:"
# echo "$PORT" "$TM_PORT" "$IS_BENCH2DRIVE" "$ROUTES" "$TEAM_AGENT" "$TEAM_CONFIG" \
#      "$CHECKPOINT_ENDPOINT" "$SAVE_PATH" "$PLANNER_TYPE" "$GPU_RANK" "$HOST" "$DEBUG_CHALLENGE"


CUDA_VISIBLE_DEVICES=${GPU_RANK} python ${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator_tcp.py \
--routes=${ROUTES} \
--repetitions=${REPETITIONS} \
--track=${CHALLENGE_TRACK_CODENAME} \
--checkpoint=${CHECKPOINT_ENDPOINT} \
--agent=${TEAM_AGENT} \
--debug=${DEBUG_MODE} \
--record=${RECORD_PATH} \
--resume=${RESUME} \
--port=${PORT} \
--traffic-manager-port=${TM_PORT} \
--gpu-rank=${GPU_RANK} \
--host=${HOST} 
# --agent-config=${TEAM_CONFIG} \ #jw) 필요 없음