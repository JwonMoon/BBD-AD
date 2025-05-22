#!/bin/bash

if [ "$#" != "2" ]; then
    echo "Usage: $0 [eval model] [log file name]"
    exit 1
fi

SCRIPT_DIR=$(dirname "$(realpath "$0")")
EVAL_MODEL=$1
LOG_FILE=$2
LOG_PATH="$SCRIPT_DIR/../jw_ws/log/$LOG_FILE"

# start logging GPU usage (background)
nvidia-smi --query-gpu=timestamp,utilization.gpu,utilization.memory,memory.total,memory.used,power.draw --format=csv -l 1 > "$LOG_PATH" &

# save PID# of background process
GPU_LOG_PID=$!

# run evaluation based on EVAL_MODEL
cd "$SCRIPT_DIR/Bench2Drive"
bash "leaderboard/scripts/run_evaluation_debug_$EVAL_MODEL.sh"

# end logging
kill $GPU_LOG_PID
