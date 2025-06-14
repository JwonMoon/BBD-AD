#!/bin/bash

SCRIPT_DIR=$(dirname "$(realpath "$0")")
# log file name(default: gpu_usage_docker.log)
LOG_FILE=${1:-gpu_usage_docker.log}

# start logging GPU usuage (background)
tegrastats --interval 1000 > "$SCRIPT_DIR/../jw_ws/dual_usage/$LOG_FILE" &

# save PID# of background process
GPU_LOG_PID=$!

# run evaluation
# cd "$SCRIPT_DIR/../B2D_tcp/Bench2Drive"
# bash "leaderboard/scripts/ros_run_tcp_backbone.sh"

# end logging
kill $GPU_LOG_PID
