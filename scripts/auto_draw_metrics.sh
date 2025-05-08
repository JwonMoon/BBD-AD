#!/bin/bash

# eval_v1 폴더 내의 서브폴더 목록
folders=("tcp_50pts" "tcp_merge_50pts" "uniad_50pts" "uniad_tiny_50pts" "vad_50pts")

# 각 폴더에 대해 명령어 실행
for folder in "${folders[@]}"; do
    echo "Processing folder: $folder"
    
    python scripts/draw_metrics.py "/root/shared_dir/B2D_Demo/jw_ws/eval_v1/$folder/metric_info.json" "/root/shared_dir/B2D_Demo/jw_ws/output/metrics/${folder}_metrics.png"
    
    echo "Completed for $folder"
    echo "-------------------"
done