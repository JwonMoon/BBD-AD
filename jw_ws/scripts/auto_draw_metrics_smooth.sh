#!/bin/bash

# eval_v1 폴더 내의 서브폴더 목록
folders=("tcp_50pts" "tcp_merge_50pts" "uniad_50pts" "uniad_tiny_50pts" "vad_50pts")

# 각 폴더에 대해 명령어 실행
for folder in "${folders[@]}"; do
    echo "Processing folder: $folder"
    
    # 첫 번째 명령어 실행 (smooth_5)
    python scripts/draw_metrics_smooth.py "/root/shared_dir/B2D_Demo/jw_ws/eval_v1/$folder/metric_info.json" "/root/shared_dir/B2D_Demo/jw_ws/output/metrics/${folder}_metrics_smooth_5.png" 5
    
    # 두 번째 명령어 실행 (smooth_10)
    python scripts/draw_metrics_smooth.py "/root/shared_dir/B2D_Demo/jw_ws/eval_v1/$folder/metric_info.json" "/root/shared_dir/B2D_Demo/jw_ws/output/metrics/${folder}_metrics_smooth_10.png" 10
    
    echo "Completed for $folder"
    echo "-------------------"
done