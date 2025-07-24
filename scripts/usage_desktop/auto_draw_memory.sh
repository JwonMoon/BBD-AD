#!/bin/bash

# eval_v1 폴더 내의 서브폴더 목록
folders=("tcp_50pts" "tcp_merge_50pts" "uniad_50pts" "uniad_tiny_50pts" "vad_50pts")

# 각 폴더에 대해 명령어 실행
for folder in "${folders[@]}"; do
    echo "Processing folder: $folder"
    
    python scripts/draw_gpu_cpu_smooth.py "/root/shared_dir/B2D_Demo/my_ws/log/$folder" "/root/shared_dir/B2D_Demo/my_ws/output/memory/${folder}_memory.png"

    echo "Completed for $folder"
    echo "-------------------"
done