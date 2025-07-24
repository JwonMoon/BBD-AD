#!/bin/bash

if [ "$#" != "2" ]; then
    echo "Usage: $0 [eval model] [log file name]"
    exit 1
fi

SCRIPT_DIR=$(dirname "$(realpath "$0")")
EVAL_MODEL=$1
LOG_FILE=$2 
LOG_PATH="$SCRIPT_DIR/../my_ws/log/$LOG_FILE"

# sysstat 설치 확인 (mpstat 사용을 위해)
if ! command -v mpstat &> /dev/null; then
    echo "mpstat not found. Please install sysstat: sudo apt install sysstat"
    exit 1
fi

# 로그 디렉토리 생성 (없을 경우)
mkdir -p "$SCRIPT_DIR/../my_ws"

# 로그 헤더 작성 (GPU + CPU + 메모리)
echo "timestamp,utilization.gpu [%],utilization.memory [%],memory.total [MiB],memory.used [MiB],power.draw [W],cpu_usage [%],memory_used [MiB],memory_total [MiB],memory_utilization [%]" > "$LOG_PATH"

# GPU 및 CPU/메모리 사용량 로깅 함수 (백그라운드)
log_usage() {
    while true; do
        # GPU 정보 (nvidia-smi)
        GPU_DATA=$(nvidia-smi --query-gpu=timestamp,utilization.gpu,utilization.memory,memory.total,memory.used,power.draw --format=csv,noheader,nounits)
        
        # CPU 사용률 (mpstat로 전체 사용률 계산: 100 - %idle)
        CPU_USAGE=$(mpstat 1 1 | tail -1 | awk '{print 100 - $NF}')
        
        # 시스템 메모리 정보 (free 명령어로 사용량/총량/점유율 계산)
        MEM_DATA=$(free -m | awk '/Mem:/ {print $3","$2","$3/$2*100}')
        MEM_USED=$(echo "$MEM_DATA" | cut -d',' -f1)
        MEM_TOTAL=$(echo "$MEM_DATA" | cut -d',' -f2)
        MEM_UTIL=$(echo "$MEM_DATA" | cut -d',' -f3)

        # GPU와 CPU/메모리 데이터 결합
        echo "$GPU_DATA, $CPU_USAGE, $MEM_USED, $MEM_TOTAL, $MEM_UTIL" >> "$LOG_PATH"
        
        sleep 1  # 1초 간격
    done
}

# 로깅 시작 (백그라운드)
log_usage &

# 백그라운드 프로세스 PID 저장
LOG_PID=$!

# 평가 스크립트 실행
cd "$SCRIPT_DIR/Bench2Drive"
bash "leaderboard/scripts/run_evaluation_debug_$EVAL_MODEL.sh"

# 로깅 종료
kill $LOG_PID

echo "Logging completed. Data saved to $LOG_PATH"
