import sys
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

# 1. 명령줄 인자 처리
if len(sys.argv) != 4:
    print("usage: python draw_smoothing.py <log file> <output file> <window_size>")
    sys.exit(1)

log_file = sys.argv[1]  
output_file = sys.argv[2]

try:
    window_size = int(sys.argv[3])  # window_size를 정수로 변환
    if window_size <= 0:
        raise ValueError("window_size must be a positive integer")
    if window_size % 2 == 0:  # Savitzky-Golay는 홀수 window size 요구
        window_size += 1
        print(f"Adjusted window_size to {window_size} (must be odd for Savitzky-Golay).")
except ValueError as e:
    print(f"Error: {e}. Please provide a valid positive integer for window_size.")
    sys.exit(1)

# 2. CSV 파일 읽기
try:
    data = pd.read_csv(log_file)
except Exception as e:
    print(f"Error reading CSV file: {e}")
    sys.exit(1)

# 3. 열 이름 정리
data.columns = data.columns.str.strip()
print("Columns in the data:", data.columns)

# jw - 인덱스 1에 해당하는 두 번째 줄 제거
# data = data.drop(index=1) 

# 4. 데이터 전처리: 숫자형으로 변환
def clean_numeric(column):
    return pd.to_numeric(column, errors='coerce')

try:
    data['utilization.gpu [%]'] = clean_numeric(data['utilization.gpu [%]'])
    data['utilization.memory [%]'] = clean_numeric(data['utilization.memory [%]'])
    data['memory.total [MiB]'] = clean_numeric(data['memory.total [MiB]'])
    data['memory.used [MiB]'] = clean_numeric(data['memory.used [MiB]'])
    data['power.draw [W]'] = clean_numeric(data['power.draw [W]'])
    data['cpu_usage [%]'] = clean_numeric(data['cpu_usage [%]'])
    data['memory_used [MiB]'] = clean_numeric(data['memory_used [MiB]'])
    data['memory_total [MiB]'] = clean_numeric(data['memory_total [MiB]'])
    data['memory_utilization [%]'] = clean_numeric(data['memory_utilization [%]'])
except Exception as e:
    print(f"Error in data preprocessing: {e}")
    sys.exit(1)

# 결측값 확인 및 처리
if data.isnull().any().any():
    print("Warning: Data contains NaN values. Filling with 0 for simplicity.")
    data = data.fillna(0)

# 5. 시간축 설정
try:
    time = pd.to_datetime(data['timestamp'])
except Exception as e:
    print(f"Error converting timestamp: {e}")
    sys.exit(1)

# 6. 데이터 평활화 (Savitzky-Golay 필터 적용)
if window_size >= len(data):
    print(f"Error: window_size ({window_size}) is larger than data length ({len(data)}). Using max odd value instead.")
    window_size = len(data) - 1 if len(data) % 2 == 0 else len(data)

# Savitzky-Golay 필터 적용 (window_size와 polyorder 설정)
polyorder = 2  # 다항식 차수 (2차 추천, 노이즈와 부드러움 균형)
data['utilization.gpu [%]_smoothed'] = savgol_filter(data['utilization.gpu [%]'], window_size, polyorder)
data['utilization.memory [%]_smoothed'] = savgol_filter(data['utilization.memory [%]'], window_size, polyorder)
data['memory.used [MiB]_smoothed'] = savgol_filter(data['memory.used [MiB]'], window_size, polyorder)
data['power.draw [W]_smoothed'] = savgol_filter(data['power.draw [W]'], window_size, polyorder)
data['cpu_usage [%]_smoothed'] = savgol_filter(data['cpu_usage [%]'], window_size, polyorder)
data['memory_used [MiB]_smoothed'] = savgol_filter(data['memory_used [MiB]'], window_size, polyorder)
data['memory_utilization [%]_smoothed'] = savgol_filter(data['memory_utilization [%]'], window_size, polyorder)

# 7. 그래프 그리기
plt.figure(figsize=(12, 14))

# GPU Utilization
plt.subplot(4, 2, 1)
plt.plot(time, data['utilization.gpu [%]_smoothed'], label='GPU Utilization (%)', color='blue', linewidth=1.5, alpha=0.8)
plt.title('GPU Utilization (Smoothed)', fontsize=10)
plt.xlabel('Time', fontsize=8)
plt.ylabel('Utilization (%)', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend(fontsize=8)

# GPU Memory Utilization
plt.subplot(4, 2, 2)
plt.plot(time, data['utilization.memory [%]_smoothed'], label='Memory Utilization (%)', color='green', linewidth=1.5, alpha=0.8)
plt.title('GPU Memory Utilization (Smoothed)', fontsize=10)
plt.xlabel('Time', fontsize=8)
plt.ylabel('Utilization (%)', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend(fontsize=8)

# GPU Memory Total (상수로 가정)
plt.subplot(4, 2, 3)
plt.axhline(y=data['memory.total [MiB]'].mean(), color='orange', linestyle='-', label='Memory Total (MiB)', alpha=0.8)
plt.title('GPU Memory Total (Assumed Constant)', fontsize=10)
plt.xlabel('Time', fontsize=8)
plt.ylabel('Memory (MiB)', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend(fontsize=8)

# GPU Memory Used
plt.subplot(4, 2, 4)
plt.plot(time, data['memory.used [MiB]_smoothed'], label='Memory Used (MiB)', color='red', linewidth=1.5, alpha=0.8)
plt.title('GPU Memory Used (Smoothed)', fontsize=10)
plt.xlabel('Time', fontsize=8)
plt.ylabel('Memory (MiB)', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend(fontsize=8)

# GPU Power Draw
plt.subplot(4, 2, 5)
plt.plot(time, data['power.draw [W]_smoothed'], label='Power Draw (W)', color='purple', linewidth=1.5, alpha=0.8)
plt.title('GPU Power Draw (Smoothed)', fontsize=10)
plt.xlabel('Time', fontsize=8)
plt.ylabel('Power (W)', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend(fontsize=8)

# CPU Usage
plt.subplot(4, 2, 6)
plt.plot(time, data['cpu_usage [%]_smoothed'], label='CPU Usage (%)', color='cyan', linewidth=1.5, alpha=0.8)
plt.title('CPU Usage (Smoothed)', fontsize=10)
plt.xlabel('Time', fontsize=8)
plt.ylabel('Usage (%)', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend(fontsize=8)

# System Memory Used
plt.subplot(4, 2, 7)
plt.plot(time, data['memory_used [MiB]_smoothed'], label='Memory Used (MiB)', color='magenta', linewidth=1.5, alpha=0.8)
plt.title('System Memory Used (Smoothed)', fontsize=10)
plt.xlabel('Time', fontsize=8)
plt.ylabel('Memory (MiB)', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend(fontsize=8)

# System Memory Utilization
plt.subplot(4, 2, 8)
plt.plot(time, data['memory_utilization [%]_smoothed'], label='Memory Utilization (%)', color='brown', linewidth=1.5, alpha=0.8)
plt.title('System Memory Utilization (Smoothed)', fontsize=10)
plt.xlabel('Time', fontsize=8)
plt.ylabel('Utilization (%)', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend(fontsize=8)

# Window Size 텍스트 추가
plt.figtext(0.5, -0.02, f'Window Size: {window_size}', ha='center', fontsize=10, bbox={"facecolor":"white", "alpha":0.5, "pad":5})

# 8. 레이아웃 조정 및 저장
plt.tight_layout()
try:
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"The graph is saved as {output_file}.")
except Exception as e:
    print(f"Error saving graph: {e}")