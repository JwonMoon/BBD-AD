import sys
import pandas as pd
import matplotlib.pyplot as plt

# 1. 명령줄 인자 처리
if len(sys.argv) != 3:
    print("usage: python draw_smoothing.py <log file> <output file>")
    sys.exit(1)

log_file = sys.argv[1]  
output_file = sys.argv[2]

# 2. CSV 파일 읽기
data = pd.read_csv(log_file)

# 3. 열 이름 정리
data.columns = data.columns.str.strip()
print(data.columns)

# 4. 데이터 전처리: 단위 제거 및 숫자로 변환
# 각 열에서 단위(%, MiB, W)를 제거하고 숫자로 변환
data['utilization.gpu [%]'] = data['utilization.gpu [%]'].str.replace('%', '').str.strip().astype(float)
data['utilization.memory [%]'] = data['utilization.memory [%]'].str.replace('%', '').str.strip().astype(float)
data['memory.total [MiB]'] = data['memory.total [MiB]'].str.replace('MiB', '').str.strip().astype(float)
data['memory.used [MiB]'] = data['memory.used [MiB]'].str.replace('MiB', '').str.strip().astype(float)
data['power.draw [W]'] = data['power.draw [W]'].str.replace('W', '').str.strip().astype(float)

# 5. 시간축 설정
time = pd.to_datetime(data['timestamp'])

# 6. 데이터 평활화 (이동 평균 적용)
window_size = 5  # 5초 간격으로 평활화 (조정 가능)
data['utilization.gpu [%]_smoothed'] = data['utilization.gpu [%]'].rolling(window=window_size, center=True).mean()
data['utilization.memory [%]_smoothed'] = data['utilization.memory [%]'].rolling(window=window_size, center=True).mean()
data['memory.used [MiB]_smoothed'] = data['memory.used [MiB]'].rolling(window=window_size, center=True).mean()
data['power.draw [W]_smoothed'] = data['power.draw [W]'].rolling(window=window_size, center=True).mean()

# 7. 그래프 그리기
plt.figure(figsize=(12, 10))

# GPU Utilization
plt.subplot(3, 2, 1)
plt.plot(time, data['utilization.gpu [%]_smoothed'], label='GPU Utilization (%)', color='blue', linewidth=1.5, alpha=0.8)
plt.title('GPU Utilization (Smoothed)', fontsize=10)
plt.xlabel('Time', fontsize=8)
plt.ylabel('Utilization (%)', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend(fontsize=8)

# Memory Utilization
plt.subplot(3, 2, 2)
plt.plot(time, data['utilization.memory [%]_smoothed'], label='Memory Utilization (%)', color='green', linewidth=1.5, alpha=0.8)
plt.title('Memory Utilization (Smoothed)', fontsize=10)
plt.xlabel('Time', fontsize=8)
plt.ylabel('Utilization (%)', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend(fontsize=8)

# Memory Total (상수이므로 수평선)
plt.subplot(3, 2, 3)
plt.axhline(y=data['memory.total [MiB]'].iloc[0], color='orange', linestyle='-', label='Memory Total (MiB)', alpha=0.8)
plt.title('Memory Total', fontsize=10)
plt.xlabel('Time', fontsize=8)
plt.ylabel('Memory (MiB)', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend(fontsize=8)

# Memory Used
plt.subplot(3, 2, 4)
plt.plot(time, data['memory.used [MiB]_smoothed'], label='Memory Used (MiB)', color='red', linewidth=1.5, alpha=0.8)
plt.title('Memory Used (Smoothed)', fontsize=10)
plt.xlabel('Time', fontsize=8)
plt.ylabel('Memory (MiB)', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend(fontsize=8)

# Power Draw
plt.subplot(3, 2, 5)
plt.plot(time, data['power.draw [W]_smoothed'], label='Power Draw (W)', color='purple', linewidth=1.5, alpha=0.8)
plt.title('Power Draw (Smoothed)', fontsize=10)
plt.xlabel('Time', fontsize=8)
plt.ylabel('Power (W)', fontsize=8)
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend(fontsize=8)

# 8. 레이아웃 조정 및 저장
plt.tight_layout()
plt.savefig(output_file, dpi=300)
print(f"The graph is saved as {output_file}.")