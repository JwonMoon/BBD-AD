import sys
import pandas as pd
import matplotlib.pyplot as plt

# 1. 명령줄 인자 처리
if len(sys.argv) != 3:
    print("usage: python draw_gpu.py <log file> <output file>")
    sys.exit(1)

log_file = sys.argv[1]  
output_file = sys.argv[2]

# 2. CSV 파일 읽기
data = pd.read_csv(log_file)

# 3. 열 이름 정리
data.columns = data.columns.str.strip()
print(data.columns)

# 4. 시간축 설정
time = pd.to_datetime(data['timestamp'])

# 5. 그래프 그리기
plt.figure(figsize=(12, 10))

plt.subplot(3, 2, 1)
plt.plot(time, data['utilization.gpu [%]'], label='GPU Utilization (%)', color='blue')
plt.title('GPU Utilization')
plt.xlabel('Time')
plt.ylabel('Utilization (%)')
plt.grid(True)
plt.legend()

plt.subplot(3, 2, 2)
plt.plot(time, data['utilization.memory [%]'], label='Memory Utilization (%)', color='green')
plt.title('Memory Utilization')
plt.xlabel('Time')
plt.ylabel('Utilization (%)')
plt.grid(True)
plt.legend()

plt.subplot(3, 2, 3)
plt.plot(time, data['memory.total [MiB]'], label='Memory Total (MiB)', color='orange')
plt.title('Memory Total')
plt.xlabel('Time')
plt.ylabel('Memory (MiB)')
plt.grid(True)
plt.legend()

plt.subplot(3, 2, 4)
plt.plot(time, data['memory.used [MiB]'], label='Memory Used (MiB)', color='red')
plt.title('Memory Used')
plt.xlabel('Time')
plt.ylabel('Memory (MiB)')
plt.grid(True)
plt.legend()

plt.subplot(3, 2, 5)
plt.plot(time, data['power.draw [W]'], label='Power Draw (W)', color='purple')
plt.title('Power Draw')
plt.xlabel('Time')
plt.ylabel('Power (W)')
plt.grid(True)
plt.legend()

# 6. 레이아웃 조정 및 저장
plt.tight_layout()
plt.savefig(output_file)  
print(f"The graph is saved as {output_file}.")