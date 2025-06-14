import sys
import pandas as pd
import matplotlib.pyplot as plt
import re
from datetime import datetime

# 1. 명령줄 인자 처리
if len(sys.argv) != 4:
    print("Usage: python draw_tegrastats_log.py <tegrastats.log> <output.png> <window_size>")
    sys.exit(1)

log_file = sys.argv[1]
output_file = sys.argv[2]

try:
    window_size = int(sys.argv[3])
    if window_size <= 0:
        raise ValueError("window_size must be positive")
except ValueError as e:
    print(f"Error: {e}")
    sys.exit(1)

# 2. 로그 파싱
pattern = re.compile(
    r"(\d{2}-\d{2}-\d{4} \d{2}:\d{2}:\d{2}) RAM (\d+)/(\d+)MB.*CPU \[(.*?)\].*GR3D_FREQ (\d+)%.*VDD_GPU_SOC (\d+)mW"
)

data = []
with open(log_file, 'r') as f:
    for line in f:
        match = pattern.search(line)
        if match:
            timestamp_str, ram_used, ram_total, cpu_raw, gr3d_freq, gpu_power = match.groups()
            timestamp = datetime.strptime(timestamp_str, "%m-%d-%Y %H:%M:%S")
            ram_used = int(ram_used)
            ram_total = int(ram_total)
            cpu_usages = [int(part.split('%')[0]) for part in cpu_raw.split(',') if '%' in part]
            cpu_avg = sum(cpu_usages) / len(cpu_usages) if cpu_usages else 0
            gr3d_freq = int(gr3d_freq)
            gpu_power = int(gpu_power)
            data.append({
                "timestamp": timestamp,
                "utilization.gpu [%]": gr3d_freq,
                "power.draw [W]": gpu_power / 1000.0,
                "cpu_usage [%]": cpu_avg,
                "memory_used [MiB]": ram_used,
                "memory_total [MiB]": ram_total,
                "memory_utilization [%]": ram_used / ram_total * 100
            })

if not data:
    print("No valid entries found in tegrastats log.")
    sys.exit(1)

df = pd.DataFrame(data)

# 3. smoothing
if window_size >= len(df):
    print(f"Warning: window_size ({window_size}) too large. Adjusting to length - 1.")
    window_size = len(df) - 1

for col in [
    'utilization.gpu [%]',
    'power.draw [W]',
    'cpu_usage [%]',
    'memory_used [MiB]',
    'memory_utilization [%]'
]:
    df[col + '_smoothed'] = df[col].rolling(window=window_size, center=True).mean()

# 4. Plotting
time = pd.to_datetime(df['timestamp'])
plt.figure(figsize=(12, 10))

# GPU Utilization
plt.subplot(3, 2, 1)
plt.plot(time, df['utilization.gpu [%]_smoothed'], label='GPU Utilization (%)', color='blue')
plt.title('GPU Utilization (Smoothed)')
plt.xlabel('Time')
plt.ylabel('Utilization (%)')
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend()

# GPU Power
plt.subplot(3, 2, 2)
plt.plot(time, df['power.draw [W]_smoothed'], label='Power Draw (W)', color='purple')
plt.title('GPU Power Draw (Smoothed)')
plt.xlabel('Time')
plt.ylabel('Power (W)')
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend()

# CPU Usage
plt.subplot(3, 2, 3)
plt.plot(time, df['cpu_usage [%]_smoothed'], label='CPU Usage (%)', color='cyan')
plt.title('CPU Usage (Smoothed)')
plt.xlabel('Time')
plt.ylabel('Usage (%)')
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend()

# System Memory Used
plt.subplot(3, 2, 4)
plt.plot(time, df['memory_used [MiB]_smoothed'], label='Memory Used (MiB)', color='magenta')
plt.title('System Memory Used (Smoothed)')
plt.xlabel('Time')
plt.ylabel('Memory (MiB)')
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend()

# System Memory Utilization
plt.subplot(3, 2, 5)
plt.plot(time, df['memory_utilization [%]_smoothed'], label='Memory Utilization (%)', color='brown')
plt.title('System Memory Utilization (Smoothed)')
plt.xlabel('Time')
plt.ylabel('Utilization (%)')
plt.grid(True, linestyle='--', alpha=0.3)
plt.legend()

# Window size 텍스트
plt.figtext(0.5, -0.02, f'Window Size: {window_size}', ha='center', fontsize=10, bbox={"facecolor":"white", "alpha":0.5, "pad":5})

# 저장
plt.tight_layout()
try:
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"The graph is saved as {output_file}.")
except Exception as e:
    print(f"Error saving graph: {e}")