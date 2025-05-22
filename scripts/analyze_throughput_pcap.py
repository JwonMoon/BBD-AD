import argparse
from scapy.all import rdpcap, IP
from collections import defaultdict
import pandas as pd
import matplotlib.pyplot as plt

# 1. 인자 처리
parser = argparse.ArgumentParser(description="Analyze throughput from a pcap file.")
parser.add_argument("pcap_file", help="Path to the pcap file (e.g., trace.pcap)")
args = parser.parse_args()

# 2. PCAP 파일 로드
packets = rdpcap(args.pcap_file)

# 3. 초 단위로 누적 전송량 계산
throughput_data = defaultdict(lambda: {'bytes': 0, 'packets': 0})
for pkt in packets:
    if IP in pkt:
        ts = int(pkt.time)
        size = len(pkt)
        throughput_data[ts]['bytes'] += size
        throughput_data[ts]['packets'] += 1

# 4. DataFrame 생성
tp_df = pd.DataFrame([
    {'timestamp': ts, 'bytes': data['bytes'], 'packets': data['packets'], 'Mbps': data['bytes'] * 8 / 1_000_000}
    for ts, data in sorted(throughput_data.items())
])

# 5. 통계 출력
avg_mbps = tp_df['Mbps'].mean()
max_mbps = tp_df['Mbps'].max()
min_mbps = tp_df['Mbps'].min()
std_mbps = tp_df['Mbps'].std()

print("📊 Throughput Summary")
print(f"- Average: {avg_mbps:.2f} Mbps")
print(f"- Max:     {max_mbps:.2f} Mbps")
print(f"- Min:     {min_mbps:.2f} Mbps")
print(f"- Stddev:  {std_mbps:.2f} Mbps")

# 6. 그래프 출력
plt.figure(figsize=(10, 4))
plt.plot(tp_df['timestamp'], tp_df['Mbps'], marker='o')
plt.axhline(avg_mbps, color='red', linestyle='--', label=f'Average: {avg_mbps:.2f} Mbps')
plt.title("📶 Throughput over Time")
plt.xlabel("Time (seconds)")
plt.ylabel("Throughput (Mbps)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

