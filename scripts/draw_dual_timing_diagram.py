import matplotlib.pyplot as plt

# 평균 시간 (단위: ms)
# TODO; by data type
# backbone_time = 185.3
# network_latency = 0.3
# branch_time = 50.2
backbone_time = 194.702
network_latency = 20.029
branch_time = 64.417
idle_gap = 15.0 

# 시작/끝 시간 계산
backbone_start = 0
backbone_end = backbone_start + backbone_time
backbone_output_time = backbone_end
branch_start = backbone_output_time + network_latency
branch_end = branch_start + branch_time
next_backbone_start = branch_end + idle_gap
next_backbone_end = next_backbone_start + backbone_time

# 시각화
fig, ax = plt.subplots(figsize=(12, 3))

# 🟦 Branch (Orin B)
ax.barh("Orin B", width=branch_time, left=branch_start, height=0.4, color='cornflowerblue')
ax.text((branch_start + branch_end) / 2, "Orin B",
        f"Branch\n{branch_time:.1f} ms", va='center', ha='center', color='white', fontsize=10)
        
# 🔵 Backbone 1 (Orin A)
ax.barh("Orin A", width=backbone_time, left=backbone_start, height=0.4, color='steelblue')
ax.text((backbone_start + backbone_end) / 2, "Orin A",
        f"Backbone\n{backbone_time:.1f} ms", va='center', ha='center', color='white', fontsize=10)

# 🔵 Backbone 2 (Orin A, for idle 이후)
ax.barh("Orin A", width=backbone_time, left=next_backbone_start, height=0.4, color='lightsteelblue')
ax.text((next_backbone_start + next_backbone_end) / 2, "Orin A",
        f"Backbone (next)", va='center', ha='center', color='black', fontsize=9)


# 🔴 Sensor input
ax.axvline(backbone_start, color='red')
ax.text(backbone_start, 1.1, "Sensor input\n0 ms", color='red', ha='center', va='bottom')

# 🔴 Control output
ax.axvline(branch_end, color='red')
ax.text(branch_end, -0.3, f"Control output\n{branch_end:.1f} ms", color='red', ha='center', va='top')

# 🕸️ Network latency
ax.plot([backbone_end, branch_start], ["Orin A", "Orin B"], linestyle='dotted', color='gray')
ax.text((backbone_end + branch_start) / 2, 0.5,
        f"{network_latency:.1f} ms", color='gray', ha='center', fontsize=9)

# 🟨 Idle Gap 표시
ax.plot([branch_end, next_backbone_start], ["Orin B", "Orin A"], linestyle='dotted', color='orange')
ax.text((branch_end + next_backbone_start) / 2, "Orin A",
        f"Idle\n{idle_gap:.1f} ms", ha='center', va='bottom', color='black', fontsize=9)

# 마무리
ax.set_xlim(0, next_backbone_end + 20)
ax.set_xlabel("Time (ms)")
ax.set_title("Distributed Inference Timing Diagram (Worst Case incl. Idle)")
ax.set_yticks(["Orin A", "Orin B"])
ax.grid(True, axis='x', linestyle='--', alpha=0.4)
plt.tight_layout()
plt.savefig("distributed_timing_with_idle.png")
plt.show()