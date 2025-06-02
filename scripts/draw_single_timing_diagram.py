import matplotlib.pyplot as plt

# 평균값 입력 (단위: ms)
T_tick = 2.728
net_delay_to_model = 26.921
T_model = 93.304
net_delay_to_tick = 18.607

# 시간 계산 (2 프레임)
# Frame 1
tick1_start = 0
tick1_end = tick1_start + T_tick

model1_start = tick1_end + net_delay_to_model
model1_end = model1_start + T_model

tick2_start = model1_end + net_delay_to_tick
tick2_end = tick2_start + T_tick

# Frame 2
model2_start = tick2_end + net_delay_to_model
model2_end = model2_start + T_model

tick3_start = model2_end + net_delay_to_tick
tick3_end = tick3_start + T_tick

# 시각화
fig, ax = plt.subplots(figsize=(12, 3))


# Model 1
ax.barh("Orin", width=T_model, left=model1_start, height=0.4, color='steelblue')
ax.text((model1_start + model1_end) / 2, "Orin", f"E2E model\n{T_model:.1f} ms", ha='center', va='center', color='white', fontsize=9)

# Tick 1
ax.barh("Desktop", width=T_tick, left=tick1_start, height=0.4, color='lightgreen')
ax.text((tick1_start + tick1_end) / 2, "Desktop", "tick", ha='center', va='center', color='black')

# Delay lines (1st frame)
ax.plot([tick1_end, model1_start], ["Desktop", "Orin"], linestyle='dotted', color='gray')
ax.text((tick1_end + model1_start) / 2, 0.5, f"{net_delay_to_model:.1f} ms", color='gray', ha='center', fontsize=9)
ax.plot([model1_end, tick2_start], ["Orin", "Desktop"], linestyle='dotted', color='gray')
ax.text((model1_end + tick2_start) / 2, 0.5, f"{net_delay_to_tick:.1f} ms", color='gray', ha='center', fontsize=9)

# Tick 2
ax.barh("Desktop", width=T_tick, left=tick2_start, height=0.4, color='lightgreen')
ax.text((tick2_start + tick2_end) / 2, "Desktop", "tick", ha='center', va='center', color='black')

# Model 2
ax.barh("Orin", width=T_model, left=model2_start, height=0.4, color='steelblue')
ax.text((model2_start + model2_end) / 2, "Orin", f"E2E model\n{T_model:.1f} ms", ha='center', va='center', color='white', fontsize=9)

# Delay lines (2nd frame)
ax.plot([tick2_end, model2_start], ["Desktop", "Orin"], linestyle='dotted', color='gray')
ax.text((tick2_end + model2_start) / 2, 0.5, f"{net_delay_to_model:.1f} ms", color='gray', ha='center', fontsize=9)
ax.plot([model2_end, tick3_start], ["Orin", "Desktop"], linestyle='dotted', color='gray')
ax.text((model2_end + tick3_start) / 2, 0.5, f"{net_delay_to_tick:.1f} ms", color='gray', ha='center', fontsize=9)

# Tick 3 (표현만)
ax.barh("Desktop", width=T_tick, left=tick3_start, height=0.4, color='lightgreen')
ax.text((tick3_start + tick3_end) / 2, "Desktop", "tick", ha='center', va='center', color='black')

# 마무리
ax.set_xlim(0, tick3_end + 50)
ax.set_xlabel("Time (ms)")
ax.set_title("Single-Orin Inference Timing Diagram (2 Frames, Mean Values)")
ax.set_yticks(["Desktop", "Orin"])
ax.grid(True, axis='x', linestyle='--', alpha=0.4)
plt.tight_layout()
plt.show()
