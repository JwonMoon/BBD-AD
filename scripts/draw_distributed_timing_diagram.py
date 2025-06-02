import matplotlib.pyplot as plt

# 평균 시간 (단위: ms)
tick_time = 1.992
sensor_delay = 142.566
backbone_time = 189.494 # #
net_delay_to_branch = 37.512
branch_time = 51.437
net_delay_to_apply = 7.355
apply_ctrl_time = 0.021
net_delay_to_tick_trigger = 40.079

# 시작 시간 계산
t1 = {}
t1["desktop_tick_start"] = 0
t1["desktop_tick_end"] = t1["desktop_tick_start"] + tick_time
t1["orinA_sensor_ready"] = t1["desktop_tick_end"] + sensor_delay
t1["orinA_backbone_end"] = t1["orinA_sensor_ready"] + backbone_time
t1["orinB_branch_start"] = t1["orinA_backbone_end"] + net_delay_to_branch
t1["orinB_branch_end"] = t1["orinB_branch_start"] + branch_time
t1["desktop_apply_start"] = t1["orinB_branch_end"] + net_delay_to_apply
t1["desktop_apply_end"] = t1["desktop_apply_start"] + apply_ctrl_time

# Frame 2
t2 = {}
t2["desktop_tick_start"] = t1["orinA_backbone_end"] + net_delay_to_tick_trigger
t2["desktop_tick_end"] = t2["desktop_tick_start"] + tick_time
t2["orinA_sensor_ready"] = t2["desktop_tick_end"] + sensor_delay
t2["orinA_backbone_end"] = t2["orinA_sensor_ready"] + backbone_time
t2["orinB_branch_start"] = t2["orinA_backbone_end"] + net_delay_to_branch
t2["orinB_branch_end"] = t2["orinB_branch_start"] + branch_time
t2["desktop_apply_start"] = t2["orinB_branch_end"] + net_delay_to_apply
t2["desktop_apply_end"] = t2["desktop_apply_start"] + apply_ctrl_time

fig, ax = plt.subplots(figsize=(14, 3))

y_locs = {"Desktop": 2, "Orin A": 1, "Orin B": 0}

MIN_WIDTH = 5  # 최소 바 길이 (ms 단위)

def draw_bar(y_label, start, end, label, color):
    y = y_locs[y_label]
    duration = end - start
    visual_width = max(duration, MIN_WIDTH)
    
    ax.barh(y, visual_width, left=start, height=0.4, color=color)
    
    ax.text(start + visual_width / 2, y, label,
            va='center', ha='center',
            color='black' if visual_width < 20 else 'white', fontsize=8)


def draw_link(x1, y1_label, x2, y2_label, label, color='gray'):
    y1 = y_locs[y1_label]
    y2 = y_locs[y2_label]
    ax.plot([x1, x2], [y1, y2], linestyle='dotted', color=color)
    if label:
        y_mid = (y1 + y2) / 2
        ax.text((x1 + x2) / 2, y_mid, label, fontsize=8, ha='center', va='bottom', color=color)

# Frame 1
draw_bar("Desktop", t1["desktop_tick_start"], t1["desktop_tick_end"], "tick", 'lightgreen')
draw_bar("Orin A", t1["orinA_sensor_ready"], t1["orinA_backbone_end"], f"Backbone\n{backbone_time:.0f} ms", 'steelblue')
draw_bar("Orin B", t1["orinB_branch_start"], t1["orinB_branch_end"], f"Branch\n{branch_time:.0f} ms", 'cornflowerblue')
# draw_bar("Desktop", t1["desktop_apply_start"], t1["desktop_apply_end"], "apply", 'indianred')

draw_link(t1["desktop_tick_end"], "Desktop", t1["orinA_sensor_ready"], "Orin A", f"{sensor_delay:.0f} ms")
draw_link(t1["orinA_backbone_end"], "Orin A", t1["orinB_branch_start"], "Orin B", f"{net_delay_to_branch:.0f} ms")
# draw_link(t1["orinB_branch_end"], "Orin B", t1["desktop_apply_start"], "Desktop", f"{net_delay_to_apply:.0f} ms")
draw_link(t1["orinA_backbone_end"], "Orin A", t2["desktop_tick_start"], "Desktop", f"{net_delay_to_tick_trigger:.0f} ms")

# Frame 2
draw_bar("Desktop", t2["desktop_tick_start"], t2["desktop_tick_end"], "tick", 'lightgreen')
draw_bar("Orin A", t2["orinA_sensor_ready"], t2["orinA_backbone_end"], f"Backbone\n{backbone_time:.0f} ms", 'steelblue')
draw_bar("Orin B", t2["orinB_branch_start"], t2["orinB_branch_end"], f"Branch\n{branch_time:.0f} ms", 'cornflowerblue')
# draw_bar("Desktop", t2["desktop_apply_start"], t2["desktop_apply_end"], "apply", 'coral')

draw_link(t2["desktop_tick_end"], "Desktop", t2["orinA_sensor_ready"], "Orin A", f"{sensor_delay:.0f} ms")
draw_link(t2["orinA_backbone_end"], "Orin A", t2["orinB_branch_start"], "Orin B", f"{net_delay_to_branch:.0f} ms")
# draw_link(t2["orinB_branch_end"], "Orin B", t2["desktop_apply_start"], "Desktop", f"{net_delay_to_apply:.0f} ms")

# 마무리
ax.set_xlim(0, t2["desktop_apply_end"] + 50)
ax.set_xlabel("Time (ms)")
ax.set_title("Dual-Orin Distributed Inference Timing Diagram (2 Frames, Mean Values)")
ax.set_yticks(list(y_locs.values()))
ax.set_yticklabels(list(y_locs.keys()))
ax.grid(True, axis='x', linestyle='--', alpha=0.3)
plt.tight_layout()
plt.show()

