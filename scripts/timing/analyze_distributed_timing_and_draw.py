import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import argparse

def summarize(col):
    return {
        "mean": round(col.mean(), 3),
        "std": round(col.std(), 3),
        "min": round(col.min(), 3),
        "max": round(col.max(), 3),
        "p95": round(col.quantile(0.95), 3),
        "p99": round(col.quantile(0.99), 3),
    }

# def draw_block(ax, start, duration, level, label, color):
#     ax.add_patch(patches.Rectangle((start, level - 0.2), duration, 0.4, color=color, alpha=0.6))
#     if label == "moved pub":
#         r_label = "pub"
#         ax.text(start + 20, level, f"{r_label}\n{duration:.2f}", ha="center", va="center", fontsize=9)
#     else:
#         ax.text(start + duration / 2, level, f"{label}\n{duration:.2f}", ha="center", va="center", fontsize=9)

def draw_block(ax, start, duration, level, label, color, threshold=7.0, vertical_offset=0.0):
    ax.add_patch(patches.Rectangle((start, level - 0.2), duration, 0.4, color=color, alpha=0.6))
    if not label:
        return
    if duration < threshold:
        x_text = start + duration + max(1.5*duration , 10)
        y_text = level + min(0.22*duration , 0.5) + vertical_offset
        ax.annotate(
            f"{label}\n{duration:.2f}",
            xy=(start + duration / 2, level),
            xytext=(x_text, y_text),
            textcoords="data",
            arrowprops=dict(arrowstyle="->", lw=0.8),
            fontsize=8,
            ha="left",
            va="center",
        )
    else:
        ax.text(start + duration / 2, level, f"{label}\n{duration:.2f}", ha="center", va="center", fontsize=9)

def draw_latency(ax, x0, y0, x1, y1, label, value, vertical_offset=0):
    ax.plot([x0, x1], [y0, y1], "k--", lw=1)
    xm, ym = (x0 + x1) / 2, (y0 + y1) / 2 + vertical_offset
    ax.text(xm, ym, f"{label}: {value:.2f}", ha="center", fontsize=8, color="black")

def draw_interval(ax, x0, y0, x1, y1, label, value, offset=0.4):
    """
    Draws a dimension line (CAD-style) with arrow heads and label above the timeline.
    """
    y = max(y0, y1) + offset  # 기준선 y축 위치

    # 치수선
    ax.annotate(
        '', xy=(x0, y), xytext=(x1, y),
        arrowprops=dict(arrowstyle='<->', lw=0.8, color='black')
    )

    # 수직 기준선 (양쪽)
    ax.plot([x0, x0], [y0 + 0.23, y], color='black', linestyle='-', lw=0.8)
    ax.plot([x1, x1], [y1 + 0.23, y], color='black', linestyle='-', lw=0.8)

    # 텍스트 (중앙)
    xm = (x0 + x1) / 2
    ax.text(xm, y + 0.05, f"{label}: {value:.2f}", ha="center", va="bottom", fontsize=8, color="black")

def draw_diagram_from_mean_durations(mean):
    fig, ax = plt.subplots(figsize=(18, 4))
    y_levels = {"evaluator": 3, "Backbone": 2, "branch": 1}

    def compute_frame(prev_end):
        f = {}
        f["T_bb_cb_start"] = prev_end
        f["T_car_tick_start"] = f["T_bb_cb_start"] + mean["D_car_tick_call"]
        f["T_bb_cb_end"] = f["T_bb_cb_start"] + mean["D_eval_tick_total"]

        f["T_bb_start"] = f["T_bb_cb_end"] + mean["D_sensor_delay"]
        f["T_bb_rx_start"] = f["T_bb_start"]
        f["T_bb_pp_start"] = f["T_bb_rx_start"] + mean["D_bb_rx"]
        f["T_bb_net_start"] = f["T_bb_pp_start"] + mean["D_bb_pp"]
        f["T_bb_tick_pub_start"] = f["T_bb_net_start"] + mean["D_bb_net"] # inlcude msg gen time
        f["T_bb_bb_pub_start"] = f["T_bb_tick_pub_start"] + mean["D_bb_tick_pub"]
        f["T_bb_bb_pub_end"] = f["T_bb_bb_pub_start"] + mean["D_bb_bb_pub"]

        f["T_br_start"] = f["T_bb_bb_pub_start"] + mean["D_transfer_delay"]
        f["T_br_rx_start"] = f["T_br_start"]
        f["T_br_net_start"] = f["T_br_rx_start"] + mean["D_br_rx"]
        f["T_br_pid_start"] = f["T_br_net_start"] + mean["D_br_net"]
        f["T_br_pub_start"] = f["T_br_pid_start"] + mean["D_br_pid"]
        f["T_br_pub_end"] = f["T_br_pub_start"] + mean["D_br_pub"]

        f["T_br_cb_start"] = f["T_br_pub_start"] + mean["D_control_delay"]
        f["T_car_ctrl_start"] = f["T_br_cb_start"] + mean["D_car_ctrl_call"]
        f["T_br_cb_end"] = f["T_br_cb_start"] + mean["D_eval_ctrl_total"]
        return f

    # 프레임 0
    f0 = compute_frame(0)
    # 프레임 1: backbone 종료 이후 D_trigger_delay 반영
    f1_start = f0["T_bb_tick_pub_start"] + mean["D_trigger_delay"]
    f1 = compute_frame(f1_start)

    # 블록 그리기 (2프레임)
    for f in [f0, f1]:
        draw_block(ax, f["T_bb_cb_start"], mean["D_eval_tick_total"], y_levels["evaluator"], "", "orange")
        draw_block(ax, f["T_car_tick_start"], mean["D_car_tick"], y_levels["evaluator"], "tick", "darkgreen")
 
        pink = "#ff3399"
        yellow = "#f7e600"
        cobalt_blue = "#0047AB"
        draw_block(ax, f["T_bb_start"], mean["D_bb_total"], y_levels["Backbone"], "", yellow)
        draw_block(ax, f["T_bb_rx_start"], mean["D_bb_rx"], y_levels["Backbone"], "rx", cobalt_blue)
        # draw_block(ax, f["T_bb_pp_start"], mean["D_bb_pp"], y_levels["Backbone"], "pp", cobalt_blue, 21)
        draw_block(ax, f["T_bb_pp_start"], mean["D_bb_pp"], y_levels["Backbone"], "pp", cobalt_blue, 6)
        draw_block(ax, f["T_bb_net_start"], mean["D_bb_net"], y_levels["Backbone"], "net", cobalt_blue)
        draw_block(ax, f["T_bb_tick_pub_start"], mean["D_bb_tick_pub"], y_levels["Backbone"], "tick pub", cobalt_blue, 0.0, 0.2)
        draw_block(ax, f["T_bb_bb_pub_start"], mean["D_bb_bb_pub"], y_levels["Backbone"], "bb pub", cobalt_blue)
        
        draw_block(ax, f["T_br_start"], mean["D_br_total"], y_levels["branch"], "", yellow)
        draw_block(ax, f["T_br_rx_start"], mean["D_br_rx"], y_levels["branch"], "rx", cobalt_blue)
        draw_block(ax, f["T_br_net_start"], mean["D_br_net"], y_levels["branch"], "net", cobalt_blue)
        draw_block(ax, f["T_br_pid_start"], mean["D_br_pid"], y_levels["branch"], "pid", cobalt_blue)
        draw_block(ax, f["T_br_pub_start"], mean["D_br_pub"], y_levels["branch"], "pub", cobalt_blue)
        
        draw_block(ax, f["T_br_cb_start"], mean["D_eval_ctrl_total"], y_levels["evaluator"], "", "purple")
        draw_block(ax, f["T_car_ctrl_start"], mean["D_car_ctrl"], y_levels["evaluator"], "ctrl", "darkgreen")

    # 프레임 간 delay 점선
    draw_latency(ax, f0["T_bb_cb_end"], y_levels["evaluator"], f0["T_bb_start"], y_levels["Backbone"], "Sensor_delay", mean["D_sensor_delay"])
    draw_latency(ax, f0["T_bb_tick_pub_start"], y_levels["Backbone"], f1["T_bb_cb_start"], y_levels["evaluator"], "Trigger_delay", mean["D_trigger_delay"], 0.2)
    draw_latency(ax, f0["T_bb_bb_pub_start"], y_levels["Backbone"], f0["T_br_start"], y_levels["branch"], "Transfer_delay", mean["D_transfer_delay"])
    draw_latency(ax, f0["T_br_pub_start"], y_levels["branch"], f0["T_br_cb_start"], y_levels["evaluator"], "Control_delay", mean["D_control_delay"], 0.4)
    
    draw_latency(ax, f1["T_bb_cb_end"], y_levels["evaluator"], f1["T_bb_start"], y_levels["Backbone"], "Sensor_delay", mean["D_sensor_delay"])
    draw_latency(ax, f1["T_bb_bb_pub_start"], y_levels["Backbone"], f1["T_br_start"], y_levels["branch"], "Transfer_delay", mean["D_transfer_delay"])
    draw_latency(ax, f1["T_br_pub_start"], y_levels["branch"], f1["T_br_cb_start"], y_levels["evaluator"], "Control_delay", mean["D_control_delay"], 0.4)

    draw_interval(ax, f0["T_bb_start"], y_levels["Backbone"], f1["T_bb_start"], y_levels["Backbone"], "tick", mean["D_tick"], 1.6)
    draw_interval(ax, f0["T_br_pub_start"], y_levels["branch"], f1["T_br_pub_start"], y_levels["branch"], "output", mean["D_output"], -0.4)


    # axis 설정
    ax.set_yticks(list(y_levels.values()))
    ax.set_yticklabels(list(y_levels.keys()))
    ax.set_xlabel("Time (ms)")
    ax.set_title("Distributed Inference Timing Diagram w/ CARLA (2 Frames, Mean Durations)")
    ax.set_xlim(-10, f1["T_br_cb_end"] + 20)
    ax.set_ylim(0.4, 3.9)
    ax.grid(True, axis='x', linestyle='--', alpha=0.3)
    plt.tight_layout()
    plt.show()

def main(args):
    base = args.scenario.strip("/")

    # 자동 경로 설정
    backbone_cb_path = f"{base}_evaluator/evaluator_backbone_cb_timing.csv"
    branch_cb_path   = f"{base}_evaluator/evaluator_branch_cb_timing.csv"
    backbone_path    = f"{base}_backbone/backbone_timing.csv"
    branch_path      = f"{base}_branch/branch_timing.csv"

    # 🔹 CSV 파일 로딩
    df_eval_bb = pd.read_csv(backbone_cb_path)
    df_eval_br = pd.read_csv(branch_cb_path)
    df_bb = pd.read_csv(backbone_path)
    df_br = pd.read_csv(branch_path)

    # 🔹 Step 기준 병합
    merged = pd.merge(df_eval_bb, df_bb, on="step", suffixes=("_bb_cb", "_bb"))
    merged = pd.merge(merged, df_br, on="step", suffixes=("", "_br"))
    merged = pd.merge(merged, df_eval_br, on="step", suffixes=("", "_br_cb"))

    # 🔹 (N+1) 비교용
    merged["T_proc_start_next"] = merged["T_proc_start"].shift(-1)
    merged["T_pub_start_next"] = merged["T_pub_start"].shift(-1)

    # 🔹 Duration 계산
    merged["D_eval_tick_total"] = (merged["T_bb_cb_end"] - merged["T_bb_cb_start"]) * 1000
    merged["D_car_tick"] = (merged["T_car_tick_end"] - merged["T_car_tick_start"]) * 1000
    merged["D_car_tick_call"] = (merged["T_car_tick_start"] - merged["T_bb_cb_start"]) * 1000
    # merged["D_car_tick_return"] = (merged["T_bb_cb_end"] - merged["T_car_tick_end"]) * 1000
    
    merged["D_sensor_delay"] = (merged["T_proc_start_next"] - merged["T_bb_cb_end"]) * 1000
    merged["D_tick"] = (merged["T_proc_start_next"] - merged["T_proc_start"]) * 1000

    merged["D_bb_total"] = (merged["T_bb_pub_end"] - merged["T_proc_start"]) * 1000
    merged["D_bb_rx"] = (merged["T_t_end"] - merged["T_proc_start"]) * 1000 # bb_start ~ rx_end
    merged["D_bb_pp"] = (merged["T_pp_end"] - merged["T_t_end"]) * 1000 
    merged["D_bb_net"] = (merged["T_bb_end"] - merged["T_pp_end"]) * 1000
    merged["D_bb_tick_pub"] = (merged["T_tick_pub_end"] - merged["T_bb_end"]) * 1000 # include tx time
    merged["D_bb_bb_pub"] = (merged["T_bb_pub_end"] - merged["T_tick_pub_end"]) * 1000 # include tx time
    
    # merged["D_backbone_before_tick_pub"] = (merged["T_tick_pub_start"] - merged["T_proc_start"]) * 1000
    # merged["D_backbone_before_bb_pub"] = (merged["T_bb_pub_start"] - merged["T_proc_start"]) * 1000
    # merged["D_backbone_for_pub"] = (merged["T_bb_pub_end"] - merged["T_bb_end"]) * 1000 # include time to make msg

    merged["D_trigger_delay"] = (merged["T_bb_cb_start"] - merged["T_tick_pub_start"]) * 1000
    merged["D_transfer_delay"] = (merged["T_cb_start"] - merged["T_bb_pub_start"]) * 1000

    merged["D_br_total"] = (merged["T_pub_end"] - merged["T_cb_start"]) * 1000
    merged["D_br_rx"] = (merged["T_rx_end"] - merged["T_cb_start"]) * 1000
    merged["D_br_net"] = (merged["T_br_end"] - merged["T_rx_end"]) * 1000
    merged["D_br_pid"] = (merged["T_pid_end"] - merged["T_br_end"]) * 1000
    merged["D_br_pub"] = (merged["T_pub_end"] - merged["T_pid_end"]) * 1000 # include

    merged["D_output"] = (merged["T_pub_start_next"] - merged["T_pub_start"]) * 1000
    merged["D_control_delay"] = (merged["T_br_cb_start"] - merged["T_pub_start"]) * 1000

    merged["D_eval_ctrl_total"] = (merged["T_br_cb_end"] - merged["T_br_cb_start"]) * 1000
    merged["D_car_ctrl"] = (merged["T_car_ctrl_end"] - merged["T_car_ctrl_start"]) * 1000
    merged["D_car_ctrl_call"] = (merged["T_car_ctrl_start"] - merged["T_br_cb_start"]) * 1000
    # merged["D_car_ctrl_return"] = (merged["T_br_cb_end"] - merged["T_car_ctrl_end"]) * 1000

    # 🔹 앞부분 필터링
    if args.skip_initial > 0:
        merged = merged[merged["step"] >= args.skip_initial]
        merged = merged[:-1]

    # 🔹 출력
    metric_list = [
        ("① evaluator tick 실행 시간", "D_eval_tick_total"),
        ("② carla tick 실행 시간", "D_car_tick"),
        ("③ carla tick 호출 시간", "D_car_tick_call"),
        # ("④ carla tick 반환 시간", "D_car_tick_return"),
        
        ("⑤ sensor delay", "D_sensor_delay"),
        
        ("⑥ backbone total 실행 시간", "D_bb_total"),
        ("⑥-1 backbone rx 실행 시간", "D_bb_rx"),
        ("⑥-2 backbone pp 실행 시간", "D_bb_pp"),
        ("⑥-3 backbone model 실행 시간", "D_bb_net"),
        ("⑥-4 backbone tick trigger msg 생성 & publish 실행 시간", "D_bb_tick_pub"),
        ("⑥-5 backbone feature msg 생성 & publish 실행 시간", "D_bb_tick_pub"),
        
        ("⑦ trigger delay", "D_trigger_delay"),
        ("⑧ transfer delay", "D_transfer_delay"),
        
        ("⑨ branch total 실행 시간", "D_br_total"),
        ("⑨-1 branch rx 실행 시간", "D_br_rx"),
        ("⑨-2 branch model 실행 시간", "D_br_net"),
        ("⑨-3 branch pid 실행 시간", "D_br_pid"),
        ("⑨-4 branch msg 생성 & publish  실행 시간", "D_br_pub"),
        
        ("⑩ control delay", "D_control_delay"),
        
        ("⑪ evaluator control 실행 시간", "D_eval_ctrl_total"),
        ("⑫ carla control 실행 시간", "D_car_ctrl"),
        ("⑬ carla control 호출 시간", "D_car_ctrl_call"),
        ("⑭-1 sensor tick", "D_tick"),
        ("⑭-2 output interval", "D_output")

    ]
    print("\n📊 평균 Duration Summary (단위: ms):")
    for label, key in metric_list:
        print(f"{label:30s} ({key}): {summarize(merged[key])}")

    # 🔹 시각화 호출
    duration_mean = merged.mean(numeric_only=True).to_dict()
    draw_diagram_from_mean_durations(duration_mean)

    # 🔹 저장
    if args.save_csv:
        merged.to_csv(args.save_csv, index=False)
        print(f"\n✅ 분석 결과 CSV 저장 완료: {args.save_csv}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Timing Analysis + Diagram")
    parser.add_argument("--scenario", required=True, help="Scenario ID (e.g., 01, 02, ...)")
    parser.add_argument("--skip_initial", type=int, default=20)
    parser.add_argument("--save_csv", type=str)
    args = parser.parse_args()
    main(args)
