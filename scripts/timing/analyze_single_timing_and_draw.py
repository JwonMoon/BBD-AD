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
#     if label == "":
#         return
#     elif label == "moved pub":
#         r_label = "pub"
#         ax.text(start + 20, level, f"{r_label}\n{duration:.2f}", ha="center", va="center", fontsize=9)
#     else:
#         ax.text(start + duration / 2, level, f"{label}\n{duration:.2f}", ha="center", va="center", fontsize=9)

def draw_block(ax, start, duration, level, label, color, threshold=5.0):
    ax.add_patch(patches.Rectangle((start, level - 0.2), duration, 0.4, color=color, alpha=0.6))
    if not label:
        return
    if duration < threshold:
        x_text = start + duration + max(1.5*duration , 2)
        y_text = level + max(0.22*duration , 0.2)
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

def draw_latency(ax, x0, y0, x1, y1, label, value):
    ax.plot([x0, x1], [y0, y1], "k--", lw=1)
    xm, ym = (x0 + x1) / 2, (y0 + y1) / 2
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
    y_levels = {"evaluator": 2, "TCP": 1}

    def compute_frame(prev_end): # start 지점을 기준으로 duration 더해서 계산
        f = {}
        f["T_tick_start"] = prev_end
        f["T_car_ctrl_start"] = f["T_tick_start"] + mean["D_car_ctrl_call"]
        f["T_car_tick_start"] = f["T_tick_start"] + mean["D_car_tick_call"]
        f["T_car_tick_end"] = f["T_car_tick_start"] + mean["D_car_tick"]
        f["T_tick_end"] = f["T_tick_start"] + mean["D_eval_tick_total"]

        f["T_tcp_start"] = f["T_tick_end"] + mean["D_sensor_delay"]
        f["T_tcp_rx_start"] = f["T_tcp_start"]
        f["T_tcp_pp_start"] = f["T_tcp_rx_start"] + mean["D_tcp_rx"]
        f["T_tcp_net_start"] = f["T_tcp_pp_start"] + mean["D_tcp_pp"]
        f["T_tcp_pid_start"] = f["T_tcp_net_start"] + mean["D_tcp_net"]
        f["T_tcp_pub_start"] = f["T_tcp_pid_start"] + mean["D_tcp_pid"]
        f["T_tcp_pub_end"] = f["T_tcp_pub_start"] + mean["D_tcp_pub"]
        return f

    # 프레임 0
    f0 = compute_frame(0)
    # 프레임 1: backbone 종료 이후 D_trigger_delay 반영
    f1_start = f0["T_tcp_pub_start"] + mean["D_control_delay"]
    f1 = compute_frame(f1_start)

    # 블록 그리기 (2프레임)
    for f in [f0, f1]:
        draw_block(ax, f["T_tick_start"], mean["D_eval_tick_total"], y_levels["evaluator"], "", "orange")
        draw_block(ax, f["T_car_ctrl_start"], mean["D_car_ctrl"], y_levels["evaluator"], "ctrl", "darkgreen")
        draw_block(ax, f["T_car_tick_start"], mean["D_car_tick"], y_levels["evaluator"], "tick", "darkgreen")
        
        pink = "#ff3399"
        yellow = "#f7e600"
        cobalt_blue = "#0047AB"
        draw_block(ax, f["T_tcp_start"], mean["D_tcp_total"], y_levels["TCP"], "", yellow)
        draw_block(ax, f["T_tcp_rx_start"], mean["D_tcp_rx"], y_levels["TCP"], "rx", cobalt_blue)
        draw_block(ax, f["T_tcp_pp_start"], mean["D_tcp_pp"], y_levels["TCP"], "pp", cobalt_blue)
        draw_block(ax, f["T_tcp_net_start"], mean["D_tcp_net"], y_levels["TCP"], "net", cobalt_blue)
        draw_block(ax, f["T_tcp_pid_start"], mean["D_tcp_pid"], y_levels["TCP"], "pid", cobalt_blue)
        draw_block(ax, f["T_tcp_pub_start"], mean["D_tcp_pub"], y_levels["TCP"], "pub", cobalt_blue)

    # 프레임 간 delay 점선
    draw_latency(ax, f0["T_tick_end"], y_levels["evaluator"], f0["T_tcp_start"], y_levels["TCP"], "Sensor_delay", mean["D_sensor_delay"])
    draw_latency(ax, f0["T_tcp_pub_start"], y_levels["TCP"], f1["T_tick_start"], y_levels["evaluator"], "Control_delay", mean["D_control_delay"])
    
    draw_latency(ax, f1["T_tick_end"], y_levels["evaluator"], f1["T_tcp_start"], y_levels["TCP"], "Sensor_delay", mean["D_sensor_delay"])
    draw_interval(ax, f0["T_tcp_start"], y_levels["TCP"], f1["T_tcp_start"], y_levels["TCP"], "tick", mean["D_tick"], 0.45)
    draw_interval(ax, f0["T_tcp_pub_start"], y_levels["TCP"], f1["T_tcp_pub_start"], y_levels["TCP"], "output", mean["D_output"], 0.7)

    # axis 설정
    ax.set_yticks(list(y_levels.values()))
    ax.set_yticklabels(list(y_levels.keys()))
    ax.set_xlabel("Time (ms)")
    ax.set_title("Monolithic Inference Timing Diagram w/ CARLA (2 Frames, Mean Durations)")
    ax.set_xlim(-10, f1["T_tcp_pub_end"] + 20)
    ax.set_ylim(0.5, 2.5)
    ax.grid(True, axis='x', linestyle='--', alpha=0.3)
    plt.tight_layout()
    plt.show()

def main(args):
    base = args.scenario.strip("/")

    # 자동 경로 설정
    evaluator_path   = f"{base}_evaluator/evaluator_tcp_timing.csv"
    tcp_path    = f"{base}_tcp/tcp_timing.csv"

    # 🔹 CSV 파일 로딩
    df_eval = pd.read_csv(evaluator_path)
    df_tcp = pd.read_csv(tcp_path)

    # 🔹 Step 기준 병합
    merged = pd.merge(df_eval, df_tcp, on="step", suffixes=("_eval", "_tcp"))

    # 🔹 (N+1) 비교용
    merged["T_proc_start_next"] = merged["T_proc_start"].shift(-1)
    merged["T_pub_start_next"] = merged["T_pub_start"].shift(-1)

    # 🔹 Duration 계산
    merged["D_eval_tick_total"] = (merged["T_tick_end"] - merged["T_tick_start"]) * 1000
    merged["D_car_ctrl"] = (merged["T_car_ctrl_end"] - merged["T_car_ctrl_start"]) * 1000
    merged["D_car_ctrl_call"] = (merged["T_car_ctrl_start"] - merged["T_tick_start"]) * 1000 # tick_start ~ carla_control_start
    merged["D_car_ctrl_return"] = (merged["T_tick_end"] - merged["T_car_ctrl_end"]) * 1000
    merged["D_car_tick"] = (merged["T_car_tick_end"] - merged["T_car_tick_start"]) * 1000
    merged["D_car_tick_call"] = (merged["T_car_tick_start"] - merged["T_tick_start"]) * 1000
    merged["D_car_tick_return"] = (merged["T_tick_end"] - merged["T_car_tick_end"]) * 1000
    
    merged["D_sensor_delay"] = (merged["T_proc_start_next"] - merged["T_tick_end"]) * 1000
    merged["D_tick"] = (merged["T_proc_start_next"] - merged["T_proc_start"]) * 1000

    merged["D_tcp_total"] = (merged["T_pub_end"] - merged["T_proc_start"]) * 1000
    merged["D_tcp_rx"] = (merged["T_t_end"] - merged["T_proc_start"]) * 1000 # tcp_start ~ rx_end
    merged["D_tcp_pp"] = (merged["T_pp_end"] - merged["T_t_end"]) * 1000 
    merged["D_tcp_net"] = (merged["T_net_end"] - merged["T_pp_end"]) * 1000
    merged["D_tcp_pid"] = (merged["T_pid_end"] - merged["T_net_end"]) * 1000
    merged["D_tcp_pub"] = (merged["T_pub_end"] - merged["T_pid_end"]) * 1000 # include tx time
    
    merged["D_output"] = (merged["T_pub_start_next"] - merged["T_pub_start"]) * 1000
    merged["D_control_delay"] = (merged["T_tick_start"] - merged["T_pub_start"]) * 1000

    # 🔹 앞부분 필터링
    if args.skip_initial > 0:
        merged = merged[merged["step"] >= args.skip_initial]
        merged = merged[:-1]

    # 🔹 출력
    metric_list = [
        ("① evaluator total 실행 시간", "D_eval_tick_total"),
        ("② carla control 실행 시간", "D_car_ctrl"),
        ("③ carla control 호출 시간", "D_car_ctrl_call"),
        ("④ carla control 반환 시간", "D_car_ctrl_return"),
        ("⑤ carla tick 실행 시간", "D_car_tick"),
        ("⑥ carla tick 호출 시간", "D_car_tick_call"),
        ("⑦ carla tick 반환 시간", "D_car_tick_return"),
        ("⑧ sensor delay", "D_sensor_delay"),
        ("⑨ tcp total 실행 시간", "D_tcp_total"),
        ("⑨-1 tcp rx 실행 시간", "D_tcp_rx"),
        ("⑨-2 tcp pp 실행 시간", "D_tcp_pp"),
        ("⑨-3 tcp model 실행 시간", "D_tcp_net"),
        ("⑨-4 tcp pid 실행 시간", "D_tcp_pid"),
        ("⑨-5 tcp msg 생성 & publish 실행 시간", "D_tcp_pub"),
        ("⑩ control delay", "D_control_delay"),
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
