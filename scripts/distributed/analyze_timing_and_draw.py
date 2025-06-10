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

def draw_block(ax, start, duration, level, label, color):
    ax.add_patch(patches.Rectangle((start, level - 0.2), duration, 0.4, color=color, alpha=0.6))
    ax.text(start + duration / 2, level, f"{label}\n{duration:.2f}", ha="center", va="center", fontsize=9)

def draw_latency(ax, x0, y0, x1, y1, label, value):
    ax.plot([x0, x1], [y0, y1], "k--", lw=1)
    xm, ym = (x0 + x1) / 2, (y0 + y1) / 2
    ax.text(xm, ym + 0.2, f"{label}: {value:.2f}", ha="center", fontsize=8, color="black")

def draw_diagram_from_mean_durations(mean):
    fig, ax = plt.subplots(figsize=(18, 4))
    y_levels = {"evaluator": 3, "Backbone": 2, "branch": 1}

    def compute_frame(prev_end):
        f = {}
        f["T_bb_cb_start"] = prev_end
        f["T_bb_cb_end"] = f["T_bb_cb_start"] + mean["eval_tick_duration"]
        f["T_proc_start"] = f["T_bb_cb_end"] + mean["sensor_delay"]
        f["T_bb_end"] = f["T_proc_start"] + mean["backbone_exec"]
        f["T_bb_end_tick_pub"] = f["T_proc_start"] + mean["backbone_exec_tick_pub"]
        f["T_bb_end_bb_pub"] = f["T_proc_start"] + mean["backbone_exec_bb_pub"]
        f["T_cb_start"] = f["T_bb_end_bb_pub"] + mean["transfer_delay"]
        f["T_pub_end"] = f["T_cb_start"] + mean["branch_exec"]
        f["T_br_cb_start"] = f["T_pub_end"] + mean["control_delay"]
        f["T_br_cb_end"] = f["T_br_cb_start"] + mean["evaluator_ctrl_exec"]
        return f

    # 프레임 0
    f0 = compute_frame(0)
    # 프레임 1: backbone 종료 이후 trigger_delay 반영
    f1_start = f0["T_bb_end_tick_pub"] + mean["trigger_delay"]
    f1 = compute_frame(f1_start)

    # 블록 그리기 (2프레임)
    for f in [f0, f1]:
        draw_block(ax, f["T_bb_cb_start"], mean["eval_tick_duration"], y_levels["evaluator"], "eval_tick", "darkgreen")
        draw_block(ax, f["T_proc_start"], mean["backbone_exec"], y_levels["Backbone"], "backbone", "navy")
        draw_block(ax, f["T_bb_end"], mean["backbone_exec_for_pub"], y_levels["Backbone"], "publish", "orange")
        draw_block(ax, f["T_cb_start"], mean["branch_exec"], y_levels["branch"], "branch", "deepskyblue")
        draw_block(ax, f["T_br_cb_start"], mean["evaluator_ctrl_exec"], y_levels["evaluator"], "control", "purple")

    # 프레임 간 delay 점선
    draw_latency(ax, f0["T_bb_cb_end"], y_levels["evaluator"], f0["T_proc_start"], y_levels["Backbone"], "Sensor_delay", mean["sensor_delay"])
    draw_latency(ax, f0["T_bb_end_tick_pub"], y_levels["Backbone"], f1["T_bb_cb_start"], y_levels["evaluator"], "Trigger_delay", mean["trigger_delay"])
    draw_latency(ax, f0["T_bb_end_bb_pub"] + 2, y_levels["Backbone"], f0["T_cb_start"], y_levels["branch"], "Transfer_delay", mean["transfer_delay"])
    draw_latency(ax, f0["T_pub_end"], y_levels["branch"], f0["T_br_cb_start"], y_levels["evaluator"], "Control_delay", mean["control_delay"])
    
    draw_latency(ax, f1["T_bb_cb_end"], y_levels["evaluator"], f1["T_proc_start"], y_levels["Backbone"], "Sensor_delay", mean["sensor_delay"])
    draw_latency(ax, f1["T_bb_end_bb_pub"] + 2, y_levels["Backbone"], f1["T_cb_start"], y_levels["branch"], "Transfer_delay", mean["transfer_delay"])
    draw_latency(ax, f1["T_pub_end"], y_levels["branch"], f1["T_br_cb_start"], y_levels["evaluator"], "Control_delay", mean["control_delay"])

    # axis 설정
    ax.set_yticks(list(y_levels.values()))
    ax.set_yticklabels(list(y_levels.keys()))
    ax.set_xlabel("Time (ms)")
    ax.set_title("Distributed Inference Timing Diagram (2 Frames, Mean Durations)")
    ax.set_xlim(-10, f1["T_br_cb_end"] + 100)
    ax.set_ylim(0.5, 3.8)
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

    # 🔹 Duration 계산
    merged["eval_tick_duration"] = (merged["T_bb_cb_end"] - merged["T_bb_cb_start"]) * 1000
    merged["carla_tick_duration"] = (merged["T_car_tick_end"] - merged["T_car_tick_start"]) * 1000
    merged["carla_tick_call"] = (merged["T_car_tick_start"] - merged["T_bb_cb_start"]) * 1000
    merged["carla_tick_return"] = (merged["T_bb_cb_end"] - merged["T_car_tick_end"]) * 1000
    merged["sensor_delay"] = (merged["T_proc_start_next"] - merged["T_bb_cb_end"]) * 1000
    merged["backbone_exec"] = (merged["T_bb_end"] - merged["T_proc_start"]) * 1000
    merged["backbone_exec_tick_pub"] = (merged["T_tick_pub_start"] - merged["T_proc_start"]) * 1000
    merged["backbone_exec_bb_pub"] = (merged["T_bb_pub_start"] - merged["T_proc_start"]) * 1000
    merged["backbone_exec_for_pub"] = (merged["T_bb_pub_end"] - merged["T_bb_end"]) * 1000
    merged["backbone_exec_total"] = (merged["backbone_exec"] + merged["backbone_exec_for_pub"]) 
    merged["trigger_delay"] = (merged["T_bb_cb_start"] - merged["T_tick_pub_start"]) * 1000
    merged["transfer_delay"] = (merged["T_cb_start"] - merged["T_bb_pub_start"]) * 1000
    merged["branch_exec"] = (merged["T_pub_end"] - merged["T_cb_start"]) * 1000
    merged["control_delay"] = (merged["T_br_cb_start"] - merged["T_pub_end"]) * 1000
    merged["evaluator_ctrl_exec"] = (merged["T_br_cb_end"] - merged["T_br_cb_start"]) * 1000
    merged["carla_ctrl_exec"] = (merged["T_car_ctrl_end"] - merged["T_car_ctrl_start"]) * 1000
    merged["carla_ctrl_call"] = (merged["T_car_ctrl_start"] - merged["T_br_cb_start"]) * 1000
    merged["carla_ctrl_return"] = (merged["T_br_cb_end"] - merged["T_car_ctrl_end"]) * 1000

    # 🔹 앞부분 필터링
    if args.skip_initial > 0:
        merged = merged[merged["step"] >= args.skip_initial]
        merged = merged[:-1]

    # 🔹 출력
    metric_list = [
        ("① evaluator tick 실행 시간", "eval_tick_duration"),
        ("② carla tick 실행 시간", "carla_tick_duration"),
        ("③ carla tick 호출 시간", "carla_tick_call"),
        ("④ carla tick 반환 시간", "carla_tick_return"),
        ("⑤ sensor delay", "sensor_delay"),
        ("⑥ backbone 실행 시간", "backbone_exec_total"),
        ("⑦ trigger delay", "trigger_delay"),
        ("⑧ transfer delay", "transfer_delay"),
        ("⑨ branch 실행 시간", "branch_exec"),
        ("⑩ control delay", "control_delay"),
        ("⑪ evaluator control 실행 시간", "evaluator_ctrl_exec"),
        ("⑫ carla control 실행 시간", "carla_ctrl_exec"),
        ("⑬ carla control 호출 시간", "carla_ctrl_call"),
        ("⑭ carla control 반환 시간", "carla_ctrl_return"),
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
    # parser.add_argument("--backbone_cb", required=True)
    # parser.add_argument("--branch_cb", required=True)
    # parser.add_argument("--backbone", required=True)
    # parser.add_argument("--branch", required=True)
    parser.add_argument("--scenario", required=True, help="Scenario ID (e.g., 01, 02, ...)")
    parser.add_argument("--skip_initial", type=int, default=20)
    parser.add_argument("--save_csv", type=str)
    args = parser.parse_args()
    main(args)
