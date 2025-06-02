import pandas as pd
import argparse

skip_initial_n=20

def summarize(col):
    return {
        "mean": round(col.mean(), 3),
        "std": round(col.std(), 3),
        "min": round(col.min(), 3),
        "max": round(col.max(), 3),
        "p95": round(col.quantile(0.95), 3),
        "p99": round(col.quantile(0.99), 3),
    }

def main(args):
    # 🔹 CSV 파일 로딩
    bb_cb_df = pd.read_csv(args.backbone_cb)
    br_cb_df = pd.read_csv(args.branch_cb)
    bb_df = pd.read_csv(args.backbone)
    br_df = pd.read_csv(args.branch)

    # 🔹 Step 기준 병합
    merged = pd.merge(bb_cb_df, bb_df, on="step", suffixes=("_bb_cb", "_bb"))
    merged = pd.merge(merged, br_df, on="step", suffixes=("", "_br"))
    merged = pd.merge(merged, br_cb_df, on="step", suffixes=("", "_br_cb"))

    # (N+1)의 값들을 사용하여 시간 차 계산
    merged["T_t_start_next"] = merged["T_t_start"].shift(-1)
    # merged["T_bb_cb_start_next"] = merged["T_bb_cb_start"].shift(-1)
    
    # 🔹 타이밍 계산 (ms)
    merged["①_tick_cb_time"] = (merged["T_bb_cb_end"] - merged["T_bb_cb_start"]) * 1000
    merged["②_sensor_delay"] = (merged["T_t_start_next"] - merged["T_bb_cb_end"]) * 1000
    merged["③_backbone_time"] = (merged["T_tx_end"] - merged["T_t_start"]) * 1000
    merged["④_net_delay_to_branch"] = (merged["T_rx_start"] - merged["T_tx_end"]) * 1000
    merged["⑤_branch_time"] = (merged["T_pub_end"] - merged["T_rx_start"]) * 1000
    merged["⑥_net_delay_to_apply"] = (merged["T_br_cb_start"] - merged["T_pub_end"]) * 1000
    merged["⑦_apply_ctrl_time"] = (merged["T_br_cb_end"] - merged["T_br_cb_start"]) * 1000
    merged["⑧_net_delay_to_tick_trigger"] = (merged["T_bb_cb_start"] - merged["T_tx_end"]) * 1000
    
    # 분석을 위한 앞쪽 step 제거 및 마지막 행 제거 (shift 영향)
    merged = merged[skip_initial_n:-1]
    
    # merged = merged['step'] >= skip_initial_n

    # 🔹 요약 출력
    metrics = [
        ("① Tick 실행 시간", "①_tick_cb_time"),
        ("② Sensor delay", "②_sensor_delay"),
        ("③ Backbone 실행", "③_backbone_time"),
        ("④ Net delay to branch", "④_net_delay_to_branch"),
        ("⑤ Branch 실행", "⑤_branch_time"),
        ("⑥ Net delay to apply control", "⑥_net_delay_to_apply"),
        ("⑦ Apply control 시간", "⑦_apply_ctrl_time"),
        ("⑧ Net delay to tick trigger", "⑧_net_delay_to_tick_trigger"),
    ]

    print("\n📊 평균값 요약:")
    for name, col in metrics:
        print(f"{name} ({col}): {summarize(merged[col])}")

    # 🔹 저장
    if args.save_csv:
        merged.to_csv(args.save_csv, index=False)
        print(f"\n✅ 분석 결과 저장됨: {args.save_csv}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Distributed Inference Timing Analyzer")
    parser.add_argument("--backbone_cb", type=str, required=True, help="evaluator_backbone_cb_timing.csv")
    parser.add_argument("--branch_cb", type=str, required=True, help="evaluator_branch_cb_timing.csv")
    parser.add_argument("--backbone", type=str, required=True, help="backbone_timing.csv")
    parser.add_argument("--branch", type=str, required=True, help="branch_timing.csv")
    parser.add_argument("--save_csv", type=str, help="Optional output merged CSV path")
    args = parser.parse_args()
    main(args)
