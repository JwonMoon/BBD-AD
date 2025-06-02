import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
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
    eval_df = pd.read_csv(args.eval)
    agent_df = pd.read_csv(args.agent)

    # 🔹 step 기준 병합
    merged = pd.merge(eval_df, agent_df, on="step", suffixes=("_eval", "_agent"))

    # 🔹 N+1 tick 시작 시간
    merged["T_start_next"] = merged["T_start"].shift(-1)

    # 🔹 시간 계산 (단위: ms)
    merged["T_tick"] = (merged["T_tick_end"] - merged["T_tick_start"]) * 1000
    merged["T_model"] = (merged["T_pub_end"] - merged["T_start"]) * 1000
    merged["T_log"] = (merged["T_log_end"] - merged["T_log_start"]) * 1000
    merged["net_delay_to_tick"] = (merged["T_tick_start"] - merged["T_pub_end"]) * 1000
    merged["net_delay_to_model"] = (merged["T_start_next"] - merged["T_tick_end"]) * 1000

    # 초기 step 제외
    merged = merged[skip_initial_n:-1]

    # 🔹 요약 출력
    metrics = [
        ("① Tick 실행 시간", "T_tick"),
        ("② Sensor delay", "net_delay_to_model"),
        ("③ model inference", "T_model"),
        ("④ Net delay to tick trigger", "net_delay_to_tick"),
    ]

    print("\n📊 평균값 요약:")
    for name, col in metrics:
        print(f"{name} ({col}): {summarize(merged[col])}")

    # 🔹 저장
    if args.save_csv:
        merged.to_csv(args.save_csv, index=False)
        print(f"\n✅ 분석 결과 저장됨: {args.save_csv}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Single Borad Inference Timing Analyzer")
    parser.add_argument("--eval", type=str, required=True, help="backbone_timing.csv")
    parser.add_argument("--agent", type=str, required=True, help="branch_timing.csv")
    parser.add_argument("--save_csv", type=str, help="Optional output merged CSV path")
    args = parser.parse_args()
    main(args)