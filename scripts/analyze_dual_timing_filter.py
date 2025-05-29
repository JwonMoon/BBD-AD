import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import argparse

def summarize(df, col):
    return {
        "mean": round(df[col].mean(), 3),
        "std": round(df[col].std(), 3),
        "min": round(df[col].min(), 3),
        "max": round(df[col].max(), 3),
        "p95": round(df[col].quantile(0.95), 3),
        "p99": round(df[col].quantile(0.99), 3),
    }

def main(backbone_path, branch_path, skip_initial_n=20):
    backbone_df = pd.read_csv(backbone_path)
    branch_df = pd.read_csv(branch_path)

    # 시간 계산 (ms)
    backbone_df["T_t"] = (backbone_df["T_t_end"] - backbone_df["T_t_start"]) * 1000
    backbone_df["T_pp"] = (backbone_df["T_p_end"] - backbone_df["T_p_start"]) * 1000
    backbone_df["T_bb"] = (backbone_df["T_b_end"] - backbone_df["T_b_start"]) * 1000
    backbone_df["T_pub"] = (backbone_df["T_tx_end"] - backbone_df["T_tx_start"]) * 1000
    backbone_df["T_log"] = (backbone_df["T_log_end"] - backbone_df["T_log_start"]) * 1000
    backbone_df["bb_total_ms"] = (backbone_df["T_tx_end"] - backbone_df["T_t_start"]) * 1000

    branch_df["T_rx"] = (branch_df["T_rx_end"] - branch_df["T_rx_start"]) * 1000
    branch_df["T_br"] = (branch_df["T_br_end"] - branch_df["T_br_start"]) * 1000
    branch_df["T_pid"] = (branch_df["T_pid_end"] - branch_df["T_pid_start"]) * 1000
    branch_df["T_pub"] = (branch_df["T_pub_end"] - branch_df["T_pub_start"]) * 1000
    branch_df["T_log"] = (branch_df["T_log_end"] - branch_df["T_log_start"]) * 1000
    branch_df["br_total_ms"] = (branch_df["T_pub_end"] - branch_df["T_rx_start"]) * 1000
    branch_df["network_latency_ms"] = (branch_df["T_rx_start"] - backbone_df["T_tx_end"]) * 1000

    # 초기 step 제외
    backbone_df = backbone_df[backbone_df["step"] >= skip_initial_n]
    branch_df = branch_df[branch_df["step"] >= skip_initial_n]

    # 🔸 Idle Gap: 다음 Backbone 시작 - 현재 Branch publish 완료
    idle_gaps = backbone_df["T_t_start"].shift(-1) - branch_df["T_pub_end"]
    idle_gaps_ms = idle_gaps[:-1] * 1000  # 마지막 row는 비교 불가하므로 제거

    print("✅ Backbone Timing Summary (after skipping initial steps):")
    for k in ["T_t", "T_pp", "T_bb", "T_pub", "T_log"]:
        print(f"{k} (ms): {summarize(backbone_df, k)}")

    print("\n✅ Branch Timing Summary (after skipping initial steps):")
    for k in ["T_rx", "T_br", "T_pid", "T_pub", "T_log"]:
        print(f"{k} (ms): {summarize(branch_df, k)}")

    print("\n🔸 Aggregated:")
    print("Backbone Total (ms):", summarize(backbone_df, "bb_total_ms"))
    print("Branch Total (ms):", summarize(branch_df, "br_total_ms"))
    print("Network Latency (ms):", summarize(branch_df, "network_latency_ms"))

    print("\n🕒 Idle Gap (between branch end and next backbone start)(ms):")
    print(summarize(pd.DataFrame({'idle_ms': idle_gaps_ms}), 'idle_ms'))
    
    # 시각화
    plt.figure(figsize=(10, 6))
    sns.lineplot(data=backbone_df, x="step", y="bb_total_ms", label="Backbone Total")
    sns.lineplot(data=branch_df, x="step", y="br_total_ms", label="Branch Total")
    sns.lineplot(data=branch_df, x="step", y="network_latency_ms", label="Network Latency")
    plt.plot(branch_df["step"][:-1], idle_gaps_ms, label="Idle Gap", linestyle='--', marker='o')
    plt.xlabel("Step")
    plt.ylabel("Time (ms)")
    plt.title(f"Timing Breakdown Over Steps (skipping first {skip_initial_n} steps)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("timing_breakdown_with_idle.png")
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--backbone", type=str, required=True, help="Path to backbone_timing.csv")
    parser.add_argument("--branch", type=str, required=True, help="Path to branch_timing.csv")
    parser.add_argument("--skip", type=int, default=20, help="Number of initial steps to skip")
    args = parser.parse_args()
    main(args.backbone, args.branch, args.skip)