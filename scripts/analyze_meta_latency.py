import os
import json
import argparse
import pandas as pd
import matplotlib.pyplot as plt

def load_logs(json_dir):
    logs = []
    for fname in sorted(os.listdir(json_dir)):
        if fname.endswith('.json'):
            with open(os.path.join(json_dir, fname), 'r') as f:
                data = json.load(f)
                logs.append(data)
    return pd.DataFrame(logs)

def analyze(df, json_dir):
    time_cols = [col for col in df.columns if col.endswith('_ms')]
    print("\n[Time Latency Summary]")
    print(df[time_cols].describe())

    # 바 차트 경로 설정
    bar_path = os.path.join(json_dir, "module_latency_bar.png")
    hist_path = os.path.join(json_dir, "total_process_hist.png")

    # 평균 바 차트
    df_mean = df[time_cols].mean().sort_values(ascending=False)
    df_mean.plot(kind='bar', title='Average Latency per Module (ms)')
    plt.ylabel('Time (ms)')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(bar_path)
    print(f"Bar chart saved to: {bar_path}")

    # 전체 latency 히스토그램
    plt.figure()
    df['total_process_step_ms'].hist(bins=30)
    plt.title('Total process_step() Latency Distribution')
    plt.xlabel('Total process_step() time (ms)')
    plt.ylabel('Frequency')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(hist_path)
    print(f"Histogram saved to: {hist_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze TCPAgentNode latency logs")
    parser.add_argument("json_dir", help="Path to directory containing JSON logs (e.g., ./eval_v1/.../meta)")
    args = parser.parse_args()

    df = load_logs(args.json_dir)
    analyze(df, args.json_dir)

