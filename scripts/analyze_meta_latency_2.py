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

def analyze(df, out_prefix=""):
    # 시간 항목들 필터링
    pd.set_option('display.max_columns', None)

    time_cols = [col for col in df.columns if col.endswith('_ms')]
    print("\n[Time Latency Summary]")
    print(df[time_cols].describe())
    print("\n[Average Latency per Module]")
    print(df[time_cols].mean().to_string())

    # 평균 바 차트
    df_mean = df[time_cols].mean().sort_values(ascending=False)
    plt.figure()
    df_mean.plot(kind='bar', title='Average Latency per Module (ms)')
    plt.ylabel('Time (ms)')
    plt.grid(True)
    plt.tight_layout()
    bar_path = os.path.join(out_prefix, "module_latency_bar.png")
    plt.savefig(bar_path)
    print(f"Bar chart saved to: {bar_path}")

    # 전체 latency 히스토그램
    if 'total_process_step_ms' in df.columns:
        plt.figure()
        df['total_process_step_ms'].hist(bins=30)
        plt.title('Total process_step() Latency Distribution')
        plt.xlabel('Total process_step() time (ms)')
        plt.ylabel('Frequency')
        plt.grid(True)
        plt.tight_layout()
        hist_path = os.path.join(out_prefix, "total_process_hist.png")
        plt.savefig(hist_path)
        print(f"Histogram saved to: {hist_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze TCPAgentNode latency logs")
    parser.add_argument("json_dir", help="Path to directory containing JSON logs (e.g., ./eval_v1/.../meta)")
    parser.add_argument("--out-prefix", default="", help="Optional prefix for output image files")
    args = parser.parse_args()

    df = load_logs(args.json_dir)
    analyze(df, out_prefix=args.out_prefix or args.json_dir)

