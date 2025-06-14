#!/usr/bin/env python3

import json
import argparse
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def parse_arguments():
    parser = argparse.ArgumentParser(description="Plot smoothed metrics from metric_info.json with averages.")
    parser.add_argument("input_file", help="Path to the input JSON file (e.g., metric_info.json)")
    parser.add_argument("output_file", help="Path to save the output plot (e.g., metrics_plot_smoothed.png)")
    parser.add_argument("window_size", type=int, help="Window size for smoothing")
    return parser.parse_args()

def load_json_data(file_path):
    with open(file_path, 'r') as f:
        data = json.load(f)
    return data

def extract_metrics_to_dataframe(data):
    # JSON 데이터를 pandas DataFrame으로 변환
    point_numbers = []
    speed_kmh = []
    inference_time_ms = []
    fps = []

    for point in sorted(data.keys(), key=int):  # 포인트 번호를 숫자로 정렬
        if point == "1": # 초기화 값 제거
            continue
        point_numbers.append(int(point))
        speed_kmh.append(data[point]["speed_kmh"])
        inference_time_ms.append(data[point]["inference_time_ms"])
        fps.append(data[point]["fps"])

    df = pd.DataFrame({
        "point_number": point_numbers,
        "speed_kmh": speed_kmh,
        "inference_time_ms": inference_time_ms,
        "fps": fps
    })
    return df

def apply_smoothing(df, window_size=5):
    # 이동 평균으로 데이터 스무딩
    df['speed_kmh_smoothed'] = df['speed_kmh'].rolling(window=window_size, center=True).mean()
    df['inference_time_ms_smoothed'] = df['inference_time_ms'].rolling(window=window_size, center=True).mean()
    df['fps_smoothed'] = df['fps'].rolling(window=window_size, center=True).mean()
    return df

def plot_metrics(df, output_file, window_size):
    # 평균값 계산 (원본 데이터 기준)
    avg_speed_kmh = np.mean(df['speed_kmh'])
    avg_inference_time_ms = np.mean(df['inference_time_ms'])
    avg_fps = np.mean(df['fps'])

    # 그래프 설정 (3개의 subplot)
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # Speed (km/h) 그래프
    ax1.plot(df['point_number'], df['speed_kmh_smoothed'], label="Speed (km/h) Smoothed", color="blue", linewidth=1.5, alpha=0.8)
    ax1.set_ylabel("Speed (km/h)")
    ax1.set_title("Speed per Frame", fontsize=10)
    ax1.grid(True, linestyle='--', alpha=0.3)
    ax1.legend(fontsize=8)

    # Inference Time (ms) 그래프
    ax2.plot(df['point_number'], df['inference_time_ms_smoothed'], label="Inference Time (ms) Smoothed", color="green", linewidth=1.5, alpha=0.8)
    ax2.set_ylabel("Inference Time (ms)")
    ax2.set_title("Inference Time per Frame", fontsize=10)
    ax2.grid(True, linestyle='--', alpha=0.3)
    ax2.legend(fontsize=8)

    # FPS 그래프
    ax3.plot(df['point_number'], df['fps_smoothed'], label="FPS Smoothed", color="red", linewidth=1.5, alpha=0.8)
    ax3.set_xlabel("Frame Number")
    ax3.set_ylabel("FPS")
    ax3.set_title("FPS per Frame", fontsize=10)
    ax3.grid(True, linestyle='--', alpha=0.3)
    ax3.legend(fontsize=8)

    # 평균값 및 Window Size 텍스트 추가
    text = (f"Average Speed: {avg_speed_kmh:.2f} km/h\n"
            f"Average Inference Time: {avg_inference_time_ms:.2f} ms\n"
            f"Average FPS: {avg_fps:.2f}\n"
            f"Window Size: {window_size}")
    plt.figtext(0.1, -0.02, text, fontsize=12, ha="left", bbox={"facecolor":"white", "alpha":0.5, "pad":5})

    # 레이아웃 조정 및 저장
    plt.tight_layout(rect=[0, 0.05, 1, 1])  # 텍스트를 위한 공간 확보
    plt.savefig(output_file, dpi=300, bbox_inches="tight")
    plt.close()

def main():
    # 인자 파싱
    args = parse_arguments()

    # JSON 데이터 로드
    data = load_json_data(args.input_file)

    # DataFrame으로 변환 및 메트릭 추출
    df = extract_metrics_to_dataframe(data)

    # 스무딩 적용
    df = apply_smoothing(df, window_size=args.window_size)

    # 그래프 그리기
    plot_metrics(df, args.output_file, args.window_size)
    print(f"The smoothed graph is saved as {args.output_file}.")

if __name__ == "__main__":
    main()