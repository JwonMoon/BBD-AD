#!/usr/bin/env python3

import json
import argparse
import matplotlib.pyplot as plt
import numpy as np

def parse_arguments():
    parser = argparse.ArgumentParser(description="Plot metrics from metric_info.json and calculate averages.")
    parser.add_argument("input_file", help="Path to the input JSON file (e.g., metric_info.json)")
    parser.add_argument("output_file", help="Path to save the output plot (e.g., metrics_plot.png)")
    return parser.parse_args()

def load_json_data(file_path):
    with open(file_path, 'r') as f:
        data = json.load(f)
    return data

def extract_metrics(data):
    point_numbers = []
    speed_kmh = []
    inference_time_ms = []
    fps = []

    # JSON 데이터에서 각 메트릭 추출
    for point in sorted(data.keys(), key=int):  # 포인트 번호를 숫자로 정렬
        if point == "1": # 초기화 값 제거
            continue
        point_numbers.append(int(point))
        speed_kmh.append(data[point]["speed_kmh"])
        inference_time_ms.append(data[point]["inference_time_ms"])
        fps.append(data[point]["fps"])

    return point_numbers, speed_kmh, inference_time_ms, fps

def plot_metrics(point_numbers, speed_kmh, inference_time_ms, fps, output_file):
    # 평균값 계산
    avg_speed_kmh = np.mean(speed_kmh)
    avg_inference_time_ms = np.mean(inference_time_ms)
    avg_fps = np.mean(fps)

    # 그래프 설정 (3개의 subplot)
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    # Speed (km/h) 그래프
    ax1.plot(point_numbers, speed_kmh, label="Speed (km/h)", color="blue")
    ax1.set_ylabel("Speed (km/h)")
    ax1.set_title("Speed per Frame", fontsize=10)
    ax1.grid(True)
    ax1.legend()

    # Inference Time (ms) 그래프
    ax2.plot(point_numbers, inference_time_ms, label="Inference Time (ms)", color="green")
    ax2.set_ylabel("Inference Time (ms)")
    ax2.set_title("Inference Time per Frame", fontsize=10)
    ax2.grid(True)
    ax2.legend()

    # FPS 그래프
    ax3.plot(point_numbers, fps, label="FPS", color="red")
    ax3.set_xlabel("Frame Number")
    ax3.set_ylabel("FPS")
    ax3.set_title("FPS per Frame", fontsize=10)
    ax3.grid(True)
    ax3.legend()

    # 평균값 텍스트 추가
    text = (f"Average Speed: {avg_speed_kmh:.2f} km/h\n"
            f"Average Inference Time: {avg_inference_time_ms:.2f} ms\n"
            f"Average FPS: {avg_fps:.2f}")
    plt.figtext(0.1, 0.01, text, fontsize=12, ha="left")

    # 레이아웃 조정 및 저장
    plt.tight_layout(rect=[0, 0.05, 1, 1])  # 텍스트를 위한 공간 확보
    plt.savefig(output_file, dpi=300, bbox_inches="tight")
    plt.close()

def main():
    # 인자 파싱
    args = parse_arguments()

    # JSON 데이터 로드
    data = load_json_data(args.input_file)

    # 메트릭 추출
    point_numbers, speed_kmh, inference_time_ms, fps = extract_metrics(data)

    # 그래프 그리기
    plot_metrics(point_numbers, speed_kmh, inference_time_ms, fps, args.output_file)
    print(f"Plot saved to {args.output_file}")

if __name__ == "__main__":
    main()