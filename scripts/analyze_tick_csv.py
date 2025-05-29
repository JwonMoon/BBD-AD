import argparse
import pandas as pd
import matplotlib.pyplot as plt

def analyze_tick_csv(csv_path):
    df = pd.read_csv(csv_path)
    
    if 'tick_duration_sec' not in df.columns:
        raise ValueError("CSV 파일에 'tick_duration_sec' 컬럼이 없습니다.")
    
    tick_ms = df['tick_duration_sec'] * 1000  # 초 → 밀리초 변환

    print("\n📊 Tick Duration 통계 요약")
    print(f"- 총 측정 수: {len(tick_ms)}개")
    print(f"- 평균 tick 시간: {tick_ms.mean():.2f} ms")
    print(f"- 최소: {tick_ms.min():.2f} ms")
    print(f"- 최대: {tick_ms.max():.2f} ms")
    print(f"- 중앙값 (50%): {tick_ms.median():.2f} ms")
    print(f"- 상위 25% 구간 (75%): {tick_ms.quantile(0.75):.2f} ms")

    high_delay_count = (tick_ms > 100).sum()
    print(f"\n⚠️ 지연이 100ms 이상인 tick 수: {high_delay_count}건")
    if high_delay_count == 0:
        print("→ 현재 설정에서는 tick 지연이 거의 없음을 의미합니다.")

    # 저장용 이름 추출
    base_name = csv_path.split('/')[-1].replace('.csv', '')

    # 히스토그램 저장
    plt.figure()
    plt.hist(tick_ms, bins=30, edgecolor='black')
    plt.title("Tick Duration Histogram (ms)")
    plt.xlabel("Tick Duration (ms)")
    plt.ylabel("Frequency")
    plt.grid(True)
    hist_path = f"{base_name}_histogram.png"
    plt.savefig(hist_path)
    print(f"\n🖼️ 히스토그램 저장됨: {hist_path}")

    # Tick 변화 그래프 저장
    plt.figure()
    plt.plot(tick_ms.values)
    plt.title("Tick Duration over Time")
    plt.xlabel("Tick Index")
    plt.ylabel("Tick Duration (ms)")
    plt.grid(True)
    line_path = f"{base_name}_tick_plot.png"
    plt.savefig(line_path)
    print(f"🖼️ Tick 변화 그래프 저장됨: {line_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CARLA Tick Duration Analyzer")
    parser.add_argument("csv_path", help="tick_*.csv 파일 경로")
    args = parser.parse_args()

    analyze_tick_csv(args.csv_path)

