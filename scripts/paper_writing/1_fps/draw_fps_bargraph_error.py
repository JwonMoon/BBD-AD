import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np  

# 한글 폰트 설정
plt.rcParams['font.family'] = 'Baekmuk Batang'
plt.rcParams['font.weight'] = 'bold'
plt.rcParams['axes.unicode_minus'] = False

# FPS 데이터
data = {
    "Hz": [30, 30, 30, 20, 20, 20, 10, 10, 10],
    "실행 환경": ["(a)단일 보드", "(b)분산 컨테이너", "(c)분산 보드"] * 3,
    "평균 FPS": [11.76, 14.036, 13.504, 11.757, 14.056, 13.714, 10.006, 10.018, 10.02],
    "표준편차": [0.066, 0.032, 0.068, 0.061, 0.044, 0.15, 0.004, 0.005, 0.007], 
}
df = pd.DataFrame(data)

# Plot
plt.figure(figsize=(8, 6))
palette = sns.color_palette("pastel")
barplot = sns.barplot(
    x="Hz",
    y="평균 FPS",
    hue="실행 환경",
    data=df,
    palette=palette,
    errorbar=None
)

# df 순서대로 막대 매핑 (df 행 수만큼만)
for bar, row in zip(barplot.patches[:len(df)], df.itertuples()):
    height = bar.get_height()
    x = bar.get_x() + bar.get_width() / 2
    std = row.표준편차
    # 에러바 추가
    plt.errorbar(x, height, yerr=std, fmt='none', ecolor='black', capsize=5, linewidth=1.5)
    # 수치 표시
    plt.text(x, height + std + 0.05, f"{height:.2f}", ha='center', va='bottom', fontsize=12, fontweight='bold')

plt.xlabel("센서 주기 (Hz)", fontsize=14)
plt.ylabel("FPS", fontsize=14)
plt.grid(True, linestyle="--", alpha=0.5)
plt.legend(title="실행 환경")
plt.tight_layout()
plt.show()