import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

# 한글 폰트 설정
plt.rcParams['font.family'] = 'Baekmuk Batang'
plt.rcParams['font.weight'] = 'bold'
plt.rcParams['axes.unicode_minus'] = False

# FPS 데이터
data = {
    "Hz": [30, 30, 20, 20, 10, 10],
    "실행 환경": ["단일 보드", "이중 보드"] * 3,
    "평균 FPS": [11.76, 13.08, 11.76, 13.14, 10.01, 10.11],
}

df = pd.DataFrame(data)

# 시각화
plt.figure(figsize=(8, 6))
barplot = sns.barplot(
    x="Hz",
    y="평균 FPS",
    hue="실행 환경",
    data=df,
    palette="pastel",
    capsize=0.1,
    # err_kws={'linewidth': 1.5}
    errorbar=None  # 표준편차 표시 안 함
)

# 막대 위에 수치 표시
for container in barplot.containers:
    for bar in container:
        height = bar.get_height()
        x = bar.get_x() + bar.get_width() / 2
        plt.text(x, height + 0.1, f"{height:.2f}", ha='center', va='bottom', fontsize=12, fontweight='bold')

# plt.title("Hz별 프레임 처리 속도(FPS) 비교", fontsize=14, weight='bold')
plt.xlabel("센서 주기 (Hz)", fontsize=14)
plt.ylabel("FPS", fontsize=14)
plt.grid(True, linestyle="--", alpha=0.5)
plt.legend(title="실행 환경")
plt.tight_layout()
plt.show()