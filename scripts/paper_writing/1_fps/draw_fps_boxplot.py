import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import seaborn as sns
import pandas as pd

# 한글 폰트 설정
plt.rcParams['font.family'] = 'Baekmuk Batang'  # 또는 'Malgun Gothic', 'AppleGothic' 등
plt.rcParams['font.weight'] = 'bold'         # 전체 텍스트 볼드 적용
plt.rcParams['axes.unicode_minus'] = False

# Data
data = {
    "Run": [1, 2, 3, 4, 5],
    "단일 보드 실행": [11.8279, 11.6526, 11.7727, 11.7545, 11.7905],
    "이중 보드 분산 실행": [13.0855, 13.1756, 13.0408, 12.9314, 13.1559],
}

# Create DataFrame
df = pd.DataFrame(data)

# Save DataFrame as CSV (optional)
# df.to_csv("fps_experiment_data.csv", index=False)  # 저장 파일명은 필요에 따라 변경 가능

# Melt DataFrame for seaborn
df_melted = df.melt(id_vars="Run", var_name="Setup", value_name="FPS")

# Plot
plt.figure(figsize=(8, 6))
sns.boxplot(x="Setup", y="FPS", data=df_melted, palette="pastel", width=0.5, showfliers=False)
# sns.stripplot(x="Setup", y="FPS", data=df_melted, color="black", size=6, jitter=True)

# plt.title("FPS 비교: 단일 보드 실행 vs 이중 보드 분산 실행")
plt.ylabel("FPS")
plt.xlabel("실행 환경")
plt.grid(True, linestyle="--", alpha=0.5)
plt.tight_layout()

# Show plot
plt.show()
