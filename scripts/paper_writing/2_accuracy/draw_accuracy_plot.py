import matplotlib.pyplot as plt
import pandas as pd
import matplotlib.font_manager as fm
from sklearn.metrics import mean_absolute_error

# 한글 폰트 설정
plt.rcParams['font.family'] = 'Baekmuk Batang'  # 또는 'Malgun Gothic', 'AppleGothic' 등
plt.rcParams['font.weight'] = 'bold'         # 전체 텍스트 볼드 적용
plt.rcParams['axes.unicode_minus'] = False

# ▶️ 실험 결과 (단일 보드 vs 분산 보드)
steer_single = [
    -0.0009495925918707189, -0.02020625263302141, -0.01712745974815668, 0.020736148289392485,
    -0.039099173511385525, 0.05486600683158362, -0.008044601239690287, 0.03063906349986393,
    0.02255932013091378, -0.009342173613215233, 0.09160550484938972, 0.00803282179506246,
    0.012077422638474033, -0.10320090249351721, 0.06664381549106628, -0.01423200863520769,
    0.09021082276721265, 0.025503566162756654, 0.01618683355791293, -0.06645859582129808
]
steer_dual = steer_single.copy()  # 동일한 경우

# ▶️ 절대 차이 계산
abs_diff = [abs(a - b) for a, b in zip(steer_single, steer_dual)]

# ▶️ 테이블 생성
df = pd.DataFrame({
    '번호': list(range(1, 21)),
    'Steer (단일)': steer_single,
    'Steer (분산)': steer_dual,
    '절대차이': abs_diff,
    'Throttle (단일)': [0.5 if i not in [10, 13, 16] else 0.05 for i in range(20)],
    'Throttle (분산)': [0.5 if i not in [10, 13, 16] else 0.05 for i in range(20)],
    'Brake (단일)': [0.0] * 20,
    'Brake (분산)': [0.0] * 20
})

# ▶️ MAE 계산
mae = mean_absolute_error(steer_single, steer_dual)
print(f"\n▶️ Steer 값의 평균 절대 오차 (MAE): {mae:.10f}")

# ▶️ 요약 테이블 출력
print("\n📋 Steer 출력 비교 테이블:")
print(df.to_string(index=False))

# ▶️ 그래프 1: Steer 값 비교
plt.figure(figsize=(10, 4))
# plt.plot(df['번호'], df['Steer (단일)'], label='단일 보드', marker='o')
# plt.plot(df['번호'], df['Steer (분산)'], label='분산 보드', marker='x', linestyle='dashed')
plt.plot(df['번호'], df['Throttle (단일)'], label='단일 보드', marker='o')
plt.plot(df['번호'], df['Throttle (분산)'], label='분산 보드', marker='x', linestyle='dashed')
# plt.plot(df['번호'], df['Brake (단일)'], label='단일 보드', marker='o')
# plt.plot(df['번호'], df['Brake (분산)'], label='분산 보드', marker='x', linestyle='dashed')

# plt.title("단일 보드 vs 분산 보드 Steer 출력 비교")
plt.xlabel("입력 샘플 번호")
# plt.ylabel("Steer 값")
plt.ylabel("Throttle 값")
# plt.ylabel("Brake 값")
plt.xticks(range(1, 21, 2))  # 1부터 20까지, 2 간격으로 눈금 표시
plt.legend()
plt.grid(True)
plt.tight_layout()
# plt.savefig("accuracy_steer.png")
plt.savefig("accuracy_throttle.png")
# plt.savefig("accuracy_brake.png")
plt.show()

# ▶️ 그래프 2: 절대값 차이
# plt.figure(figsize=(10, 3))
# plt.bar(df['번호'], df['절대차이'])
# plt.title("Steer 출력 절대 차이")
# plt.xlabel("입력 프레임 번호")
# plt.ylabel("|Steer 차이|")
# plt.grid(True)
# plt.tight_layout()
# plt.savefig("steer_절대차이.png")
# plt.show()