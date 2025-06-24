import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# 한글 폰트 설정
plt.rcParams['font.family'] = 'Baekmuk Batang'  # 또는 'Malgun Gothic', 'AppleGothic' 등
plt.rcParams['font.weight'] = 'bold'         # 전체 텍스트 볼드 적용
plt.rcParams['axes.unicode_minus'] = False

# 데이터
hz_list = [10, 20, 30]
frames = [50, 100, 200]

data = {
    10: {"single": [45, 95, 195], "dual": [46, 96, 196]},
    20: {"single": [22, 54, 109], "dual": [26, 59, 129]},
    30: {"single": [13, 32, 68], "dual": [15, 38, 81]},
}

for hz in hz_list:
    single = data[hz]["single"]
    dual = data[hz]["dual"]

    # 손실률 = 100 - 수신률
    single_loss = [100 - (r / f * 100) for r, f in zip(single, frames)]
    dual_loss = [100 - (r / f * 100) for r, f in zip(dual, frames)]

    plt.figure()
    plt.plot(frames, single_loss, marker='o', label='단일 보드')
    plt.plot(frames, dual_loss, marker='o', label='분산 보드')
    # plt.title(f'{hz}Hz 입력 주기에서의 Frame 손실률 비교')
    plt.xlabel('총 발행 프레임 수')
    plt.ylabel('손실률 (%)')
    plt.ylim(0, 100)
    plt.xticks(frames)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(f'frame_drop_{hz}hz.png')
    plt.show()
