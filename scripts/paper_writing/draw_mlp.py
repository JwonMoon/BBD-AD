import matplotlib.pyplot as plt
import numpy as np

def draw_mlp(layer_sizes):
    fig, ax = plt.subplots()
    v_spacing = 1
    h_spacing = 5
    radius = 0.2

    # 노드 좌표 계산
    node_positions = {}
    for i, layer_size in enumerate(layer_sizes):
        layer_x = i * h_spacing
        layer_ys = np.linspace(-layer_size/2, layer_size/2, layer_size)
        for j, y in enumerate(layer_ys):
            node_positions[(i, j)] = (layer_x, y)

    # 노드 그리기
    for pos in node_positions.values():
        circle = plt.Circle(pos, radius, color='lightblue', ec='k')
        ax.add_artist(circle)

    # 연결선 그리기
    for i in range(len(layer_sizes)-1):
        for j in range(layer_sizes[i]):
            for k in range(layer_sizes[i+1]):
                x1, y1 = node_positions[(i, j)]
                x2, y2 = node_positions[(i+1, k)]
                ax.plot([x1, x2], [y1, y2], 'gray', linewidth=0.5)

    ax.set_aspect('equal')
    ax.axis('off')
    plt.show()

# 예시: 입력 8, 은닉 3층 (16, 16, 12), 출력 4
draw_mlp([8, 16, 16, 12, 4])
