# TCP backbone to branch output message

# Flattened CNN feature: (512 x 8 x 29) = 118784
float32[] cnn_feature

# Measurement feature: 128
float32[] measurement_feature

# Traj GRU hidden states: (4 x 256) = 1024
float32[] traj_hidden_state

# Vehicle speed (original, m/s)
float32 speed

# Normalized speed (for model input)
float32 gt_velocity

# Target point (2차원)
float32[2] target_point

# One-hot encoded driving command (길이 6)
float32[6] command

# 예측된 경로 (pred_len x 2 = 4 x 2 = 8)
float32[8] pred_wp

# 현재 스텝 번호 (디버깅용)
int32 step