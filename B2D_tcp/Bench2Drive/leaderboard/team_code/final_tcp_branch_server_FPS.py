#!/usr/bin/env python3
import socket
import json
import datetime
import pathlib
import time
import torch
import numpy as np
import csv
from collections import OrderedDict
from TCP.model_branch_ver2 import TCPBranch
from TCP.config import GlobalConfig
import os
import select  # 비블로킹 모드를 위해 추가
import msgpack

SAVE_PATH = os.environ.get('SAVE_PATH', None)
PLANNER_TYPE = os.environ.get('PLANNER_TYPE', 'only_ctrl')

def restore_tensor(msg, config):
    # j_traj, measurement_feature, cnn_feature 복원
    j_traj = torch.from_numpy(
        np.frombuffer(msg['j_traj'], dtype=np.float32).reshape(msg['j_traj_shape']).copy()
    ).cuda()  # shape: (1, 256)

    measurement_feature = torch.from_numpy(
        np.frombuffer(msg['measurement_feature'], dtype=np.float32).reshape(msg['measurement_shape']).copy()
    ).cuda()  # shape: (1, 128)

    cnn_feature = torch.from_numpy(
        np.frombuffer(msg['cnn_feature'], dtype=np.float32).reshape(msg['cnn_feature_shape']).copy()
    ).cuda()  # shape: (1, 1000, 8, 28)

    gt_velocity = torch.tensor([[msg['gt_velocity']]], dtype=torch.float32).cuda()  # shape: (1, 1)

    target = torch.tensor(msg['target_point'], dtype=torch.float32).view(1, 2).cuda()  # shape: (1, 2)

    command = torch.tensor(msg['command'], dtype=torch.float32).view(1, 6).cuda()  # shape: (1, 6)

    state = torch.cat([gt_velocity / 12.0, target, command], dim=1)  # shape: (1, 9)

    return j_traj, measurement_feature, cnn_feature, gt_velocity, target, command, state

def start_branch_server(ckpt_path, debug_mode, orin1_ip='192.168.20.1', port_recv=9999, port_send=9998):
    debug_mode = int(debug_mode)

    # 모델 초기화
    config = GlobalConfig()
    model = TCPBranch(config)
    ckpt = torch.load(ckpt_path, map_location="cuda", weights_only=True)  # weights_only=True 추가
    model.load_state_dict({k.replace("model.", ""): v for k, v in ckpt["state_dict"].items()}, strict=False)
    model.cuda().eval()

    # TCP 수신 (Backbone → Branch)
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # [추가] 수신 버퍼 크기 설정 (예: 64KB)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
    recv_sock.bind(('0.0.0.0', port_recv))
    recv_sock.listen(1)
    print(f"[BranchNode] Listening for backbone at {port_recv}")
    
    # [추가] 비블로킹 모드 설정
    recv_sock.setblocking(False)
    conn_recv, _ = None, None
    while not conn_recv:
        r, _, _ = select.select([recv_sock], [], [], 1.0)  # 1초 타임아웃
        if r:
            conn_recv, addr = recv_sock.accept()
            conn_recv.setblocking(False)  # 연결된 소켓도 비블로킹 모드로 설정
            print(f"[BranchNode] Accepted connection from {addr}")

    # TCP 송신 (Branch → Relay)
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # [추가] 송신 버퍼 크기 설정 (예: 64KB)
    send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
    send_sock.connect((orin1_ip, port_send))
    print(f"[BranchNode] Connected to relay at {orin1_ip}:{port_send}")

    # 로그 설정
    step = 0
    if debug_mode > 0 and SAVE_PATH:
        now = datetime.datetime.now()
        save_path = pathlib.Path(SAVE_PATH) / f"tcp_branch_{now.strftime('%m_%d_%H_%M_%S')}"
        save_path.mkdir(parents=True, exist_ok=True)
        # meta_path = save_path / "meta_branch"
        # meta_path.mkdir()
        log_file = save_path / "branch_timing.csv"
        with open(log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'step', 'T_cb_start',
                # 'T_rx_start', 
                # 'T_recv_start', 'T_recv_end',
                # 'T_restore_start', 'T_restore_end',
                # 'T_rx_end',
                # 'T_br_start',
                # 'T_br_branch_traj', 'T_br_pred_wp',  # 추가
                # 'T_br_init_att', 'T_br_join_ctrl', 'T_br_branch_ctrl',
                # 'T_br_action_head', 'T_br_future_feature_action',
                # 'T_br_end',
                # 'T_pid_start', 'T_pid_end',
                'T_tx_start', 'T_tx_end',
                # 'T_br_log_start', 'T_br_log_end'
            ])
    else:
        save_path = None
        log_file = None

    while True:
        try:
            # [변경] 비블로킹 방식으로 메시지 길이 수신
            r, _, _ = select.select([conn_recv], [], [], 0.01)  # 10ms 타임아웃
            if not r:
                continue  # 데이터가 없으면 루프 재시작
            T_cb_start = time.time()
            # 1. Receive
            T_rx_start = time.time()
            # print(f"[BranchNode] Before recv(4): {time.time() - T_rx_start}")
            msg_len_data = conn_recv.recv(4)
            # print(f"[BranchNode] After recv(4): {time.time() - T_rx_start}")
            
            if not msg_len_data:
                print("[BranchNode] Connection closed by backbone")
                break
            msg_len = int.from_bytes(msg_len_data, byteorder='big')

            data = b''
            T_recv_start = time.time()
            while len(data) < msg_len:
                r, _, _ = select.select([conn_recv], [], [], 0.01)  # 10ms 타임아웃
                if not r:
                    continue
                chunk = conn_recv.recv(msg_len - len(data))
                if not chunk:
                    print("[BranchNode] Connection closed during data receive")
                    break
                data += chunk
            # print(f"[BranchNode] Step {step}: Data size = {len(data)} bytes")
            T_recv_end = time.time()

            T_restore_start = time.time()
            msg = msgpack.unpackb(data, raw=False)  # msgpack 역직렬화
            # msg = msgpack.unpackb(data, raw=False)  # msgpack 역직렬화
            step = msg['step']
            # cnn, meas, traj, gt_vel, target, cmd, pred_wp, state = restore_tensor(msg, config) #ver1
            j_traj, measurement_feature, cnn_feature, gt_velocity, target, command, state = restore_tensor(msg, config) #ver2
            T_restore_end = time.time()
            T_rx_end = time.time()

            # 2. Inference
            T_br_start = time.time()
            # pred = model(cnn, meas, traj) #ver1
            pred = model(j_traj, measurement_feature, cnn_feature, target) #ver2
            T_br_end = time.time()

            # 3. PID
            T_pid_start = time.time()
            steer_ctrl, throttle_ctrl, brake_ctrl, meta_ctrl = model.process_action(pred, int(torch.argmax(command)), gt_velocity, target)
            steer_traj, throttle_traj, brake_traj, meta_traj = model.control_pid(pred['pred_wp'], gt_velocity, target)
            T_pid_end = time.time()

            # 3-1. PLANNER_TYPE별 제어 선택
            if PLANNER_TYPE == 'only_ctrl':
                control_steer = float(np.clip(steer_ctrl, -1, 1))
                control_throttle = float(np.clip(throttle_ctrl, 0, 0.75))
                control_brake = np.clip(float(brake_ctrl), 0, 1)
                # control_brake = float(brake_ctrl > 0.5)
                control_agent = 'only_ctrl'
                # meta = meta_ctrl
            elif PLANNER_TYPE == 'only_traj':
                control_steer = float(np.clip(steer_traj, -1, 1))
                control_throttle = float(np.clip(throttle_traj, 0, 0.75))
                control_brake = np.clip(float(brake_traj), 0, 1)
                # control_brake = float(brake_traj > 0.5)
                control_agent = 'only_traj'
                # meta = meta_traj
            elif PLANNER_TYPE == 'merge_ctrl_traj':
                alpha = 0.5
                control_steer = float(np.clip(alpha * steer_traj + (1 - alpha) * steer_ctrl, -1, 1))
                control_throttle = float(np.clip(alpha * throttle_traj + (1 - alpha) * throttle_ctrl, 0, 0.75))
                control_brake = max(np.clip(float(brake_ctrl), 0, 1), np.clip(float(brake_traj), 0, 1))
                # control_brake = float(max(brake_ctrl, brake_traj > 0.5))
                control_agent = 'merge_ctrl_traj'
                # meta = meta_traj
            else:
                raise ValueError(f"Unknown PLANNER_TYPE: {PLANNER_TYPE}")

            # self.get_logger().warning(f"- process_step(): clipping")
            if abs(control_steer) > 0.07:   ## In turning
                speed_threshold = 1.0   ## Avoid stuck during turning
            else:
                speed_threshold = 1.5   ## Avoid pass stop/red light/collision
            if float(msg['speed']) > speed_threshold:
                max_throttle = 0.05
            else:
                max_throttle = 0.5
            control_throttle = np.clip(control_throttle, a_min=0.0, a_max=max_throttle)

            if control_brake > 0:
                control_brake = 1.0
            if control_brake > 0.5:
                control_throttle = 0.0


            # 4. Send control
            control = {
                'step': step,
                'steer': control_steer,
                'throttle': control_throttle,
                'brake': control_brake
            }
            print(f"[PUB CONTROL] step={step}, steer={control_steer}, throttle={control_throttle}, brake={control_brake}") #debug

            payload = msgpack.packb(control, use_bin_type=True)  # MessagePack 직렬화
            T_tx_start = time.time()
            send_sock.sendall(len(payload).to_bytes(4, byteorder='big') + payload)
            T_tx_end = time.time()

            # 5. Logging
            if debug_mode > 0 and log_file:
                T_log_start = time.time()
                # meta.update({
                #     'step': step,
                #     'agent': control_agent,
                #     'cnn_feature': cnn.view(-1).tolist()[:10],
                #     'measurement_feature': meas.view(-1).tolist()[:10],
                #     'traj_hidden_state': traj.view(-1).tolist()[:10],
                #     'gt_velocity': float(gt_vel.item()),
                #     'target_point': target.view(-1).tolist(),
                #     'command': cmd.view(-1).tolist(),
                #     'pred_wp': pred_wp.view(-1).tolist()[:10],
                #     'steer': control_steer,
                #     'throttle': control_throttle,
                #     'brake': control_brake
                # })

                # with open(save_path / 'meta_branch' / f'{step:04d}.json', 'w') as f:
                #     json.dump(meta, f, indent=4)

                with open(log_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        step, T_cb_start,
                        # T_rx_start, 
                        # T_recv_start, T_recv_end,
                        # T_restore_start, T_restore_end,
                        # T_rx_end,
                        # T_br_start,
                        # pred['timing']['branch_traj'],  # 추가
                        # pred['timing']['pred_wp'],      # 추가
                        # pred['timing']['init_att'],
                        # pred['timing']['join_ctrl'],
                        # pred['timing']['branch_ctrl'],
                        # pred['timing']['action_head'],
                        # pred['timing']['future_feature_action'],
                        # T_br_end,
                        # T_pid_start, T_pid_end,
                        T_tx_start, T_tx_end,
                        # T_log_start, time.time()
                    ])
        except Exception as e:
            print(f"[BranchNode] Error at step {step}: {e}")
            break

    conn_recv.close()
    recv_sock.close()
    send_sock.close()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--ckpt-path', required=True)
    parser.add_argument('--debug-mode', default=0)
    args = parser.parse_args()

    start_branch_server(args.ckpt_path, args.debug_mode)