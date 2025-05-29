#!/usr/bin/env python
import os
import json
import datetime
import pathlib
import time
import cv2
import math
from collections import OrderedDict
import csv

import torch
import numpy as np
from PIL import Image as PILImage
from torchvision import transforms as T

from TCP.config import GlobalConfig
from team_code.planner import RoutePlanner
from scipy.optimize import fsolve

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from carla_msgs.msg import CarlaEgoVehicleControl
from tf_transformations import euler_from_quaternion
from TCP.model_branch import TCPBranch
from tcp_msgs.msg import TCPBackboneOutput

SAVE_PATH = os.environ.get('SAVE_PATH', None)
PLANNER_TYPE = os.environ.get('PLANNER_TYPE', None)

EARTH_RADIUS_EQUA = 6378137.0

class TCPBranchNode(Node):
    def __init__(self, ckpt_path, save_path, debug_mode):
        super().__init__('tcp_branch_node')

        self.ckpt_path = ckpt_path
        # self.save_path = save_path if save_path else '/root/shared_dir/B2D_Demo/B2D_tcp/Bench2Drive/eval_v1/'
        self.debug_mode = int(debug_mode)
        self.step = -1

        self.config = GlobalConfig()
        self.net = TCPBranch(self.config)

        ckpt = torch.load(self.ckpt_path, map_location="cuda", weights_only=True)
        ckpt = ckpt["state_dict"]
        new_state_dict = OrderedDict()
        for key, value in ckpt.items():
            new_key = key.replace("model.", "")
            new_state_dict[new_key] = value
        self.net.load_state_dict(new_state_dict, strict=False)
        self.net.cuda()
        self.net.eval()
        
        self.pid_metadata = {} 
        
        self.subscription = self.create_subscription(TCPBackboneOutput, '/tcp/backbone_output', self.backbone_callback, QoSProfile(depth=1))

        self.control_pub = self.create_publisher(CarlaEgoVehicleControl, '/tcp/vehicle_control_cmd', QoSProfile(depth=1))

        if (self.debug_mode > 0) and SAVE_PATH:
            now = datetime.datetime.now()
            string = f"tcp_agent_{now.strftime('%m_%d_%H_%M_%S')}"
            self.save_path = pathlib.Path(SAVE_PATH) / string
            self.save_path.mkdir(parents=True, exist_ok=True)
            # timing
            (self.save_path / 'meta_branch').mkdir()
            self.log_file = self.save_path / 'branch_timing.csv'
            with open(self.log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'step', 'T_rx_start', 'T_rx_end', 'T_br_start', 'T_br_end', 'T_pid_start', 'T_pid_end', 'T_pub_start', 'T_pub_end', 'T_log_start', 'T_log_end'
                ])

    def backbone_callback(self, msg):
        if self.debug_mode > 1:
            self.get_logger().warning(f"[TCPBranchNode] Process step called, step={self.step}")

        # timing
        T_rx_start = time.time()
        # time) ROS2 시계 기준 시간 기록
        ros_time_ns = self.get_clock().now().nanoseconds
        
        self.step = msg.step

        # Tensor 복원
        cnn_feature = torch.tensor(msg.cnn_feature, dtype=torch.float32).view(1, 512, 8, 29).cuda()
        measurement_feature = torch.tensor(msg.measurement_feature, dtype=torch.float32).view(1, 128).cuda()
        traj_hidden_state = torch.tensor(msg.traj_hidden_state, dtype=torch.float32).view(1, self.config.pred_len, 256).cuda()

        speed = msg.speed
        gt_velocity = torch.tensor([[msg.gt_velocity]], dtype=torch.float32).cuda()
        target_point = torch.tensor([msg.target_point], dtype=torch.float32).cuda()
        command = torch.tensor(msg.command, dtype=torch.float32).view(1, 6).cuda()
        state = torch.cat([gt_velocity / 12.0, target_point, command], dim=1)
        pred_wp = torch.tensor(msg.pred_wp, dtype=torch.float32).view(1, self.config.pred_len, 2).cuda()
        
        T_rx_end = time.time()
        
        # Inference
        # self.get_logger().warning(f"- backbone_callback(): branch net()")
        T_br_start = time.time()
        # break net!
        pred = self.net(cnn_feature, measurement_feature, traj_hidden_state)
        T_br_end = time.time()

        # STEP 3: PID 계산
        # self.get_logger().warning(f"- backbone_callback(): control_pid()")
        T_pid_start = time.time()
        steer_ctrl, throttle_ctrl, brake_ctrl, metadata = self.net.process_action(pred, int(torch.argmax(command)), gt_velocity, target_point)
        steer_traj, throttle_traj, brake_traj, metadata_traj = self.net.control_pid(pred_wp, gt_velocity, target_point)
        T_pid_end = time.time()

        # self.get_logger().info(f"[DEBUG] gt_velocity: {gt_velocity}")
        # self.get_logger().info(f"[DEBUG] target_point: {target_point}")
        # self.get_logger().info(f"[DEBUG] pred_wp.cpu().numpy(): {pred['pred_wp'].cpu().numpy()}")
        # self.get_logger().info(f"[DEBUG] steer_ctrl: {steer_ctrl}, throttle_ctrl: {throttle_ctrl}, brake_ctrl: {brake_ctrl}")
        # self.get_logger().info(f"[DEBUG] steer_traj: {steer_traj}, throttle_traj: {throttle_traj}, brake_traj: {brake_traj}")

        # STEP 4: 제어 명령 생성 및 publish
        # self.get_logger().warning(f"- backbone_callback(): generate control")
        control = CarlaEgoVehicleControl()
        if PLANNER_TYPE == 'only_traj':
            self.pid_metadata = metadata_traj
            self.pid_metadata['agent'] = 'only_traj'
            control.steer = np.clip(float(steer_traj), -1, 1)
            control.throttle = np.clip(float(throttle_traj), 0, 0.75)
            control.brake = np.clip(float(brake_traj), 0, 1)
        elif PLANNER_TYPE == 'only_ctrl':
            self.pid_metadata = metadata
            self.pid_metadata['agent'] = 'only_ctrl'
            control.steer = np.clip(float(steer_ctrl), -1, 1)
            control.throttle = np.clip(float(throttle_ctrl), 0, 0.75)
            control.brake = np.clip(float(brake_ctrl), 0, 1)
        elif PLANNER_TYPE == 'merge_ctrl_traj':
            self.pid_metadata = metadata_traj
            self.pid_metadata['agent'] = 'merge_ctrl_traj'
            alpha = 0.5
            control.steer = np.clip(alpha * steer_traj + (1 - alpha) * steer_ctrl, -1, 1)
            control.throttle = np.clip(alpha * throttle_traj + (1 - alpha) * throttle_ctrl, 0, 0.75)
            control.brake = max(np.clip(float(brake_ctrl), 0, 1), np.clip(float(brake_traj), 0, 1))
            # self.get_logger().warning(f"[DEBUG] steer={control.steer}") 
            # self.get_logger().warning(f"[DEBUG] throttle={control.throttle}") 
            # self.get_logger().warning(f"[DEBUG] brake={control.brake}") 

        # self.get_logger().warning(f"- backbone_callback(): clipping")
        if abs(control.steer) > 0.07:   ## In turning
            speed_threshold = 1.0   ## Avoid stuck during turning
        else:
            speed_threshold = 1.5   ## Avoid pass stop/red light/collision
        if float(msg.speed) > speed_threshold:
            max_throttle = 0.05
        else:
            max_throttle = 0.5
        control.throttle = np.clip(control.throttle, a_min=0.0, a_max=max_throttle)
        # self.get_logger().warning(f"[DEBUG] throttle={control.throttle}")

        if control.brake > 0:
            control.brake = 1.0
        if control.brake > 0.5:
            control.throttle = 0.0
        # self.get_logger().warning(f"[DEBUG] brake={control.brake}")

        # self.get_logger().warning(f"- backbone_callback(): control_pub()")
        # self.get_logger().warning(f"[TCPBranchNode] Published control: steer={control.steer:.3f}, throttle={control.throttle:.3f}, brake={control.brake:.3f}")

        T_pub_start = time.time()
        self.control_pub.publish(control)
        T_pub_end = time.time()

        # STEP 5: 결과 저장
        if self.debug_mode > 0:
            T_log_start = time.time()
            self.pid_metadata = {
                'step': self.step,
                'cnn_feature': cnn_feature.view(-1).tolist()[:10],
                'measurement_feature': measurement_feature.view(-1).tolist()[:10],
                'traj_hidden_state': traj_hidden_state.view(-1).tolist()[:10],
                
                'ros_time_ns': ros_time_ns,
                'branch_inference_ms': (T_br_end -T_br_start) * 1000,
                'pid_calc_ms': (T_pid_end - T_pid_start) * 1000,
                'publish_ms': (T_pub_end - T_pub_start) * 1000,
                'total_process_step_ms': (T_pub_end - T_rx_start) * 1000,

                'steer_ctrl': float(steer_ctrl),
                'steer_traj': float(steer_traj),
                'throttle_ctrl': float(throttle_ctrl),
                'throttle_traj': float(throttle_traj),
                'brake_ctrl': float(brake_ctrl),
                'brake_traj': float(brake_traj) if isinstance(brake_traj, float) else float(np.array(brake_traj).item()),  # 또는 bool(brake_traj)
                
                'steer': control.steer,
                'throttle': control.throttle,
                'brake': control.brake,
            }
            
            if SAVE_PATH and self.step % 1 == 0:
                self.save()

            # timing log 저장
            if self.log_file:
                with open(self.log_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        self.step,
                        T_rx_start, T_rx_end,
                        T_br_start, T_br_end,
                        T_pid_start, T_pid_end,
                        T_pub_start, T_pub_end,
                        T_log_start, time.time()
                    ])
            if self.debug_mode > 1:
                self.get_logger().warning(f"Process step finished, step={self.step}")

    def save(self):
        frame = self.step
        with open(self.save_path / 'meta_branch' / (f'%04d.json' % frame), 'w') as outfile:
            json.dump(self.pid_metadata, outfile, indent=4)

    def destroy(self):
        del self.net
        torch.cuda.empty_cache()
        self.get_logger().info("TCP Agent Node destroyed")
        
        #jw) todo: 최종 결과 출력

def main():
    import argparse
    parser = argparse.ArgumentParser(description='TCP Agent Node for CARLA')
    parser.add_argument('--ckpt-path', required=True, help='Path to model checkpoint')
    parser.add_argument('--save-path', default=None, help='Path to save debug outputs')
    parser.add_argument('--debug-mode', type=int, default=0, help='Level of debug mode')
    # parser.add_argument('--img-input', default='raw', help='Type for input camera image')
    # parser.add_argument('--img-k', type=float, default=1.0, help='Input camera image resolution ratio')
    args = parser.parse_args()

    rclpy.init()
    node = TCPBranchNode(args.ckpt_path, args.save_path, args.debug_mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TCP Branch Node")
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
