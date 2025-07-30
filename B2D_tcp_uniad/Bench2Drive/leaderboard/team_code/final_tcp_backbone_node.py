#!/usr/bin/env python

import os
import json
import datetime
import pathlib
import time
import cv2
import math
from collections import OrderedDict

import torch
import numpy as np
from PIL import Image as PILImage
from torchvision import transforms as T

# from TCP.model import TCP
from TCP.model_backbone_ver2 import TCPBackbone
from TCP.config import GlobalConfig
from team_code.planner import RoutePlanner
from scipy.optimize import fsolve

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image, NavSatFix, Imu, CompressedImage
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaRoute, CarlaEgoVehicleStatus, CarlaGnssRoute
from tf_transformations import euler_from_quaternion
from bbd_msgs.msg import BBDBackboneOutput, TickTrigger
import csv
import threading, queue
import socket
import msgpack

SAVE_PATH = os.environ.get('SAVE_PATH', None)

EARTH_RADIUS_EQUA = 6378137.0

class TCPBackboneNode(Node):
    def __init__(self, ckpt_path, save_path, debug_mode, img_input, img_k):
        super().__init__('tcp_backbone_node')

        self.ckpt_path = ckpt_path
        # self.save_path = save_path if save_path else '/root/shared_dir/BBD-AD/B2D_tcp/Bench2Drive/eval_v1/'
        self.debug_mode = debug_mode
        self.img_input = img_input if img_input else 'raw'
        self.img_k = float(img_k) if img_k else 1.0
        self.step = 0
        self.initialized = False
        self.try_proc_num = 0

        self.config = GlobalConfig()
        self.net = TCPBackbone(self.config)

        ckpt = torch.load(self.ckpt_path, map_location="cuda", weights_only=True)
        ckpt = ckpt["state_dict"]
        new_state_dict = OrderedDict()
        for key, value in ckpt.items():
            new_key = key.replace("model.", "")
            new_state_dict[new_key] = value
        self.net.load_state_dict(new_state_dict, strict=False)
        self.net.cuda()
        self.net.eval()

        self._im_transform = T.Compose([ # cpu 연산
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        self.lat_ref = 0.0
        self.lon_ref = 0.0
        self._route_planner = None

        self.rgb_front = None
        self.rgb_front_left = None
        self.rgb_front_right = None
        self.gps = [0.0, 0.0]
        self.imu = 0.0
        self.speed = 0.0
        self._global_plan_gps = None

        self.gps_received = False
        self.imu_received = False
        self.speed_received = False
        self.global_plan_gps_received = False
        
        # self.pid_metadata = {} 
        
        # 클래스 초기화 시
        self.send_queue = queue.Queue(maxsize=3)
        self.sender_thread = threading.Thread(target=self._tcp_sender_thread, daemon=True)
        self.sender_thread.start()

        # BEST_EFFORT QoS 프로파일 생성
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        if self.img_input == 'raw':
            self.create_subscription(Image, '/carla/hero/CAM_FRONT/image', self.image_front_callback, best_effort_qos)
            self.create_subscription(Image, '/carla/hero/CAM_FRONT_LEFT/image', self.image_front_left_callback, best_effort_qos)
            self.create_subscription(Image, '/carla/hero/CAM_FRONT_RIGHT/image', self.image_front_right_callback, best_effort_qos)
        elif self.img_input == 'compressed':
            self.create_subscription(CompressedImage, '/image_front/compressed', self.compressed_image_front_callback, 1)
            self.create_subscription(CompressedImage, '/image_front_left/compressed', self.compressed_image_front_left_callback, 1)
            self.create_subscription(CompressedImage, '/image_front_right/compressed', self.compressed_image_front_right_callback, 1)
        else:
            print("img_input type is wrong !")

        self.create_subscription(NavSatFix, '/carla/hero/GPS', self.gps_callback, best_effort_qos)
        self.create_subscription(Imu, '/carla/hero/IMU', self.imu_callback, best_effort_qos)
        self.create_subscription(CarlaEgoVehicleStatus, '/carla/hero/vehicle_status', self.vehicle_status_callback, best_effort_qos)
        self.create_subscription(CarlaGnssRoute, '/carla/hero/global_plan_gps', self.global_plan_gps_callback, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        
        # self.backbone_publisher = self.create_publisher(BBDBackboneOutput, '/tcp/backbone_output', QoSProfile(depth=1))
        self.backbone_publisher = self.create_publisher(BBDBackboneOutput, '/tcp/backbone_output', best_effort_qos)

        # [추가] TCP 소켓 연결 (Orin2로 연결)
        self.tcp_ip = "192.168.20.2"  # Orin2
        self.tcp_port = 9999          # 포트는 맞춰서 사용
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)  # 송신 버퍼 크기 64KB
        try:
            self.sock.connect((self.tcp_ip, self.tcp_port))
            print(f"[TCPBackboneNode] Connected to TCP server at {self.tcp_ip}:{self.tcp_port}")
        except Exception as e:
            print(f"[TCPBackboneNode] TCP connection failed: {e}")
            exit(1)

        if self.debug_mode > 0 and SAVE_PATH:
            now = datetime.datetime.now()
            string = f"tcp_backbone_{now.strftime('%m_%d_%H_%M_%S')}"
            self.save_path = pathlib.Path(SAVE_PATH) / string
            self.save_path.mkdir(parents=True, exist_ok=True)
            
            (self.save_path / 'meta_backbone').mkdir()
            self.log_file = self.save_path / 'backbone_timing.csv'
            with open(self.log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'step', 'try_proc_num', 
                    'T_proc_start', 
                    # 'T_t_start', 'T_t_end', 
                    # 'T_pp_start', 
                    # 'T_rgb_start',
                    # 'T_im_trans_start', 'T_im_trans_end',
                    # 'T_tocuda_start', 'T_tocuda_end',
                    # 'T_rgb_end',
                    # 'T_one_hot_start', 'T_one_hot_end',
                    # 'T_tick_start', 'T_tick_end',
                    # 'T_stack_start', 'T_stack_end',
                    # 'T_state_start', 'T_state_end',
                    # 'T_pp_end', 
                    # 'T_bb_start',
                    
                    # 'T_bb_perception',
                    # 'T_bb_speed_branch',
                    # 'T_bb_measurements',
                    # 'T_bb_join_traj',
                    # 'T_bb_branch_traj',
                    # 'T_bb_pred_wp',
                    
                    # 'T_bb_end', 
                    # 'T_tx_bb_start', 'T_tx_bb_end', 
                    'T_bb_pub_start', 'T_bb_pub_end', 
                    # 'T_log_start', 'T_log_end'
                ])

            if self.debug_mode > 2:
                (self.save_path / 'rgb_front').mkdir()

    # 데이터를 큐에 넣는 비동기 전송 함수
    def send_via_tcp_async(self, msg):
        # print(f"[ASYNC] publishing start, step={msg[step]}")
        self.send_queue.put(msg)  # 비동기 전송 요청 (non-blocking)
        # print(f"[ASYNC] publishing end, step={msg[step]}")

    # 실제 송신을 담당하는 스레드 함수
    def _tcp_sender_thread(self):
        while True:
            try:
                raw_msg = self.send_queue.get(timeout=0.1)  # 블로킹 get 2)msg gen 병목

                # 이 안에서 직렬화
                packed_msg = msgpack.packb(raw_msg, use_bin_type=True)
                length_prefix = len(packed_msg).to_bytes(4, 'big')   

                self.sock.sendall(length_prefix + packed_msg)
                # print(f"[TCPBackboneNode] [Thread] Sent step {msg.step}, size={len(compressed_msg)}")
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[TCPBackboneNode] [Thread] TCP send failed: {e}")


    def send_via_tcp(self, msg):
        try:
            # msgpack으로 직렬화
            payload = {
                'j_traj': np.frombuffer(msg['j_traj'], dtype=np.float32).tolist(),
                'j_traj_shape': msg['j_traj_shape'],
                'measurement_feature': np.frombuffer(msg['measurement_feature'], dtype=np.float32).tolist(),
                'measurement_shape': msg['measurement_shape'],
                'cnn_feature': np.frombuffer(msg['cnn_feature'], dtype=np.float32).tolist(),
                'cnn_feature_shape': msg['cnn_feature_shape'],
                'gt_velocity': float(msg['gt_velocity']),
                'target_point': [float(x) for x in msg['target_point']],
                'command': [float(x) for x in msg['command']],
                'step': int(msg['step'])
            }
            serialized_msg = msgpack.packb(payload, use_bin_type=True)
            length = len(serialized_msg).to_bytes(4, byteorder='big')
            self.sock.sendall(length + serialized_msg)
            # print(f"[TCPBackboneNode] Sent msgpack data at step {msg['step']}, size={len(serialized_msg)} bytes")
        except Exception as e:
            print(f"[TCPBackboneNode] TCP send failed: {e}")


    def decode_image(self, msg):
        try:
            if self.img_input == 'raw':
                # self.get_logger().info(f"decode_image called, step={self.step}")
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                img_bgra = img_data.reshape((msg.height, msg.width, 4))
                return cv2.cvtColor(img_bgra, cv2.COLOR_BGRA2BGR)

            elif self.img_input == 'compressed':
                # self.get_logger().info(f"decode_compressed_image called, step={self.step}")
                np_arr = np.frombuffer(msg.data, np.uint8)
                image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                return image
            else:
                print("img_input type is wrong !")

        except Exception as e:
            return None

    def image_front_callback(self, msg):
        # self.get_logger().info(f"[image_front_callback] step {self.step}")
        self.rgb_front = self.decode_image(msg)
        self.try_process_step()
        # self.get_logger().info(f"[image_front_callback] finished, step {self.step}")

    def image_front_left_callback(self, msg):
        # self.get_logger().info(f"[image_front_left_callback] step {self.step}")
        self.rgb_front_left = self.decode_image(msg)
        self.try_process_step()
        # self.get_logger().info(f"[image_front_left_callback] finished, step {self.step}")

    def image_front_right_callback(self, msg):
        # self.get_logger().info(f"[image_front_right_callback] step {self.step}")
        self.rgb_front_right = self.decode_image(msg)
        self.try_process_step()
        # self.get_logger().info(f"[image_front_right_callback] finished, step {self.step}")

    def compressed_image_front_callback(self, msg):
        # self.get_logger().info(f"[compressed_image_front_callback] step {self.step}")
        self.rgb_front = self.decode_image(msg)
        self.try_process_step()
        # self.get_logger().info(f"[compressed_image_front_callback] finished, step {self.step}")

    def compressed_image_front_left_callback(self, msg):
        # self.get_logger().info(f"[compressed_image_front_left_callback] step {self.step}")
        self.rgb_front_left = self.decode_image(msg)
        self.try_process_step()
        # self.get_logger().info(f"[compressed_image_front_left_callback] finished, step {self.step}")

    def compressed_image_front_right_callback(self, msg):
        # self.get_logger().info(f"[compressed_image_front_right_callback] step {self.step}")
        self.rgb_front_right = self.decode_image(msg)
        self.try_process_step()
        # self.get_logger().info(f"[compressed_image_front_right_callback] finished, step {self.step}")

    def gps_callback(self, msg):
        # self.get_logger().info(f"[gps_callback] step {self.step}")
        self.gps = [msg.latitude, msg.longitude]
        self.gps_received = True
        self.try_process_step()
        # self.get_logger().info(f"[gps_callback] finished, step {self.step}, lat={msg.latitude}, lon={msg.longitude}") #debug

    def imu_callback(self, msg):
        # self.get_logger().info(f"[imu_callback] step {self.step}")
        # quaternion -> yaw
        orientation_q = msg.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        
        yaw = (np.pi / 2 - yaw) % (2 * np.pi) #jw) ENU to NED

        self.imu = yaw  # yaw in radians
        self.imu_received = True
        self.try_process_step()
        # self.get_logger().info(f"[imu_callback] finished, step {self.step}, imu={self.imu}") #debug

    def vehicle_status_callback(self, msg):
        # self.get_logger().info(f"[vehicle_status_callback] step {self.step}")
        self.speed = msg.velocity
        self.speed_received = True
        self.try_process_step()
        # self.get_logger().info(f"[vehicle_status_callback] finished, step {self.step},  speed={self.speed}") #debug

    def global_plan_gps_callback(self, msg):
        # self.get_logger().info(f"[global_plan_gps_callback] coordinates={len(msg.coordinates)}, road_options={len(msg.road_options)}")
        self._global_plan_gps = []
        for navsat, road_option in zip(msg.coordinates, msg.road_options):
            self._global_plan_gps.append((
            {
                'lon': navsat.longitude,
                'lat': navsat.latitude,
                'z': navsat.altitude
            }, 
            road_option
            ))
            
        if not self.global_plan_gps_received:
            self.global_plan_gps_received = True

        if not self.initialized:
            self._init()
        # print(f"GPS plan road_options: {[opt for _, opt in self._global_plan_gps]}")
        # print(f"[DEBUG] First global_plan_gps point: lon={self._global_plan_gps[0][0]['lon']}, lat={self._global_plan_gps[0][0]['lat']}")
        self.try_process_step()

    def _init(self):  
        if self.global_plan_gps_received is False:
            return

        try:
            locx, locy = self._global_plan[0][0]['location']['x'], self._global_plan[0][0]['location']['y']
            lon, lat = self._global_plan_gps[0][0]['lon'], self._global_plan_gps[0][0]['lat']
            E = EARTH_RADIUS_EQUA
            def equations(vars):
                x, y = vars
                eq1 = lon * math.cos(x * math.pi / 180) - (locx * x * 180) / (math.pi * E) - math.cos(x * math.pi / 180) * y
                eq2 = math.log(math.tan((lat + 90) * math.pi / 360)) * E * math.cos(x * math.pi / 180) + locy - math.cos(x * math.pi / 180) * E * math.log(math.tan((90 + x) * math.pi / 360))
                return [eq1, eq2]
            initial_guess = [0, 0]
            solution = fsolve(equations, initial_guess)
            self.lat_ref, self.lon_ref = solution[0], solution[1]
        except Exception as e:
            print(e, flush=True)
            self.lat_ref, self.lon_ref = 0, 0
        # print(f"[_init()] lat_ref: {self.lat_ref}, lon_ref: {self.lon_ref}, save_path: {self.save_path}")

        self._route_planner = RoutePlanner(4.0, 50.0, lat_ref=self.lat_ref, lon_ref=self.lon_ref)
        # self._route_planner.set_route(self._global_plan, False)
        self._route_planner.set_route(self._global_plan_gps, True)
        self.initialized = True

    def try_process_step(self):
        if self.initialized and self.rgb_front is not None and self.rgb_front_left is not None and self.rgb_front_right is not None and self.gps_received and self.imu_received and self.speed_received:
            # print(f"let's try process_step(), step {self.step}")
            self.process_step()
            # print(f"process_step() returned, step {self.step}")
            
            self.rgb_front = None
            self.rgb_front_left = None
            self.rgb_front_right = None
            self.gps_received = False
            self.imu_received = False
            self.speed_received = False
            self.try_proc_num = 0
        else:
            self.try_proc_num += 1
            # if self.debug_mode > 1:
                # print("-----------------------------------------------")
                # print("too early to do process_step()")
                # if not self.initialized:
                #     print(f"initialized = {self.initialized}")
                # if self.rgb_front is None:
                #     print(f"rgb_front = {self.rgb_front is not None}")
                # if self.rgb_front_left is None:
                #     print(f"rgb_front_left = {self.rgb_front_left is not None}")
                # if self.rgb_front_right is None:
                #     print(f"rgb_front_right = {self.rgb_front_right is not None}")
                # if not self.gps_received:
                #     print(f"gps_received = {self.gps_received}")
                # if not self.imu_received:
                #     print(f"imu_received = {self.imu_received}")
                # if not self.speed_received:
                #     print(f"speed_received = {self.speed_received}")
                # if not self.global_plan_gps_received:
                #     print(f"global_plan_gps_received = {self.global_plan_gps_received}")
                # print("-----------------------------------------------")

    def tick(self):
        self.step += 1

        if self.debug_mode > 1:
            self.get_logger().info(f"[TCPBackboneNode] Tick called, GPS={self.gps}, step={self.step}") #debug
        if not all([self.rgb_front is not None, self.rgb_front_left is not None, self.rgb_front_right is not None, self.initialized]):
            self.get_logger().warning(f"- tick(): No rgb_front or not initialized at step {self.step}")
            return None

        front = self.rgb_front[:, int(200*self.img_k):int(1400*self.img_k), :]
        left = self.rgb_front_left[:, :int(1400*self.img_k), :]
        right = self.rgb_front_right[:, int(200*self.img_k):, :]
        rgb_concate = np.concatenate((left, front, right), axis=1)

        rgb = torch.from_numpy(rgb_concate).permute(2, 0, 1).unsqueeze(0).float()
        rgb = torch.nn.functional.interpolate(rgb, size=(256, 900), mode='bilinear', align_corners=False)
        rgb = rgb.squeeze(0).permute(1, 2, 0).byte().numpy()

		# bev = cv2.cvtColor(input_data['bev'][1][:, :, :3], cv2.COLOR_BGR2RGB)
        gps = self.gps_to_location(self.gps)
        # print(f"[DEBUG] Agent GPS to local: x={gps[0]}, y={gps[1]}")
        speed = self.speed
        imu = self.imu

        result = {
            'rgb': rgb,
            'rgb_front': front,
            'gps': gps,
            'speed': speed,
            'imu': imu
        }
        pos = result['gps']
        next_wp, next_cmd = self._route_planner.run_step(pos)
        result['next_command'] = next_cmd

        # print(f"[DEBUG] Next WP: x={next_wp[0]}, y={next_wp[1]}")
        # print(f"[DEBUG] Distance to WP: dx={next_wp[0]-gps[0]}, dy={next_wp[1]-gps[1]}")
        # print(f"[DEBUG] next_cmd: {next_cmd}")

        theta = imu - np.pi / 2
        R = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
        local_command_point = np.array([next_wp[0] - pos[0], next_wp[1] - pos[1]])
        local_command_point = R.dot(local_command_point)
        result['target_point'] = tuple(local_command_point)
        # print(f"[DEBUG] Local target_point (vehicle frame): x={local_command_point[0]:.3f}, y={local_command_point[1]:.3f}")
        if self.debug_mode > 1:
            self.get_logger().info(f"Tick finished, step={self.step}") #debug        

        return result

    def publish_async(self, msg):
        def _task():
            print(f"[ASYNC] publishing start, step={msg.step}")
            self.backbone_publisher.publish(msg)
            print(f"[ASYNC] publishing done, step={msg.step}")
        threading.Thread(target=_task, daemon=True).start()
        # threading.Thread(target=self.backbone_publisher.publish, args=(msg,), daemon=True).start()

    @torch.no_grad()
    def process_step(self):
        T_proc_start = time.time()
        ros_time_ns = self.get_clock().now().nanoseconds # ROS2 시계 기준 시간 기록
        
        if self.debug_mode > 1:
            self.get_logger().warning(f"[TCPBackboneNode] Process step called, step={self.step}")

        # STEP 1: tick
        # T_t_start = time.time()
        tick_data = self.tick()
        # T_t_end = time.time()

        if tick_data is None or self.step < self.config.seq_len:
            self.get_logger().warning(f"- process_step(): No tick data or step < seq_len at step {self.step}, retutn control 0")
            return

        # STEP 2: input preprocessing
        # T_pp_start = time.time()
        # T_rgb_start = time.time()
        
        # # ver.1) split
        # # 1. 이미지 정규화 및 텐서 변환 (HWC → CHW, float, 정규화)
        # # 2. 배치 차원 추가 (→ shape: (1, 3, H, W))
        # T_im_trans_start = time.time()
        # rgb_tensor = self._im_transform(tick_data['rgb']).unsqueeze(0)  # shape: (3, H, W), dtype: float32, normalized
        # T_im_trans_end = time.time()
        
        # # 3. GPU로 이동 + dtype 변환
        # T_tocuda_start = time.time()
        # rgb_tensor = rgb_tensor.to('cuda', dtype=torch.float32)
        # T_tocuda_end = time.time()

        # # 4. 최종 변수 할당
        # rgb = rgb_tensor
        
        # # ver.2) original
        rgb = self._im_transform(tick_data['rgb']).unsqueeze(0).to('cuda', dtype=torch.float32)

        # T_rgb_end = time.time()

        # T_one_hot_start = time.time()
        gt_velocity = torch.FloatTensor([tick_data['speed']]).to('cuda', dtype=torch.float32)
        command = tick_data['next_command']
        command = 4 if command < 0 else command - 1
        assert command in [0, 1, 2, 3, 4, 5]
        cmd_one_hot = [0] * 6
        cmd_one_hot[command] = 1
        cmd_one_hot = torch.tensor(cmd_one_hot).view(1, 6).to('cuda', dtype=torch.float32)
        # T_one_hot_end = time.time()

        # T_tick_start = time.time()
        speed = torch.FloatTensor([float(tick_data['speed'])]).view(1, 1).to('cuda', dtype=torch.float32) / 12
        tick_data['target_point'] = [
            torch.FloatTensor([tick_data['target_point'][0]]),
            torch.FloatTensor([tick_data['target_point'][1]])
        ]
        # T_tick_end = time.time()

        # T_stack_start = time.time()
        target_point = torch.stack(tick_data['target_point'], dim=1).to('cuda', dtype=torch.float32)
        # T_stack_end = time.time()

        # T_state_start = time.time()
        state = torch.cat([speed, target_point, cmd_one_hot], 1)
        # T_state_end = time.time()

        # T_pp_end = time.time()

        # STEP 3: 모델 추론
        # T_bb_start = time.time()
        # break net!
        # cnn_feature, measurement_feature, traj_hidden_state, backbone_outputs = self.net(rgb, state, target_point) #ver.1
        j_traj, measurement_feature, cnn_feature, backbone_outputs = self.net(rgb, state, target_point) #ver.2
        # T_bb_end = time.time()

        # STEP 4: 메시지 생성 & Publish
        # T_tx_bb_start = time.time()

        msg_raw = {
            'step': self.step,
            'j_traj': j_traj.cpu().numpy().astype(np.float32).tobytes(),
            'j_traj_shape': list(j_traj.shape),
            'measurement_feature': measurement_feature.cpu().numpy().astype(np.float32).tobytes(),
            'measurement_shape': list(measurement_feature.shape),     
            'cnn_feature': cnn_feature.cpu().numpy().astype(np.float32).tobytes(),
            'cnn_feature_shape': list(cnn_feature.shape),
            'gt_velocity': float(gt_velocity.item()),
            'target_point': target_point.cpu().numpy().astype(np.float32).tolist(),
            'command': cmd_one_hot.view(-1).tolist(),
            'speed' : tick_data['speed'] #jw) tmp
        }

        T_bb_pub_start = time.time()
        # self.backbone_publisher.publish(msg) # 1)

        # self.publish_async(msg)
        # print(f"[MAIN] returned from publish_async(), step={self.step}") # 2)

        # self.send_via_tcp(msg)  # 블로킹으로 전송 # 3)

        # self.send_via_tcp_async(msg) # non-blocking 4)
        self.send_via_tcp_async(msg_raw) # non-blocking 4)
        # print(f"[MAIN] returned from send_via_tcp(), step={self.step}")

        T_bb_pub_end = time.time()

        # STEP 5: 결과 저장
        if self.debug_mode > 0:
            # T_log_start = time.time()

            # self.pid_metadata = {
            #     'step': self.step,
            #     'cnn_feature': list(msg.cnn_feature[:10]),
            #     'measurement_feature': list(msg.measurement_feature),
            #     'traj_hidden_state': list(msg.traj_hidden_state[:10]),
            #     'speed': float(tick_data['speed']),  # 원래 속도,
            #     'gt_velocity' : float(gt_velocity.item()), # 정규화된 속도 입력
            #     'target_point' : [float(target_point[0, 0].item()), float(target_point[0, 1].item())],
            #     'command' : cmd_one_hot.view(-1).tolist(),

            #     'ros_time_ns': ros_time_ns,
            #     'tick_ms': (T_t_end - T_t_start) * 1000,
            #     'preprocess_ms': (T_pp_end - T_pp_start) * 1000,
            #     'backbone_inference_ms': (T_bb_end - T_bb_start) * 1000,
            #     'tx_bb_msg_ms': (T_bb_pub_start - T_tx_bb_start) * 1000,
            #     'bb_publish_ms': (T_bb_pub_end - T_bb_pub_start) * 1000,
            #     'total_process_step_ms': (T_bb_pub_end - T_proc_start) * 1000
            # }
            
            if SAVE_PATH and self.step % 1 == 0 and self.debug_mode > 2:
                # self.save(tick_data)
                self.save_async(tick_data)

            # timing log 저장
            if self.log_file:
                with open(self.log_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        self.step, self.try_proc_num,
                        T_proc_start,
                        # T_t_start, T_t_end,
                        # T_pp_start, 
                        # T_rgb_start, 
                        # T_im_trans_start, T_im_trans_end,
                        # T_tocuda_start, T_tocuda_end,
                        # T_rgb_end,
                        # T_one_hot_start, T_one_hot_end,
                        # T_tick_start, T_tick_end,
                        # T_stack_start, T_stack_end,
                        # T_state_start, T_state_end,
                        # T_pp_end,
                        # T_bb_start, 
                        
                        # backbone_outputs['timing']['perception'],
                        # backbone_outputs['timing']['speed_branch'],
                        # backbone_outputs['timing']['measurements'],
                        # backbone_outputs['timing']['join_traj'],
                        
                        # T_bb_end,
                        # T_tx_bb_start, T_bb_pub_start,
                        T_bb_pub_start, T_bb_pub_end,
                        # T_log_start, time.time()
                    ])
            
            # if self.debug_mode > 1:       
            #     T_log_end = time.time()
            #     self.get_logger().warning(f"[TCPBackboneNode] Process step finished, step={self.step}")


    def save(self, tick_data):
        frame = self.step
        PILImage.fromarray(tick_data['rgb_front']).save(self.save_path / 'rgb_front' / (f'%04d.png' % frame))
        # with open(self.save_path / 'meta_backbone' / (f'%04d.json' % frame), 'w') as outfile:
        #     json.dump(self.pid_metadata, outfile, indent=4)

    def save_async(self, tick_data):
        def save_task():
            self.save(tick_data)
        threading.Thread(target=save_task, daemon=True).start()

    def gps_to_location(self, gps):
        lat, lon = gps
        scale = math.cos(self.lat_ref * math.pi / 180.0)
        my = math.log(math.tan((lat + 90) * math.pi / 360.0)) * (EARTH_RADIUS_EQUA * scale)
        mx = (lon * (math.pi * EARTH_RADIUS_EQUA * scale)) / 180.0
        y = scale * EARTH_RADIUS_EQUA * math.log(math.tan((90.0 + self.lat_ref) * math.pi / 360.0)) - my
        x = mx - scale * self.lon_ref * math.pi * EARTH_RADIUS_EQUA / 180.0
        return np.array([x, y])

    def destroy(self):
        del self.net
        torch.cuda.empty_cache()
        self.get_logger().info("TCP Agent Node destroyed")
        
def main():
    import argparse
    parser = argparse.ArgumentParser(description='TCP Agent Node for CARLA')
    parser.add_argument('--ckpt-path', required=True, help='Path to model checkpoint')
    parser.add_argument('--save-path', default=None, help='Path to save debug outputs')
    parser.add_argument('--debug-mode', type=int, default=0, help='Level of debug mode')
    parser.add_argument('--img-input', default='raw', help='Type for input camera image')
    parser.add_argument('--img-k', type=float, default=1.0, help='Input camera image resolution ratio')
    args = parser.parse_args()

    rclpy.init()
    node = TCPBackboneNode(args.ckpt_path, args.save_path, args.debug_mode, args.img_input, args.img_k)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TCP Backbone Node")
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()