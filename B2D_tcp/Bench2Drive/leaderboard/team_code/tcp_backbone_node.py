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

# from TCP.model import TCP
from TCP.config import GlobalConfig
from team_code.planner import RoutePlanner
from scipy.optimize import fsolve

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, NavSatFix, Imu, CompressedImage
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaRoute, CarlaEgoVehicleStatus, CarlaGnssRoute
from tf_transformations import euler_from_quaternion
from TCP.model_backbone import TCPBackbone
from tcp_msgs.msg import TCPBackboneOutput

SAVE_PATH = os.environ.get('SAVE_PATH', None)

EARTH_RADIUS_EQUA = 6378137.0

class TCPBackboneNode(Node):
    def __init__(self, ckpt_path, save_path, debug_mode, img_input, img_k):
        super().__init__('tcp_backbone_node')

        self.ckpt_path = ckpt_path
        # self.save_path = save_path if save_path else '/tmp/tcp_agent'
        self.debug_mode = debug_mode
        self.img_input = img_input if img_input else 'raw'
        self.img_k = float(img_k) if img_k else 1.0
        self.step = 0
        self.initialized = False
        self.wall_start = time.time()

        self.config = GlobalConfig()
        # self.net = TCP(self.config)
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

        self._im_transform = T.Compose([
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
        self._global_plan = None
        self._global_plan_gps = None

        self.gps_received = False
        self.imu_received = False
        self.speed_received = False
        self.global_plan_received = False
        self.global_plan_gps_received = False
        
        self.pid_metadata = {} 
        
        if self.img_input == 'raw':
            self.create_subscription(Image, '/carla/hero/CAM_FRONT/image', self.image_front_callback, 1)
            self.create_subscription(Image, '/carla/hero/CAM_FRONT_LEFT/image', self.image_front_left_callback, 1)
            self.create_subscription(Image, '/carla/hero/CAM_FRONT_RIGHT/image', self.image_front_right_callback, 1)
        elif self.img_input == 'compressed':
            self.create_subscription(CompressedImage, '/image/compressed', self.compressed_image_front_callback, 1)
            self.create_subscription(CompressedImage, '/image_left/compressed', self.compressed_image_front_left_callback, 1)
            self.create_subscription(CompressedImage, '/image_right/compressed', self.compressed_image_front_right_callback, 1)
        else:
            print("img_input type is wrong !")

        self.create_subscription(NavSatFix, '/carla/hero/GPS', self.gps_callback, 1)
        self.create_subscription(Imu, '/carla/hero/IMU', self.imu_callback, 1)
        self.create_subscription(CarlaEgoVehicleStatus, '/carla/hero/vehicle_status', self.vehicle_status_callback, 1)
        self.create_subscription(CarlaGnssRoute, '/carla/hero/global_plan_gps', self.global_plan_gps_callback, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(CarlaRoute, '/carla/hero/global_plan', self.global_plan_callback, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # self.control_pub = self.create_publisher(CarlaEgoVehicleControl, '/tcp/vehicle_control_cmd', QoSProfile(depth=1))
        self.backbone_publisher = self.create_publisher(TCPBackboneOutput, '/tcp/backbone_output', QoSProfile(depth=1))

        if SAVE_PATH:
            now = datetime.datetime.now()
            string = f"tcp_agent_{now.strftime('%m_%d_%H_%M_%S')}"
            self.save_path = pathlib.Path(SAVE_PATH) / string
            self.save_path.mkdir(parents=True, exist_ok=True)
            (self.save_path / 'rgb_front').mkdir()
            (self.save_path / 'meta_backbone').mkdir()
            
            # timing
            self.log_file = self.save_path / 'meta_backbone' / 'backbone_timing.csv'
            with open(self.log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'step', 'T_s', 'T_p_start', 'T_p_end', 'T_b_start', 'T_b_end', 'T_tx_start', 'T_tx_end'
                ])


        if self.debug_mode:
            self.timer = self.create_timer(1.0, self.debug_callback)

    def decode_image(self, msg):
        try:
            self.get_logger().info(f"decode_image called, step={self.step}")
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            img_bgra = img_data.reshape((msg.height, msg.width, 4))
            return cv2.cvtColor(img_bgra, cv2.COLOR_BGRA2BGR)
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

    def decode_compressed_image(self, msg):
        try:
            # self.get_logger().info(f"decode_compressed_image called, step={self.step}")
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            return image
        except Exception as e:
            # self.get_logger().error(f"Image decode failed: {e}")
            return None

    def compressed_image_front_callback(self, msg):
        # self.get_logger().info(f"[compressed_image_front_callback] step {self.step}")
        self.rgb_front = self.decode_compressed_image(msg)
        self.try_process_step()
        # self.get_logger().info(f"[compressed_image_front_callback] finished, step {self.step}")

    def compressed_image_front_left_callback(self, msg):
        # self.get_logger().info(f"[compressed_image_front_left_callback] step {self.step}")
        self.rgb_front_left = self.decode_compressed_image(msg)
        self.try_process_step()
        # self.get_logger().info(f"[compressed_image_front_left_callback] finished, step {self.step}")

    def compressed_image_front_right_callback(self, msg):
        # self.get_logger().info(f"[compressed_image_front_right_callback] step {self.step}")
        self.rgb_front_right = self.decode_compressed_image(msg)
        self.try_process_step()
        # self.get_logger().info(f"[compressed_image_front_right_callback] finished, step {self.step}")

    def try_process_step(self):
        if self.initialized and self.rgb_front is not None and self.rgb_front_left is not None and self.rgb_front_right is not None and self.gps_received and self.imu_received and self.speed_received:
            print(f"let's try process_step(), step {self.step}")
            self.process_step()
            print(f"process_step() returned, step {self.step}")
            
            self.rgb_front = None
            self.rgb_front_left = None
            self.rgb_front_right = None
            self.gps_received = False
            self.imu_received = False
            self.speed_received = False
        else:
            print("-----------------------------------------------")
            print("too early to do process_step()")
            if not self.initialized:
                print(f"initialized = {self.initialized}")
            if self.rgb_front is None:
                print(f"rgb_front = {self.rgb_front is not None}")
            if self.rgb_front_left is None:
                print(f"rgb_front_left = {self.rgb_front_left is not None}")
            if self.rgb_front_right is None:
                print(f"rgb_front_right = {self.rgb_front_right is not None}")
            if not self.gps_received:
                print(f"gps_received = {self.gps_received}")
            if not self.imu_received:
                print(f"imu_received = {self.imu_received}")
            if not self.speed_received:
                print(f"speed_received = {self.speed_received}")
            if not self.global_plan_received:
                print(f"global_plan_received = {self.global_plan_received}")
            if not self.global_plan_gps_received:
                print(f"global_plan_gps_received = {self.global_plan_gps_received}")
            print("-----------------------------------------------")


    def gps_callback(self, msg):
        self.gps = [msg.latitude, msg.longitude]
        self.gps_received = True
        self.try_process_step()
        # self.get_logger().info(f"[gps_callback] lat={msg.latitude}, lon={msg.longitude}") #debug

    def imu_callback(self, msg):
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
        self.get_logger().info(f"[imu_callback] imu={self.imu}") #debug

    def vehicle_status_callback(self, msg):
        self.speed = msg.velocity
        self.speed_received = True
        self.try_process_step()
        # self.get_logger().info(f"[vehicle_status_callback] speed={self.speed}") #debug

    def global_plan_callback(self, msg):
        self.get_logger().info(f"[global_plan_callback] poses={len(msg.poses)}, road_options={len(msg.road_options)}")
        self._global_plan = []
        for pose, road_option in zip(msg.poses, msg.road_options):
            self._global_plan.append(({
                'location': {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z
                }
            }, road_option))

        if not self.global_plan_received:
            self.global_plan_received = True

        if not self.initialized:
            self._init()
        print(f"World plan road_options: {[opt for _, opt in self._global_plan]}")
        print(f"[DEBUG] First global_plan_world_coord point: x={self._global_plan[0][0]['location']['x']}, y={self._global_plan[0][0]['location']['y']}")
        self.try_process_step()

    def global_plan_gps_callback(self, msg):
        self.get_logger().info(f"[global_plan_gps_callback] coordinates={len(msg.coordinates)}, road_options={len(msg.road_options)}")
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
        print(f"GPS plan road_options: {[opt for _, opt in self._global_plan_gps]}")
        print(f"[DEBUG] First global_plan_gps point: lon={self._global_plan_gps[0][0]['lon']}, lat={self._global_plan_gps[0][0]['lat']}")
        self.try_process_step()

    def _init(self):  #jw) this
        if self.global_plan_received is False or self.global_plan_gps_received is False:
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
        print(f"[_init()] lat_ref: {self.lat_ref}, lon_ref: {self.lon_ref}, save_path: {self.save_path}")
        #
        self._route_planner = RoutePlanner(4.0, 50.0, lat_ref=self.lat_ref, lon_ref=self.lon_ref)
        # self._route_planner.set_route(self._global_plan, False)
        self._route_planner.set_route(self._global_plan_gps, True)
        self.initialized = True

    def tick(self):
        self.step += 1

        self.get_logger().info(f"Tick called, GPS={self.gps}, step={self.step}") #debug
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

        # rgb_resized = cv2.resize(rgb, (900, 256))
        # rgb_front = cv2.cvtColor(rgb_resized, cv2.COLOR_BGR2RGB)

        gps = self.gps_to_location(self.gps)
        print(f"[DEBUG] Agent GPS to local: x={gps[0]}, y={gps[1]}")
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
        print(f"[DEBUG] Next WP: x={next_wp[0]}, y={next_wp[1]}")
        print(f"[DEBUG] Distance to WP: dx={next_wp[0]-gps[0]}, dy={next_wp[1]-gps[1]}")

        result['next_command'] = next_cmd
        print(f"[DEBUG] next_cmd: {next_cmd}")

        self.get_logger().info(f"- tick(): Next waypoint: {next_wp}, cmd: {next_cmd}")    #debug

        theta = imu - np.pi / 2
        R = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
        local_command_point = np.array([next_wp[0] - pos[0], next_wp[1] - pos[1]])
        local_command_point = R.dot(local_command_point)
        print(f"[DEBUG] Local target_point (vehicle frame): x={local_command_point[0]:.3f}, y={local_command_point[1]:.3f}")
        result['target_point'] = tuple(local_command_point)

        self.get_logger().info(f"Tick finished, step={self.step}") #debug
        
        return result

    @torch.no_grad()
    def process_step(self):
        self.get_logger().warning(f"[TCPBackboneNode] Process step called, step={self.step}")

        tick_data = self.tick()
        if tick_data is None or self.step < self.config.seq_len:
            self.get_logger().warning(f"- process_step(): No tick data or step < seq_len at step {self.step}, retutn control 0")
            rgb = self._im_transform(tick_data['rgb']).unsqueeze(0)
            control = CarlaEgoVehicleControl()
            self.control_pub.publish(control)
            return

        # time) ROS2 시계 기준 시간 기록
        ros_time_ns = self.get_clock().now().nanoseconds

        # STEP 0: 전체 시간 측정 시작
        #timing
        T_s = time.time()  # Sensor 입력 수신 기준 시간

        # STEP 1: 이미지 전처리
        T_p_start = time.time()
        rgb = self._im_transform(tick_data['rgb']).unsqueeze(0).to('cuda', dtype=torch.float32)
        T_p_end = time.time()

        gt_velocity = torch.FloatTensor([tick_data['speed']]).to('cuda', dtype=torch.float32)
        command = tick_data['next_command']
        command = 4 if command < 0 else command - 1
        assert command in [0, 1, 2, 3, 4, 5]
        cmd_one_hot = [0] * 6
        cmd_one_hot[command] = 1
        cmd_one_hot = torch.tensor(cmd_one_hot).view(1, 6).to('cuda', dtype=torch.float32)

        speed = torch.FloatTensor([float(tick_data['speed'])]).view(1, 1).to('cuda', dtype=torch.float32) / 12
        tick_data['target_point'] = [
            torch.FloatTensor([tick_data['target_point'][0]]),
            torch.FloatTensor([tick_data['target_point'][1]])
        ]
        target_point = torch.stack(tick_data['target_point'], dim=1).to('cuda', dtype=torch.float32)
        state = torch.cat([speed, target_point, cmd_one_hot], 1)

        # STEP 2: 모델 추론
        self.get_logger().warning(f"- process_step(): backbone net()")
        T_b_start = time.time()
        # break net!
        cnn_feature, measurement_feature, traj_hidden_state, backbone_outputs = self.net(rgb, state, target_point)
        T_b_end = time.time()

        # 메시지 생성
        msg = TCPBackboneOutput()
        msg.cnn_feature = cnn_feature.flatten().tolist()
        msg.measurement_feature = measurement_feature.flatten().tolist()
        msg.traj_hidden_state = traj_hidden_state.flatten().tolist()
        msg.speed = float(tick_data['speed'])  # 원래 속도
        msg.gt_velocity = float(gt_velocity.item())  # 정규화된 속도 입력
        msg.target_point = [float(target_point[0, 0].item()), float(target_point[0, 1].item())]
        msg.command = cmd_one_hot.view(-1).tolist()
        msg.pred_wp = backbone_outputs['pred_wp'].view(-1).tolist()
        msg.step = self.step

        # Publish 시작/종료 시간 측정
        T_tx_start = time.time()
        self.backbone_publisher.publish(msg)
        T_tx_end = time.time()

        # 결과 저장
        self.pid_metadata = {
            'step': self.step,
            'cnn_feature': list(msg.cnn_feature[:10]),
            'measurement_feature': list(msg.measurement_feature),
            'traj_hidden_state': list(msg.traj_hidden_state[:10]),
            'speed': float(tick_data['speed']),  # 원래 속도,
            'gt_velocity' : float(gt_velocity.item()), # 정규화된 속도 입력
            'target_point' : [float(target_point[0, 0].item()), float(target_point[0, 1].item())],
            'command' : cmd_one_hot.view(-1).tolist(),

            'ros_time_ns': ros_time_ns,
            'image_preprocess_ms': (T_p_end - T_p_start) * 1000,
            'backbone_inference_ms': (T_b_end - T_b_start) * 1000,
            'publish_ms': (T_tx_end - T_b_end) * 1000,
            'total_process_step_ms': (T_tx_end - T_s) * 1000
        }
        
        if SAVE_PATH and self.step % 1 == 0:
            self.save(tick_data)

        # timing log 저장
        if self.log_file:
            with open(self.log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    self.step,
                    T_s,
                    T_p_start,
                    T_p_end,
                    T_b_start,
                    T_b_end,
                    T_tx_start,
                    T_tx_end
                ])
        else:
            print(f"log_file !!!!!!!!!!: {self.log_file}")
                
        self.get_logger().warning(f"Process step finished, step={self.step}")

    def save(self, tick_data):
        frame = self.step
        PILImage.fromarray(tick_data['rgb_front']).save(self.save_path / 'rgb_front' / (f'%04d.png' % frame))
        with open(self.save_path / 'meta_backbone' / (f'%04d.json' % frame), 'w') as outfile:
            json.dump(self.pid_metadata, outfile, indent=4)

    def gps_to_location(self, gps):
        lat, lon = gps
        scale = math.cos(self.lat_ref * math.pi / 180.0)
        my = math.log(math.tan((lat + 90) * math.pi / 360.0)) * (EARTH_RADIUS_EQUA * scale)
        mx = (lon * (math.pi * EARTH_RADIUS_EQUA * scale)) / 180.0
        y = scale * EARTH_RADIUS_EQUA * math.log(math.tan((90.0 + self.lat_ref) * math.pi / 360.0)) - my
        x = mx - scale * self.lon_ref * math.pi * EARTH_RADIUS_EQUA / 180.0
        return np.array([x, y])

    def debug_callback(self):
        pass

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
    parser.add_argument('--debug', default=True, help='Run in debug mode')
    parser.add_argument('--img-input', default='raw', help='Type for input camera image')
    parser.add_argument('--img-k', type=float, default=1.0, help='Input camera image resolution ratio')
    args = parser.parse_args()

    rclpy.init()
    node = TCPBackboneNode(args.ckpt_path, args.save_path, args.debug, args.img_input, args.img_k)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TCP Agent Node")
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
