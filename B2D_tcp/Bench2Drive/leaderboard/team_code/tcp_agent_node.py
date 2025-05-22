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

from TCP.model import TCP
from TCP.config import GlobalConfig
from team_code.planner import RoutePlanner
from scipy.optimize import fsolve

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, NavSatFix, Imu, CompressedImage
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaRoute, CarlaEgoVehicleStatus
# from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion

SAVE_PATH = os.environ.get('SAVE_PATH', None)
IS_BENCH2DRIVE = os.environ.get('IS_BENCH2DRIVE', None)
PLANNER_TYPE = os.environ.get('PLANNER_TYPE', None)
EARTH_RADIUS_EQUA = 6378137.0
# IMG_K = 0.5

class TCPAgentNode(Node):
    def __init__(self, ckpt_path, save_path, debug_mode, img_input, img_k):
        super().__init__('tcp_agent_node')

        self.ckpt_path = ckpt_path
        self.save_path = save_path if save_path else '/tmp/tcp_agent'
        self.debug_mode = debug_mode
        self.img_input = img_input if img_input else 'raw'
        self.img_k = float(img_k) if img_k else 1.0
        self.step = -1
        self.initialized = False
        self.wall_start = time.time()

        self.config = GlobalConfig()
        self.net = TCP(self.config)

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
        self.speed = 0.0
        self.imu = 0.0
        self.gps_received = False
        self.imu_received = False
        self.speed_received = False
        self.global_plan = None
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
        self.create_subscription(CarlaRoute, '/carla/hero/global_plan', self.global_plan_callback, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.control_pub = self.create_publisher(CarlaEgoVehicleControl, '/tcp/vehicle_control_cmd', QoSProfile(depth=1))

        if SAVE_PATH:
            now = datetime.datetime.now()
            string = f"tcp_agent_{now.strftime('%m_%d_%H_%M_%S')}"
            self.save_path = pathlib.Path(SAVE_PATH) / string
            self.save_path.mkdir(parents=True, exist_ok=True)
            (self.save_path / 'rgb_front').mkdir()
            (self.save_path / 'meta').mkdir()

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
        self.get_logger().info(f"[image_front_callback] step {self.step}")
        self.rgb_front = self.decode_image(msg)
        self.try_process_step()
        self.get_logger().info(f"[image_front_callback] finished, step {self.step}")

    def image_front_left_callback(self, msg):
        self.get_logger().info(f"[image_front_left_callback] step {self.step}")
        self.rgb_front_left = self.decode_image(msg)
        self.try_process_step()
        self.get_logger().info(f"[image_front_left_callback] finished, step {self.step}")

    def image_front_right_callback(self, msg):
        self.get_logger().info(f"[image_front_right_callback] step {self.step}")
        self.rgb_front_right = self.decode_image(msg)
        self.try_process_step()
        self.get_logger().info(f"[image_front_right_callback] finished, step {self.step}")

    def decode_compressed_image(self, msg):
        try:
            self.get_logger().info(f"decode_compressed_image called, step={self.step}")
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            return image
        except Exception as e:
            self.get_logger().error(f"Image decode failed: {e}")
            return None

    def compressed_image_front_callback(self, msg):
        self.get_logger().info(f"[image_front_callback] step {self.step}")
        self.rgb_front = self.decode_compressed_image(msg)
        self.try_process_step()
        self.get_logger().info(f"[image_front_callback] finished, step {self.step}")

    def compressed_image_front_left_callback(self, msg):
        self.get_logger().info(f"[image_front_left_callback] step {self.step}")
        self.rgb_front_left = self.decode_compressed_image(msg)
        self.try_process_step()
        self.get_logger().info(f"[image_front_left_callback] finished, step {self.step}")

    def compressed_image_front_right_callback(self, msg):
        self.get_logger().info(f"[image_front_right_callback] step {self.step}")
        self.rgb_front_right = self.decode_compressed_image(msg)
        self.try_process_step()
        self.get_logger().info(f"[image_front_right_callback] finished, step {self.step}")


    def try_process_step(self):
        print("-----------------------------------------------")
        print(f"initialized = {self.initialized}")
        print(f"rgb_front = {self.rgb_front is not None}")
        print(f"rgb_front_left = {self.rgb_front_left is not None}")
        print(f"rgb_front_right = {self.rgb_front_right is not None}")
        print(f"gps_received = {self.gps_received}")
        print(f"imu_received = {self.imu_received}")
        print(f"speed_received = {self.speed_received}")
        print("-----------------------------------------------")

        if self.initialized and self.rgb_front is not None and self.rgb_front_left is not None and self.rgb_front_right is not None and self.gps_received and self.imu_received and self.speed_received:
        # if self.initialized and self.rgb_front is not None:
        # if self.initialized and self.rgb_front is not None and self.rgb_front_left is not None and self.rgb_front_right is not None and self.speed_received and self.gps_received:
            print(f"let's try process_step(), step {self.step}")
            # control = CarlaEgoVehicleControl()
            # self.control_pub.publish(control)
            self.process_step()
            print(f"process_step() returned, step {self.step}")
            
            self.rgb_front = None
            self.rgb_front_left = None
            self.rgb_front_right = None
            self.gps_received = False
            self.imu_received = False
            self.speed_received = False
        else:
            print("too early to do process_step()")

    def gps_callback(self, msg):
        self.gps = [msg.latitude, msg.longitude]
        self.gps_received = True
        self.try_process_step()
        self.get_logger().info(f"[gps_callback] lat={msg.latitude}, lon={msg.longitude}") #debug

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

        self.imu = yaw  # yaw in radians
        # self.imu = msg.data if not math.isnan(msg.data) else 0.0
        self.imu_received = True
        self.try_process_step()
        self.get_logger().info(f"[imu_callback] imu={self.imu}") #debug

    def vehicle_status_callback(self, msg):
        self.speed = msg.velocity
        self.speed_received = True
        self.try_process_step()
        self.get_logger().info(f"[vehicle_status_callback] speed={self.speed}") #debug

    def global_plan_callback(self, msg):
        # self.get_logger().info(f"[global_plan_callback] poses={len(msg.poses)}, road_options={len(msg.road_options)}")
        self.global_plan = []
        for pose, road_option in zip(msg.poses, msg.road_options):
            self.global_plan.append(({
                'position': {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z
                }
            }, road_option))
        if not self.initialized:
            self._init()
        print(f"[DEBUG] First global_plan point: x={self.global_plan[0][0]['position']['x']}, y={self.global_plan[0][0]['position']['y']}")
        self.try_process_step()
        # self.get_logger().info(f"[global_plan_callback] Global plan received with {len(self.global_plan)} waypoints")

    def _init(self):
        self.lat_ref, self.lon_ref = 0.0, 0.0
        self._route_planner = RoutePlanner(4.0, 50.0, lat_ref=self.lat_ref, lon_ref=self.lon_ref)
        self._route_planner.set_route(self.global_plan, False)
        self.initialized = True
        self.get_logger().info(f"Initialized with lat_ref={self.lat_ref}, lon_ref={self.lon_ref}")

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

        self.get_logger().info(f"- tick(): Speed: {self.speed}") #debugc
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
        self.get_logger().info(f"- tick(): Next waypoint: {next_wp}, cmd: {next_cmd}")    #debug

        theta = imu - np.pi / 2
        R = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
        local_command_point = np.array([next_wp[0] - pos[0], next_wp[1] - pos[1]])
        local_command_point = R.dot(local_command_point)
        result['target_point'] = tuple(local_command_point)
        self.get_logger().info(f"Tick finished, step={self.step}") #debug
        return result

    @torch.no_grad()
    def process_step(self):
        self.get_logger().warning(f"Process step called, step={self.step}")
        tick_data = self.tick()
        if tick_data is None or self.step < self.config.seq_len:
            self.get_logger().warning(f"- process_step(): No tick data or setp < seq_len at step {self.step}")
            return

        gt_velocity = torch.FloatTensor([tick_data['speed']]).to('cuda', dtype=torch.float32)
        command = tick_data['next_command']
        self.get_logger().warning(f"- process_step(): next_command={command}")

        if command < 0:
            command = 4
        command -= 1
        cmd_one_hot = [0] * 6
        cmd_one_hot[command] = 1
        cmd_one_hot = torch.tensor(cmd_one_hot).view(1, 6).to('cuda', dtype=torch.float32)
        speed = torch.FloatTensor([float(tick_data['speed'])]).view(1, 1).to('cuda', dtype=torch.float32) / 12
        rgb = self._im_transform(tick_data['rgb']).unsqueeze(0).to('cuda', dtype=torch.float32)

        tick_data['target_point'] = [
            torch.FloatTensor([tick_data['target_point'][0]]),
            torch.FloatTensor([tick_data['target_point'][1]])
        ]
        target_point = torch.stack(tick_data['target_point'], dim=1).to('cuda', dtype=torch.float32)
        state = torch.cat([speed, target_point, cmd_one_hot], 1)

        self.get_logger().warning(f"- process_step(): net()")
        pred = self.net(rgb, state, target_point)
        steer_ctrl, throttle_ctrl, brake_ctrl, metadata = self.net.process_action(pred, tick_data['next_command'], gt_velocity, target_point)
        self.get_logger().warning(f"- process_step(): control_pid()")
        steer_traj, throttle_traj, brake_traj, metadata_traj = self.net.control_pid(pred['pred_wp'], gt_velocity, target_point)

        self.get_logger().info(f"state: {state}") #debug
        self.get_logger().info(f"pred_wp: {pred['pred_wp']}") #debug
        self.get_logger().info(f"steer_ctrl: {steer_ctrl}, throttle_ctrl: {throttle_ctrl}, brake_ctrl: {brake_ctrl}") #debug
        self.get_logger().info(f"steer_traj: {steer_traj}, throttle_traj: {throttle_traj}, brake_traj: {brake_traj}") #debug

        self.get_logger().warning(f"- process_step(): clipping")
        control = CarlaEgoVehicleControl()
        if PLANNER_TYPE == 'only_traj':
            self.pid_metadata = metadata_traj
            control.steer = np.clip(float(steer_traj), -1, 1)
            control.throttle = np.clip(float(throttle_traj), 0, 0.75)
            control.brake = np.clip(float(brake_traj), 0, 1)
        elif PLANNER_TYPE == 'only_ctrl':
            self.pid_metadata = metadata
            control.steer = np.clip(float(steer_ctrl), -1, 1)
            control.throttle = np.clip(float(throttle_ctrl), 0, 0.75)
            control.brake = np.clip(float(brake_ctrl), 0, 1)
        elif PLANNER_TYPE == 'merge_ctrl_traj':
            self.pid_metadata = metadata_traj
            alpha = 0.5
            control.steer = np.clip(alpha * steer_traj + (1 - alpha) * steer_ctrl, -1, 1)

            self.get_logger().warning(f"[debug1] throttle={control.throttle}") #debug
            control.throttle = np.clip(alpha * throttle_traj + (1 - alpha) * throttle_ctrl, 0, 0.75)
            self.get_logger().warning(f"[debug2] throttle={control.throttle}") #debug
            control.brake = max(np.clip(float(brake_ctrl), 0, 1), np.clip(float(brake_traj), 0, 1))

        if abs(control.steer) > 0.07:
            speed_threshold = 1.0
        else:
            speed_threshold = 1.5

        self.get_logger().warning(f"[debug3] throttle={control.throttle}") #debug
        control.throttle = np.clip(control.throttle, 0.0, 0.05 if float(tick_data['speed']) > speed_threshold else 0.5)
        self.get_logger().warning(f"[debug4] throttle={control.throttle}") #debug

        if control.brake > 0:
            control.brake = 1.0
        if control.brake > 0.5:
            control.throttle = 0.0

        self.pid_metadata.update({
            'steer': control.steer,
            'throttle': control.throttle,
            'brake': control.brake
        })

        self.get_logger().warning(f"[PUB CONTROL] steer={control.steer}, throttle={control.throttle}, brake={control.brake}") #debug
        control.steer *= -1
        self.control_pub.publish(control)
        self.get_logger().warning(f"- process_step(): control_pub()")
        if SAVE_PATH and self.step % 1 == 0:
            self.save(tick_data)
        self.get_logger().warning(f"Process step finished, step={self.step}")

    def save(self, tick_data):
        frame = self.step
        PILImage.fromarray(tick_data['rgb_front']).save(self.save_path / 'rgb_front' / (f'%04d.png' % frame))
        with open(self.save_path / 'meta' / (f'%04d.json' % frame), 'w') as outfile:
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
    node = TCPAgentNode(args.ckpt_path, args.save_path, args.debug, args.img_input, args.img_k)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TCP Agent Node")
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
