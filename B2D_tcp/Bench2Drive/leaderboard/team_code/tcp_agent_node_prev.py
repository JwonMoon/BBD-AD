import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, NavSatFix
from std_msgs.msg import Float32
from carla_msgs.msg import CarlaEgoVehicleControl
from geometry_msgs.msg import PoseArray
import cv2
import numpy as np
import torch
from torchvision import transforms as T
from TCP.model import TCP
from TCP.config import GlobalConfig
from team_code.planner import RoutePlanner
import math
import time
from collections import OrderedDict
from PIL import Image
import os
import argparse

class TCPAgentNode(Node):
    def __init__(self, ckpt_path, save_path, debug_mode):
        super().__init__('tcp_agent_node')
        self.config = GlobalConfig()
        self.net = TCP(self.config).cuda().eval()
        # 체크포인트 경로 수정 필요
        # ckpt_path = "/path/to/tcp_b2d.ckpt"  # 여기를 네 경로로 변경
        ckpt = torch.load(ckpt_path, map_location="cuda", weights_only=True)
        self.net.load_state_dict({k.replace("model.", ""): v for k, v in ckpt["state_dict"].items()})
        self.lat_ref, self.lon_ref = 0.0, 0.0
        self.route_planner = RoutePlanner(4.0, 50.0, lat_ref=self.lat_ref, lon_ref=self.lon_ref)
        self.initialized = False
        self.step = -1
        self.im_transform = T.Compose([
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        self.input_data = {}
        self.global_plan = None
        self.debug_mode = debug_mode  # 디버깅 모드 (필요하면 False로)
        self.save_path = save_path  # 저장 경로 수정 필요

        # Subscriptions
        # self.create_subscription(Image, '/carla/ego_vehicle/rgb_front/image', self.image_callback, 10)
        # self.create_subscription(Image, '/carla/ego_vehicle/rgb_front_left/image', self.image_callback, 10)
        # self.create_subscription(Image, '/carla/ego_vehicle/rgb_front_right/image', self.image_callback, 10)
        # self.create_subscription(Image, '/carla/ego_vehicle/bev/image', self.bev_callback, 10)
        self.create_subscription(NavSatFix, '/carla/ego_vehicle/gnss', self.gps_callback, 10)
        self.create_subscription(Imu, '/carla/ego_vehicle/imu', self.imu_callback, 10)
        self.create_subscription(Float32, '/carla/ego_vehicle/speed', self.speed_callback, 10)
        self.create_subscription(PoseArray, '/carla/ego_vehicle/global_plan', self.plan_callback, 10)

        # Publisher
        self.control_pub = self.create_publisher(CarlaEgoVehicleControl, '/carla/ego_vehicle/vehicle_control_cmd', 10)

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        # 디버깅용
        self.fps_log = []
        self.inference_times = []

    def image_callback(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        key = msg.header.frame_id.split('/')[-1].replace('image', '').upper()
        self.input_data[key] = (msg.header.stamp, img)

    def bev_callback(self, msg):
        bev = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        bev = cv2.cvtColor(bev, cv2.COLOR_BGR2RGB)
        self.input_data['bev'] = (msg.header.stamp, bev)

    def gps_callback(self, msg):
        self.input_data['GNSS'] = (msg.header.stamp, [msg.latitude, msg.longitude, msg.altitude])

    def imu_callback(self, msg):
        compass = math.atan2(2.0 * (msg.orientation.w * msg.orientation.z + msg.orientation.x * msg.orientation.y),
                            1.0 - 2.0 * (msg.orientation.y**2 + msg.orientation.z**2))
        self.input_data['IMU'] = (msg.header.stamp, [compass])

    def speed_callback(self, msg):
        self.input_data['SPEED'] = (msg.header.stamp, {'speed': msg.data})

    def plan_callback(self, msg):
        self.global_plan = [{'lat': pose.position.x, 'lon': pose.position.y, 'z': 0.0} for pose in msg.poses]
        if not self.initialized and self.global_plan:
            self._init()

    def timer_callback(self):
        self.process_data()

    def process_data(self):
        # required_keys = ['CAM_FRONT', 'CAM_FRONT_LEFT', 'CAM_FRONT_RIGHT', 'GNSS', 'SPEED', 'IMU']
        required_keys = ['GNSS', 'SPEED', 'IMU']
        if not all(k in self.input_data for k in required_keys):
            return

        if not self.initialized:
            self._init()

        self.step += 1
        start_time = time.time()

        tick_data = self.tick(self.input_data)
        if self.step < self.config.seq_len:
            control = CarlaEgoVehicleControl()
            self.control_pub.publish(control)
            return

        rgb = self.im_transform(tick_data['rgb']).unsqueeze(0).to('cuda')
        speed = torch.FloatTensor([tick_data['speed']]).view(1, 1).to('cuda') / 12
        target_point = torch.stack([
            torch.FloatTensor([tick_data['target_point'][0]]),
            torch.FloatTensor([tick_data['target_point'][1]])
        ], dim=1).to('cuda')
        cmd = tick_data['next_command']
        if cmd < 0:
            cmd = 4
        cmd -= 1
        cmd_one_hot = torch.zeros(1, 6).to('cuda')
        cmd_one_hot[0, cmd] = 1
        state = torch.cat([speed, target_point, cmd_one_hot], 1)

        inference_start = time.time()
        pred = self.net(rgb, state, target_point)
        inference_end = time.time()
        self.inference_times.append(inference_end - inference_start)

        steer, throttle, brake, _ = self.net.process_action(pred, tick_data['next_command'],
                                                          torch.FloatTensor([tick_data['speed']]).to('cuda'),
                                                          target_point)

        control = CarlaEgoVehicleControl()
        control.steer = float(steer)
        control.throttle = float(throttle)
        control.brake = float(brake)
        self.control_pub.publish(control)

        if self.debug_mode:
            self.save(tick_data)

        frame_time = time.time() - start_time
        fps = 1.0 / frame_time if frame_time > 0 else 0
        self.fps_log.append(fps)
        self.get_logger().info(f"Step: {self.step}, FPS: {fps:.2f}, Inference Time: {(inference_end - inference_start)*1000:.2f}ms")

        self.input_data = {}  # 다음 주기를 위해 초기화

    def tick(self, input_data):
        front_img = input_data['CAM_FRONT'][1][:, 200:1400]
        front_left_img = input_data['CAM_FRONT_LEFT'][1][:, :1400]
        front_right_img = input_data['CAM_FRONT_RIGHT'][1][:, 200:]
        gps = input_data['GPS'][1][:2]
        speed = input_data['SPEED'][1]['speed']
        compass = input_data['IMU'][1][0] or 0.0
        bev = input_data.get('bev', None)

        rgb = np.concatenate((front_left_img, front_img, front_right_img), axis=1)
        rgb = torch.from_numpy(rgb).permute(2, 0, 1).unsqueeze(0).float()
        rgb = torch.nn.functional.interpolate(rgb, size=(256, 900), mode='bilinear', align_corners=False)
        rgb = rgb.squeeze(0).permute(1, 2, 0).byte().numpy()

        result = {'rgb': rgb, 'gps': gps, 'speed': speed, 'compass': compass}
        if bev is not None:
            result['bev'] = bev[1]
        pos = self.gps_to_location(result['gps'])
        result['gps'] = pos
        next_wp, next_cmd = self._route_planner.run_step(pos)
        result['next_command'] = next_cmd.value
        theta = compass - np.pi / 2
        R = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
        local_command_point = np.array([next_wp[0] - pos[0], next_wp[1] - pos[1]])
        local_command_point = R.dot(local_command_point)
        result['target_point'] = tuple(local_command_point)
        return result

    def save(self, tick_data):
        frame = self.step
        os.makedirs(f"{self.save_path}/bev", exist_ok=True)
        if 'bev' in tick_data:
            Image.fromarray(tick_data['bev']).save(f"{self.save_path}/bev/{frame:04d}.png")

    def _init(self):
        if self.global_plan is None:
            self.global_plan = [{'lat': 0.0, 'lon': 0.0, 'z': 0.0}]
        self.route_planner.set_route(self.global_plan, True)
        self.initialized = True

    def gps_to_location(self, gps):
        lat, lon = gps
        scale = math.cos(self.lat_ref * math.pi / 180.0)
        my = math.log(math.tan((lat + 90) * math.pi / 360.0)) * (6378137.0 * scale)
        mx = (lon * (math.pi * 6378137.0 * scale)) / 180.0
        y = scale * 6378137.0 * math.log(math.tan((90.0 + self.lat_ref) * math.pi / 360.0)) - my
        x = mx - scale * self.lon_ref * math.pi * 6378137.0 / 180.0
        return np.array([x, y])

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ckpt-path", type=str, default='./../Bench2DriveZoo/ckpts/tcp_b2d.ckpt',
                        help="Path to checkpoint")
    parser.add_argument("--save-path", type=str, default='./eval_v1',
                        help="Path to save debugging results")
    parser.add_argument("--debug", type=bool, default=False,
                        help="Run with debug output")
    args = parser.parse_args()

    rclpy.init()
    node = TCPAgentNode(ckpt_path=args.ckpt_path, save_path=args.save_path, debug_mode=args.debug)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()