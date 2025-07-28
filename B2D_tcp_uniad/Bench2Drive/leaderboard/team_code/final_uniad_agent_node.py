#!/usr/bin/env python

import os
import json
import datetime
import pathlib
import time
import cv2
from collections import deque
import math
from collections import OrderedDict
import torch
import numpy as np
from PIL import Image as PILImage
from torchvision import transforms as T
from scipy.optimize import fsolve

#uniad
from pid_controller import PIDController #jw
# from Bench2DriveZoo.team_code.pid_controller import PIDController
from planner import RoutePlanner #jw
# from Bench2DriveZoo.team_code.planner import RoutePlanner
# from leaderboard.autoagents import autonomous_agent
from mmcv import Config
from mmcv.models import build_model
from mmcv.utils import (get_dist_info, init_dist, load_checkpoint,wrap_fp16_model)
from mmcv.datasets.pipelines import Compose
from mmcv.parallel.collate import collate as  mm_collate_to_batch_form
from mmcv.core.bbox import get_box_type
from pyquaternion import Quaternion

#jw
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, NavSatFix, Imu, CompressedImage
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaRoute, CarlaEgoVehicleStatus, CarlaGnssRoute
from tf_transformations import euler_from_quaternion
from bbd_msgs.msg import BBDBranchOutput
import csv

SAVE_PATH = os.environ.get('SAVE_PATH', None)
PLANNER_TYPE = os.environ.get('PLANNER_TYPE', None)

EARTH_RADIUS_EQUA = 6378137.0

class UniADAgentNode(Node):
    #jw) setup() -> __init__()
    def __init__(self, config_ckpt_path, save_path, debug_mode, img_input):
        super().__init__('uniad_agent_node')

        #uniad
        self.pidcontroller = PIDController() 
        self.config_path = config_ckpt_path.split('+')[0]
        self.ckpt_path = config_ckpt_path.split('+')[1]
        # self.save_path = save_path if save_path else '/tmp/uniad_agent'
        self.debug_mode = debug_mode
        self.img_input = img_input if img_input else 'raw'
        self.step = 0
        self.initialized = False
        self.try_proc_num = 0

        #uniad
        cfg = Config.fromfile(self.config_path)
        cfg.model['motion_head']['anchor_info_path'] = os.path.join('Bench2DriveZoo',cfg.model['motion_head']['anchor_info_path'])
        if hasattr(cfg, 'plugin'):
            if cfg.plugin:
                import importlib
                if hasattr(cfg, 'plugin_dir'):
                    plugin_dir = cfg.plugin_dir
                    plugin_dir = os.path.join("Bench2DriveZoo", plugin_dir)
                    _module_dir = os.path.dirname(plugin_dir)
                    _module_dir = _module_dir.split('/')
                    _module_path = _module_dir[0]
                    for m in _module_dir[1:]:
                        _module_path = _module_path + '.' + m
                    print(_module_path)
                    plg_lib = importlib.import_module(_module_path) 
                    
        self.model = build_model(cfg.model, train_cfg=cfg.get('train_cfg'), test_cfg=cfg.get('test_cfg')) #jw) partitioning: base_e2e 안에 model 변경하기
        checkpoint = load_checkpoint(self.model, self.ckpt_path, map_location='cpu', strict=True)
        
        self.model.cuda()
        self.model.eval()

        self.inference_only_pipeline = []
        for inference_only_pipeline in cfg.inference_only_pipeline:
            if inference_only_pipeline["type"] not in ['LoadMultiViewImageFromFilesInCeph']:
                self.inference_only_pipeline.append(inference_only_pipeline)
        self.inference_only_pipeline = Compose(self.inference_only_pipeline)
        self.takeover = False
        # self.stop_time = 0
        # self.takeover_time = 0

        self._im_transform = T.Compose([
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        # self.last_steers = deque()

        self.lat_ref, self.lon_ref = 42.0, 2.0
        # self.lat_ref, lon_ref = 0.0 #tcp
        self._route_planner = None

        control = BBDBranchOutput()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 0.0	
        self.prev_control = control

        # write extrinsics directly
        self.lidar2img = {
        'CAM_FRONT':np.array([[ 1.14251841e+03,  8.00000000e+02,  0.00000000e+00, -9.52000000e+02],
                                  [ 0.00000000e+00,  4.50000000e+02, -1.14251841e+03, -8.09704417e+02],
                                  [ 0.00000000e+00,  1.00000000e+00,  0.00000000e+00, -1.19000000e+00],
                                 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]),
          'CAM_FRONT_LEFT':np.array([[ 6.03961325e-14,  1.39475744e+03,  0.00000000e+00, -9.20539908e+02],
                                   [-3.68618420e+02,  2.58109396e+02, -1.14251841e+03, -6.47296750e+02],
                                   [-8.19152044e-01,  5.73576436e-01,  0.00000000e+00, -8.29094072e-01],
                                   [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]),
          'CAM_FRONT_RIGHT':np.array([[ 1.31064327e+03, -4.77035138e+02,  0.00000000e+00,-4.06010608e+02],
                                       [ 3.68618420e+02,  2.58109396e+02, -1.14251841e+03,-6.47296750e+02],
                                    [ 8.19152044e-01,  5.73576436e-01,  0.00000000e+00,-8.29094072e-01],
                                    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]]),
         'CAM_BACK':np.array([[-5.60166031e+02, -8.00000000e+02,  0.00000000e+00, -1.28800000e+03],
                     [ 5.51091060e-14, -4.50000000e+02, -5.60166031e+02, -8.58939847e+02],
                     [ 1.22464680e-16, -1.00000000e+00,  0.00000000e+00, -1.61000000e+00],
                     [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]),
        'CAM_BACK_LEFT':np.array([[-1.14251841e+03,  8.00000000e+02,  0.00000000e+00, -6.84385123e+02],
                                  [-4.22861679e+02, -1.53909064e+02, -1.14251841e+03, -4.96004706e+02],
                                  [-9.39692621e-01, -3.42020143e-01,  0.00000000e+00, -4.92889531e-01],
                                  [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]),
  
        'CAM_BACK_RIGHT': np.array([[ 3.60989788e+02, -1.34723223e+03,  0.00000000e+00, -1.04238127e+02],
                                    [ 4.22861679e+02, -1.53909064e+02, -1.14251841e+03, -4.96004706e+02],
                                    [ 9.39692621e-01, -3.42020143e-01,  0.00000000e+00, -4.92889531e-01],
                                    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
        }
        self.lidar2cam = {
        'CAM_FRONT':np.array([[ 1.  ,  0.  ,  0.  ,  0.  ],
                                 [ 0.  ,  0.  , -1.  , -0.24],
                                 [ 0.  ,  1.  ,  0.  , -1.19],
                              [ 0.  ,  0.  ,  0.  ,  1.  ]]),
        'CAM_FRONT_LEFT':np.array([[ 0.57357644,  0.81915204,  0.  , -0.22517331],
                                      [ 0.        ,  0.        , -1.  , -0.24      ],
                                   [-0.81915204,  0.57357644,  0.  , -0.82909407],
                                   [ 0.        ,  0.        ,  0.  ,  1.        ]]),
          'CAM_FRONT_RIGHT':np.array([[ 0.57357644, -0.81915204, 0.  ,  0.22517331],
                                   [ 0.        ,  0.        , -1.  , -0.24      ],
                                   [ 0.81915204,  0.57357644,  0.  , -0.82909407],
                                   [ 0.        ,  0.        ,  0.  ,  1.        ]]),
        'CAM_BACK':np.array([[-1. ,  0.,  0.,  0.  ],
                             [ 0. ,  0., -1., -0.24],
                             [ 0. , -1.,  0., -1.61],
                             [ 0. ,  0.,  0.,  1.  ]]),
     
        'CAM_BACK_LEFT':np.array([[-0.34202014,  0.93969262,  0.  , -0.25388956],
                                  [ 0.        ,  0.        , -1.  , -0.24      ],
                                  [-0.93969262, -0.34202014,  0.  , -0.49288953],
                                  [ 0.        ,  0.        ,  0.  ,  1.        ]]),
  
        'CAM_BACK_RIGHT':np.array([[-0.34202014, -0.93969262,  0.  ,  0.25388956],
                                  [ 0.        ,  0.         , -1.  , -0.24      ],
                                  [ 0.93969262, -0.34202014 ,  0.  , -0.49288953],
                                  [ 0.        ,  0.         ,  0.  ,  1.        ]])
        }
        self.lidar2ego = np.array([[ 0. ,  1. ,  0. , -0.39],
                                   [-1. ,  0. ,  0. ,  0.  ],
                                   [ 0. ,  0. ,  1. ,  1.84],
                                   [ 0. ,  0. ,  0. ,  1.  ]])
        
        topdown_extrinsics =  np.array([[0.0, -0.0, -1.0, 50.0], [0.0, 1.0, -0.0, 0.0], [1.0, -0.0, 0.0, -0.0], [0.0, 0.0, 0.0, 1.0]])
        unreal2cam = np.array([[0,1,0,0], [0,0,-1,0], [1,0,0,0], [0,0,0,1]])
        self.coor2topdown = unreal2cam @ topdown_extrinsics
        topdown_intrinsics = np.array([[548.993771650447, 0.0, 256.0, 0], [0.0, 548.993771650447, 256.0, 0], [0.0, 0.0, 1.0, 0], [0, 0, 0, 1.0]])
        self.coor2topdown = topdown_intrinsics @ self.coor2topdown

        #jw)
        self.rgb_front = None
        self.rgb_front_left = None
        self.rgb_front_right = None
        self.rgb_back = None
        self.rgb_back_left = None
        self.rgb_back_right = None
        self.gps = [0.0, 0.0]
        self.compass = 0.0
        self.acceleration = 0.0
        self.angular_velocity = 0.0
        self.speed = 0.0
        self._global_plan = None
        self._global_plan_gps = None

        self.gps_received = False
        self.imu_received = False
        self.speed_received = False
        self.global_plan_gps_received = False
        
        # self.pid_metadata = {} 
        
        if self.img_input == 'raw':
            self.create_subscription(Image, '/carla/hero/CAM_FRONT/image', self.image_front_callback, 1)
            self.create_subscription(Image, '/carla/hero/CAM_FRONT_LEFT/image', self.image_front_left_callback, 1)
            self.create_subscription(Image, '/carla/hero/CAM_FRONT_RIGHT/image', self.image_front_right_callback, 1)
            self.create_subscription(Image, '/carla/hero/CAM_BACK/image', self.image_back_callback, 1)
            self.create_subscription(Image, '/carla/hero/CAM_BACK_LEFT/image', self.image_back_left_callback, 1)
            self.create_subscription(Image, '/carla/hero/CAM_BACK_RIGHT/image', self.image_back_right_callback, 1)
        elif self.img_input == 'compressed':
            self.create_subscription(CompressedImage, '/image_front/compressed', self.compressed_image_front_callback, 1)
            self.create_subscription(CompressedImage, '/image_front_left/compressed', self.compressed_image_front_left_callback, 1)
            self.create_subscription(CompressedImage, '/image_front_right/compressed', self.compressed_image_front_right_callback, 1)
            self.create_subscription(CompressedImage, '/image_back/compressed', self.compressed_image_back_callback, 1)
            self.create_subscription(CompressedImage, '/image_back_left/compressed', self.compressed_image_back_left_callback, 1)
            self.create_subscription(CompressedImage, '/image_back_right/compressed', self.compressed_image_back_right_callback, 1)
        else:
            print("img_input type is wrong !")

        self.create_subscription(NavSatFix, '/carla/hero/GPS', self.gps_callback, 1)
        self.create_subscription(Imu, '/carla/hero/IMU', self.imu_callback, 1)
        self.create_subscription(CarlaEgoVehicleStatus, '/carla/hero/vehicle_status', self.vehicle_status_callback, 1)
        self.create_subscription(CarlaGnssRoute, '/carla/hero/global_plan_gps', self.global_plan_gps_callback, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.control_pub = self.create_publisher(BBDBranchOutput, '/uniad/vehicle_control_cmd', QoSProfile(depth=1))

        if self.debug_mode > 0 and SAVE_PATH:
            now = datetime.datetime.now()
            string = f"uniad_agent_{now.strftime('%m_%d_%H_%M_%S')}"
            self.save_path = pathlib.Path(SAVE_PATH) / string
            self.save_path.mkdir(parents=True, exist_ok=True)

            # (self.save_path / 'meta').mkdir()
            self.log_file = self.save_path / 'uniad_timing.csv'
            with open(self.log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'step', 'try_proc_num', 
                    'T_proc_start',
                    # 'T_t_start', 'T_t_end', 
                    # 'T_pp_start', 'T_pp_end', 
                    # 'T_net_start', 
                    
                    # 'T_net_perception',
                    # 'T_net_speed_branch',
                    # 'T_net_measurements',
                    # 'T_net_join_traj',
                    # 'T_net_branch_traj',
                    # 'T_net_pred_wp',
                    # 'T_net_init_att',
                    # 'T_net_join_ctrl',
                    # 'T_net_branch_ctrl',
                    # 'T_net_action_head',
                    # 'T_net_future_feature_action',
                    
                    # 'T_net_end', 
                    # 'T_pid_start', 'T_pid_end', 
                    # 'T_gen_msg_start', 'T_gen_msg_end', 
                    'T_pub_start', 'T_pub_end', 
                    # 'T_log_start', 'T_log_end'
                ])

            if self.debug_mode > 2:
                (self.save_path / 'rgb_front').mkdir()

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

    def image_back_callback(self, msg):
        # self.get_logger().info(f"[image_back_callback] step {self.step}")
        self.rgb_back = self.decode_image(msg)
        self.try_process_step()
        # self.get_logger().info(f"[image_back_callback] finished, step {self.step}")

    def image_back_left_callback(self, msg):
        # self.get_logger().info(f"[image_back_left_callback] step {self.step}")
        self.rgb_back_left = self.decode_image(msg)
        self.try_process_step()
        # self.get_logger().info(f"[image_back_left_callback] finished, step {self.step}")

    def image_back_right_callback(self, msg):
        # self.get_logger().info(f"[image_back_right_callback] step {self.step}")
        self.rgb_back_right = self.decode_image(msg)
        self.try_process_step()
        # self.get_logger().info(f"[image_back_right_callback] finished, step {self.step}")

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

    def compressed_image_back_callback(self, msg):
        # self.get_logger().info(f"[compressed_image_back_callback] step {self.step}")
        self.rgb_back = self.decode_image(msg)
        self.try_process_step()
        # self.get_logger().info(f"[compressed_image_back_callback] finished, step {self.step}")

    def compressed_image_back_left_callback(self, msg):
        # self.get_logger().info(f"[compressed_image_back_left_callback] step {self.step}")
        self.rgb_back_left = self.decode_image(msg)
        self.try_process_step()
        # self.get_logger().info(f"[compressed_image_back_left_callback] finished, step {self.step}")

    def compressed_image_back_right_callback(self, msg):
        # self.get_logger().info(f"[compressed_image_back_right_callback] step {self.step}")
        self.rgb_back_right = self.decode_image(msg)
        self.try_process_step()
        # self.get_logger().info(f"[compressed_image_back_right_callback] finished, step {self.step}")

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

        self.compass = yaw  # yaw in radians
        # self.acceleration = msg.linear_acceleration
        self.acceleration = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ], dtype=np.float32)
        # self.angular_velocity = msg.angular_velocity
        self.angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=np.float32)
        self.imu_received = True
        self.try_process_step()
        # self.get_logger().info(f"[imu_callback] finished, step {self.step}, compass={self.compass}") #debug

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
        #
        self._route_planner = RoutePlanner(4.0, 50.0, lat_ref=self.lat_ref, lon_ref=self.lon_ref)
        # self._route_planner.set_route(self._global_plan, False)
        self._route_planner.set_route(self._global_plan_gps, True)
        self.initialized = True

    def try_process_step(self):
        if self.initialized and self.rgb_front is not None and self.rgb_front_left is not None and self.rgb_front_right is not None and \
           self.rgb_back is not None and self.rgb_back_left is not None and self.rgb_back_right is not None and self.gps_received and self.imu_received and self.speed_received:
            # print(f"let's try process_step(), step {self.step}")
            self.process_step()
            # print(f"process_step() returned, step {self.step}")
            
            self.rgb_front = None
            self.rgb_front_left = None
            self.rgb_front_right = None
            self.rgb_back = None
            self.rgb_back_left = None
            self.rgb_back_right = None
            self.gps_received = False
            self.imu_received = False
            self.speed_received = False
            self.try_proc_num = 0
        else:
            self.try_proc_num += 1
            if self.debug_mode > 1:
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
                if self.rgb_back is None:
                    print(f"rgb_back = {self.rgb_back is not None}")
                if self.rgb_back_left is None:
                    print(f"rgb_back_left = {self.rgb_back_left is not None}")
                if self.rgb_back_right is None:
                    print(f"rgb_back_right = {self.rgb_back_right is not None}")
                if not self.gps_received:
                    print(f"gps_received = {self.gps_received}")
                if not self.imu_received:
                    print(f"imu_received = {self.imu_received}")
                if not self.speed_received:
                    print(f"speed_received = {self.speed_received}")
                if not self.global_plan_gps_received:
                    print(f"global_plan_gps_received = {self.global_plan_gps_received}")
                print("-----------------------------------------------")

    def tick(self):
        self.step += 1
        
        if self.debug_mode > 1:
            self.get_logger().info(f"[UniADBackboneNode] Tick called, GPS={self.gps}, step={self.step}") #debug
        if not all([self.rgb_front is not None, self.rgb_front_left is not None, self.rgb_front_right is not None, self.rgb_back is not None, self.rgb_back_left is not None, self.rgb_back_right is not None, self.initialized]):
            self.get_logger().warning(f"- tick(): No rgb_front or not initialized at step {self.step}")
            return None

        img_data = [self.rgb_front, self.rgb_front_left, self.rgb_front_right, self.rgb_back, self.rgb_back_left, self.rgb_back_right]
        cams = ['CAM_FRONT', 'CAM_FRONT_LEFT', 'CAM_FRONT_RIGHT', 'CAM_BACK', 'CAM_BACK_LEFT', 'CAM_BACK_RIGHT']
        imgs = dict(zip(cams, img_data))

		# bev = cv2.cvtColor(input_data['bev'][1][:, :, :3], cv2.COLOR_BGR2RGB)
        gps = self.gps_to_location(self.gps)
        # print(f"[DEBUG] Agent GPS to local: x={gps[0]}, y={gps[1]}")
        speed = self.speed
        compass = self.compass
        acceleration = self.acceleration
        angular_velocity = self.angular_velocity
        # pos = self.gps_to_location(gps)
        # near_node, near_command = self._route_planner.run_step(pos)
        near_node, near_command = self._route_planner.run_step(gps)

        result = {
            'imgs': imgs,
            'gps': gps,
            # 'pos':pos,
            'speed': speed,
            'compass': compass,
            # 'bev': bev,
            'acceleration':acceleration,
            'angular_velocity':angular_velocity,
            'command_near':near_command,
            'command_near_xy':near_node
        }
        return result

    @torch.no_grad()
    def process_step(self):
        T_proc_start = time.time()
        ros_time_ns = self.get_clock().now().nanoseconds # ROS2 시계 기준 시간 기록

        if self.debug_mode > 1:
            self.get_logger().warning(f"Process step called, step={self.step}")

        # STEP 1: tick
        # T_t_start = time.time()
        tick_data = self.tick()
        # T_t_end = time.time()

        # STEP 2: vehicle status organizing
        results = {}
        results['lidar2img'] = []
        results['lidar2cam'] = []
        results['img'] = []
        results['folder'] = ' '
        results['scene_token'] = ' '  
        results['frame_idx'] = 0
        results['timestamp'] = self.step / 20
        results['box_type_3d'], _ = get_box_type('LiDAR')
  
        for cam in ['CAM_FRONT','CAM_FRONT_LEFT','CAM_FRONT_RIGHT','CAM_BACK','CAM_BACK_LEFT','CAM_BACK_RIGHT']:
            results['lidar2img'].append(self.lidar2img[cam])
            results['lidar2cam'].append(self.lidar2cam[cam])
            results['img'].append(tick_data['imgs'][cam])
        results['lidar2img'] = np.stack(results['lidar2img'],axis=0)
        results['lidar2cam'] = np.stack(results['lidar2cam'],axis=0)

        raw_theta = tick_data['compass']   if not np.isnan(tick_data['compass']) else 0
        ego_theta = -raw_theta + np.pi/2
        rotation = list(Quaternion(axis=[0, 0, 1], radians=ego_theta))

        can_bus = np.zeros(18)
        # can_bus[0] = tick_data['pos'][0]
        # can_bus[1] = -tick_data['pos'][1]
        can_bus[3:7] = rotation
        can_bus[7] = tick_data['speed']
        can_bus[10:13] = tick_data['acceleration']
        can_bus[11] *= -1
        can_bus[13:16] = -tick_data['angular_velocity']
        can_bus[16] = ego_theta
        can_bus[17] = ego_theta / np.pi * 180 
        results['can_bus'] = can_bus
        command = tick_data['command_near']
        if command < 0:
            command = 4
        command -= 1
        results['command'] = command

        theta_to_lidar = raw_theta
        command_near_xy = np.array([tick_data['command_near_xy'][0]-can_bus[0],-tick_data['command_near_xy'][1]-can_bus[1]])
        rotation_matrix = np.array([[np.cos(theta_to_lidar),-np.sin(theta_to_lidar)],[np.sin(theta_to_lidar),np.cos(theta_to_lidar)]])
        local_command_xy = rotation_matrix @ command_near_xy

        ego2world = np.eye(4)
        ego2world[0:3,0:3] = Quaternion(axis=[0, 0, 1], radians=ego_theta).rotation_matrix
        ego2world[0:2,3] = can_bus[0:2]
        lidar2global = ego2world @ self.lidar2ego
        results['l2g_r_mat'] = lidar2global[0:3,0:3]
        results['l2g_t'] = lidar2global[0:3,3]
        stacked_imgs = np.stack(results['img'],axis=-1)
        results['img_shape'] = stacked_imgs.shape
        results['ori_shape'] = stacked_imgs.shape
        results['pad_shape'] = stacked_imgs.shape
        results = self.inference_only_pipeline(results)
        self.device="cuda"

        # STEP 3: making input data
        # T_pp_start = time.time()
        input_data_batch = mm_collate_to_batch_form([results], samples_per_gpu=1)
        for key, data in input_data_batch.items():
            if key != 'img_metas':
                if torch.is_tensor(data[0]):
                    data[0] = data[0].to(self.device)
        # T_pp_end = time.time()
        
        # STEP 4: model inference
        # self.get_logger().warning(f"- process_step(): model()")
        # T_net_start = time.time()
        output_data_batch = self.model(input_data_batch, return_loss=False, rescale=True)
        out_truck =  output_data_batch[0]['planning']['result_planning']['sdc_traj'][0].cpu().numpy()
        # T_net_end = time.time()
        
        # STEP 3: PID 계산
        # self.get_logger().warning(f"- process_step(): control_pid()")
        # T_pid_start = time.time()
        steer_traj, throttle_traj, brake_traj, metadata_traj = self.pidcontroller.control_pid(out_truck, tick_data['speed'], local_command_xy)
        if brake_traj < 0.05: brake_traj = 0.0
        if throttle_traj > brake_traj: brake_traj = 0.0
        if tick_data['speed']>5:
            throttle_traj = 0
        # self.get_logger().info(f"[DEBUG] steer_traj: {steer_traj}, throttle_traj: {throttle_traj}, brake_traj: {brake_traj}, metadata_traj: {metadata_traj}")
        # T_pid_end = time.time()
        
        # STEP 4: 제어 명령 생성 및 publish
        # self.get_logger().warning(f"- process_step(): generate control")
        control = BBDBranchOutput()
        # self.pid_metadata = metadata_traj
        # self.pid_metadata['agent'] = 'only_traj'
        control.steer = np.clip(float(steer_traj), -1, 1)
        control.throttle = np.clip(float(throttle_traj), 0, 0.75)
        control.brake = np.clip(float(brake_traj), 0, 1)
        # self.pid_metadata['steer'] = control.steer
        # self.pid_metadata['throttle'] = control.throttle
        # self.pid_metadata['brake'] = control.brake
        # self.pid_metadata['steer_traj'] = float(steer_traj)
        # self.pid_metadata['throttle_traj'] = float(throttle_traj)
        # self.pid_metadata['brake_traj'] = float(brake_traj)
        # self.pid_metadata['plan'] = out_truck.tolist()
        # metric_info = self.get_metric_info()
        # self.metric_info[self.step] = metric_info
        # if SAVE_PATH is not None and self.step % 1 == 0:
        #     self.save(tick_data)
        self.prev_control = control
        control.step = self.step

        self.get_logger().warning(f"[PUB CONTROL] step={self.step}, steer={control.steer}, throttle={control.throttle}, brake={control.brake}") #debug

        T_pub_start = time.time()
        self.control_pub.publish(control)
        T_pub_end = time.time()

        # STEP 5: 결과 저장
        if self.debug_mode > 0:
            # T_log_start = time.time()
            # self.pid_metadata = {
            #     'step': self.step,
            #     'ros_time_ns': ros_time_ns,
            #     'tick_ms': (T_t_end - T_t_start) * 1000,
            #     'preprocess_ms': (T_pp_end - T_pp_start) * 1000,
            #     'net_inference_ms': (T_net_end - T_net_start) * 1000,
            #     'pid_calc_ms': (T_pid_end - T_pid_start) * 1000,
            #     'gen_msg_ms': (T_pub_start - T_pid_end) * 1000,
            #     'publish_ms': (T_pub_end - T_pub_start) * 1000,
            #     'total_process_step_ms': (T_pub_end - T_proc_start) * 1000,

            #     'steer_ctrl': float(steer_ctrl),
            #     'steer_traj': float(steer_traj),
            #     'throttle_ctrl': float(throttle_ctrl),
            #     'throttle_traj': float(throttle_traj),
            #     'brake_ctrl': float(brake_ctrl),
            #     'brake_traj': float(brake_traj),
            #     'steer': control.steer,
            #     'throttle': control.throttle,
            #     'brake': control.brake,
            # }
            
            if SAVE_PATH and self.step % 1 == 0 and self.debug_mode > 2:
                self.save(tick_data)

            # timing log 저장
            if self.log_file:
                with open(self.log_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        self.step, self.try_proc_num,
                        T_proc_start,
                        # T_t_start, T_t_end,
                        # T_pp_start, T_pp_end,
                        # T_net_start, 
                        
                        # pred['timing']['perception'],
                        # pred['timing']['speed_branch'],
                        # pred['timing']['measurements'],
                        # pred['timing']['join_traj'],
                        # pred['timing']['branch_traj'],
                        # pred['timing']['pred_wp'],
                        # pred['timing']['init_att'],
                        # pred['timing']['join_ctrl'],
                        # pred['timing']['branch_ctrl'],
                        # pred['timing']['action_head'],
                        # pred['timing']['future_feature_action'],
                        
                        # T_net_end,
                        # T_pid_start, T_pid_end,
                        # T_pid_end, T_pub_start, # gen msg time
                        T_pub_start, T_pub_end,
                        # T_log_start, time.time()
                    ])
            
            # if self.debug_mode > 1: 
            #     T_log_end = time.time()
            #     self.get_logger().warning(f"[UniADAgentNode] Process step finished, step={self.step}")

    def save(self, tick_data):
        frame = self.step
        PILImage.fromarray(tick_data['rgb_front']).save(self.save_path / 'rgb_front' / (f'%04d.png' % frame))
        # with open(self.save_path / 'meta' / (f'%04d.json' % frame), 'w') as outfile:
        #     json.dump(self.pid_metadata, outfile, indent=4)

    def gps_to_location(self, gps):
        lat, lon = gps
        scale = math.cos(self.lat_ref * math.pi / 180.0)
        my = math.log(math.tan((lat + 90) * math.pi / 360.0)) * (EARTH_RADIUS_EQUA * scale)
        mx = (lon * (math.pi * EARTH_RADIUS_EQUA * scale)) / 180.0
        y = scale * EARTH_RADIUS_EQUA * math.log(math.tan((90.0 + self.lat_ref) * math.pi / 360.0)) - my
        x = mx - scale * self.lon_ref * math.pi * EARTH_RADIUS_EQUA / 180.0
        return np.array([x, y])

    def destroy(self):
        del self.model
        torch.cuda.empty_cache()
        self.get_logger().info("UniAD Agent Node destroyed")
        
def main():
    import argparse
    parser = argparse.ArgumentParser(description='UniAD Agent Node for CARLA')
    parser.add_argument('--config-ckpt-path', required=True, help='Path to model config and checkpoint')
    parser.add_argument('--save-path', default=None, help='Path to save debug outputs')
    parser.add_argument('--debug-mode', type=int, default=0, help='Level of debug mode')
    parser.add_argument('--img-input', default='raw', help='Type for input camera image')
    args = parser.parse_args()

    rclpy.init()
    node = UniADAgentNode(args.config_ckpt_path, args.save_path, args.debug_mode, args.img_input)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down UniAD Agent Node")
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
