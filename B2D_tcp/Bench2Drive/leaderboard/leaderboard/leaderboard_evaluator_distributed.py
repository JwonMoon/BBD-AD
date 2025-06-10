#!/usr/bin/env python
# Copyright (c) 2018-2019 Intel Corporation.
# authors: German Ros (german.ros@intel.com), Felipe Codevilla (felipe.alcm@gmail.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
CARLA Challenge Evaluator Routes

Provisional code to evaluate Autonomous Agents for the CARLA Autonomous Driving challenge
"""
from __future__ import print_function

import traceback
import argparse
from argparse import RawTextHelpFormatter
from distutils.version import LooseVersion
import importlib
import os
import pkg_resources
import sys
import carla
import signal

import socket
import subprocess
import time
from datetime import datetime
# jw
from xml.etree import ElementTree as ET

from srunner.scenariomanager.carla_data_provider import *
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog

from leaderboard.scenarios.scenario_manager_distributed import ScenarioManager
from leaderboard.scenarios.route_scenario import RouteScenario
from leaderboard.envs.sensor_interface import SensorConfigurationInvalid
from leaderboard.autoagents.agent_wrapper import AgentError, validate_sensor_configuration, TickRuntimeError
from leaderboard.utils.statistics_manager import StatisticsManager, FAILURE_MESSAGES
from leaderboard.utils.route_indexer import RouteIndexer
# from leaderboard.utils.route_manipulation import downsample_route # 다운샘플링용
from leaderboard.autoagents.ros_base_agent import BridgeHelper, ROSBaseAgent
from leaderboard.autoagents.autonomous_agent import AutonomousAgent

# jw
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaRoute, CarlaGnssRoute
# from carla_msgs.srv import SpawnObject, DestroyObject
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import NavSatFix
from tcp_msgs.msg import TickTrigger, TCPBranchOutput
import pathlib, csv

import atexit

SAVE_PATH = os.environ.get('SAVE_PATH', None)

sensors_to_icons = {
    'sensor.camera.rgb':        'carla_camera',
    'sensor.lidar.ray_cast':    'carla_lidar',
    'sensor.other.radar':       'carla_radar',
    'sensor.other.gnss':        'carla_gnss',
    'sensor.other.imu':         'carla_imu',
    'sensor.opendrive_map':     'carla_opendrive_map',
    'sensor.speedometer':       'carla_speedometer'
}

import socket

def find_free_port(starting_port):
    port = starting_port
    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(("localhost", port))
                return port
        except OSError:
            port += 1

def get_weather_id(weather_conditions):
    from xml.etree import ElementTree as ET
    tree = ET.parse('leaderboard/data/weather.xml')
    root = tree.getroot()
    def conditions_match(weather, conditions):
        for (key, value) in weather:
            if key == 'route_percentage' : continue
            if str(getattr(conditions, key))!= value:
                return False
        return True
    for case in root.findall('case'):
        weather = case[0].items()
        if conditions_match(weather, weather_conditions):
            return case.items()[0][1]
    return None

class EvaluatorAgent(Node, ROSBaseAgent):
    ROS_VERSION = 2

    def __init__(self, carla_host, carla_port, debug=False):
        # rclpy 초기화
        rclpy.init(args=None)
        # ROSBaseAgent.__init__(self, self.ROS_VERSION, carla_host, carla_port, debug)
        # ROSBaseAgent의 브릿지만 초기화
        AutonomousAgent.__init__(self, carla_host, carla_port, debug)  # ROSBaseAgent.__init__ 건너뜀
        Node.__init__(self, 'evaluator_node')

        self._control_subscriber = self.create_subscription(TCPBranchOutput, '/tcp/vehicle_control_cmd', self._vehicle_control_cmd_callback, QoSProfile(depth=1))
        self._control_subscriber = self.create_subscription(TickTrigger, '/tcp/tick_trigger', self._tick_trigger_callback, QoSProfile(depth=1))
        
        self._path_publisher = self.create_publisher(CarlaRoute, "/carla/hero/global_plan", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._path_gps_publisher = self.create_publisher(CarlaGnssRoute, "/carla/hero/global_plan_gps", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
         
        self._scenario_manager = None
        self._control = None
        self._is_agent_ready = False

        if debug and SAVE_PATH:
            now = datetime.now()
            string = f"tcp_agent_{now.strftime('%m_%d_%H_%M_%S')}"
            self.save_path = pathlib.Path(SAVE_PATH) / string
            self.save_path.mkdir(parents=True, exist_ok=True)
            
            # timing
            self.log_file_bb = self.save_path / 'evaluator_backbone_cb_timing.csv'
            with open(self.log_file_bb, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'step', 'T_bb_cb_start', 'T_car_tick_start', 'T_car_tick_end', 'T_bb_cb_end' # jw changed
                ])
            self.log_file_br = self.save_path / 'evaluator_branch_cb_timing.csv'
            with open(self.log_file_br, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'step', 'T_br_cb_start', 'T_car_ctrl_start', 'T_car_ctrl_end', 'T_br_cb_end'
                ])

    def sensors(self): #original
	    return [
				{
					'type': 'sensor.camera.rgb',
					'x': 0.80, 'y': 0.0, 'z': 1.60,
					'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
					# 'width': 1600, 'height': 900, 'fov': 70,
					# 'width': 1440, 'height': 810, 'fov': 70, #0.9
					# 'width': 1280, 'height': 720, 'fov': 70, #0.8
					# 'width': 1120, 'height': 630, 'fov': 70, #0.7
					'width': 1040, 'height': 585, 'fov': 70, #0.65
					# 'width': 960, 'height': 540, 'fov': 70, #0.6
					# 'width': 800, 'height': 450, 'fov': 70, #0.5
					# 'width': 480, 'height': 270, 'fov': 70, #0.3
					# 'width': 160, 'height': 90, 'fov': 70, #0.1
					'id': 'CAM_FRONT'
					},
				{
					'type': 'sensor.camera.rgb',
					'x': 0.27, 'y': -0.55, 'z': 1.60,
					'roll': 0.0, 'pitch': 0.0, 'yaw': -55.0,
					# 'width': 1600, 'height': 900, 'fov': 70,
					# 'width': 1440, 'height': 810, 'fov': 70, #0.9
					# 'width': 1280, 'height': 720, 'fov': 70, #0.8
					# 'width': 1120, 'height': 630, 'fov': 70, #0.7
					'width': 1040, 'height': 585, 'fov': 70, #0.65
					# 'width': 960, 'height': 540, 'fov': 70, #0.6
					# 'width': 800, 'height': 450, 'fov': 70, #0.5
					# 'width': 480, 'height': 270, 'fov': 70, #0.3
					# 'width': 160, 'height': 90, 'fov': 70, #0.1
					'id': 'CAM_FRONT_LEFT'
					},
				{
					'type': 'sensor.camera.rgb',
					'x': 0.27, 'y': 0.55, 'z': 1.60,
					'roll': 0.0, 'pitch': 0.0, 'yaw': 55.0,
					# 'width': 1600, 'height': 900, 'fov': 70,
					# 'width': 1440, 'height': 810, 'fov': 70, #0.9
					# 'width': 1280, 'height': 720, 'fov': 70, #0.8
					# 'width': 1120, 'height': 630, 'fov': 70, #0.7
					'width': 1040, 'height': 585, 'fov': 70, #0.65
					# 'width': 960, 'height': 540, 'fov': 70, #0.6
					# 'width': 800, 'height': 450, 'fov': 70, #0.5
					# 'width': 480, 'height': 270, 'fov': 70, #0.3
					# 'width': 160, 'height': 90, 'fov': 70, #0.1
					'id': 'CAM_FRONT_RIGHT'
					},
				# imu
				{
					'type': 'sensor.other.imu',
					'x': -1.4, 'y': 0.0, 'z': 0.0,
					'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
					'sensor_tick': 0.05,
					'id': 'IMU'
					},
				# gps
				{
					'type': 'sensor.other.gnss',
					'x': -1.4, 'y': 0.0, 'z': 0.0,
					'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
					'sensor_tick': 0.01,
					'id': 'GPS'
					}]
				# # speed
				# {
				# 	'type': 'sensor.speedometer',
				# 	'reading_frequency': 20,
				# 	'id': 'SPEED'
				# 	},
                # {	
                #     'type': 'sensor.camera.rgb',
                #     'x': 0.0, 'y': 0.0, 'z': 50.0,
                #     'roll': 0.0, 'pitch': -90.0, 'yaw': 0.0,
                #     'width': 512, 'height': 512, 'fov': 5 * 10.0,
                #     'id': 'bev'
                # }]
    
    #ros2_agent function
    @staticmethod
    def get_ros_version():
        return EvaluatorAgent.ROS_VERSION
    
    def destroy(self):
        self.get_logger().info("Destroying EvaluatorAgent")
        # super().destroy()
        super(AutonomousAgent, self).destroy()  # ROSBaseAgent의 destroy 건너뛰기
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    
    def set_global_plan(self, global_plan_gps, global_plan_world_coord):
        super(EvaluatorAgent, self).set_global_plan(global_plan_gps, global_plan_world_coord)
        print(f"Global plan set with global_plan_gps / len: {len(global_plan_gps)} waypoints")
        print(f"Global plan set with global_plan_world_coord / len : {len(global_plan_world_coord)} waypoints")

        path = CarlaRoute()
        for wp in global_plan_world_coord:  # 직접 입력 사용
            path.road_options.append(wp[1])
            pose = BridgeHelper.carla2ros_pose(
                wp[0].location.x, wp[0].location.y, wp[0].location.z,
                wp[0].rotation.roll, wp[0].rotation.pitch, wp[0].rotation.yaw,
                to_quat=True
            )
            path.poses.append(
                Pose(position=Point(**pose["position"]), orientation=Quaternion(**pose["orientation"])))
        self._path_publisher.publish(path)
        print(f"Publishing global plan: poses={len(path.poses)}, road_options={len(path.road_options)}")
 
        path_gps = CarlaGnssRoute()
        for wp in global_plan_gps:  # 직접 입력 사용
            path_gps.road_options.append(wp[1])
            path_gps.coordinates.append(
                NavSatFix(latitude=wp[0]["lat"], longitude=wp[0]["lon"], altitude=wp[0]["z"]))
        self._path_gps_publisher.publish(path_gps)
        print(f"Publishing global plan gps: poses={len(path_gps.coordinates)}, road_options={len(path_gps.road_options)}")
        
    def set_scenario_manager(self, scenario_manager):
        self._scenario_manager = scenario_manager

    #ros_base_agent fuction
    def run_step(self):
        timeout = 1.0
        start_time = time.time()
        while self._control is None and (time.time() - start_time) < timeout:
            # print(f"@@ [run_setp] get sleep, self._control={self._control}")
            rclpy.spin_once(self, timeout_sec=0.001)

        if self._control:
            control = self._control
            self._control = None
            return control
        else:
            print("[EvaluatorAgent] No control received in time. Returning default VehicleControl.")
            return carla.VehicleControl()  # 기본적으로 멈춘 상태

        # #jw) ros node
        # while self._control is None:
        #     rclpy.spin_once(self, timeout_sec=0.01)

        # control = self._control
        # self._control = None
        # return control

    def _tick_trigger_callback(self, msg):
        if self._is_agent_ready is False:
            print(f"[Evaluator ROS2] set _is_agent_ready True, step={msg.step}")
            self._is_agent_ready = True
        if msg.trigger and self._scenario_manager is not None:
            print(f"[Evaluator ROS2] Received tick trigger, step={msg.step}")
            try:
                T_bb_cb_start, T_car_tick_start, T_car_tick_end, T_bb_cb_end = self._scenario_manager._tick_simulation(msg.step)
            except Exception as e:
                print(f"tick_callback error: {e}")
        
        # timing log 저장
        if self.log_file_bb:
            with open(self.log_file_bb, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    msg.step,
                    T_bb_cb_start, T_car_tick_start, T_car_tick_end, T_bb_cb_end
                ])

    
    def _vehicle_control_cmd_callback(self, msg):
        # T_br_cb_start = time.time()
        self._control = carla.VehicleControl(
            steer=msg.steer, 
            throttle=msg.throttle, 
            brake=msg.brake,
            hand_brake=msg.hand_brake, 
            reverse=msg.reverse,
            manual_gear_shift=msg.manual_gear_shift, 
            gear=msg.gear
        )
        if self._scenario_manager is not None:
            print(f"[Evaluator ROS2] Received control_cmd")
            try:
                T_br_cb_start, T_car_ctrl_satrt, T_car_ctrl_end, T_br_cb_end = self._scenario_manager._apply_control(self._control, msg.step)
            except Exception as e:
                print(f"tick_callback error: {e}")
        # T_br_cb_end = time.time()

        # timing log 저장
        if self.log_file_br:
            with open(self.log_file_br, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    msg.step,
                    T_br_cb_start, T_car_ctrl_satrt, T_car_ctrl_end, T_br_cb_end
                ])

class LeaderboardEvaluator(object):
    """
    Main class of the Leaderboard. Everything is handled from here,
    from parsing the given files, to preparing the simulation, to running the route.
    """

    # Tunable parameters
    client_timeout = 300.0  # in seconds
    # frame_rate = 20.0      # in Hz

    def __init__(self, args, statistics_manager):
        """
        Setup CARLA client and world
        Setup ScenarioManager
        """
        self.world = None
        self.manager = None
        self.sensors = None
        self.sensors_initialized = False
        self.sensor_icons = []
        self.agent_instance = None
        self.route_scenario = None
        self.frame_rate = 10.0      # in Hz #jw

        self.statistics_manager = statistics_manager

        # Setup the simulation
        self.client, self.client_timeout, self.traffic_manager = self._setup_simulation(args)

        dist = pkg_resources.get_distribution("carla")
        if dist.version != 'leaderboard':
            if LooseVersion(dist.version) < LooseVersion('0.9.10'):
                raise ImportError("CARLA version 0.9.10.1 or newer required. CARLA version found: {}".format(dist))

        # Load agent 
        #jw)
        # module_name = os.path.basename(args.agent).split('.')[0]
        # sys.path.insert(0, os.path.dirname(args.agent))
        # self.module_agent = importlib.import_module(module_name)

        # Create the ScenarioManager
        self.manager = ScenarioManager(args.timeout, self.statistics_manager, args.debug)

        # Time control for summary purposes
        self._start_time = GameTime.get_time()
        self._end_time = None

        # Prepare the agent timer
        self._agent_watchdog = None
        signal.signal(signal.SIGINT, self._signal_handler)

        self._client_timed_out = False

    def _signal_handler(self, signum, frame):
        """
        Terminate scenario ticking when receiving a signal interrupt.
        Either the agent initialization watchdog is triggered, or the runtime one at scenario manager
        """
        if self._agent_watchdog and not self._agent_watchdog.get_status():
            raise RuntimeError("Timeout: Agent took longer than {}s to setup".format(self.client_timeout))
        elif self.manager:
            self.manager.signal_handler(signum, frame)

    def __del__(self):
        """
        Cleanup and delete actors, ScenarioManager and CARLA world
        """
        if hasattr(self, 'manager') and self.manager:
            del self.manager
        if hasattr(self, 'world') and self.world:
            del self.world

    def _get_running_status(self):
        """
        returns:
           bool: False if watchdog exception occured, True otherwise
        """
        if self._agent_watchdog:
            return self._agent_watchdog.get_status()
        return False

    def _cleanup(self):
        """
        Remove and destroy all actors
        """
        CarlaDataProvider.cleanup()

        if self._agent_watchdog:
            self._agent_watchdog.stop()

        try:
            if self.agent_instance:
                self.agent_instance.destroy()
                self.agent_instance = None
        except Exception as e:
            print("\n\033[91mFailed to stop the agent:", flush=True)
            print(f"\n{traceback.format_exc()}\033[0m", flush=True)

        if self.route_scenario:
            self.route_scenario.remove_all_actors()
            self.route_scenario = None
            if self.statistics_manager:
                self.statistics_manager.remove_scenario()

        if self.manager:
            self._client_timed_out = not self.manager.get_running_status()
            self.manager.cleanup()

        # Make sure no sensors are left streaming
        alive_sensors = self.world.get_actors().filter('*sensor*')
        for sensor in alive_sensors:
            sensor.stop()
            sensor.destroy()

    def _setup_simulation(self, args):
        """
        Prepares the simulation by getting the client, and setting up the world and traffic manager settings
        """
        # self.carla_path = os.environ["CARLA_ROOT"]
        # args.port = find_free_port(args.port)
        # cmd1 = f"{os.path.join(self.carla_path, 'CarlaUE4.sh')} -RenderOffScreen -nosound -carla-rpc-port={args.port} -graphicsadapter={args.gpu_rank}"
        # self.server = subprocess.Popen(cmd1, shell=True, preexec_fn=os.setsid)
        # print(cmd1, self.server.returncode, flush=True)
        # atexit.register(os.killpg, self.server.pid, signal.SIGKILL)
        # time.sleep(30)
            
        attempts = 0
        num_max_restarts = 20
        while attempts < num_max_restarts:
            try:
                client = carla.Client(args.host, args.port)
                if args.timeout:
                    client_timeout = args.timeout
                client.set_timeout(client_timeout)

                settings = carla.WorldSettings(
                    synchronous_mode = True,
                    fixed_delta_seconds = 1.0 / self.frame_rate,
                    deterministic_ragdolls = True,
                    spectator_as_ego = False #jw
                )
                client.get_world().apply_settings(settings)
                print(f"load_world success , attempts={attempts}", flush=True)
                break
            except Exception as e:
                print(f"load_world failed , attempts={attempts}", flush=True)
                print(e, flush=True)
                attempts += 1
                time.sleep(5)
        attempts = 0
        num_max_restarts = 40
        while attempts < num_max_restarts:
            try:
                args.traffic_manager_port = find_free_port(args.traffic_manager_port)
                traffic_manager = client.get_trafficmanager(args.traffic_manager_port)
                traffic_manager.set_synchronous_mode(True)
                traffic_manager.set_hybrid_physics_mode(True)
                print(f"traffic_manager init success, try_time={attempts}", flush=True)
                break
            except Exception as e:
                print(f"traffic_manager init fail, try_time={attempts}", flush=True)
                print(e, flush=True)
                attempts += 1
                time.sleep(5)
        return client, client_timeout, traffic_manager

    def _reset_world_settings(self):
        """
        Changes the modified world settings back to asynchronous
        """
        # Has simulation failed?
        if self.world and self.manager and not self._client_timed_out:
            # Reset to asynchronous mode
            self.world.tick()  # TODO: Make sure all scenario actors have been destroyed
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            settings.deterministic_ragdolls = False
            settings.spectator_as_ego = True
            self.world.apply_settings(settings)

            # Make the TM back to async
            self.traffic_manager.set_synchronous_mode(False)
            self.traffic_manager.set_hybrid_physics_mode(False)

    def _load_and_wait_for_world(self, args, town):
        """
        Load a new CARLA world without changing the settings and provide data to CarlaDataProvider
        """
        self.world = self.client.load_world(town, reset_settings=False)

        # Large Map settings are always reset, for some reason
        settings = self.world.get_settings()
        settings.tile_stream_distance = 650
        settings.actor_active_distance = 650
        self.world.apply_settings(settings)

        self.world.reset_all_traffic_lights()
        CarlaDataProvider.set_client(self.client)
        CarlaDataProvider.set_traffic_manager_port(args.traffic_manager_port)
        CarlaDataProvider.set_world(self.world)

        # This must be here so that all route repetitions use the same 'unmodified' seed
        self.traffic_manager.set_random_device_seed(args.traffic_manager_seed)

        # Wait for the world to be ready
        self.world.tick()

        map_name = CarlaDataProvider.get_map().name.split("/")[-1]
        if map_name != town:
            raise Exception("The CARLA server uses the wrong map!"
                            " This scenario requires the use of map {}".format(town))

    def _register_statistics(self, route_index, entry_status, crash_message=""):
        """
        Computes and saves the route statistics
        """
        print("\033[1m> Registering the route statistics\033[0m", flush=True)
        self.statistics_manager.save_entry_status(entry_status)
        self.statistics_manager.compute_route_statistics(
            route_index, self.manager.scenario_duration_system, self.manager.scenario_duration_game, crash_message
        )

    def _load_and_run_scenario(self, args, config):
        """
        Load and run the scenario given by config.

        Depending on what code fails, the simulation will either stop the route and
        continue from the next one, or report a crash and stop.
        """
        crash_message = ""
        entry_status = "Started"

        print("\n\033[1m========= Preparing {} (repetition {}) =========\033[0m".format(config.name, config.repetition_index), flush=True)

        # Prepare the statistics of the route
        route_name = f"{config.name}_rep{config.repetition_index}"
        scenario_name = config.scenario_configs[0].name
        town_name = str(config.town)
        weather_id = get_weather_id(config.weather[0][1])
        currentDateAndTime = datetime.now()
        currentTime = currentDateAndTime.strftime("%m_%d_%H_%M_%S")
        save_name = f"{route_name}_{town_name}_{scenario_name}_{weather_id}_{currentTime}"
        self.statistics_manager.create_route_data(route_name, scenario_name, weather_id, save_name, town_name, config.index)

        print("\033[1m> Loading the world\033[0m", flush=True)

        # Load the world and the scenario
        try:
            self._load_and_wait_for_world(args, config.town)
            self.route_scenario = RouteScenario(world=self.world, config=config, debug_mode=args.debug)
            self.statistics_manager.set_scenario(self.route_scenario)

        except Exception:
            # The scenario is wrong -> set the ejecution to crashed and stop
            print("\n\033[91mThe scenario could not be loaded:", flush=True)
            print(f"\n{traceback.format_exc()}\033[0m", flush=True)

            entry_status, crash_message = FAILURE_MESSAGES["Simulation"]
            self._register_statistics(config.index, entry_status, crash_message)
            self._cleanup()
            return True

        print("\033[1m> Setting up the agent\033[0m", flush=True)

        # Set up the user's agent, and the timer to avoid freezing the simulation
        try:
            self._agent_watchdog = Watchdog(args.timeout)
            self._agent_watchdog.start()

            # agent_class_name = getattr(self.module_agent, 'get_entry_point')()
            # agent_class_obj = getattr(self.module_agent, agent_class_name)

            # self.agent_instance = agent_class_obj(args.host, args.port, args.debug)
            # self.agent_instance.set_global_plan(self.route_scenario.gps_route, self.route_scenario.route)
            # args.agent_config = args.agent_config + '+' + save_name
            # self.agent_instance.setup(args.agent_config)
            #jw)
            self.agent_instance = EvaluatorAgent(args.host, args.port, args.debug > 0)
            
            # print("RouteScenario.gps_route:", self.route_scenario.gps_route)
            # print("RouteScenario.route:", self.route_scenario.route)
            self.agent_instance.set_global_plan(self.route_scenario.gps_route, self.route_scenario.route)
            self.agent_instance.set_scenario_manager(self.manager)  # jw) change run_step()

            # Check and store the sensors
            if not self.sensors:
                self.sensors = self.agent_instance.sensors()
                track = self.agent_instance.track

                validate_sensor_configuration(self.sensors, track, args.track)

                self.sensor_icons = [sensors_to_icons[sensor['type']] for sensor in self.sensors]
                self.statistics_manager.save_sensors(self.sensor_icons)
                self.statistics_manager.write_statistics()

                self.sensors_initialized = True
            
            self._agent_watchdog.stop()
            self._agent_watchdog = None

        except SensorConfigurationInvalid as e:
            # The sensors are invalid -> set the ejecution to rejected and stop
            print("\n\033[91mThe sensor's configuration used is invalid:", flush=True)
            print(f"{e}\033[0m\n", flush=True)

            entry_status, crash_message = FAILURE_MESSAGES["Sensors"]
            self._register_statistics(config.index, entry_status, crash_message)
            self._cleanup()
            return True

        except Exception as e:
            # The agent setup has failed -> start the next route
            print("\n\033[91mCould not set up the required agent:", flush=True)
            print(f"\n{traceback.format_exc()}\033[0m", flush=True)
            print(f"{e}\033[0m\n", flush=True)

            entry_status, crash_message = FAILURE_MESSAGES["Agent_init"]
            self._register_statistics(config.index, entry_status, crash_message)
            self._cleanup()
            return True

        print("\033[1m> Running the route\033[0m", flush=True)

        # Run the scenario
        try:
            # Load scenario and run it
            if args.record:
                self.client.start_recorder("{}/{}_rep{}.log".format(args.record, config.name, config.repetition_index))
                
            print("\033[1m>>> load_scenario\033[0m", flush=True)
            self.manager.load_scenario(self.route_scenario, self.agent_instance, config.index, config.repetition_index)
            self.manager.tick_count = 0
            

            print(f"\033[1m>>> >>> self.agent_instance._is_agent_ready: {self.agent_instance._is_agent_ready}\033[0m", flush=True)
            print("\033[1m>>> run_scenario\033[0m", flush=True)
            self.manager.run_scenario()
            # jw) orin-warm-up
            while True:
                if self.agent_instance._is_agent_ready is True:
                    break
                else:
                    # ROS 콜백 처리를 위해 spin_once 호출
                    rclpy.spin_once(self.agent_instance, timeout_sec=0.01)
                    # print("\033[1m>>> >>> carla tick before run_scenario\033[0m", flush=True)
                    CarlaDataProvider.get_world().tick()
                    time.sleep(0.5)
            rclpy.spin(self.agent_instance)

        except AgentError:
            # The agent has failed -> stop the route
            print("\n\033[91mStopping the route, the agent has crashed:", flush=True)
            print(f"\n{traceback.format_exc()}\033[0m")

            entry_status, crash_message = FAILURE_MESSAGES["Agent_runtime"]

        except KeyboardInterrupt:
            return True
        
        except TickRuntimeError:
            entry_status, crash_message = "Started", "TickRuntime"
        
        except Exception:
            print("\n\033[91mError during the simulation:", flush=True)
            print(f"\n{traceback.format_exc()}\033[0m", flush=True)

            entry_status, crash_message = FAILURE_MESSAGES["Simulation"]

        # Stop the scenario
        try:
            print("\033[1m> Stopping the route\033[0m", flush=True)
            self.manager.stop_scenario()
            self._register_statistics(config.index, entry_status, crash_message)

            if args.record:
                self.client.stop_recorder()

            self._cleanup()

        except Exception:
            print("\n\033[91mFailed to stop the scenario, the statistics might be empty:", flush=True)
            print(f"\n{traceback.format_exc()}\033[0m", flush=True)

            _, crash_message = FAILURE_MESSAGES["Simulation"]

        # If the simulation crashed, stop the leaderboard, for the rest, move to the next route
        return crash_message == "Simulation crashed"

    def run(self, args):
        """
        Run the challenge mode
        """
        route_indexer = RouteIndexer(args.routes, args.repetitions, args.routes_subset)

        if args.resume:
            resume = route_indexer.validate_and_resume(args.checkpoint)
        else:
            resume = False

        if resume:
            self.statistics_manager.add_file_records(args.checkpoint)
        else:
            self.statistics_manager.clear_records()
        self.statistics_manager.save_progress(route_indexer.index, route_indexer.total)
        self.statistics_manager.write_statistics()

        crashed = False
        t1 = time.time()
        while route_indexer.peek() and not crashed:

            # Run the scenario
            config = route_indexer.get_next_config()
            crashed = self._load_and_run_scenario(args, config)
            print("--crashed:", crashed, flush=True)
            # Save the progress and write the route statistics
            self.statistics_manager.save_progress(route_indexer.index, route_indexer.total)
            self.statistics_manager.write_statistics()
            if crashed:
                print(f'{route_indexer.index} crash, [{route_indexer.index}/{route_indexer.total}], please restart', flush=True)
                break

        # Go back to asynchronous mode
        self._reset_world_settings()

        if not crashed:
            # Save global statistics
            print(f"cost time={time.time()-t1}", flush=True)
            print("\033[1m> Registering the global statistics\033[0m", flush=True)
            self.statistics_manager.compute_global_statistics()
            self.statistics_manager.validate_and_write_statistics(self.sensors_initialized, crashed)
        
        if crashed:
            cmd2 = "ps -ef | grep '-graphicsadapter="+ str(args.gpu_rank) + "' | grep -v grep | awk '{print $2}' | xargs -r kill -9"
            server = subprocess.Popen(cmd2, shell=True, preexec_fn=os.setsid)
            atexit.register(os.killpg, server.pid, signal.SIGKILL)

        return crashed

def main():
    description = "CARLA AD Leaderboard Evaluation: evaluate your Agent in CARLA scenarios\n"

    # general parameters
    parser = argparse.ArgumentParser(description=description, formatter_class=RawTextHelpFormatter)
    parser.add_argument('--host', default='localhost',
                        help='IP of the host server (default: localhost)')
    parser.add_argument('--port', default=2000, type=int,
                        help='TCP port to listen to (default: 2000)')
    parser.add_argument('--traffic-manager-port', default=8000, type=int,
                        help='Port to use for the TrafficManager (default: 8000)')
    parser.add_argument('--traffic-manager-seed', default=0, type=int,
                        help='Seed used by the TrafficManager (default: 0)')
    parser.add_argument('--debug', type=int,
                        help='Run with debug output', default=0)
    parser.add_argument('--record', type=str, default='',
                        help='Use CARLA recording feature to create a recording of the scenario')
    parser.add_argument('--timeout', default=100.0, type=float,
                        help='Set the CARLA client timeout value in seconds')

    # simulation setup
    parser.add_argument('--routes', required=True,
                        help='Name of the routes file to be executed.')
    parser.add_argument('--routes-subset', default='', type=str,
                        help='Execute a specific set of routes')
    parser.add_argument('--repetitions', type=int, default=1,
                        help='Number of repetitions per route.')

    # agent-related options
    parser.add_argument("-a", "--agent", type=str,
                        help="Path to Agent's py file to evaluate", required=True)
    parser.add_argument("--agent-config", type=str,
                        help="Path to Agent's configuration file", default="")

    parser.add_argument("--track", type=str, default='SENSORS',
                        help="Participation track: SENSORS, MAP")
    parser.add_argument('--resume', type=bool, default=False,
                        help='Resume execution from last checkpoint?')
    parser.add_argument("--checkpoint", type=str, default='./simulation_results.json',
                        help="Path to checkpoint used for saving statistics and resuming")
    parser.add_argument("--debug-checkpoint", type=str, default='./live_results.txt',
                        help="Path to checkpoint used for saving live results")
    parser.add_argument("--gpu-rank", type=int, default=0)
    parser.add_argument('--save-path', default=None, help='Path to save debug outputs')
    
    arguments = parser.parse_args()

    statistics_manager = StatisticsManager(arguments.checkpoint, arguments.debug_checkpoint)
    leaderboard_evaluator = LeaderboardEvaluator(arguments, statistics_manager)
    crashed = leaderboard_evaluator.run(arguments)

    del leaderboard_evaluator

    if crashed:
        sys.exit(-1)
    else:
        sys.exit(0)

if __name__ == '__main__':
    main()
