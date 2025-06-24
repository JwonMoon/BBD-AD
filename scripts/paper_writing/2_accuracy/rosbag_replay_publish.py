#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sqlite3
import sys
import termios
import tty
import select
from std_msgs.msg import Header

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import CompressedImage, NavSatFix, Imu
from carla_msgs.msg import CarlaEgoVehicleStatus, CarlaGnssRoute

TOPIC_TYPES = {
    '/image/compressed': CompressedImage,
    '/image_left/compressed': CompressedImage,
    '/image_right/compressed': CompressedImage,
    '/carla/hero/GPS': NavSatFix,
    '/carla/hero/IMU': Imu,
    '/carla/hero/vehicle_status': CarlaEgoVehicleStatus,
    '/carla/hero/global_plan_gps': CarlaGnssRoute
}
TARGET_TOPICS = list(TOPIC_TYPES.keys())[:-1]
INPUT_SET_COUNT = 20

def get_keypress():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def fetch_messages_from_db3(db3_path):
    con = sqlite3.connect(db3_path)
    cur = con.cursor()
    cur.execute("SELECT id, name FROM topics")
    topic_map = dict(cur.fetchall())
    cur.execute("SELECT topic_id, timestamp, data FROM messages")

    all_msgs = {topic: [] for topic in TOPIC_TYPES}
    for topic_id, timestamp, data in cur.fetchall():
        topic_name = topic_map.get(topic_id, None)
        if topic_name in all_msgs:
            all_msgs[topic_name].append((timestamp, data))
    con.close()
    return all_msgs

def find_closest_sets(all_msgs):
    sets = []
    used_timestamps = set()
    gps_msgs = sorted(all_msgs['/carla/hero/GPS'], key=lambda x: x[0])

    prev_ts = None  # 직전에 사용한 GPS timestamp

    for timestamp, _ in gps_msgs:
        if prev_ts is not None and abs(timestamp - prev_ts) < 1e9:
            continue  # 1초 안 지나면 skip

        candidate = {}
        is_valid = True
        for topic in TARGET_TOPICS:
            msgs = sorted(all_msgs[topic], key=lambda x: abs(x[0] - timestamp))
            if not msgs:
                is_valid = False
                break
            closest = msgs[0]
            if closest[0] in used_timestamps:
                is_valid = False
                break
            candidate[topic] = closest

        if is_valid:
            for msg in candidate.values():
                used_timestamps.add(msg[0])
            sets.append(candidate)
            prev_ts = timestamp  # 다음 반복을 위한 기준 갱신
            if len(sets) >= INPUT_SET_COUNT:
                break

    return sets

def get_msg_type_str(msg_cls):
    return msg_cls.__module__.replace('.', '/') + '/' + msg_cls.__name__

class RosbagReplayer(Node):
    def __init__(self, db3_path):
        super().__init__('rosbag_replayer')
        self.all_msgs = fetch_messages_from_db3(db3_path)
        self.msg_sets = find_closest_sets(self.all_msgs)
        self.current_idx = 0

        # QoS profiles
        # best_effort_qos = QoSProfile(
        #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #     durability=DurabilityPolicy.VOLATILE,
        #     depth=1
        # )
        transient_local_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Create publishers with appropriate QoS
        self._pubs = {}
        for topic in TOPIC_TYPES:
            # if topic in [
            #     '/image/compressed', '/image_left/compressed', '/image_right/compressed',
            #     '/carla/hero/GPS', '/carla/hero/IMU', '/carla/hero/vehicle_status'
            # ]:
            #     self._pubs[topic] = self.create_publisher(TOPIC_TYPES[topic], topic, best_effort_qos)
            if topic == '/carla/hero/global_plan_gps':
                self._pubs[topic] = self.create_publisher(TOPIC_TYPES[topic], topic, transient_local_qos)
            else:
                self._pubs[topic] = self.create_publisher(TOPIC_TYPES[topic], topic, 1)

        self.publish_global_plan_from_rosbag()
        self.get_logger().info('Ready. Press space to publish each input set.')
        self.timer = self.create_timer(0.1, self.loop)

    def publish_global_plan_from_rosbag(self):
        gps_route_msgs = self.all_msgs['/carla/hero/global_plan_gps']
        if not gps_route_msgs:
            self.get_logger().warn('No global_plan_gps found in rosbag.')
            return
        ts, data = gps_route_msgs[0]
        msg_type = get_message(get_msg_type_str(TOPIC_TYPES['/carla/hero/global_plan_gps']))
        msg = deserialize_message(data, msg_type)
        self._pubs['/carla/hero/global_plan_gps'].publish(msg)
        self.get_logger().info('Published global plan from rosbag.')

    def loop(self):
        key = get_keypress()
        if key == ' ' and self.current_idx < len(self.msg_sets):
            msg_set = self.msg_sets[self.current_idx]
            for topic, (ts, data) in msg_set.items():
                msg_type = get_message(get_msg_type_str(TOPIC_TYPES[topic]))
                msg = deserialize_message(data, msg_type)
                if hasattr(msg, 'header'):
                    msg.header = Header()
                    msg.header.stamp = self.get_clock().now().to_msg()
                self._pubs[topic].publish(msg)
            self.get_logger().info(f'Published input set {self.current_idx + 1}')
            self.current_idx += 1
        elif key == 'q':
            self.get_logger().info('Exiting...')
            rclpy.shutdown()

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    if len(sys.argv) < 2:
        print("Usage: rosbag_replay_publish_qos.py <path_to_rosbag.db3>")
        sys.exit(1)
    node = RosbagReplayer(sys.argv[1])
    rclpy.spin(node)
