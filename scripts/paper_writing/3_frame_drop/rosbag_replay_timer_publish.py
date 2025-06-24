#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sqlite3
import sys
import argparse
import termios
import tty
import select
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage, NavSatFix, Imu
from carla_msgs.msg import CarlaEgoVehicleStatus, CarlaGnssRoute
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy

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

def find_input_sets(all_msgs, count=INPUT_SET_COUNT, min_gap_nsec=1e9):
    sets = []
    used_timestamps = set()
    gps_msgs = sorted(all_msgs['/carla/hero/GPS'], key=lambda x: x[0])
    last_ts = 0
    for timestamp, _ in gps_msgs:
        if timestamp - last_ts < min_gap_nsec:
            continue
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
            last_ts = timestamp
            if len(sets) >= count:
                break
    return sets

def get_msg_type_str(msg_cls):
    return msg_cls.__module__.replace('.', '/') + '/' + msg_cls.__name__

def get_keypress():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class RosbagTimerPublisher(Node):
    def __init__(self, db3_path, hz, repeat):
        super().__init__('rosbag_timer_publisher')
        self.all_msgs = fetch_messages_from_db3(db3_path)
        self.msg_sets = find_input_sets(self.all_msgs)
        self.total_sets = len(self.msg_sets)
        self.repeat = repeat
        self.hz = hz
        self.current_idx = 0
        self.started = False

        self._pubs = {
            topic: self.create_publisher(
                TOPIC_TYPES[topic], topic,
                QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                if topic == '/carla/hero/global_plan_gps' else 1
            )
            for topic in TOPIC_TYPES
        }

        self.publish_global_plan()
        self.get_logger().info(f'총 {self.total_sets}개의 입력 세트를 준비했습니다.')
        self.get_logger().info('스페이스 바를 누르면 발행을 시작합니다. (q로 종료)')

        self.timer = self.create_timer(1.0 / self.hz, self.timer_callback)
        self.keypress_timer = self.create_timer(0.1, self.wait_for_start)

    def publish_global_plan(self):
        route_msgs = self.all_msgs['/carla/hero/global_plan_gps']
        if not route_msgs:
            self.get_logger().warn('No global_plan_gps found in rosbag.')
            return
        ts, data = route_msgs[0]
        msg_type = get_message(get_msg_type_str(TOPIC_TYPES['/carla/hero/global_plan_gps']))
        msg = deserialize_message(data, msg_type)
        self._pubs['/carla/hero/global_plan_gps'].publish(msg)
        self.get_logger().info('Published global plan from rosbag.')

    def wait_for_start(self):
        key = get_keypress()
        if key == ' ':
            self.started = True
            self.get_logger().info(f'{self.hz}Hz로 {self.repeat}회 발행을 시작합니다.')
            self.destroy_timer(self.keypress_timer)
        elif key == 'q':
            self.get_logger().info('사용자에 의해 종료됨.')
            rclpy.shutdown()

    def timer_callback(self):
        if not self.started:
            return
        if self.current_idx >= self.repeat:
            self.get_logger().info('지정된 횟수 발행 완료. 종료합니다.')
            rclpy.shutdown()
            return
        msg_set = self.msg_sets[self.current_idx % self.total_sets]
        for topic, (ts, data) in msg_set.items():
            msg_type = get_message(get_msg_type_str(TOPIC_TYPES[topic]))
            msg = deserialize_message(data, msg_type)
            if hasattr(msg, 'header'):
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
            self._pubs[topic].publish(msg)
        self.get_logger().info(f'[{self.current_idx+1}] 입력 세트 발행')
        self.current_idx += 1

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    parser = argparse.ArgumentParser()
    parser.add_argument('db3', help='입력 rosbag db3 경로')
    parser.add_argument('--hz', type=float, default=10.0, help='발행 주기 (Hz)')
    parser.add_argument('--repeat', type=int, default=50, help='총 반복 횟수')
    args = parser.parse_args()

    rclpy.init()
    node = RosbagTimerPublisher(args.db3, args.hz, args.repeat)
    rclpy.spin(node)
