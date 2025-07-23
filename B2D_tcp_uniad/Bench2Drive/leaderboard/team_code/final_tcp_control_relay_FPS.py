#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tcp_msgs.msg import TCPBranchOutput
import socket
import json
import threading
import pathlib
import csv
import time
from datetime import datetime
from rclpy.qos import QoSProfile
import msgpack  # 추가: msgpack 라이브러리 임포트

SAVE_PATH = pathlib.Path('./../../jw_ws/dual_eval/')
DEBUG_MODE = 1

class TCPControlRelay(Node):
    def __init__(self, port=9998):
        super().__init__('tcp_control_relay')
        self.control_pub = self.create_publisher(TCPBranchOutput, '/tcp/vehicle_control_cmd', QoSProfile(depth=1))

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.listen(1)
        self.get_logger().info(f"[Relay] Waiting for TCP connection on port {port}...")

        self.conn, addr = self.sock.accept()
        self.get_logger().info(f"[Relay] Connected to {addr}")

        if DEBUG_MODE > 0:
            now = datetime.now()
            log_dir = SAVE_PATH / f"tcp_relay_{now.strftime('%m_%d_%H_%M_%S')}"
            log_dir.mkdir(parents=True, exist_ok=True)
            # self.meta_path = log_dir / "meta_relay"
            # self.meta_path.mkdir(exist_ok=True)
            self.log_file = log_dir / "relay_timing.csv"
            with open(self.log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "step", "T_rl_rx_start", 
                    # "T_rl_msg_ready", 
                    "T_rl_pub_start", "T_rl_pub_end"
                ])
        else:
            # self.meta_path = None
            self.log_file = None

        self.recv_thread = threading.Thread(target=self.recv_loop, daemon=True)
        self.recv_thread.start()

    def recv_loop(self):
        while rclpy.ok():
            try:
                T_rl_rx_start = time.time()

                header = self.conn.recv(4)
                if not header:
                    continue
                msg_len = int.from_bytes(header, byteorder='big')
                data = b''
                while len(data) < msg_len:
                    packet = self.conn.recv(msg_len - len(data))
                    if not packet:
                        break
                    data += packet

                T_rl_msg_ready = time.time()
                # msg_dict = json.loads(data.decode('utf-8'))
                msg_dict = msgpack.unpackb(data, raw=False)  # msgpack 역직렬화

                msg = TCPBranchOutput()
                msg.step = msg_dict['step']
                msg.steer = float(msg_dict['steer'])
                msg.throttle = float(msg_dict['throttle'])
                msg.brake = float(msg_dict['brake'])

                T_rl_pub_start = time.time()
                self.control_pub.publish(msg)
                T_rl_pub_end = time.time()

                # self.get_logger().info(f"[Relay] Published control step={msg.step}, steer={msg.steer:.3f}, throttle={msg.throttle:.3f}, brake={msg.brake:.3f}")

                # Logging
                if DEBUG_MODE > 0:
                    # JSON 저장
                    # with open(self.meta_path / f'{msg.step:04d}.json', 'w') as f:
                    #     json.dump({
                    #         'step': msg.step,
                    #         'steer': msg.steer,
                    #         'throttle': msg.throttle,
                    #         'brake': msg.brake,
                    #         'T_rl_rx_start': T_rx_start,
                    #         'T_rl_msg_ready': T_msg_ready,
                    #         'T_rl_pub_done': T_pub_done,
                    #         'duration_recv_to_publish': (T_pub_done - T_rx_start) * 1000
                    #     }, f, indent=4)

                    # CSV 저장
                    with open(self.log_file, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([
                            msg.step, T_rl_rx_start, 
                            # T_rl_msg_ready, 
                            T_rl_pub_start, T_rl_pub_end
                        ])

            except Exception as e:
                self.get_logger().error(f"[Relay] TCP receive error: {e}")
                break

def main():
    rclpy.init()
    node = TCPControlRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "conn"):
            node.conn.close()
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
