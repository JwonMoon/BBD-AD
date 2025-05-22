import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from rosbags.highlevel import AnyReader

def analyze_intervals(bag_path, topics):
    bag_path = Path(bag_path)
    if not bag_path.exists():
        print(f"[ERROR] Bag path does not exist: {bag_path}")
        return

    with AnyReader([bag_path]) as reader:
        topic_timestamps = {topic: [] for topic in topics}
        topic_sizes = {topic: [] for topic in topics}

        for connection, timestamp, rawdata in reader.messages():
            if connection.topic in topics:
                topic_timestamps[connection.topic].append(timestamp * 1e-9)  # ns → s
                topic_sizes[connection.topic].append(len(rawdata))

        for topic in topics:
            timestamps = topic_timestamps[topic]
            sizes = topic_sizes[topic]

            if len(timestamps) < 2:
                print(f"[WARN] Not enough messages for topic {topic}")
                continue

            intervals = np.diff(timestamps)
            mean = np.mean(intervals)
            avg_size_kb = np.mean(sizes) / 1024

            print(f"\n📊 Topic: {topic}")
            print(f"  Count: {len(timestamps)}")
            print(f"  Mean interval: {mean * 1000:.3f} ms  🔁 {1 / mean:.2f} Hz")
            print(f"  Min: {np.min(intervals) * 1000:.3f} ms 🔁 {1 / np.min(intervals):.2f} Hz")
            print(f"  Max: {np.max(intervals) * 1000:.3f} ms 🔁 {1 / np.max(intervals):.2f} Hz") 
            print(f"  Avg msg size: {avg_size_kb:.1f} KB")

            # Save histogram
            plt.figure()
            plt.hist(intervals * 1000, bins=30, edgecolor='black')
            plt.title(f"Interval Histogram: {topic}")
            plt.xlabel("Interval (ms)")
            plt.ylabel("Frequency")
            plt.grid(True)

            fname = topic.replace("/", "_").strip("_") + "_hist.png"
            plt.savefig(fname)
            print(f"  Histogram saved to: {fname}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ROS2 bag sensor interval analyzer")
    parser.add_argument('bag_path', help='Path to the ROS2 bag directory')
    parser.add_argument('img_type', help='Camera image input type')
    #parser.add_argument('--img-type', default='raw', choices=['raw', 'compressed'], help='Camera image input type')
    args = parser.parse_args()
    
    if args.img_type == 'raw':
        topics = [
            '/carla/hero/CAM_FRONT/image',
            '/carla/hero/CAM_FRONT_LEFT/image',
            '/carla/hero/CAM_FRONT_RIGHT/image',
        ]
    else:
        topics = [
            '/image/compressed',
            '/image_left/compressed',
            '/image_right/compressed',
        ]
    
    topics += [
        '/carla/hero/GPS',
        '/carla/hero/IMU',
        '/carla/hero/vehicle_status',
        '/carla/hero/vehicle_control_cmd',
    ]

    analyze_intervals(args.bag_path, topics)

