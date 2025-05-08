import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from rosbags.highlevel import AnyReader

def analyze_intervals(bag_path, topics):
    with AnyReader([Path(bag_path)]) as reader:
        topic_timestamps = {topic: [] for topic in topics}

        for connection, timestamp, rawdata in reader.messages():
            if connection.topic in topics:
                topic_timestamps[connection.topic].append(timestamp * 1e-9)  # ns → s

        for topic, timestamps in topic_timestamps.items():
            if len(timestamps) < 2:
                print(f"[WARN] Not enough messages for topic {topic}")
                continue

            intervals = np.diff(timestamps)
            mean = np.mean(intervals)
            print(f"\n📊 {topic}")
            print(f"  Count: {len(timestamps)}")
            print(f"  Mean interval: {mean * 1000:.3f} ms  ≈ {1 / mean:.2f} Hz")
            print(f"  Min: {np.min(intervals) * 1000:.3f} ms")
            print(f"  Max: {np.max(intervals) * 1000:.3f} ms")

            # Plot
            plt.figure()
            plt.hist(intervals * 1000, bins=30, edgecolor='black')
            plt.title(f"Interval Histogram: {topic}")
            plt.xlabel("Interval (ms)")
            plt.ylabel("Frequency")
            plt.grid(True)

        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ROS2 bag sensor interval analyzer")
    parser.add_argument('bag_path', help='Path to the ROS2 bag directory')
    args = parser.parse_args()

    topics = [
        '/carla/hero/CAM_FRONT/image',
        '/carla/hero/CAM_FRONT_LEFT/image',
        '/carla/hero/CAM_FRONT_RIGHT/image',
    ]

    analyze_intervals(args.bag_path, topics)

