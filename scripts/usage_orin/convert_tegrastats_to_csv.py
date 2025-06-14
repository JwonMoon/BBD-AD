import re
import sys
import pandas as pd
from datetime import datetime

def parse_tegrastats_line(line):
    match = re.match(
        r"(\d{2}-\d{2}-\d{4} \d{2}:\d{2}:\d{2}) RAM (\d+)/(\d+)MB.*CPU \[(.*?)\].*GR3D_FREQ (\d+)%.*VDD_GPU_SOC (\d+)mW",
        line
    )
    if not match:
        return None

    timestamp_str, ram_used, ram_total, cpu_raw, gr3d_freq, gpu_power = match.groups()
    timestamp = datetime.strptime(timestamp_str, "%m-%d-%Y %H:%M:%S")
    ram_used = int(ram_used)
    ram_total = int(ram_total)
    cpu_usages = [int(x.split('%')[0]) for x in cpu_raw.split(',') if '%' in x]
    cpu_avg = sum(cpu_usages) / len(cpu_usages) if cpu_usages else 0
    gr3d_freq = int(gr3d_freq)
    gpu_power = int(gpu_power)

    return {
        "timestamp": timestamp,
        "utilization.gpu [%]": gr3d_freq,
        "memory_used [MiB]": ram_used,
        "memory_total [MiB]": ram_total,
        "cpu_usage [%]": cpu_avg,
        "power.draw [W]": gpu_power / 1000  # mW → W
    }

def convert_log_file(input_path, output_path):
    with open(input_path, "r") as f:
        lines = f.readlines()

    parsed = [parse_tegrastats_line(line) for line in lines]
    parsed = [row for row in parsed if row is not None]
    df = pd.DataFrame(parsed)
    df.to_csv(output_path, index=False)
    print(f"Saved converted CSV to: {output_path}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python convert_tegrastats_to_csv.py <input_log_file> <output_csv_file>")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    convert_log_file(input_file, output_file)
