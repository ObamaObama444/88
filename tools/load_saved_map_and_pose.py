#!/usr/bin/env python3
import argparse
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomReader(Node):
    def __init__(self, topic):
        super().__init__("load_map_pose_odom_reader")
        self.msg = None
        self.sub = self.create_subscription(Odometry, topic, self.cb, 10)

    def cb(self, msg):
        self.msg = msg


def get_odom(topic="/hardware/odom", timeout=3.0):
    rclpy.init()
    node = OdomReader(topic)

    start = time.time()
    while time.time() - start < timeout and node.msg is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    msg = node.msg
    node.destroy_node()
    rclpy.shutdown()

    if msg is None:
        return 0.0, 0.0, 0.0

    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return float(p.x), float(p.y), quat_to_yaw(q)


def run_cmd(cmd):
    print("+", " ".join(cmd))
    subprocess.run(cmd, check=False)


def kill_old_processes():
    patterns = [
        "map_server",
        "lifecycle_manager_saved_map",
        "static_transform_publisher",
        "slam_toolbox",
    ]

    for p in patterns:
        subprocess.run(["pkill", "-f", p], check=False)

    time.sleep(0.8)


def fix_map_yaml(map_yaml):
    path = Path(map_yaml)
    if not path.exists():
        print(f"ERROR: map yaml not found: {map_yaml}")
        sys.exit(1)

    pgm = path.with_suffix(".pgm")
    if not pgm.exists():
        print(f"ERROR: map image not found: {pgm}")
        sys.exit(1)

    lines = path.read_text().splitlines()
    new_lines = []
    changed = False

    for line in lines:
        if line.startswith("image:"):
            new_lines.append(f"image: {pgm.name}")
            changed = True
        else:
            new_lines.append(line)

    if not changed:
        new_lines.insert(0, f"image: {pgm.name}")

    path.write_text("\n".join(new_lines) + "\n")
    print(f"Map yaml fixed: image: {pgm.name}")


def compute_map_to_odom(map_x, map_y, map_yaw, odom_x, odom_y, odom_yaw):
    # Нужно T_map_odom такое, чтобы:
    # T_map_odom * T_odom_base = T_map_base
    yaw = map_yaw - odom_yaw

    c = math.cos(yaw)
    s = math.sin(yaw)

    x = map_x - (c * odom_x - s * odom_y)
    y = map_y - (s * odom_x + c * odom_y)

    return x, y, yaw


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", default="/src/maps/labyrinth_map.yaml")
    parser.add_argument("--x", type=float, required=True, help="x робота на карте")
    parser.add_argument("--y", type=float, required=True, help="y робота на карте")
    parser.add_argument("--yaw-deg", type=float, required=True, help="направление робота на карте в градусах")
    parser.add_argument("--odom-topic", default="/hardware/odom")
    args = parser.parse_args()

    map_yaml = args.map
    map_x = args.x
    map_y = args.y
    map_yaw = math.radians(args.yaw_deg)

    print("=== LOAD SAVED MAP AND SET ROVER POSE ===")
    print(f"map: {map_yaml}")
    print(f"target pose on map: x={map_x:.3f}, y={map_y:.3f}, yaw={args.yaw_deg:.1f} deg")

    fix_map_yaml(map_yaml)
    kill_old_processes()

    print()
    print("Starting saved map...")
    map_proc = subprocess.Popen([
        "ros2", "launch", "/src/misc/saved_map_launch.py",
        f"map:={map_yaml}",
    ])

    time.sleep(2.0)

    print()
    print(f"Reading odom from {args.odom_topic}...")
    odom_x, odom_y, odom_yaw = get_odom(args.odom_topic, timeout=3.0)
    print(f"current odom: x={odom_x:.3f}, y={odom_y:.3f}, yaw={math.degrees(odom_yaw):.1f} deg")

    tf_x, tf_y, tf_yaw = compute_map_to_odom(
        map_x, map_y, map_yaw,
        odom_x, odom_y, odom_yaw
    )

    print()
    print("Computed map -> odom:")
    print(f"x={tf_x:.3f}, y={tf_y:.3f}, yaw={math.degrees(tf_yaw):.1f} deg")

    print()
    print("Starting static transform map -> odom...")
    tf_proc = subprocess.Popen([
        "ros2", "run", "tf2_ros", "static_transform_publisher",
        "--x", str(tf_x),
        "--y", str(tf_y),
        "--z", "0",
        "--yaw", str(tf_yaw),
        "--pitch", "0",
        "--roll", "0",
        "--frame-id", "map",
        "--child-frame-id", "odom",
    ])

    print()
    print("READY.")
    print("Check:")
    print("  ros2 topic echo /map --once")
    print("  timeout 5 ros2 run tf2_ros tf2_echo map base_link")
    print()
    print("Keep this terminal open. Press Ctrl+C to stop map and pose publisher.")

    try:
        while True:
            time.sleep(1)
            if map_proc.poll() is not None:
                print("WARNING: map_server launch exited")
                break
            if tf_proc.poll() is not None:
                print("WARNING: static_transform_publisher exited")
                break
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        for proc in [tf_proc, map_proc]:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                time.sleep(0.5)
                if proc.poll() is None:
                    proc.kill()


if __name__ == "__main__":
    main()
