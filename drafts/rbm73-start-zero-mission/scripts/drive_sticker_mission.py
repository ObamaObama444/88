#!/usr/bin/env python3
import json
import math
import time
import urllib.request
from collections import Counter, defaultdict

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose


POINTS_FILE = "/src/maps/sticker_points.yaml"
STATS_URL = "http://localhost:8989/stats"
COUNTING_ENABLE_URL = "http://localhost:8989/counting/enable"
COUNTING_DISABLE_URL = "http://localhost:8989/counting/disable"

TARGET_CLASSES = ["apple", "donut", "car", "motorcycle"]

SCAN_SECONDS = 5.0
NAV_TIMEOUT = 80.0


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class StickerMission(Node):
    def __init__(self):
        super().__init__("drive_sticker_mission")

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/mission/status", 10)

        with open(POINTS_FILE, "r") as f:
            data = yaml.safe_load(f)

        self.points = data["stickers"]

        self.detected_by_sticker = {}
        self.counts = Counter()
        self.points_by_class = defaultdict(list)

        self.status(f"MISSION_LOADED points={len(self.points)}")

    def status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        for _ in range(20):
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

    def make_pose(self, p):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(p["x"])
        pose.pose.position.y = float(p["y"])
        pose.pose.position.z = 0.0

        qz, qw = yaw_to_quat(float(p["yaw"]))
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def go_to(self, p, timeout=NAV_TIMEOUT):
        self.status(f"GO_TO {p['id']} x={p['x']} y={p['y']} yaw={p['yaw']}")

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.status("ERROR_NAVIGATE_TO_POSE_NOT_AVAILABLE")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(p)

        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.status(f"NAV_REJECTED {p['id']}")
            return False

        result_future = goal_handle.get_result_async()
        start = time.time()

        while not result_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if time.time() - start > timeout:
                self.status(f"NAV_TIMEOUT {p['id']}")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                self.stop_robot()
                return False

        self.stop_robot()
        self.status(f"ARRIVED {p['id']}")
        return True

    def read_site_stats(self):
        try:
            with urllib.request.urlopen(STATS_URL, timeout=0.5) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except Exception as e:
            self.status(f"STATS_READ_ERROR {e}")
            return None

    def call_site_url(self, url, label):
        try:
            with urllib.request.urlopen(url, timeout=0.5) as resp:
                data = resp.read().decode("utf-8")
                self.status(f"{label} OK {data}")
                return True
        except Exception as e:
            self.status(f"{label} ERROR {e}")
            return False

    def scan_here(self, p):
        self.status(f"SCAN_START {p['id']} seconds={SCAN_SECONDS}")

        # Включаем подсчёт только когда робот уже приехал и стоит
        self.call_site_url(COUNTING_ENABLE_URL, "COUNTING_ENABLE")

        observations = Counter()
        start = time.time()

        while time.time() - start < SCAN_SECONDS and rclpy.ok():
            stats = self.read_site_stats()

            if stats:
                current = stats.get("current", {})

                for cls in TARGET_CLASSES:
                    value = int(current.get(cls, 0) or 0)
                    if value > 0:
                        observations[cls] += value

            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.15)

        # После скана сразу выключаем подсчёт, чтобы во время движения total не рос
        self.call_site_url(COUNTING_DISABLE_URL, "COUNTING_DISABLE")

        if not observations:
            self.status(f"NO_DETECTION {p['id']}")
            return None

        # Послабление для apple:
        # если яблоко моргнуло хотя бы один раз за время скана, считаем точку яблоком.
        if observations.get("apple", 0) >= 1:
            best_class = "apple"
            votes = observations["apple"]
        else:
            best_class, votes = observations.most_common(1)[0]

        self.detected_by_sticker[p["id"]] = best_class
        self.counts[best_class] += 1
        self.points_by_class[best_class].append(p)

        self.status(
            f"DETECTED {p['id']} class={best_class} votes={votes} "
            f"observations={dict(observations)} counts={dict(self.counts)}"
        )

        return best_class

    def run(self):
        self.status("MISSION_START_SCAN_ALL_STICKERS")
        self.call_site_url(COUNTING_DISABLE_URL, "COUNTING_DISABLE_AT_START")
        self.call_site_url(COUNTING_ENABLE_URL, "COUNTING_ENABLE")

        for p in self.points:
            ok = self.go_to(p)

            if not ok:
                self.status(f"SKIP_SCAN_NAV_FAILED {p['id']}")
                continue

            self.scan_here(p)

        self.call_site_url(COUNTING_DISABLE_URL, "COUNTING_DISABLE")
        self.status(f"SCAN_FINISHED detected_by_sticker={self.detected_by_sticker}")
        self.status(f"FINAL_COUNTS {dict(self.counts)}")

        if not self.counts:
            self.status("ERROR_NO_STICKERS_DETECTED")
            self.stop_robot()
            return

        min_count = min(self.counts.values())
        rare_classes = [cls for cls, cnt in self.counts.items() if cnt == min_count]

        # Если несколько классов встретились одинаково редко - берём первый
        target_class = rare_classes[0]
        target_point = self.points_by_class[target_class][0]

        self.status(
            f"TARGET_CLASS {target_class} count={min_count} "
            f"target_point={target_point['id']}"
        )

        ok = self.go_to(target_point, timeout=NAV_TIMEOUT)

        if ok:
            self.status("MISSION_FINISHED_STOPPED_NEAR_RAREST_STICKER")
        else:
            self.status("MISSION_FINISHED_FINAL_NAV_FAILED")

        self.stop_robot()


def main():
    rclpy.init()
    node = StickerMission()

    try:
        node.run()
    except KeyboardInterrupt:
        node.status("MISSION_INTERRUPTED")
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
