#!/usr/bin/env python3
import json
import math
import time
import yaml
from collections import Counter, defaultdict

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class StaticStickerMission(Node):
    def __init__(self):
        super().__init__("static_sticker_mission")

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/mission/status", 10)

        self.det_sub = self.create_subscription(
            String,
            "/stickers/detection",
            self.detection_cb,
            10
        )

        with open("/src/maps/sticker_points.yaml", "r") as f:
            data = yaml.safe_load(f)

        self.stickers = data["stickers"]

        self.current_sticker = None
        self.current_detections = []

        self.detected_by_sticker = {}
        self.counts = Counter()
        self.points_by_class = defaultdict(list)

        self.get_logger().info(f"Loaded stickers: {len(self.stickers)}")

    def status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def detection_cb(self, msg):
        if self.current_sticker is None:
            return

        try:
            detections = json.loads(msg.data)
        except Exception:
            return

        for d in detections:
            cls = d.get("class")
            conf = float(d.get("conf", 0.0))
            bbox = d.get("bbox", [0, 0, 0, 0])

            if not cls or conf < 0.30:
                continue

            # Сохраняем все наблюдения за время скана
            self.current_detections.append({
                "class": cls,
                "conf": conf,
                "bbox": bbox,
                "time": time.time()
            })

    def make_pose(self, p):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(p["x"])
        pose.pose.position.y = float(p["y"])
        pose.pose.position.z = 0.0

        qz, qw = yaw_to_quat(float(p.get("yaw", 0.0)))
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        for _ in range(12):
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

    def go_to(self, p, timeout=80.0):
        self.status(f"GO_TO {p['id']} x={p['x']} y={p['y']} yaw={p['yaw']}")

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.status("ERROR_NAV_ACTION_NOT_AVAILABLE")
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

        start_time = time.time()
        while not result_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if time.time() - start_time > timeout:
                self.status(f"NAV_TIMEOUT {p['id']}")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                self.stop_robot()
                return False

        self.stop_robot()
        self.status(f"ARRIVED {p['id']}")
        return True

    def scan_sticker(self, p, seconds=3.0):
        self.current_sticker = p
        self.current_detections = []

        self.status(f"SCAN_START {p['id']}")

        start_time = time.time()
        while time.time() - start_time < seconds and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.current_sticker = None

        if not self.current_detections:
            self.status(f"NO_DETECTION {p['id']}")
            return None

        # Выбираем самый стабильный класс:
        # сначала по количеству наблюдений, потом по средней уверенности.
        by_class = defaultdict(list)
        for d in self.current_detections:
            by_class[d["class"]].append(d["conf"])

        best_class = None
        best_score = None

        for cls, confs in by_class.items():
            count = len(confs)
            avg_conf = sum(confs) / len(confs)
            score = (count, avg_conf)

            if best_score is None or score > best_score:
                best_score = score
                best_class = cls

        self.detected_by_sticker[p["id"]] = best_class
        self.counts[best_class] += 1
        self.points_by_class[best_class].append(p)

        self.status(
            f"DETECTED {p['id']} class={best_class} "
            f"observations={best_score[0]} avg_conf={best_score[1]:.2f} "
            f"counts={dict(self.counts)}"
        )

        return best_class

    def run(self):
        self.status("MISSION_START_STATIC_STICKERS")

        for p in self.stickers:
            ok = self.go_to(p)
            if not ok:
                self.status(f"SKIPPED {p['id']} BECAUSE_NAV_FAILED")
                continue

            self.scan_sticker(p, seconds=3.0)

        self.status(f"SCAN_FINISHED detected_by_sticker={self.detected_by_sticker}")
        self.status(f"FINAL_COUNTS {dict(self.counts)}")

        if not self.counts:
            self.status("ERROR_NO_STICKERS_DETECTED")
            self.stop_robot()
            return

        min_count = min(self.counts.values())
        rare_classes = [cls for cls, cnt in self.counts.items() if cnt == min_count]
        target_class = rare_classes[0]

        target_point = self.points_by_class[target_class][0]

        self.status(
            f"TARGET_CLASS {target_class} count={min_count} "
            f"target_point={target_point['id']}"
        )

        ok = self.go_to(target_point, timeout=80.0)

        if ok:
            self.status("MISSION_FINISHED_STOPPED_NEAR_TARGET")
        else:
            self.status("MISSION_FINISHED_BUT_FINAL_NAV_FAILED")

        self.stop_robot()


def main():
    rclpy.init()
    node = StaticStickerMission()

    try:
        node.run()
    except KeyboardInterrupt:
        node.stop_robot()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
