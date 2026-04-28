#!/usr/bin/env python3
import json
import math
import time
from collections import Counter, defaultdict

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32MultiArray
from slam_toolbox.srv import Reset


TARGET_CLASSES = {
    "apple",
    "car",
    "donut",
    "motorcycle",
}


# Новые точки на свежей карте.
# Формат: ("название", x, y, yaw_degrees)
SCAN_ROUTE = [
    ("sticker_1", 1.415, -1.966, -52.966),
    ("sticker_2", 0.524, -1.013, 123.690),
    ("sticker_3", 2.562, -1.230, -58.924),
    ("sticker_4", 2.967, -0.233, -64.141),
    ("sticker_5", 1.808, -1.171, 123.153),
    ("sticker_6", 2.232, 0.854, 42.955),
    ("sticker_7", 1.395, 0.943, -140.767),
    ("sticker_8", 0.618, -0.229, -67.984),
]


def yaw_to_quat(yaw_rad: float):
    return math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_node")

        self.goal_pub = self.create_publisher(PoseStamped, "/goal", 10)
        self.stop_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.status_sub = self.create_subscription(
            String,
            "/goal/status",
            self.on_goal_status,
            10
        )

        self.feedback_sub = self.create_subscription(
            Float32MultiArray,
            "/goal/feedback",
            self.on_goal_feedback,
            10
        )

        self.det_sub = self.create_subscription(
            String,
            "/sticker_detections",
            self.on_detections,
            10
        )

        self.cancel_client = self.create_client(Reset, "/goal/cancel")

        self.last_status = ""
        self.distance_remaining = None
        self.latest_detections = []

        self.found = []
        self.class_to_places = defaultdict(list)

    def on_goal_status(self, msg: String):
        self.last_status = msg.data
        self.get_logger().info(f"/goal/status: {msg.data}")

    def on_goal_feedback(self, msg: Float32MultiArray):
        if len(msg.data) >= 1:
            self.distance_remaining = float(msg.data[0])

    def on_detections(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        detections = []

        for d in data.get("detections", []):
            name = d.get("class_name")
            conf = float(d.get("confidence", 0.0))
            area = float(d.get("area", 0.0))

            if name not in TARGET_CLASSES:
                continue
            if conf < 0.35:
                continue
            if area < 800:
                continue

            detections.append({
                "class_name": name,
                "confidence": conf,
                "area": area,
                "bbox": d.get("bbox"),
                "cx": d.get("cx"),
                "cy": d.get("cy"),
            })

        self.latest_detections = detections

    def make_goal(self, x, y, yaw_deg, frame_id="map"):
        yaw_rad = math.radians(float(yaw_deg))
        qz, qw = yaw_to_quat(yaw_rad)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        return msg

    def publish_stop(self, seconds=1.0):
        msg = Twist()
        end = time.time() + seconds

        while time.time() < end:
            self.stop_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.05)

    def cancel_goal(self):
        if self.cancel_client.wait_for_service(timeout_sec=1.0):
            req = Reset.Request()
            future = self.cancel_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            self.get_logger().info("Cancel sent to /goal/cancel")
        else:
            self.get_logger().warn("/goal/cancel service not available")

        self.publish_stop(1.0)

    def go_to(self, name, x, y, yaw_deg, timeout=80.0, stop_dist=0.25):
        self.last_status = ""
        self.distance_remaining = None

        goal = self.make_goal(x, y, yaw_deg)

        self.get_logger().info(
            f"Going to {name}: x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f}"
        )

        for _ in range(3):
            self.goal_pub.publish(goal)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

        start = time.time()

        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            if "GOAL SUCCESS" in self.last_status:
                self.get_logger().info(f"Reached {name}")
                self.publish_stop(0.5)
                return True

            if "GOAL ABORTED" in self.last_status or "GOAL REJECTED" in self.last_status:
                self.get_logger().warn(f"Goal failed at {name}: {self.last_status}")
                self.publish_stop()
                return False

            if self.distance_remaining is not None and self.distance_remaining < stop_dist:
                self.get_logger().info(
                    f"Close enough to {name}: {self.distance_remaining:.2f} m"
                )
                self.cancel_goal()
                return True

        self.get_logger().warn(f"Timeout on {name}, cancelling")
        self.cancel_goal()
        return False

    def scan_here(self, place_name, x, y, yaw_deg, seconds=4.0):
        self.get_logger().info(f"Scanning at {place_name} for {seconds:.1f} sec")

        self.latest_detections = []

        samples = []
        start = time.time()

        while time.time() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.latest_detections:
                best = max(
                    self.latest_detections,
                    key=lambda d: (d["confidence"], d["area"])
                )
                samples.append(best["class_name"])

        if not samples:
            self.get_logger().warn(f"No sticker detected at {place_name}")
            return None

        counts = Counter(samples)
        class_name, votes = counts.most_common(1)[0]

        record = {
            "place": place_name,
            "class_name": class_name,
            "votes": votes,
            "x": x,
            "y": y,
            "yaw_deg": yaw_deg,
        }

        self.found.append(record)
        self.class_to_places[class_name].append(record)

        self.get_logger().info(
            f"Sticker at {place_name}: {class_name}, votes={votes}, all={dict(counts)}"
        )

        return record

    def choose_rarest(self):
        class_counts = Counter()

        for item in self.found:
            class_counts[item["class_name"]] += 1

        self.get_logger().info(f"Final counts: {dict(class_counts)}")

        if not class_counts:
            return None, None

        min_count = min(class_counts.values())
        rare_classes = [
            cls for cls, count in class_counts.items()
            if count == min_count
        ]

        rare_class = sorted(rare_classes)[0]
        target_place = self.class_to_places[rare_class][0]

        self.get_logger().info(
            f"Rarest class: {rare_class}, count={min_count}, target={target_place['place']}"
        )

        return rare_class, target_place

    def print_summary(self):
        print()
        print("========== MISSION SUMMARY ==========")

        if not self.found:
            print("No stickers found")
        else:
            for item in self.found:
                print(
                    f"{item['place']}: {item['class_name']} "
                    f"votes={item['votes']} "
                    f"x={item['x']:.3f} y={item['y']:.3f} yaw={item['yaw_deg']:.1f}"
                )

            counts = Counter([x["class_name"] for x in self.found])
            print("COUNTS:", dict(counts))

        print("=====================================")
        print()

    def run(self):
        self.get_logger().info("Mission started")

        for name, x, y, yaw_deg in SCAN_ROUTE:
            ok = self.go_to(name, x, y, yaw_deg, timeout=80.0, stop_dist=0.25)

            if not ok:
                self.get_logger().warn(f"Skipping scan at {name}, navigation failed")
                continue

            self.scan_here(name, x, y, yaw_deg, seconds=4.0)

        self.print_summary()

        rare_class, target = self.choose_rarest()

        if target is None:
            self.get_logger().error("No stickers found. Mission failed.")
            self.publish_stop()
            return

        self.get_logger().info(
            f"Going to final target: {target['place']} class={rare_class}"
        )

        self.go_to(
            "final_" + target["place"],
            target["x"],
            target["y"],
            target["yaw_deg"],
            timeout=80.0,
            stop_dist=0.35,
        )

        self.publish_stop(2.0)
        self.get_logger().info("Mission finished")


def main():
    rclpy.init()
    node = MissionNode()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("KeyboardInterrupt, cancelling")
        node.cancel_goal()
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
