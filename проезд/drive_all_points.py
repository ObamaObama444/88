#!/usr/bin/env python3
import math
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


POINTS_FILE = "/src/maps/sticker_points.yaml"
NAV_TIMEOUT = 80.0
WAIT_AFTER_ARRIVE = 2.0


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class DriveAllPoints(Node):
    def __init__(self):
        super().__init__("drive_all_points")

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/mission/status", 10)

        with open(POINTS_FILE, "r") as f:
            data = yaml.safe_load(f)

        self.points = data["stickers"]
        self.status(f"LOADED {len(self.points)} points")

    def status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        for _ in range(15):
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

    def go_to(self, p):
        self.status(f"GO_TO {p['id']} x={p['x']} y={p['y']} yaw={p['yaw']}")

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.status("ERROR: /navigate_to_pose not available")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(p)

        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.status(f"REJECTED {p['id']}")
            return False

        result_future = goal_handle.get_result_async()
        start_time = time.time()

        while not result_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if time.time() - start_time > NAV_TIMEOUT:
                self.status(f"TIMEOUT {p['id']}")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                self.stop_robot()
                return False

        self.stop_robot()
        self.status(f"ARRIVED {p['id']}")
        time.sleep(WAIT_AFTER_ARRIVE)
        return True

    def run(self):
        self.status("START_DRIVE_ALL_POINTS")

        for p in self.points:
            self.go_to(p)

        self.stop_robot()
        self.status("FINISHED_ALL_POINTS")


def main():
    rclpy.init()
    node = DriveAllPoints()

    try:
        node.run()
    except KeyboardInterrupt:
        node.status("INTERRUPTED")
        node.stop_robot()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
