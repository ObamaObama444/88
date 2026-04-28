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


START_FILE = "/src/maps/start_pose.yaml"
NAV_TIMEOUT = 80.0


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class DriveToStart(Node):
    def __init__(self):
        super().__init__("drive_to_start")

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/mission/status", 10)

        with open(START_FILE, "r") as f:
            self.start = yaml.safe_load(f)["start"]

    def status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def stop_robot(self):
        msg = Twist()
        for _ in range(20):
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

    def make_pose(self):
        p = self.start

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

    def run(self):
        p = self.start
        self.status(f"RETURN_TO_START x={p['x']} y={p['y']} yaw={p['yaw']}")

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.status("ERROR_NAVIGATE_TO_POSE_NOT_AVAILABLE")
            self.stop_robot()
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose()

        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.status("START_GOAL_REJECTED")
            self.stop_robot()
            return

        result_future = goal_handle.get_result_async()
        start_time = time.time()

        while not result_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if time.time() - start_time > NAV_TIMEOUT:
                self.status("START_NAV_TIMEOUT")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                self.stop_robot()
                return

        self.stop_robot()
        self.status("ARRIVED_TO_START")


def main():
    rclpy.init()
    node = DriveToStart()

    try:
        node.run()
    except KeyboardInterrupt:
        node.status("RETURN_TO_START_INTERRUPTED")
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
