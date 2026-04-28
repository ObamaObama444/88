#!/usr/bin/env python3
import math
import time
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32MultiArray
from slam_toolbox.srv import Reset


def yaw_to_quat(yaw: float):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class MissionBase(Node):
    def __init__(self):
        super().__init__("mission_base")

        self.goal_pub = self.create_publisher(PoseStamped, "/goal", 10)
        self.stop_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.status_sub = self.create_subscription(
            String,
            "/goal/status",
            self.on_status,
            10
        )

        self.feedback_sub = self.create_subscription(
            Float32MultiArray,
            "/goal/feedback",
            self.on_feedback,
            10
        )

        self.cancel_client = self.create_client(Reset, "/goal/cancel")

        self.last_status = ""
        self.distance_remaining = None

    def on_status(self, msg: String):
        self.last_status = msg.data
        self.get_logger().info(f"/goal/status: {msg.data}")

    def on_feedback(self, msg: Float32MultiArray):
        if len(msg.data) >= 1:
            self.distance_remaining = float(msg.data[0])

    def make_goal(self, x, y, yaw, frame_id="map"):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        qz, qw = yaw_to_quat(float(yaw))
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

        self.publish_stop()

    def go_to(self, x, y, yaw, timeout=30.0, stop_dist=0.25):
        self.last_status = ""
        self.distance_remaining = None

        goal = self.make_goal(x, y, yaw)
        self.get_logger().info(
            f"Sending goal: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f} deg"
        )

        # Несколько публикаций, чтобы goal_proxy точно получил цель
        for _ in range(3):
            self.goal_pub.publish(goal)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            if "GOAL SUCCESS" in self.last_status:
                self.get_logger().info("Goal reached")
                self.publish_stop(0.5)
                return True

            if "GOAL ABORTED" in self.last_status or "GOAL REJECTED" in self.last_status:
                self.get_logger().warn(f"Goal failed: {self.last_status}")
                self.publish_stop()
                return False

            # Страховка: если осталось совсем мало, останавливаем
            if self.distance_remaining is not None and self.distance_remaining < stop_dist:
                self.get_logger().info(f"Close enough: {self.distance_remaining:.2f} m")
                self.cancel_goal()
                return True

        self.get_logger().warn("Goal timeout, cancelling")
        self.cancel_goal()
        return False

    def run_route(self):
        # Тестовые точки рядом с текущей позицией, которую мы видели в TF:
        # x примерно -0.61, y примерно 0.17, yaw примерно -78 градусов.
        route = [
            ("test_1", -0.55, 0.20, math.radians(-78)),
            ("test_2", -0.50, 0.25, math.radians(-78)),
        ]

        self.get_logger().info("Mission test started")

        for name, x, y, yaw in route:
            self.get_logger().info(f"Going to {name}")
            ok = self.go_to(x, y, yaw, timeout=30.0)
            if not ok:
                self.get_logger().warn(f"Failed on {name}, stopping mission")
                self.cancel_goal()
                return
            self.get_logger().info(f"Reached {name}, pause")
            time.sleep(1.0)

        self.get_logger().info("Mission test finished")
        self.publish_stop()


def main():
    rclpy.init()
    node = MissionBase()

    try:
        node.run_route()
    except KeyboardInterrupt:
        node.get_logger().warn("KeyboardInterrupt, cancelling")
        node.cancel_goal()
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
