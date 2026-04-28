#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist

class CancelNav(Node):
    def __init__(self):
        super().__init__('cancel_nav')
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def cancel(self):
        if self.cancel_client.wait_for_service(timeout_sec=2.0):
            req = CancelGoal.Request()
            future = self.cancel_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            self.get_logger().info('Cancel request sent to /navigate_to_pose')
        else:
            self.get_logger().warn('Cancel service not available')

        msg = Twist()
        for _ in range(30):
            self.stop_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.05)

def main():
    rclpy.init()
    node = CancelNav()
    node.cancel()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
