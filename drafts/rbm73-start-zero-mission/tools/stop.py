#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Stop(Node):
    def __init__(self):
        super().__init__('stop_robot')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

def main():
    rclpy.init()
    node = Stop()
    msg = Twist()
    for _ in range(40):
        node.pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.05)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
