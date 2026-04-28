#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Stopper(Node):
    def __init__(self):
        super().__init__('stopper')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

def main():
    rclpy.init()
    node = Stopper()
    msg = Twist()
    end = time.time() + 2.0
    while time.time() < end:
        node.pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
