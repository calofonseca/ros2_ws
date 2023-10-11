#!/usr/bin/env python3
from typing import List, Optional
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("HELLO " + str(self.counter_))
        self.counter_ += 1
    

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node) #Node kept alive until Ctrl-C
    rclpy.shutdown()

if __name__ == '__main__':
    main()