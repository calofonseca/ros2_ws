#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__("pose_subscriber")
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.get_logger().info("Pose Subscriber node has been started")
        

    def pose_callback(self, msg: Pose):
        #msg = Twist()
        #msg.linear.x = 2.0
        #msg.angular.z = 1.0
        #self.cmv_vel_pub_.publish(msg)
        self.get_logger().info("Recevied: " + str(msg.x))


def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node) #Node kept alive until Ctrl-C
    rclpy.shutdown()
