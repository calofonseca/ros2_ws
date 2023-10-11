#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__("wall_follower")
        self.cmv_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.get_logger().info("Wal Follower node has been started")
        

    def pose_callback(self, msg: Pose):
        self.get_logger().info("Recevied: " + str(msg))
        if msg.x > 9.0 or msg.x < 2.0 or msg.y > 9.0 or msg.y < 2.0:
            self.send_velocity_command(1.0, 0.9)
        else:        
            self.send_velocity_command(5.0, 0.0)

    def send_velocity_command(self, x, a):
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = a
        self.cmv_vel_pub_.publish(msg)
        self.get_logger().info("Published: " + str(msg))


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node) #Node kept alive until Ctrl-C
    rclpy.shutdown()
