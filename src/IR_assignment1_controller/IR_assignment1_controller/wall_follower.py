#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollowingNode(Node):
    def __init__(self):
        super().__init__('wall_following_node')
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/lidar',
            self.laser_callback,
            10
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.error_sum = 0
        self.prev_error = 0
        self.Kp = 1.0
        self.Ki = 0.1
        self.Kd = 0.01
        self.max_speed = 0.5
        self.min_speed = 0.1

    def laser_callback(self, msg):

        self.get_logger().info("Received: " + str(msg))

        # Assuming the wall is to the right of the robot
        # and LIDAR data ranges array is 360 degrees
         # Extract right side measurements
        right_distance = min(msg.ranges[320:640])
        desired_distance = 1.0  # Desired distance from the wall
        error = desired_distance - right_distance

        # Proportional control
        P = self.Kp * error

        # Integral control
        self.error_sum += error
        I = self.Ki * self.error_sum

        # Derivative control
        D = self.Kd * (error - self.prev_error)
        self.prev_error = error

        control_output = P + I + D

        # Variable speed control
        # Slow down when error is large, speed up when error is small
        speed = self.max_speed - abs(error) * 0.1  # Example speed control formula
        speed = max(min(speed, self.max_speed), self.min_speed)  # Clamp speed to min/max range

        self.send_velocity_command(speed, control_output)

    def send_velocity_command(self, speed, control_output):
        # Create and publish a Twist message to control the robot
        twist_msg = Twist()
        twist_msg.linear.x = speed  # Forward speed
        twist_msg.angular.z = control_output  # Angular speed
        self.cmd_pub.publish(twist_msg)
        self.get_logger().info("Published: " + str(twist_msg))


def main():
    rclpy.init()
    node = WallFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
