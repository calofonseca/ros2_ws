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
        self.max_speed = 1.0
        self.min_speed = 0.5
        self.get_logger().info("Wall Follower Node has been started")

    def laser_callback(self, msg):
        # Your existing laser_callback code...
        
        # Conditions and states
        wall_detected = min(msg.ranges[320:640]) < 3.5  # example condition
        stopping_condition_detected = self.check_stopping_condition(msg)  # Implement this method
        
        if stopping_condition_detected:
            self.state = "stopping"
        elif wall_detected:
            self.state = "following_wall"
        else:
            self.state = "wandering"

        # Decision-making and executing behaviors
        if self.state == "wandering":
            self.wander()
        elif self.state == "following_wall":
            self.follow_wall(msg)
        elif self.state == "stopping":
            self.stop_at_goal()

    def wander(self):
        # Implement logic to wander around
        self.send_command(0.1, 0.0)
        return
    
    def follow_wall(self, msg):
        self.get_logger().info("Received: " + str(len(msg.ranges)))

        # Assuming the LIDAR data ranges array is 360 degrees
        # Extract right and left side measurements
        right_distance = min(msg.ranges[270:360])  # Adjust the angles as per your LIDAR orientation
        self.get_logger().info("Right: " + str(right_distance))
        left_distance = min(msg.ranges[0:90])      # Adjust the angles as per your LIDAR orientation
        self.get_logger().info("Left: " + str(left_distance))
        desired_distance = 2.0  # Desired distance from the wall
    
        # Check which side the wall is closer to
        if right_distance < left_distance:
            error = desired_distance - right_distance
            self.get_logger().info("Following wall on the right")
        else:
            error = desired_distance - left_distance
            self.get_logger().info("Following wall on the left")

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
        speed = self.max_speed - abs(error) * 0.1  # Example speed control formula
        speed = max(min(speed, self.max_speed), self.min_speed)  # Clamp speed to min/max range

        # Decide direction based on the side of the wall
        if right_distance < left_distance:
            # If following right wall, control_output remains same
            pass
        else:
            # If following left wall, invert the direction of control_output
            control_output = -control_output
    
        self.send_command(0.1, control_output)
        return
    
    def stop_at_goal(self):
        # Implement logic to stop or change behavior at the goal
        self.send_command(0.0, 0.0)
        return
    
    def check_stopping_condition(self, msg):
        # Implement logic to check for stopping condition based on sensor data
        # Example: If LIDAR data show a particular pattern (e.g., a sharp tip)
        # ...
        return False  # Replace with actual logic       
        
    def send_command(self, speed, control_output):
        # Create and publish a Twist message to control the robot
        self.get_logger().info("Trying to send: " + str(speed) + " control" + str(control_output))
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
