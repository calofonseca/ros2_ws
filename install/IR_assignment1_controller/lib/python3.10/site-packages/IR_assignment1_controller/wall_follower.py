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
        self.range = 2.0
        self.Kp = 0.1
        self.Ki = 0.0
        self.Kd = 0.0
        self.distance = 1
        self.max_windup = 10
        self.error_log = []
        self.following_side = None  # 'left' or 'right'
        self.get_logger().info("Wall Follower Node has been started")

    def laser_callback(self, msg):

        regions_ = {
            'bright': min(min(msg.ranges[0:102]), self.range),    # Back-right
            'right': min(min(msg.ranges[102:290]), self.range),   # Right
            'fright': min(min(msg.ranges[290:330]), self.range),  # Front-right
            'front': min(min(msg.ranges[330:390]), self.range),   # Front
            'fleft': min(min(msg.ranges[390:430]), self.range),   # Front-left
            'left': min(min(msg.ranges[430:618]), self.range),    # Left
            'bleft': min(min(msg.ranges[618:720]), self.range),   # Back-left
            'right_early': sum([i for i in msg.ranges[190:220] if i != float('inf')]) / len([i for i in msg.ranges[190:220] if i != float('inf')]) if [i for i in msg.ranges[190:220] if i != float('inf')] else float('10'),  # Average for 102-216 degrees
            'right_late': sum([i for i in msg.ranges[140:170] if i != float('inf')]) / len([i for i in msg.ranges[140:170] if i != float('inf')]) if [i for i in msg.ranges[140:170] if i != float('inf')] else float('10'),   # Average for 216-320 degrees
            'left_early': sum([i for i in msg.ranges[400:504] if i != float('inf')]) / len([i for i in msg.ranges[400:504] if i != float('inf')]) if [i for i in msg.ranges[400:504] if i != float('inf')] else float('1e10'),   # Average for 400-504 degrees
            'left_late': sum([i for i in msg.ranges[504:618] if i != float('inf')]) / len([i for i in msg.ranges[504:618] if i != float('inf')]) if [i for i in msg.ranges[504:618] if i != float('inf')] else float('1e10'),    # Average for 504-618 degrees
        }

        self.get_logger().info(str(regions_))

        if self.end_condition(regions_):
            self.end_at_goal()
        elif self.is_wall_detected(regions_):
            if self.following_side is None:
                self.choose_following_side(regions_)  # Choose side when first detected a wall
            if self.following_side is not None:
                self.get_logger().info(f"Following Wall on " + str(self.following_side))
                self.follow_wall(regions_)
        else:
            self.get_logger().info(f"Wander")
            self.wander()

    def choose_following_side(self, regions):
        # Choose the side to follow based on certain logic. 
        # For example, choose the side that has the closest wall when first detected.
        if regions['left'] < regions['right']:
            self.following_side = 'left'
            self.get_logger().info(f"Chosen to follow: {self.following_side} wall")
        elif regions['left'] > regions['right']: 
            self.following_side = 'right'
            self.get_logger().info(f"Chosen to follow: {self.following_side} wall")
        

    def wander(self):
        # Logic to wander around
        self.send_command(0.5, 0.0)

    def is_wall_detected(self, regions):
        return regions['fright'] < self.range or regions['fleft'] < self.range or regions['right'] < self.range or regions['left'] < self.range


    def end_condition(self, regions):
        # You might determine a specific condition based on region readings
        # For instance, when all frontal regions have no obstacle:
        #return regions['bright'] > self.range and regions['right'] > self.range and regions['fright'] > self.range and regions['front'] > self.range and regions['fleft'] > self.range and regions['bleft'] > self.range and regions['left'] < self.distance
        return False

    def follow_wall(self, regions):
        # Define the early and late sections for both sides
        early_avg= regions['right_early'] if self.following_side == 'right' else regions['left_early']
        late_avg = regions['right_late'] if self.following_side == 'right' else regions['left_late']

        # The 'angle' difference will be the difference in these averages
        self.get_logger().info(f"early avg: " + str(early_avg))
        self.get_logger().info(f"late avg: " + str(late_avg))

        angle_diff = early_avg - late_avg # on right if positive is heading out of the wall and negative is going to the wall 

        self.get_logger().info(f"Difference: " + str(angle_diff))

        # Control logic based on the angle difference
        # The idea is if angle_diff is positive, it means robot is getting farther from the wall 
        # on the 'late' section and needs to turn towards the wall. Negative would mean the opposite.
        if self.following_side == "right":
            angle_diff = -angle_diff  # reverse the sign for right side
        
        # Use angle_diff as the error for PID
        error = angle_diff

        # Rest of the PID logic remains the same
        self.error_log.append(error)
        self.error_sum = min(max(self.error_sum + error, -self.max_windup), self.max_windup)

        control_output = (
            self.Kp * error +
            self.Ki * self.error_sum +
            self.Kd * (error - self.prev_error)
        )

        speed = 0.5

        self.prev_error = error

        self.get_logger().info(f"Control Output: " + str(control_output))
            
        # Sending control commands
        self.send_command(speed, control_output)

    #def follow_wall(self, regions):
    #    # As an example, let’s say you want to follow a wall on the right. 1.23-1 = 0.23
    #    error = regions[self.following_side] - self.distance
    #    self.get_logger().info(f"Error: " + str(error))

    #    if self.following_side == "right":
    #        error = -error

    #    self.error_log.append(error)

    #    # Updating error for next iteration
    #    self.error_sum = min(max(self.error_sum + error, -self.max_windup), self.max_windup)
    #    # PID Control
    #    control_output = (
    #        self.Kp * error +
    #        self.Ki * self.error_sum +
    #        self.Kd * (error - self.prev_error)
    #    )

    #    # Limiting speed and control output
    #    speed = min(max(0.5, 1.2 - abs(error)), 1.2)  # Clip speed between 0.5 and 1.5
    #    speed = 0.5
    #    self.get_logger().info(f"Control Out: " + str(control_output))
        #control_output = min(max(control_output, -1.0), 1.0)  # Clip control output between -1 and 1
        #self.get_logger().info(f"Control Output: " + str(control_output))

        
    #    self.prev_error = error
        
        # Sending control commands
    #    self.send_command(speed, control_output)


    def end_at_goal(self):
        # Logic to stop or change behavior at the goal
        self.send_command(0.0, 0.0)
        self.save_errors_to_file()

    def send_command(self, speed, control_output):
        # Create and publish a Twist message to control the robot
        twist_msg = Twist()
        twist_msg.linear.x = float(speed)  # Forward speed
        twist_msg.angular.z = float(control_output)  # Angular speed
        self.cmd_pub.publish(twist_msg)
    
    def save_errors_to_file(self):
        with open("error_log.txt", "a") as file:
            for error in self.error_log:
                file.write(f"{error}\n")

def main():
    rclpy.init()
    node = WallFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
