#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time 

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

        #Variables and parameters
        self.error_sum = 0
        self.ended = False
        self.prev_error = 0
        self.range = 10.0
        self.max_windup = 10
        self.error_log = []
        self.following_side = None  # 'left' or 'right'
        self.start_time = None

        self.Kp = 1.0
        self.Ki = 0.0
        self.Kd = 0.2
        self.distance = 1
        
        self.get_logger().info("Wall Follower Node has been started")

    def laser_callback(self, msg):

        #Separating the regions for calculus of the algorithm
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
            'left_early': sum([i for i in msg.ranges[400:504] if i != float('inf')]) / len([i for i in msg.ranges[400:504] if i != float('inf')]) if [i for i in msg.ranges[400:504] if i != float('inf')] else float('10'),   # Average for 400-504 degrees
            'left_late': sum([i for i in msg.ranges[504:618] if i != float('inf')]) / len([i for i in msg.ranges[504:618] if i != float('inf')]) if [i for i in msg.ranges[504:618] if i != float('inf')] else float('10'),    # Average for 504-618 degrees,
            'msg_left': msg.ranges[390:670],
            'msg_right': msg.ranges[50:330]   
        }

        #Subsumption architecture
        if not self.ended:
            if self.end_condition(regions_) :
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
        max_range = self.range-8.75  

        if regions['left'] < max_range or regions['right'] < max_range:
            if regions['left'] < regions['right']:
                self.following_side = 'left'
                self.get_logger().info(f"Chosen to follow: {self.following_side} wall")
            else:
                self.following_side = 'right'
                self.get_logger().info(f"Chosen to follow: {self.following_side} wall")
        elif regions['fleft'] < max_range or regions['fright'] < max_range:
            if regions['fleft'] < regions['fright']:
                self.following_side = 'left'
                self.get_logger().info(f"Chosen to follow: {self.following_side} wall")
                self.rotate_robot('left')
            else:
                self.following_side = 'right'
                self.get_logger().info(f"Chosen to follow: {self.following_side} wall")
                self.rotate_robot('right')
        elif regions['front'] < max_range:
            import random
            self.following_side = random.choice(['left', 'right'])
            self.get_logger().info(f"Chosen to follow: {self.following_side} wall due to obstacle in front")
            self.rotate_robot(self.following_side)
        elif regions['bright'] < max_range or regions['bleft'] < max_range:
            self.rotate_robot('around')
            self.following_side = None  # Reset to not following any wall
        else:
            self.wander()
            self.following_side = None  # If both are at max range or no walls detected

    def rotate_robot(self, direction):
        if direction == 'left':
            self.send_command(1, -1)
            time.sleep(1.5)  # This should be tweaked based on real robot's behavior
            self.send_command(0.0, 0.0)
        elif direction == 'right':
            self.send_command(1, 1)
            time.sleep(1.5)  # This should be tweaked based on real robot's behavior
            self.send_command(0.0, 0.0)
        elif direction == 'around':
            self.send_command(0.0, -1)
            time.sleep(6.0)  # This should be tweaked based on real robot's behavior
            self.send_command(0.0, 0.0)
            time.sleep(1.0)


    def wander(self):
        # Logic to wander around
        self.send_command(0.5, 0.0)

    def is_wall_detected(self, regions):
        return regions['fright'] < self.range or regions['fleft'] < self.range or regions['right'] < self.range or regions['left'] < self.range or regions['front'] < self.range or regions['bleft'] < self.range or regions['bright'] < self.range

    def end_condition(self, regions):
        wall_length = 0
        consecutive_wall_points = 0
        max_wall_lenght = 0
        if self.following_side == "left":
            readings = regions['msg_left']
        elif self.following_side == "right":
            readings = regions['msg_right']
        else: 
            readings = None
        
        if readings is not None:
            for r in readings:  # Right side based on your regions
                if r < self.range:  # Check if there's an object/wall
                    delta_s = r * 0.5 * (3.141592653589793 / 180)
                    wall_length += delta_s
                    consecutive_wall_points += 1
                else:
                    # Reset if there's a break in the wall
                    if wall_length >= max_wall_lenght:
                            max_wall_lenght = wall_length
                    wall_length = 0
                    consecutive_wall_points = 0
                
            self.get_logger().info("wall lenght " + str(max_wall_lenght))
            if self.following_side == "left":
                if abs(max_wall_lenght - 1.0) < 0.05 and regions['fright'] >= self.range and regions['fleft'] >= self.range and regions['right'] >= self.range and regions['bleft'] >= self.range and regions['bright'] >= self.range:  
                    self.get_logger().info("Detected a wall of approximately 1 meter in length!")
                    return True
            elif self.following_side == "right":
                if abs(max_wall_lenght - 1.0) < 0.05 and regions['fright'] >= self.range and regions['fleft'] >= self.range and regions['left'] >= self.range and regions['bleft'] >= self.range and regions['bright'] >= self.range:  
                    self.get_logger().info("Detected a wall of approximately 1 meter in length!")
                    return True

            
        
        return False    

    def follow_wall(self, regions):

        if self.start_time is None:
            self.start_time = time.time()
    
        # As an example, let’s say you want to follow a wall on the right. 1.23-1 = 0.23
        error = regions[self.following_side + "_early"] - self.distance
        self.get_logger().info(f"Error: " + str(error))

        if self.following_side == "right":
            error = -error

        self.error_log.append(error)

        # Updating error for next iteration
        self.error_log.append(error)
        self.error_sum = min(max(self.error_sum + error, -self.max_windup), self.max_windup)
        # PID Control
        control_output = (
            self.Kp * error +
            self.Ki * self.error_sum +
            self.Kd * (error - self.prev_error)
        )

        # Limiting speed and control output
        speed = min(max(0.5, 1.2 - abs(error)), 1.2)  # Clip speed between 0.5 and 1.5
        speed = 1.0
        self.get_logger().info(f"Control Out: " + str(control_output))
        #control_output = min(max(control_output, -1.0), 1.0)  # Clip control output between -1 and 1
        #self.get_logger().info(f"Control Output: " + str(control_output))

        
        self.prev_error = error
        
        # Sending control commands
        self.send_command(speed, control_output)


    def end_at_goal(self):
        # Logic to stop or change behavior at the goal
        self.ended = True
        self.send_command(0.0, 0.0)
        self.calculate_KPIs()

    def send_command(self, speed, control_output):
        # Create and publish a Twist message to control the robot
        twist_msg = Twist()
        twist_msg.linear.x = float(speed)  # Forward speed
        twist_msg.angular.z = float(control_output)  # Angular speed
        self.cmd_pub.publish(twist_msg)
        self.get_logger().info(f"Sent: " + str(twist_msg))
    
    def calculate_KPIs(self):
        end_time = time.time()
        LTime = end_time - self.start_time
        N = len(self.error_log)
        
        # Calculate KPIs
        loss = sum([abs(error) for error in self.error_log]) / N
        RMSE = (sum([error**2 for error in self.error_log]) / N)**0.5
        MError = max([abs(error) for error in self.error_log])

        # Output KPIs
        self.get_logger().info(f"KPIs:\nLoss: {loss}\nRMSE: {RMSE}\nMError: {MError}\nLTime: {LTime} seconds")

def main():
    rclpy.init()
    node = WallFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
