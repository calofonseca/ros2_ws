from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
import pathlib

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', 'src/IR_assignment1_controller/worlds/envent.sdf'],
            output='screen'
        ),
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
            ],
            output='screen'
        ),
        Node(
            package='IR_assignment1_controller',
            executable='wall_follower',
            output='screen'
        ),
    ])


#ros2 launch IR_assignment1_controller simulation_launch.py