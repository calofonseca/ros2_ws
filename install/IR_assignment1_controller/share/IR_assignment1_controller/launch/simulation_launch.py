from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', 'src/IR_assignment1_controller/worlds/envent.sdf'],
            output='screen'
        ),
        TimerAction(
            period=5.0,  # Modify delay duration as needed
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    arguments=[
                        'cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                        'lidar@sensor_msgs/msg/LaserScan]ignition.msgs.LaserScan'
                    ],
                    output='screen'
                )
            ]
        ),
        Node(
            package='IR_assignment1_controller',
            executable='wall_follower',
            output='screen'
        ),
    ])

#ros2 launch IR_assignment1_controller simulation_launch.py

#cd ros2_ws/src/IR_assignment1_controller/worlds/
#ign gazebo envent.sdf 
#ros2 run IR_assignment1_controller wall_follower 
#ros2 run ros_gz_bridge parameter_bridge cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
#ros2 run ros_gz_bridge parameter_bridge lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan


