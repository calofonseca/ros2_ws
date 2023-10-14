from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch Ignition Gazebo with the specified environment SDF
        ExecuteProcess(
            cmd=['./launch_ignition.sh', 'envent.sdf'],
            output='screen'
        ),
        # Launch the parameter_bridge for cmd_vel
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                'cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
            ],
            output='screen'
        ),
        # Launch the parameter_bridge for lidar
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                'lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'
            ],
            output='screen'
        ),
        # Launch the wall follower node
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


