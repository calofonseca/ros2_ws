# Reactive wall following robot with laser scanner sensor

This project implements a wall-following algorithm in python for differential drive 2 wheeled robot with a laser scanner sensor using the [Robot Operating System 2 (ROS2)](http://www.ros.org/) libraries, [Ignition Gazebo](http://gazebosim.org/) as simulator and [Python](https://www.python.org/) as programming language. The proposed wall-following algorithm makes a robot wander at random until a wall is found, then follows the wall - through an implemented control to keep a constant distance from it in the outside of a question mark shaped “?" wall, stoping at the bottom dot.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Test Environment

* [Ubuntu 22.04](https://ubuntu.com/blog/tag/22-04-lts) - Operating System
* [ROS2 Iron](https://docs.ros.org/en/iron/index.html) - Set of software libraries and tools used to build and interact with the robot
* [Ignition Gazebo Fortress](http://gazebosim.org/) - Tool used to simulation
* [Python 3.10.12](https://www.python.org/) - Programming language
    
### Setup

After clone the project navigate to the project folder
```
cd ros2_ws/
```
Run colcon build [check here how to setup colcon]( https://colcon.readthedocs.io/en/released/user/installation.html)
```
colcon build
```

### Launch the simulation

Return to the project folder 
```
ros2 launch IR_assignment1_controller simulation_launch.py
```

## Running with multiple terminals

To run the with multiple terminals it will be necessary to use 4 (or 5) different terminals. Follow the steps starting at project root folder.

### First Terminal
*Do not close this terminal*
1) Run the command to change to the world workspace:
```
cd ros2_ws/src/IR_assignment1_controller/worlds/
```
2) Launch the simulation world:
```
ign gazebo environment.sdf 
```

### Second Terminal
*Do not close this terminal*

1) Run the command to setup the lidar bridge:
```
ros2 run ros_gz_bridge parameter_bridge lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan
```

### Third Terminal
*Do not close this terminal*

1) Run the command to setup the command bridge:
```
ros2 run ros_gz_bridge parameter_bridge cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
```

### Fourth Terminal
*Do not close this terminal*

1) Change to the base workspace:
```
cd ../../..
```
2) Build the environment:
```
colcon build
```
3) Run the wall-following script:
```
ros2 run IR_assignment1_controller wall_follower 
```

### Fifth Terminal

1) Run rviz2 to see the lidar measurments:
```
rviz2
```
2) Select add at the bottom left, and then select laserscaner and click Ok. 

3) Change the fized frame parameter to: "vehicle_blue/chassis/gpu_lidar"


## Authors

* [Tiago Fonseca](https://github.com/calofonseca) up202302320
