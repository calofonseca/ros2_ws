# Reactive wall following robot with laser scanner sensor

This project implements a wall-following algorithm in python for differential drive 2 wheeled robot with a laser scanner sensor using the [Robot Operating System 2 (ROS2)](http://www.ros.org/) libraries, [Ignition Gazebo](http://gazebosim.org/) as simulator and [Python](https://www.python.org/) as programming language. The proposed wall-following algorithm makes a robot wander at random until a wall is found, then follows the wall - through an implemented control to keep a constant distance from it - in the outside of a question mark shaped “?" wall.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Test Environment
- ROS: Indigo
- Ubuntu: 14.04LTS
- Python: 2.7.6
- (Gazebo: 7.12.0)

### Project structure

#### Catkin Workspace Folder
    .
    ├── catkin_ws               # Catkin Workspace
    │   ├── src                 # Source files 
    │   │   ├── two-wheeled-robot-motion-planning
    │   │   │   ├── scripts     # Folder that contains the follow wall scripts
    │   │   │   └── ...
    │   │   └── ...
    │   ├── devel               # Folder created after compilation
    │   │   ├── setup.bash      # Shell Script to add environment variables to your path
    │   │   └── ...             # etc.
    │   └── build               # Compiled files
    └── simulation_ws

#### Simulation Workspace Folder
    .
    └── simulation_ws           # Simulation Workspace
        ├── src                 # Source files 
        │   ├── two-wheeled-robot-simulation
        │   │   ├── m2wr_description    # Folder that contains the robot definition
        │   │   ├── my_worlds           # Folder that contains world's definitions
        │   │   └── ...
        │   └── ...
        ├── devel               # Folder created after compilation
        │   ├── setup.bash      # Shell Script to add environment variables to your path
        │   └── ...             # etc.
        └── build               # Compiled files
    
### Setup

After clone the project navigate to the project folder
```
cd ros-wall-follower-2-wheeled-robot-master
```
Navigate to Catkin Workspace and run `catkin_make`
```
cd catkin_ws
catkin_make
```
Return to the project folder 
```
cd ..
```
Navigate to Simulation Workspace and run `catkin_make`
```
cd simulation_ws
catkin_make
```
## Running with multiple terminals

To run the with multiple terminals it will be necessary to use 4 (or 5) different terminals. Follow the steps starting at project root folder.

### First Terminal
*Do not close this terminal*
1) Run the command to sart up the ignition gazebo environment:
```
cd ros2_ws/src/IR_assignment1_controller/worlds/
ign gazebo envent.sdf 
```

2) Configure the lidar bride:

```
roslaunch my_worlds V_world.launch
```
or 
```
roslaunch my_worlds W_world.launch
```

### Second Terminal

1) Run the command to setup the lidar bridge:
```
ros2 run ros_gz_bridge parameter_bridge cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
#ros2 run ros_gz_bridge parameter_bridge lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan
```

2) Spawn the Mobile 2 Wheeled Robot on the World:
```
roslaunch m2wr_description spawn.launch y:=1
```

### Third Terminal
*Do not close this terminal*

1) Run the command to setup the environments variables on this terminal:
```
source catkin_ws/devel/setup.bash
```
2) Run the wall-following script:
```
rosrun two-wheeled-robot-motion-planning follow_wall.py
```
## Built With

* [Ubuntu 22.04](https://ubuntu.com/blog/tag/22-04-lts) - Operating System
* [ROS2 Iron](https://docs.ros.org/en/iron/index.html) - Set of software libraries and tools used to build and interact with the robot
* [Ignition Gazebo Fortress](http://gazebosim.org/) - Tool used to simulation
* [Python 3.10.12](https://www.python.org/) - Programming language

## Authors

* [Tiago Fonseca](https://github.com/calofonseca) up202302320
