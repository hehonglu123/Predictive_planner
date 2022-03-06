# Instruction
## System
Ubuntu (20.04) with ROS noetic
wsl2 on win11 OK

## Binary Packages:
* lcov: sudo apt-get install locv
* ipopt: sudo apt-get install ros-noetic-ipopt
* fcl: sudo apt-get install ros-noetic-fcl
* bullet inverse dynamics float64: sudo apt-get install libbullet-extras-dev
* ompl: sudo apt-get install ros-noetic-ompl
* taskflow: sudo apt-get install ros-noetic-taskflow

## Python Packages
* catkin_tools
* QP: pip install qpsolvers
* General Robotice Toolbox:
* etc.

## ROS Packages (to be built in catkin_ws):
* tesseract (use wstool): https://github.com/tesseract-robotics/tesseract_python
* robotraconteur (noetic branch): https://github.com/robotraconteur/robotraconteur
* RobotRaconteur_Gazebo_Server_Plugin: https://github.com/johnwason/RobotRaconteur_Gazebo_Server_Plugin
* robotraconteur_companion: 



## workspace build command (using catkin tools):
```
rosdep install --from-paths . --ignore-src --rosdistro noetic -y
catkin build
```

## Path & Source
```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export GAZEBO_MODEL_PATH=~/Predictive_planner/models
export GAZEBO_PLUGIN_PATH=~/catkin_ws/devel/lib
```



## Running instructions


`roscore` to start a roscore for gazebo

`./start_gazebo` to start gazebo and RR_gazebo plugin only

`./start_all` to start everything

open separate terminals and run
`python client_same.py`
