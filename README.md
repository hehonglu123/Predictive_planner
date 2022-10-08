# Overview
Robotic arm real time centralized planner, with predicted future (fixed horizon) collision avoidance.

# Instruction
*Tesseract Setup*
![](images/tess_viewer.png)
*Execution*
Sawyer 20 Steps                 |  ABB 20 Steps
:------------------------------:|:-------------------------:
![](images/Sawyer_20_step.gif)  | ![](images/ABB_20_step.gif)


2 Robot Predictive Move         | 2 Robot Predictive Move      
:------------------------------:|:-------------------------:
![](images/move2.gif)           | ![](images/2robots.gif)

*SoftWare Architecture*

![](images/diagram.jpg)



## System
* Ubuntu (20.04) with ROS noetic
(wsl2 on win11 OK)
* Python3.8

* Simulation robot service (https://drive.google.com/file/d/1lNFGjh11DI32MYN8jgVuNADaLYteue4R/view?usp=sharing & [dotnet3 runtime](https://docs.microsoft.com/en-us/dotnet/core/install/linux-ubuntu))
* Swig 4.0.2 or later
* Robot Raconteur C# Native (`sudo apt-get install librobotraconteur-net-native`)

## Python Packages
* catkin_tools: `sudo apt-get install python3-catkin-tools`
* QP: `pip install qpsolvers`
* General Robotice Toolbox: `pip install general-robotics-toolbox`
* Tesseract: `pip install tesseract-robotics tesseract-robotics-viewer` (may need to upgrade pip first)

## ROS Packages (to be built in catkin_ws):
* robotraconteur (*noetic branch*): https://github.com/robotraconteur/robotraconteur
* RobotRaconteur_Gazebo_Server_Plugin: https://github.com/johnwason/RobotRaconteur_Gazebo_Server_Plugin
* robotraconteur_companion: https://github.com/robotraconteur/robotraconteur_companion



## workspace build command (using catkin tools):
```
rosdep install --from-paths . --ignore-src --rosdistro noetic -y
catkin config --cmake-args -DROBOTRACONTEUR_ROS=1
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

* `./start_all` to start simulation environment
* `python planner.py` to bring up planner
* `python client_sawyer.py` or ` python client_abb.py`

