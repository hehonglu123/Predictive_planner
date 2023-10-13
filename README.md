# Overview
Robotic arm real-time centralized planner, with predicted future (fixed horizon) collision avoidance.

*Tesseract Setup*
![](images/tess_viewer.png)
*Execution*
Sawyer 20 Steps                 |  ABB 20 Steps
:------------------------------:|:-------------------------:
![](images/Sawyer_20_step.gif)  | ![](images/ABB_20_step.gif)


2 Robot Predictive Move         | 2 Robot Predictive Move      
:------------------------------:|:-------------------------:
![](images/move2.gif)           | ![](images/2robots.gif)

*Software Architecture*

![](images/diagram.jpg)


# Algorithm
For any linear system $x_{k+1}=Ax_k+Bu_k$, the future state at $k+N$ could be written as 

```math
x_{k+N}=A^Nx_k+\begin{bmatrix} A^{N-1}B & A^{N-2}B &  ... & B \end{bmatrix} \begin{bmatrix}u_k \\ u_{k+1} \\ ... \\ u_{k+N-1}\end{bmatrix} .
```
## N-step lookahead QP
To track a goal at future N-step with a quadratic cost: $|x_d-x_{k+N}|$, it could be solved with quadratic programming
```math
min_{\mathbf{u}_{all}} ||x_d-A^Nx_k-J_N\mathbf{u}_{all}||.
```
If the desired goal is moving or the goal is to track a trajectory, similarly
```math
\begin{bmatrix}x_{k+1} \\ x_{k+2} \\ ... \\ x_{k+N}\end{bmatrix}=\begin{bmatrix}A \\ A^2 \\ ... \\ A^N\end{bmatrix}x_k+\begin{bmatrix} B & 0 & ... & 0 \\ AB & B & ... & 0 \\ A^{N-1}B & A^{N-2}B &  ... & B \end{bmatrix} \begin{bmatrix}u_k \\ u_{k+1} \\ ... \\ u_{k+N-1}\end{bmatrix},
```
```math
min_{\mathbf{u}_{all}} ||\mathbf{x_{all}}-A_{stack}x_k-J_{all}\mathbf{u}_{all}||.
```
## N-step lookahead Trajectory Optimization
Rather than calculating the new trajectory every loop, it's possible to optimize the trajectory by $\delta\mathbf{x_{all}}$ by adjusting  $\delta\mathbf{u_{all}}$
```math
min_{\delta\mathbf{u}_{all}} ||x_{k+N}-x_{pred}-J_N\delta\mathbf{u}_{all}||,
```
or
```math
min_{\delta\mathbf{u}_{all}} ||\mathbf{x_{all}}-\mathbf{x_{pred}}-J_{all}\delta\mathbf{u}_{all}||.
```
This is more helpful for collision avoidance than standard multi-step QP because collision constraint is a quadratic constraint, this way it's possible to steer the trajectory to the opposite collision direction. It doesn't matter if it's a first-order system, where the control input is *velocity* that is already $\delta x$.

## Collision Constraint
Usually, a control barrier function is used to drive the trajectory away from the collision. Tesseract provides collision distance and vector between two objects, so at $i_{th}$ step this constraint could be written as
```math
J_i\delta \mathbf{u_{all}} \vec{d} < h(d),
```
where $\vec{d}$ is the normalized collision vector from a point on the robot to a point in the environment (sign flipped if penetrating), and $h(d)$ is the barrier function depending on the collision distance (negative if penetrating).

# Instruction

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

* `./start_all` to start the simulation environment
* `python planner.py` to bring up planner
* `python client_sawyer.py` or ` python client_abb.py`

