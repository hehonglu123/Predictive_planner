#!/bin/sh
cd ~/GazeboModelRobotRaconteurDriver-2021-04-18
# RR URL: rr+tcp://localhost:58654?service=robot

dotnet GazeboModelRobotRaconteurDriver.dll --gazebo-url=rr+tcp://localhost:11346/?service=GazeboServer --robotraconteur-tcp-port=58654 --robotraconteur-nodename=sawyer  --model-name=sawyer --robot-info-file=/mnt/c/Users/hehon/Desktop/RobotRaconteur/RR_Project/simulation/config/sawyer_robot_default_config.yml
