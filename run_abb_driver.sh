#!/bin/sh
cd ~/GazeboModelRobotRaconteurDriver-2021-04-18
# RR URL: rr+tcp://localhost:23333?service=robot
dotnet GazeboModelRobotRaconteurDriver.dll --gazebo-url=rr+tcp://localhost:11346/?service=GazeboServer --robotraconteur-tcp-port=58655 --robotraconteur-nodename=abb --model-name=abb --robot-info-file=/mnt/c/Users/hehon/Desktop/Predictive_planner/config/abb1200.yml
