#!/bin/bash

FCUURL=$1

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/install/setup.bash
roslaunch offboard offboard.launch fcu_url:=${FCUURL} 2>&1 > /mavros.log &
bash
