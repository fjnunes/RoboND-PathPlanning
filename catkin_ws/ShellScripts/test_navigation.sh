#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/RoboND-PathPlanning/catkin_ws/World/UdacityRoom2.world
export TURTLEBOT_GAZEBO_MAP_FILE=/home/workspace/RoboND-PathPlanning/catkin_ws/World/UdacityRoom2.yaml

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &