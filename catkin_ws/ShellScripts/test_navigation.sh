#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/RoboND-PathPlanning/catkin_ws/src/home_service_robot/World/UdacityRoom.world

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.lauch " &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &