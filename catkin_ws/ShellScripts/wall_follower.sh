#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/RoboND-PathPlanning/catkin_ws/src/home_service_robot/World/UdacityRoom.world

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm  -e  " rosrun home_service_robot wall_follower " &