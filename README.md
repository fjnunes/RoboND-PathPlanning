# RoboND-PathPlanning

### Task List
Here's a detailed task list of the steps you should take in order to implement this package with your home service robot to autonomously map an environment:
1. Create a **wall_follower** package.
2. Create a **wall_follower** C++ node by cloning this repo.
3. Edit the wall_follower C++ **node name** and change it to **wall_follower**.
4. Edit the wall_follower C++ subscriber and publisher **topics name**.
5. Write a **wall_follower.sh** shell script that launch the **turtlebot_world.launch**, **gmapping_demo.launch**, **view_navigation.launch**, and the **wall_follower** node.
6. Edit the **CMakeLists.txt** file and add directories, executable, and target link libraries.
7. Build your **catkin_ws**.
8. Run your **wall_follower.sh** shell script to autonomously map the environment.
9. Once you are satisfied with the map, kill the wall_follower terminal and save your map in both **pgm** and **yaml** formats in the **World** directory of your **catkin_ws/src**.
