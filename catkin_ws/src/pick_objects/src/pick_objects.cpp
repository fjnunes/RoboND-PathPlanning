#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//actionlib::SimpleClientGoalState moveToGoal(move_base_msgs::MoveBaseGoal *goal) {  
//  return ac.getState();
//}

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickUpGoal;
  move_base_msgs::MoveBaseGoal dropOffGoal;

  // set up the frame parameters
  pickUpGoal.target_pose.header.frame_id = "map";
  pickUpGoal.target_pose.header.stamp = ros::Time::now();
  dropOffGoal.target_pose.header.frame_id = "map";
  dropOffGoal.target_pose.header.stamp = ros::Time::now();
  
  // Define a position and orientation for the robot to reach
  pickUpGoal.target_pose.pose.position.x = 4.0;
  pickUpGoal.target_pose.pose.position.y = 6.0;
  pickUpGoal.target_pose.pose.orientation.w = 1.0;
  
  dropOffGoal.target_pose.pose.position.x = -3.0;
  dropOffGoal.target_pose.pose.position.y = 4.0;
  dropOffGoal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup goal");
  ac.sendGoal(pickUpGoal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Reached pickup goal");
  else
    ROS_INFO("The base failed to move to the pickup goal");

  // wait for 5 seconds
  ROS_INFO("Waiting 5 seconds");
  ros::Duration(5.0).sleep();
  
  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending dropoff goal");
  ac.sendGoal(dropOffGoal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Reached dropoff goal");
  else
    ROS_INFO("The base failed to move to the dropoff goal");

  return 0;
}