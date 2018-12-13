/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>

// %EndTag(INCLUDES)%

// %Tag(INIT)%

ros::Publisher marker_pub;
visualization_msgs::Marker marker;
geometry_msgs::Point pickup_position;
geometry_msgs::Point dropoff_position;
const float distance_threshold = 0.2;
bool hasObject;

float euclidean_distance(geometry_msgs::Point pointA, geometry_msgs::Point pointB) 
{
  return sqrt(fabs(pow(pointB.x - pointA.x, 2) - pow(pointB.y - pointA.y, 2))); 
}

void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // ROS_INFO("x: [%f], y: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y);
  
    geometry_msgs::Point current_position = msg->pose.pose.position;

  	float pickup_distance = euclidean_distance(current_position, pickup_position);
  	float dropoff_distance = euclidean_distance(current_position, dropoff_position);

    // ROS_INFO("Pickup-> [%f], Dropoff-> [%f]", pickup_distance, dropoff_distance); 
  
    if (!hasObject && (pickup_distance <= distance_threshold))
    {
        ROS_INFO("Reached pickup position");
        // Pause 5 seconds
        ROS_INFO("Waiting 5 seconds");
        ros::Duration(5.0).sleep();
        // Hide the marker
        ROS_INFO("Hiding the marker");
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        hasObject = true;
    }
    else if (hasObject && (dropoff_distance <= distance_threshold))
    {
        ROS_INFO("Reached dropoff position");
        // Pause 5 seconds
        ROS_INFO("Waiting 5 seconds");
        ros::Duration(5.0).sleep();
        // Publish the marker at the drop off zone
        ROS_INFO("Publishing the marker at the drop off zone");
        marker.pose.position = dropoff_position;
        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);
        hasObject = false;
    }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber amcl_pose_sub = n.subscribe("amcl_pose", 2, amclPoseCallback);
  
  hasObject = false;
  pickup_position.x = 4.0;
  pickup_position.y = 6.0;
  pickup_position.z = 0.0;

  dropoff_position.x = -3.0;
  dropoff_position.y = 4.0;
  dropoff_position.z = 0.0;
  
// %EndTag(INIT)%

  // Set our initial shape type to be a cube
// %Tag(SHAPE_INIT)%
  uint32_t shape = visualization_msgs::Marker::CUBE;
// %EndTag(SHAPE_INIT)%

// %Tag(MARKER_INIT)%
//  while (ros::ok())
//  {
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
// %EndTag(MARKER_INIT)%

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
// %Tag(NS_ID)%
    marker.ns = "add_markers";
    marker.id = 0;
// %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
// %Tag(TYPE)%
    marker.type = visualization_msgs::Marker::CUBE; //shape;
// %EndTag(TYPE)%

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
// %Tag(ACTION)%
    marker.action = visualization_msgs::Marker::ADD;
// %EndTag(ACTION)%

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
// %Tag(POSE)%
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
// %EndTag(POSE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
// %Tag(SCALE)%
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
// %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
// %Tag(COLOR)%
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(LIFETIME)%
    marker.lifetime = ros::Duration();
// %EndTag(LIFETIME)%

    // Publish the marker
// %Tag(PUBLISH)%
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
  
    // Publish the marker at the pickup zone
    ROS_INFO("Publishing the marker at the pickup zone");
    marker.pose.position = pickup_position;
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);

  
  /**
   ros::spin() will enter a loop, pumping callbacks.  With this version, all
   callbacks will be called from within this thread (the main one).  ros::spin()
   will exit when Ctrl-C is pressed, or the node is shutdown by the master.
  */
    ros::spin();
  
// %EndTag(PUBLISH)%

// %Tag(SLEEP_END)%
//    r.sleep();
//  }
// %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%