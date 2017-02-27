
/**********************************************************************************************************************************

  Description:This program is used for adding the end effector frame in to the TF tree with respect to camera.
  Auther     :Hirenbhai Patel & Avinash Jain

 **********************************************************************************************************************************/

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include<tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include<iostream>
#include <math.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "add_end_effactor_frame"); // create a node "add_end_effactor_frame"
  ros::NodeHandle node;

  tf::TransformBroadcaster br; // create a object of broadcaster
  tf::Transform transform; // create a object of transform

  ros::Rate rate(10.0);
  while (node.ok())
  {
    transform.setOrigin( tf::Vector3(0,0.07,0.065) ); // set the origin of the end-effector frame with respect to ar_marker. The input values should be in meter
    tf::Quaternion q;
    q.setRPY(3.14159, 0, 0);// set the orientation of the end-effector frame with respect to ar_marker. The input values should be in radian.
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ar_marker_0", "end_effector_frame")); // broadcast the transformation in to the TF-tree.
    rate.sleep();
  }
  return 0;
}
