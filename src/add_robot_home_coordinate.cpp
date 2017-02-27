
/**********************************************************************************************************************************

  Description:This program is used for adding the Robot origin(world co-ordinate)frame in to the TF tree with respect to camera.
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
  ros::init(argc, argv, "add_robot_home_coordinate"); // create a node "add_robot_home_coordinate"
  ros::NodeHandle node;
  tf::TransformBroadcaster br; // create a object of broadcaster
  tf::Transform transform; // create a object of transform

  ros::Rate rate(10.0);
  while (node.ok())
  {
    transform.setOrigin( tf::Vector3(0.65,0,0.1) ); // set the origin of the robot origin (world co-ordinate) frame with respect to end_effector_frame. The input values should be in meter
    tf::Quaternion q;
    q.setRPY(3.14159, 0, 3.14159); // set the orientation of the robot origin (world co-ordinate) frame with respect to end_effector_frame. The input values should be in radian.
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "robot_home_coordinate", "end_effector_frame"));  // broadcast the transformation in to the TF-tree.
    rate.sleep();
  }
  return 0;
}
