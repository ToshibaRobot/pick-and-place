
///**********************************************************************************************************************************

//  Description:This program is used for adding the alver marker frame in to the TF tree with respect to camera.
//  Author     :Hirenbhai Patel & Avinash Jain

// **********************************************************************************************************************************/

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include<tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include<iostream>
#include <math.h>


void cb(ar_track_alvar_msgs::AlvarMarkers req) {
  tf::TransformBroadcaster tf_br; // create a object for broadcaster
  tf::TransformListener listener; // create a object for listener
  tf::Transform transform; // create a object for transform
  if (!req.markers.empty()) {
    tf::Quaternion q(req.markers[0].pose.pose.orientation.x, req.markers[0].pose.pose.orientation.y, req.markers[0].pose.pose.orientation.z, req.markers[0].pose.pose.orientation.w); // get the position and orientation from the marker and add it to the Quaternion
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); // get orientation of the marker

    //give the position and orientation of the marker to the transform to set origin and the orientation
    transform.setOrigin( tf::Vector3(req.markers[0].pose.pose.position.x, req.markers[0].pose.pose.position.y, req.markers[0].pose.pose.position.z) ); // input values should be in metre
    transform.setRotation(tf::Quaternion( req.markers[0].pose.pose.orientation.x, req.markers[0].pose.pose.orientation.y, req.markers[0].pose.pose.orientation.z, req.markers[0].pose.pose.orientation.w)); // input value should be in radian

    try{
      listener.waitForTransform("/camera_link", "/ar_marker_0", ros::Time::now(), ros::Duration(1000.0));
      tf_br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "marker_frame", "/camera_link")); // get the transform between two frames and add it to the TF-tree
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_pose"); // create a node "tf_pose"
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("ar_pose_marker", 1, cb); // subscribe a topic "ar_pose_marker" from alver marker package
  ros::spin();
  return 0;
  
}


//#include <ros/ros.h>
//#include <tf/transform_datatypes.h>
//#include <ar_track_alvar_msgs/AlvarMarkers.h>
//#include<tf/transform_broadcaster.h>
//#include <tf/transform_listener.h>
//#include<iostream>
//#include <math.h>
//void cb(ar_track_alvar_msgs::AlvarMarkers req) {

//  ROS_INFO("INSIDE main ar_listener . . . . .");

//  tf::TransformBroadcaster tf_br;
//  tf::TransformListener listener;
//  tf::Transform transform;
//  if (!req.markers.empty()) {
//    tf::Quaternion q(req.markers[0].pose.pose.orientation.x, req.markers[0].pose.pose.orientation.y, req.markers[0].pose.pose.orientation.z, req.markers[0].pose.pose.orientation.w);
//    tf::Matrix3x3 m(q);
//    double roll, pitch, yaw;
//    m.getRPY(roll, pitch, yaw);

//    transform.setOrigin( tf::Vector3(req.markers[0].pose.pose.position.x, req.markers[0].pose.pose.position.y, req.markers[0].pose.pose.position.z) );
//    transform.setRotation(tf::Quaternion( req.markers[0].pose.pose.orientation.x, req.markers[0].pose.pose.orientation.y, req.markers[0].pose.pose.orientation.z, req.markers[0].pose.pose.orientation.w));
//    try{
//      listener.waitForTransform("/camera_link", "/marker", ros::Time::now(), ros::Duration(1.0));
//      tf_br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "marker", "camera_link"));
//    }
//    catch (tf::TransformException ex){
//      ROS_ERROR("%s",ex.what());
//      ros::Duration(1.0).sleep();
//    }
//  } // if
//}
//int main(int argc, char **argv) {
//  ros::init(argc, argv, "tf_pose");
//  ros::NodeHandle nh;
//  //  ROS_INFO("INSIDE main ar_listener . . . . .");
//  ros::Subscriber sub = nh.subscribe("ar_pose_marker", 1, cb);
//  ros::spin();
//  return 0;

//}
