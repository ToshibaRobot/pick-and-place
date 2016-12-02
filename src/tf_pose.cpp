#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include<tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include<iostream>
#include <math.h>


void cb(ar_track_alvar_msgs::AlvarMarkers req) {
  tf::TransformBroadcaster tf_br;
  tf::TransformListener listener;
  tf::Transform transform;
  if (!req.markers.empty()) {
    tf::Quaternion q(req.markers[0].pose.pose.orientation.x, req.markers[0].pose.pose.orientation.y, req.markers[0].pose.pose.orientation.z, req.markers[0].pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    transform.setOrigin( tf::Vector3(req.markers[0].pose.pose.position.x, req.markers[0].pose.pose.position.y, req.markers[0].pose.pose.position.z) );
    transform.setRotation(tf::Quaternion( req.markers[0].pose.pose.orientation.x, req.markers[0].pose.pose.orientation.y, req.markers[0].pose.pose.orientation.z, req.markers[0].pose.pose.orientation.w));

    try{
      listener.waitForTransform("/camera_link", "/marker_frame", ros::Time::now(), ros::Duration(1.0));
      tf_br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "marker_frame", "camera_link"));
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
}

void cb1(geometry_msgs::Point req) {
  tf::TransformBroadcaster tf_br1;
  tf::TransformListener listener1;
  tf::Transform transform1;
  //if (req.Point) {
    //tf::Quaternion q(req.x, req.y, req.z, req.);
    //tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    //m.getRPY(roll, pitch, yaw);

    transform1.setOrigin( tf::Vector3(req.x, req.y, req.z) );
    //transform.setRotation(tf::Quaternion( req.markers[0].pose.pose.orientation.x, req.markers[0].pose.pose.orientation.y, req.markers[0].pose.pose.orientation.z, req.markers[0].pose.pose.orientation.w));

    try{
      listener1.waitForTransform("/world_frame", "/gripper_frame", ros::Time::now(), ros::Duration(1.0));
      tf_br1.sendTransform(tf::StampedTransform(transform1.inverse(), ros::Time::now(), "gripper_frame", "world_frame"));
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
  }
//  }
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_pose");
  ros::NodeHandle nh;
  //  ROS_INFO("INSIDE main ar_listener . . . . .");
  ros::Subscriber sub = nh.subscribe("ar_pose_marker", 1, cb);
  ros::Subscriber sub1 = nh.subscribe("current_pose", 1, cb1);
  ros::spin();
  return 0;
  
}
