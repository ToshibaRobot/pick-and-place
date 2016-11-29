#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

void cloudCB(const sensor_msgs::PointCloud2 &input)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
    pcl::fromROSMsg(input, cloud);
    pcl::io::savePCDFileASCII ("/home/ros/catkin_ws/src/chapter6_tutorials/data/dataRGB_pcd.pcd", cloud);
}

main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_write");

    ros::NodeHandle nh;
    ros::Subscriber bat_sub = nh.subscribe("/kinect2/hd/points", 10, cloudCB);

    ros::spin();

    return 0;
}


