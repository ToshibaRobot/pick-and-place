/**********************************************************************************************************************************

  Description:This program is used for displaying the final object in the pcl visualizer .
  Auther     :Hirenbhai Patel & Avinash Jain

 **********************************************************************************************************************************/

#include <iostream>
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class cloudHandler
{
public:
    cloudHandler()
    : viewer("Cloud Viewer")
    {
        pcl_sub = nh.subscribe("pcl_filtered3", 10, &cloudHandler::cloudCB, this); // subscribe the topic "pcl_filtered3"
        viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB, this); // create a timer for viewer
    }

    //this function will tack the data from ros message data type to this node and display the cloud data in to a viewer
    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg(input, cloud);

        viewer.showCloud(cloud.makeShared());
    }

    //timer for updating the data in to the viewer
    void timerCB(const ros::TimerEvent&)
    {
        if (viewer.wasStopped())
        {
            ros::shutdown();
        }
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    pcl::visualization::CloudViewer viewer;
    ros::Timer viewer_timer;
};

main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_visualize"); // create a node "pcl_visualize"

    cloudHandler handler; // create a object of a class

    ros::spin();

    return 0;
}

