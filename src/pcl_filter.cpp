/**********************************************************************************************************************************

  Description:This program is used for detecting the objects from the conveyor belt. It will give the centre co-ordinate of the object
              with respect to the camera.
  Auther     :Hirenbhai Patel & Avinash Jain

 **********************************************************************************************************************************/

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <dynamic_reconfigure/server.h>
#include <pick_and_place/pcd_dataConfig.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include "pick_and_place/object_coordinate.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include<tf/transform_broadcaster.h>
//belowed all variable is used for Dynamic configuration of the Node. Please look at the ros.org website for details.
double_t leafsize_x=0.01f,leafsize_y=0.01f,leafsize_z=0.01f,filter_mean,filter_thresold,segmentation_thresold,ClusterTolerance;
int32_t segmentation_maxiteration,ClusterMinSize,ClusterMaxSize;
double_t passFilterMin_x,passFilterMin_y,passFilterMin_z,passFilterMax_x,passFilterMax_y,passFilterMax_z;


class cloudHandler
{
public:
  cloudHandler()
  {

    pcl_sub = nh.subscribe("/camera/depth_registered/points", 10, &cloudHandler::cloudCB, this); // subscribe the topic "/kinect2/qhd/points" from iai_kinnect package

    // various publishers for analysing the points cloud
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered", 1);
    pcl_pub1 = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered1", 1);
    pcl_pub2 = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered2", 1);
    pcl_pub3 = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered3", 1);
    pub = nh.advertise<pick_and_place::object_coordinate>("/object_current_pos",1000);

 //   pcl_pub4 = nh.advertise<sensor_msgs::PointCloud2>("pcl_transformed", 1);

  }

  tf::TransformListener *tf_listener;
  tf::Transform transform;

  void cloudCB(const sensor_msgs::PointCloud2& input)
  {

//    tf_listener->waitForTransform("/world", input.header.frame_id, input.header.stamp, ros::Duration(5.0));
//     pcl_ros::transformPointCloud("/world", input, &input, tf_listener);



//    //tf_listener->waitForTransform("/world", input.header.frame_id, input.header.stamp, ros::Duration(5.0));
//   // pcl_ros::transformPointCloud("/world", input, &input, tf_listener);

//  //  tf_listener->lookupTransform(input.header.frame_id, "/world",ros::Time(0), &tf_listener);
//  //   pcl_ros::transformPointCloud (input, input, &tf_listener);


//  //  pcl_pub4.publish(input); //Publish the cluster on the "pcl_filtered3" topic

//   //waiting to get available tranformation between cloud1 and reference frame
//      rTfListener->waitForTransform(sReferenceFrame, (*rInputCloud1).header.frame_id, (*rInputCloud1).header.stamp, ros::Duration(5.0));
//      // transforming point cloud from camera frame to reference frame
//      pcl_ros::transformPointCloud(sReferenceFrame, *rInputCloud1, rTransformedToWorldCloud1, *rTfListener);
//      // converting from ros sensor msg to pcl type. so that we can add it together.
//      pcl::fromROSMsg(rTransformedToWorldCloud1, pclTransformedToWorldCloud1);
    pick_and_place::object_coordinate object_pose_msg;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2 output_passFilter;

    // read the cloud
    pcl::fromROSMsg(input, *cloud); // Read the point cloud for further processing

    // Downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelSampler;
    voxelSampler.setInputCloud(cloud->makeShared());
    voxelSampler.setLeafSize(leafsize_x,leafsize_x,leafsize_x);// leafsize = 0.01 = 1 cm The values to be tacken is in meters. The values of the leafsize should be in meters
    voxelSampler.filter(*cloud_downsampled);

    // Convert the the point cloud to ROS Massage type and publish on the topic "pcl_filtered"
    pcl::toROSMsg(*cloud_downsampled, output);
    pcl_pub.publish(output);

    //Filter the Point cloud in X-Direction both from positive and negative
    pcl::PassThrough<pcl::PointXYZRGB> pass_x;
    pass_x.setInputCloud (cloud_downsampled->makeShared());
    pass_x.setFilterFieldName ("x");
    pass_x.setFilterLimits (passFilterMin_x,passFilterMax_x); //passFilterMin_x = 1 = 1 m  The values to be tacken is in meters. The values of the Filterlimits should be in meters
    pass_x.filter (*cloud_downsampled);

    //Filter the Point cloud in Y-Direction both from positive and negative
    pcl::PassThrough<pcl::PointXYZRGB> pass_y;
    pass_y.setInputCloud (cloud_downsampled->makeShared());
    pass_y.setFilterFieldName ("y");
    pass_y.setFilterLimits (passFilterMin_y,passFilterMax_y); //passFilterMin_y = 1 = 1 m  The values to be tacken is in meters. The values of the Filterlimits should be in meters
    pass_y.filter (*cloud_downsampled);

    //Filter the Point cloud in Z-Direction both from positive and negative
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass_z;
    pass_z.setInputCloud (cloud_downsampled->makeShared());
    pass_z.setFilterFieldName ("z");
    pass_z.setFilterLimits (passFilterMin_z,passFilterMax_z); //passFilterMin_z = 1 = 1 m  The values to be tacken is in meters. The values of the Filterlimits should be in meters
    pass_z.filter (*cloud_filtered);
    pass_z.filter(*indices);

    // remove all points who have a distance larger than filter_thresold standard deviation of the mean distance
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statFilter;
    statFilter.setInputCloud(cloud_filtered->makeShared());
    statFilter.setMeanK(filter_mean); // The number of neighbors to analyze for each point
    statFilter.setStddevMulThresh(filter_thresold); // filter_thresold should be standard deviation.
    statFilter.filter(*cloud_filtered);

    pcl::toROSMsg(*cloud_filtered, output_passFilter);
    pcl_pub1.publish(output_passFilter); // Publish the filter cloud on the topic "pcl_filtered1"


    //pcl::PCDWriter writer;


    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered->makeShared());
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (ClusterTolerance); // The value of ClusterTolerence should be in meters
    ec.setMinClusterSize (ClusterMinSize); // ClusterMinSize is number of points.
    ec.setMaxClusterSize (ClusterMaxSize); // ClusterMaxSize is number of points.
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    // Below loop will extract the number of cluster one by one.
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      //std::stringstream ss;
      //ss << "cloud_cluster_" << j << ".pcd";
      //writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*

      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloud_cluster,centroid); // calculate the centre point of the cluster
      cout<<"x :"<<centroid[0]<<endl;
      cout<<"y :"<<centroid[1]<<endl;
      cout<<"z :"<<centroid[2]<<endl;

      object_pose_msg.PosX=centroid[0];
      object_pose_msg.PosY=centroid[1];
      object_pose_msg.PosZ=centroid[2];

      pcl::toROSMsg(*cloud_cluster, output);
      pcl_pub3.publish(output); //Publish the cluster on the "pcl_filtered3" topic
      j++;


      pub.publish(object_pose_msg);
    }

  }

protected:
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Publisher pcl_pub;
  ros::Publisher pcl_pub1;
  ros::Publisher pcl_pub2;
  ros::Publisher pcl_pub3;
  ros::Publisher pcl_pub4;
  ros::Publisher pub ;

};


//This is the function for getting the vlues of the varialble from the Dynamic reconfiguration file.
void callback(pick_and_place::pcd_dataConfig &config, uint32_t level)
{
  leafsize_x = config.leafsize_x; // leafsize for downsampling the cloud in x-direction
  leafsize_y = config.leafsize_y; // leafsize for downsampling the cloud in y-direction
  leafsize_z = config.leafsize_z; // leafsize for downsampling the cloud in z-direction
  filter_mean = config.filter_mean; // The number of neighbors to analyze for each point
  filter_thresold = config.filter_thresold; // Standard deviation
  segmentation_thresold = config.segmentation_thresold;
  segmentation_maxiteration = config.segmentation_maxiteration;
  passFilterMin_x = config.passFilterMin_x; // minumum values for filtering in negative x-direction
  passFilterMin_y = config.passFilterMin_y; // minumum values for filtering in negative y-direction
  passFilterMin_z = config.passFilterMin_z; // minumum values for filtering in negative z-direction
  passFilterMax_x = config.passFilterMax_x; // minumum values for filtering in positive x-direction
  passFilterMax_y = config.passFilterMax_y; // minumum values for filtering in positive y-direction
  passFilterMax_z = config.passFilterMax_z; // minumum values for filtering in positive z-direction
  ClusterTolerance = config.ClusterTolerance;
  ClusterMaxSize = config.ClusterMaxSize; // Maximum number of points in a cluster
  ClusterMinSize = config.ClusterMinSize; // Minimum number of points in a cluster

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_filter");

  //create dynamic reconfiguration node
  dynamic_reconfigure::Server<pick_and_place::pcd_dataConfig> server;
  dynamic_reconfigure::Server<pick_and_place::pcd_dataConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  cloudHandler handler; // create a object for a class

  ros::spin();

  return 0;
}


