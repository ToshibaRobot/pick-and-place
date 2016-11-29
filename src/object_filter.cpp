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
#include <chapter6_tutorials/pcd_dataConfig.h>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include<vector>

using namespace std;

double_t leafsize_x=0.01f,leafsize_y=0.01f,leafsize_z=0.01f,filter_mean,filter_thresold,segmentation_thresold,ClusterTolerance;
int32_t segmentation_maxiteration,ClusterMinSize,ClusterMaxSize;
double_t passFilterMin_x,passFilterMin_y,passFilterMin_z,passFilterMax_x,passFilterMax_y,passFilterMax_z;

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("/kinect2/hd/points", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered", 1);
        pcl_pub1 = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered1", 1);
        pcl_pub2 = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered2", 1);
        pcl_pub4 = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered3", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2& input)
    {
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
         sensor_msgs::PointCloud2 output;
         //sensor_msgs::PointCloud2 output_passFilter;

        pcl::fromROSMsg(input, *cloud);

        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZRGB> voxelSampler;
        voxelSampler.setInputCloud(cloud->makeShared());
        voxelSampler.setLeafSize(leafsize_x,leafsize_x,leafsize_x);
        voxelSampler.filter(*cloud_downsampled);

        pcl::toROSMsg(*cloud_downsampled, output);
        pcl_pub.publish(output);



        pcl::PassThrough<pcl::PointXYZRGB> pass_x;
        pass_x.setInputCloud (cloud_downsampled->makeShared());
        pass_x.setFilterFieldName ("x");
        pass_x.setFilterLimits (passFilterMin_x,passFilterMax_x);
        pass_x.filter (*cloud_downsampled);

        pcl::PassThrough<pcl::PointXYZRGB> pass_y;
        pass_y.setInputCloud (cloud_downsampled->makeShared());
        pass_y.setFilterFieldName ("y");
        pass_y.setFilterLimits (passFilterMin_y,passFilterMax_y);
        pass_y.filter (*cloud_downsampled);

        pcl::PassThrough<pcl::PointXYZRGB> pass_z;
        pass_z.setInputCloud (cloud_downsampled->makeShared());
        pass_z.setFilterFieldName ("z");
        pass_z.setFilterLimits (passFilterMin_z,passFilterMax_z);
        pass_z.filter (*cloud_filtered);


        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statFilter;
        statFilter.setInputCloud(cloud_filtered->makeShared());
        statFilter.setMeanK(filter_mean);
        statFilter.setStddevMulThresh(filter_thresold);
        statFilter.filter(*cloud_filtered);

        pcl::toROSMsg(*cloud_filtered, output);
        pcl_pub1.publish(output);

        //std::vector<int> points_remove;

//        int red,green,blue;
//        for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
//        {
//            red=int(cloud_filtered->points[i].r);
//            green=int(cloud_filtered->points[i].g);
//            blue=int(cloud_filtered->points[i].b);

//            if(red<=255 && red>=250 && green<=255 && green>=250 && blue<=255 && blue>=250)
//            {
//              cloud_filtered->points[i].rgb=0;
//            }

//        }






//        pcl::toROSMsg(*cloud_filtered, output);
//        pcl_pub2.publish(output);

////        // Create the segmentation object for the planar model and set all the parameters

//        pcl::SACSegmentation<pcl::PointXYZ> seg;
//        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::PCDWriter writer;
//        seg.setOptimizeCoefficients (true);
//        seg.setModelType (pcl::SACMODEL_PLANE);
//        seg.setMethodType (pcl::SAC_RANSAC);
//        seg.setMaxIterations (segmentation_maxiteration);
//        seg.setDistanceThreshold (segmentation_thresold);

//        int i=0, nr_points = (int) cloud_filtered->points.size ();
//          while (cloud_filtered->points.size () > 0.3 * nr_points)
//          {
//            // Segment the largest planar component from the remaining cloud
//            seg.setInputCloud (cloud_filtered);
//            seg.segment (*inliers, *coefficients);
//            if (inliers->indices.size () == 0)
//            {
//              std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//              break;
//            }

//            // Extract the planar inliers from the input cloud
//            pcl::ExtractIndices<pcl::PointXYZ> extract;
//            extract.setInputCloud (cloud_filtered);
//            extract.setIndices (inliers);
//            extract.setNegative (false);

//            // Get the points associated with the planar surface
//            extract.filter (*cloud_plane);
//            std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

//            // Remove the planar inliers, extract the rest
//            extract.setNegative (true);
//            extract.filter (*cloud_f);
//            *cloud_filtered = *cloud_f;
//          }

//          pcl::toROSMsg(*cloud_filtered, output);
//          pcl_pub2.publish(output);

          // Creating the KdTree object for the search method of the extraction
          pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
          tree->setInputCloud (cloud_filtered);

          std::vector<pcl::PointIndices> cluster_indices;
          pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
          ec.setClusterTolerance (ClusterTolerance); // 2cm
          ec.setMinClusterSize (ClusterMinSize);
          ec.setMaxClusterSize (ClusterMaxSize);
          ec.setSearchMethod (tree);
          ec.setInputCloud (cloud_filtered);
          ec.extract (cluster_indices);

        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
          int j = 0;
          for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
          {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
              cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
            std::stringstream ss;
            ss << "cloud_cluster_" << j << ".pcd";
            writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
            j++;
            std::string pcl_clust="pcl_cluster"+j;
            pcl_pub3 = nh.advertise<sensor_msgs::PointCloud2>("pcl_clust", 1);
            pcl::toROSMsg(*cloud_cluster, output);

            pcl_pub3.publish(output);
          }

      output.header.frame_id = "odom";

        //output.header.frame_id = "odom";
        //pcl::io::savePCDFileASCII ("/home/ros/catkin_ws/src/chapter6_tutorials/data/data_pcd.pcd", cloud);
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
    ros::Publisher pcl_pub1;
    ros::Publisher pcl_pub2;
    ros::Publisher pcl_pub3;
    ros::Publisher pcl_pub4;
};

void callback(chapter6_tutorials::pcd_dataConfig &config, uint32_t level)
{

  leafsize_x = config.leafsize_x;
  leafsize_y = config.leafsize_y;
  leafsize_z = config.leafsize_z;
  filter_mean = config.filter_mean;
  filter_thresold = config.filter_thresold;
  segmentation_thresold = config.segmentation_thresold;
  segmentation_maxiteration = config.segmentation_maxiteration;
  passFilterMin_x = config.passFilterMin_x;
  passFilterMin_y = config.passFilterMin_y;
  passFilterMin_z = config.passFilterMin_z;
  passFilterMax_x = config.passFilterMax_x;
  passFilterMax_y = config.passFilterMax_y;
  passFilterMax_z = config.passFilterMax_z;
  ClusterTolerance = config.ClusterTolerance;
  ClusterMaxSize = config.ClusterMaxSize;
  ClusterMinSize = config.ClusterMinSize;
 //           config.str_param.c_str(),
  //          config.bool_param?"True":"False",
   //         config.size);
}

main(int argc, char** argv)
{
    ros::init(argc, argv, "object_filter");

    dynamic_reconfigure::Server<chapter6_tutorials::pcd_dataConfig> server;
    dynamic_reconfigure::Server<chapter6_tutorials::pcd_dataConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
    cloudHandler handler;

    ros::spin();

    return 0;
}


