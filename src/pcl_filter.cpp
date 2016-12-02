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

double_t leafsize_x=0.01f,leafsize_y=0.01f,leafsize_z=0.01f,filter_mean,filter_thresold,segmentation_thresold,ClusterTolerance;
int32_t segmentation_maxiteration,ClusterMinSize,ClusterMaxSize;
double_t passFilterMin_x,passFilterMin_y,passFilterMin_z,passFilterMax_x,passFilterMax_y,passFilterMax_z;

class cloudHandler
{
public:
  cloudHandler()
  {
    pcl_sub = nh.subscribe("/kinect2/qhd/points", 10, &cloudHandler::cloudCB, this);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered", 1);
    pcl_pub1 = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered1", 1);
    pcl_pub2 = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered2", 1);
    pcl_pub3 = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered3", 1);
  }




  void cloudCB(const sensor_msgs::PointCloud2& input)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2 output_passFilter;

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

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass_z;
    pass_z.setInputCloud (cloud_downsampled->makeShared());
    pass_z.setFilterFieldName ("z");
    pass_z.setFilterLimits (passFilterMin_z,passFilterMax_z);
    pass_z.filter (*cloud_filtered);
    pass_z.filter(*indices);


    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statFilter;
    statFilter.setInputCloud(cloud_filtered->makeShared());
    statFilter.setMeanK(filter_mean);
    statFilter.setStddevMulThresh(filter_thresold);
    statFilter.filter(*cloud_filtered);

    pcl::toROSMsg(*cloud_filtered, output_passFilter);
    pcl_pub1.publish(output_passFilter);

    //        pcl::toROSMsg(*cloud_filtered, output);
    //        pcl_pub2.publish(output);

    //        pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
    //        pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    //        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    //        normal_estimator.setSearchMethod (tree);
    //        normal_estimator.setInputCloud (cloud_filtered->makeShared());
    //        normal_estimator.setKSearch (50);
    //        normal_estimator.compute (*normals);



    //          pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    //          reg.setMinClusterSize (ClusterMinSize);
    //          reg.setMaxClusterSize (ClusterMaxSize);
    //          reg.setSearchMethod (tree);
    //          reg.setNumberOfNeighbours (30);
    //          reg.setInputCloud (cloud_filtered->makeShared());
    //          //reg.setIndices (indices);
    //          reg.setInputNormals (normals);
    //          reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    //          reg.setCurvatureThreshold (ClusterTolerance);

    //          std::vector <pcl::PointIndices> clusters;
    //          reg.extract (clusters);

    //                  pcl::toROSMsg(*cloud_filtered, output);
    //                  pcl_pub2.publish(output);
    //          std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    //          std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
    //          std::cout << "These are the indices of the points of the initial" <<
    //            std::endl << "cloud that belong to the first cluster:" << std::endl;
    //          int counter = 0;
    //          while (counter < clusters[0].indices.size ())
    //          {
    //            std::cout << clusters[0].indices[counter] << ", ";
    //            counter++;
    //            if (counter % 10 == 0)
    //              std::cout << std::endl;
    //          }
    //          std::cout << std::endl;

    //          cloud_f = reg.getColoredCloud();


    //          pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    //          pcl::visualization::CloudViewer viewer ("Cluster viewer");
    //          viewer.showCloud(colored_cloud);
    //          while (!viewer.wasStopped ())
    //          {
    //          }


    //        // Create the segmentation object for the planar model and set all the parameters

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

    //          pcl::toROSMsg(*cloud_f, output);
    //          pcl_pub3.publish(output);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered->makeShared());

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
    //  ss << "cloud_cluster_" << j << ".pcd";
    //  writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*

      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloud_cluster,centroid);
      cout<<"x :"<<centroid[0]<<endl;
      cout<<"y :"<<centroid[1]<<endl;
      cout<<"z :"<<centroid[2]<<endl;

      //object_dim_calc(cloud_cluster,j);

      pcl::toROSMsg(*cloud_cluster, output);
      pcl_pub3.publish(output);

      j++;
    }



    //output.header.frame_id = "odom";
    //pcl::io::savePCDFileASCII ("/home/ros/catkin_ws/src/pick_and_place/data/data_pcd.pcd", cloud);
  }

  void object_dim_calc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr  clustered_cloud,int cluster_count)
  {
    // compute principal direction

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*clustered_cloud, centroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*clustered_cloud, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
    eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

    // move the points to the that reference frame
    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    p2w.block<3,3>(0,0) = eigDx.transpose();
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cPoints (new pcl::PointCloud<pcl::PointXYZRGB>);

    //(new pcl::PointCloud<pcl::PointXYZ>)
    pcl::transformPointCloud(*clustered_cloud, *cPoints, p2w);

    //pcl::PointXYZ min_pt, max_pt;
    pcl::PointXYZRGB minPt, maxPt;
    //pcl::getMinMax3D (*cPoints, minPt, maxPt);

    //calculating the Min and Max points in each cluster
    minPt.x = cPoints->points[0].x;
    maxPt.x = cPoints->points[0].x;
    minPt.y = cPoints->points[0].y;
    maxPt.y = cPoints->points[0].y;
    minPt.z = cPoints->points[0].z;
    maxPt.z = cPoints->points[0].z;

    for (size_t i = 1; i < cPoints->points.size (); ++i)
    {
      if(cPoints->points[i].x <= minPt.x )
        minPt.x = cPoints->points[i].x;
      else if(cPoints->points[i].y <= minPt.y )
        minPt.y = cPoints->points[i].y;
      else if(cPoints->points[i].z <= minPt.z )
        minPt.z = cPoints->points[i].z;
      else if(cPoints->points[i].x >= maxPt.x )
        maxPt.x = cPoints->points[i].x;
      else if(cPoints->points[i].y >= maxPt.y )
        maxPt.y = cPoints->points[i].y;
      else if(cPoints->points[i].z >= maxPt.z )
        maxPt.z = cPoints->points[i].z;
    }

    //Dimensions of the Bounded Box
    std::cout<<"Cluster : "<<cluster_count +1 <<std::endl;
    ROS_INFO("The length of the box is: %f",maxPt.x - minPt.x);
    ROS_INFO("The width of the box is: %f",maxPt.y - minPt.y);
    ROS_INFO("The depth of the box is: %f",maxPt.z - minPt.z);

    std::stringstream sh,sl,sb,sd;
    sh<<  "Cluster : "<<cluster_count +1      <<std::endl;
    sl << "Length  : "  << maxPt.x - minPt.x  <<std::endl;
    sb << "Breadth : "  << maxPt.y - minPt.y  <<std::endl;
    sd << "Depth   : "  << maxPt.z - minPt.z  <<std::endl;



    const Eigen::Vector3f mean_diag = 0.5f*(maxPt.getVector3fMap() + minPt.getVector3fMap());

    // final transform
    const Eigen::Quaternionf qfinal(eigDx);
    const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();

    // draw the cloud and the box
    pcl::visualization::PCLVisualizer viewer;
    viewer.addPointCloud(clustered_cloud);
    viewer.addCube(tfinal, qfinal, maxPt.x - minPt.x, maxPt.y - minPt.y, maxPt.z - minPt.z);
    viewer.addText(sh.str(),100,250,"heading", 0);
    viewer.addText(sl.str(),100,200,"length", 0);
    viewer.addText(sb.str(),100,150,"breadth", 0);
    viewer.addText(sd.str(),100,100,"depth", 0);
    viewer.spin();
  }

protected:
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Publisher pcl_pub;
  ros::Publisher pcl_pub1;
  ros::Publisher pcl_pub2;
  ros::Publisher pcl_pub3;
};

void callback(pick_and_place::pcd_dataConfig &config, uint32_t level)
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
  ros::init(argc, argv, "pcl_filter");

  dynamic_reconfigure::Server<pick_and_place::pcd_dataConfig> server;
  dynamic_reconfigure::Server<pick_and_place::pcd_dataConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  cloudHandler handler;

  ros::spin();

  return 0;
}


