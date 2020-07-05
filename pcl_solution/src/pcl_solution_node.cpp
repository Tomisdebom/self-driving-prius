#include <iostream>
#include <vector> 

#include <boost/foreach.hpp>

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <math.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/common/centroid.h>

#include "vision_msgs/Detection3DArray.h"
#include "vision_msgs/Detection3D.h"
#include "vision_msgs/ObjectHypothesisWithPose.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>

#include <Eigen/Dense>

#include <pcl/common/common.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PCLconverter{

  // Initialize all subscribers and publishers
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/point_cloud", 1, &PCLconverter::callback, this);
  ros::Publisher det = nh.advertise<vision_msgs::Detection3DArray>("pcl_solution_node/detections", 1);

  public:
  void callback(const sensor_msgs::PointCloud2& total_cloud)
  {
    // Initialize cofficients and inliers
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Initialize the full pointcloud and convert it to a ROS message to be able to analyse it
    pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(total_cloud, *full_cloud);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Set all configurations for the filter   
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.3);
    seg.setInputCloud (full_cloud);
    seg.segment (*inliers, *coefficients);
    
    // Extract all inliers from the full cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(full_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);

    // Initialize the final cloud and tree 
    std::vector<pcl::PointIndices> final_cloud;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    // Set all configurations for the filter
    tree->setInputCloud (obstacle_cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> clus;
    clus.setClusterTolerance (0.5);
    clus.setMinClusterSize (10);
    clus.setMaxClusterSize (25000);
    clus.setSearchMethod(tree);
    clus.setInputCloud(obstacle_cloud);

    // Extract final cloud after filtering the cluster 
    final_cloud.clear();
    clus.extract(final_cloud);
  
    // Initialize vectors for centroid and minimum and maximum points in the obstacle cloud
    Eigen::Vector4f centroid;
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;	

    // Initialize detection3D detection and detection3Darray
    vision_msgs::Detection3D detection;
    vision_msgs::Detection3DArray detections;

    // Loop trough the final cloud and compute centroids for each obstacle cluster, also get minimum and maximum point of each cloud
    for(int i=0;i<final_cloud.size();i++){
      pcl::compute3DCentroid(*obstacle_cloud,final_cloud[i],centroid);
      pcl::getMinMax3D(*obstacle_cloud, final_cloud[i],min_pt, max_pt);

      // Set a position to each detections
      detection.bbox.center.position.x = centroid[0];
      detection.bbox.center.position.y = centroid[1];
      detection.bbox.center.position.z = centroid[2];

      // Set a size to each detections
      detection.bbox.size.x = fabs(max_pt[0] - min_pt[0]);
      detection.bbox.size.y = fabs(max_pt[1] - min_pt[1]);
      detection.bbox.size.z = fabs(max_pt[2] - min_pt[2]);

      // Add the pointcloud header to eaach detection
      detection.header = total_cloud.header;
      
      // Add each detection to the list of deteections
      detections.detections.push_back(detection);
    } 

    detections.header = total_cloud.header;
    det.publish(detections);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  PCLconverter ic;
  ros::spin();
  return (0);
}
