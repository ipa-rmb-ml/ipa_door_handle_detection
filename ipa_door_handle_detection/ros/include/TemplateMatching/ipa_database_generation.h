#include <iostream>
#include <ros/ros.h>
#include <utility>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/common/projection_matrix.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/registration/icp.h>

#include <pcl/registration/icp_nl.h>
#include <boost/thread/mutex.hpp>

#include<stdio.h>
#include<cstdlib>
#include<string.h>
#include<fstream>
#include<dirent.h>
#include <bits/stdc++.h> 
#include "sensor_msgs/Image.h"


class DoorhandleDatabaseGeneration
{
public:

    DoorhandleDatabaseGeneration(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg);

	void initCameraNode(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg);

	void pointcloudCallback_1(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);	

	void pointcloudCallback_2(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);

	std::string getFilePathFromParameter(int dist, int angle_XZ, int angle_YZ);

private:
	ros::Publisher pub_;
	ros::NodeHandle nh_;
	sensor_msgs::PointCloud2::Ptr point_cloud_out_msg_;
	ros::Subscriber point_cloud_sub_;
	Eigen::Vector3f setup_;


};
