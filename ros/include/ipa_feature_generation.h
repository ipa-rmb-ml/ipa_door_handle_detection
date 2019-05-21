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




class FeatureGeneration
{
public:

FeatureGeneration(std::string file_path_to_point_clouds,std::string handle_type);

static void StartFeatureGeneration(std::string file_path_to_point_clouds);

void generateDepthImages(std::string file_path_to_point_clouds, std::string handle_type);

void createDirectory(std::string path);

private:
std::string TEMPLATE_PATH_;
std::string BASE_PATH_;
std::string targetPathFeatures_;

};