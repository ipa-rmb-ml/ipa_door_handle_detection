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




class DoorHandleTemplateGeneration
{
public:

DoorHandleTemplateGeneration(std::string file_path_to_point_clouds,std::string handle_type);

static void StartTemplateGeneration(std::string file_path_to_point_clouds);

void generateTemplatePCLFiles(std::string file_path_to_point_clouds, std::string handle_type);

void createDirectory(std::string path);

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > generateTemplateAlignmentObject(std::vector <pcl::PointIndices> clusters,pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced_pc, pcl::ModelCoefficients::Ptr plane_coeff);


private:

std::string targetPathXYZRGB_;
std::string targetPathNormals_;
std::string targetPathFeatures_;
std::string targetPathPCA_;
std::string targetPathEigen_;
std::string targetPathBB_;
std::string TEMPLATE_PATH_;
std::string BASE_PATH_;

};