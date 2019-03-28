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

#include <boost/thread/mutex.hpp>


// definition of the settings for the camera --> might be adapted --> read in from TXT file

#define TOPIC_POINT_CLOUD_IN "/pico_flexx/points"
#define TOPIC_POINT_CLOUD_OUT1 "/template_matching"
#define TOPIC_POINT_CLOUD_OUT2 "/door_handle"
#define POINT_CLOUD_TEMPLATE_PATH "/home/robot/Desktop/rmb-ml"
#define CAMERA_LINK "/pico_flexx_optical_frame"

// creates vector with pointclouds where each represents a cluster idintified by region growing
class StartHandleDetection
{
public:

	StartHandleDetection(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg);

	void initCameraNode(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg);

	void pointcloudCallback_1(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);	

	void pointcloudCallback_2(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);	

private:
	ros::Publisher pub1_;
	ros::Publisher pub2_;
	ros::NodeHandle nh_;
	ros::Subscriber point_cloud_sub1_;
	ros::Subscriber point_cloud_sub2_;
	sensor_msgs::PointCloud2::Ptr point_cloud_out_msg_;
	
	
	double max_dist_1_;
	double max_dist_2_;
	double overlap_ratio_;
	double diag_BB3D_lim_;

	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_scenery_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr handle_point_cloud_;

	std::string filePathXYZRGB_;
	std::string filePathNormals_;
	std::string filePathFeatures_;
	std::string filePathPCATransformations_;
	std::string filePathBBInformations_;
};

