#include <iostream>
#include <ros/ros.h>
#include <utility>
#include <math.h>

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
#include <pcl/common/transforms.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/common/projection_matrix.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/correspondence.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>


#include <boost/thread/mutex.hpp>

// struct for storing point cloud plane information

	struct planeInformation
	{
		pcl::PointIndices::Ptr plane_point_cloud_indices;
		pcl::ModelCoefficients::Ptr plane_coeff;
	};

	struct pcaInformation
	{
		Eigen::Matrix4f pca_transformation;
		Eigen::Vector3f bounding_box_3D;
	};

// creates vector with pointclouds where each represents a cluster idintified by region growing
class PointCloudSegmentation
{
public:

	PointCloudSegmentation();

	//main function og the segmentation class
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > segmentPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr changePointCloudColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);

	planeInformation detectPlaneInPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

	bool  alignCylinderToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud,pcl::PointCloud<pcl::Normal>::Ptr input_point_cloud_normals, pcl::ModelCoefficients::Ptr plane_coeff);

	double checkOrientationAndGeometry(pcl::ModelCoefficients::Ptr cylinder_coeff,pcl::ModelCoefficients::Ptr plane_point_cloud_indices);

	pcaInformation calculatePCA(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud);

	// removing all object points that are too distant from the door plane
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr minimizePointCloudToObject(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,pcl::PointIndices::Ptr plane_pc_indices,pcl::ModelCoefficients::Ptr plane_coeff);
	
	// apply region growing to identifie all cluster in front of the detected door
	std::vector <pcl::PointIndices> findClustersByRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced_pc);

	// generate vector of pointclouds representing each of the cluster
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > generateAlignmentObject(std::vector <pcl::PointIndices> clusters,pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced_pc,pcl::ModelCoefficients::Ptr plane_coeff);

	// used to project cluster points on detected plane to remove all addition points of the point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr projectPointsOnPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder_points,pcl::ModelCoefficients::Ptr plane_coeff);

	bool checkBB3DOrientation(Eigen::Matrix4f,pcl::ModelCoefficients::Ptr cylinder_coeff);




private:
// filter points by distance
double min_point_to_plane_dist_;
double max_point_to_plane_dist_;

double min_height_door_handle_;
double max_height_door_handle_;

double max_door_robot_;

double cylinder_rad_min_;
double cylinder_rad_max_;

int max_num_iter_;
double distance_thres_;

int min_cluster_size_;
int max_cluster_size_;

double max_diff_norm_axis_;
double inlier_ratio_thres_;

double angle_thres_x_;

};

