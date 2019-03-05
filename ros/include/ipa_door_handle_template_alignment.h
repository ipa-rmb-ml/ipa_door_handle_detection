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
#include <pcl/registration/correspondence_rejection_median_distance.h>
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
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/don.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/search/organized.h>


#include <iostream>
#include <fstream>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/features/principal_curvatures.h>
#include <math.h>

#include <boost/thread/mutex.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <stdlib.h>                                                             
#include <stdio.h>    
#include <string>
#include <cstdlib>

struct icpInformation
{
 Eigen::Matrix4f icp_transformation;
 double icp_fitness_score;
};


// based on PointCloudDataImport class 
// generating the template databasa for various door handle types 
class FeatureCloudGeneration
{
public:

FeatureCloudGeneration();

// file load different types
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >  loadGeneratedTemplatePCLXYZ(std::string filePath);
std::vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> >  loadGeneratedTemplatePCLFeatures(std::string filePath);
std::vector<pcl::PointCloud<pcl::Normal>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::Normal>::Ptr> >  loadGeneratedTemplatePCLNormals(std::string filePath);
std::vector<Eigen::Matrix4f> loadGeneratedPCATransformations(std::string filePath);
std::vector<Eigen::Vector3f> loadGeneratedBBInformation(std::string filePath);

icpInformation icpBasedTemplateAlignment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_template_point_cloud);

Eigen::Matrix4f  featureBasedTemplateAlignment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud,pcl::PointCloud<pcl::Normal>::Ptr input_cloud_normals,pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud,pcl::PointCloud<pcl::FPFHSignature33>::Ptr template_cloud_features,pcl::PointCloud<pcl::Normal>::Ptr template_cloud_normals);

pcl::PointCloud<pcl::Normal>::Ptr calculateSurfaceNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud);

pcl::PointCloud<pcl::FPFHSignature33>::Ptr calculate3DFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr downSamplePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud);

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > generateAlignmentObject(std::vector <pcl::PointIndices> clusters,pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced_pc, pcl::ModelCoefficients::Ptr plane_coeff);

std::vector<int> estimateCorrespondences(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud_1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud_2, double max_dist, double overlap_ratio);

// hardcoded values for ipa_database_generation and ipa_door_handle_template_alignment -> later create .CSV with desired parameters and put in into to directory
static std::string getFilePathFromParameter(int dist, int angle_1, int angle_2);


private:
double alignment_eps_; //
double alignment_thres_;

int max_num_iter_icp_ref_;
double corresp_dist_step_;
int max_num_iter_; //1000
double similarity_thres_; //0.9f

double rad_search_dist_; //0.03
float voxel_grid_size_; //0.005f;

};
