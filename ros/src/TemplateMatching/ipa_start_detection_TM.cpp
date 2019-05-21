
#include "TemplateMatching/ipa_start_detection_TM.h"
#include "TemplateMatching/ipa_door_handle_segmentation.h"
#include "TemplateMatching/ipa_door_handle_template_alignment.h"

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>





// init relevant paths, topics and nodes (see initCameraNode)
StartHandleDetectionTM::StartHandleDetectionTM(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg) :
nh_(nh), point_cloud_out_msg_(point_cloud_out_msg)
{
	std::string camera_link = "pico_flexx_optical_frame" ;
	std::string PATH_TO_TEMPLATE_DIR = "/home/rmb-ml/Desktop/TemplateDataBase";

	filePathXYZRGB_ = PATH_TO_TEMPLATE_DIR + "/templateDataXYZRGB/"; // only for testing -> change later
	filePathPCATransformations_ = PATH_TO_TEMPLATE_DIR +"/templateDataPCATrafo/";
	filePathBBInformations_ = PATH_TO_TEMPLATE_DIR +"/templateDataBB/";

	// correspondence estimation function
	max_dist_1_ = 0.01; //first crit
	max_dist_2_ = 0.005; // refinement after best template
	overlap_ratio_ =0.9; // overlp ration of template and cluster during corresp matching

	// geometrical handle constrains in cm
	max_handle_lenght_ = 15;
	max_handle_width_ = 5;
	max_handle_height_ = 5;
	diag_BB3D_lim_ = 15;

	initCameraNode(nh,point_cloud_out_msg);	
}





// MAIN CALCULATION GOES BELOW
void StartHandleDetectionTM::pointcloudCallback_1(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
		
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

	//create new point cloud in pcl format: pointcloud_in_pcl_format
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_in_pcl_format(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane_points(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr published_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

	//transform imported pointcloud point_cloud_msg to pointcloud_in_pcl_format
	pcl::fromROSMsg(*point_cloud_msg, *pointcloud_in_pcl_format);
	
	// ==================== ACTUAL CALCULATION:START ==========================================================================================================

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_pca (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr cluster_pca_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_pca(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_pca_best(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assumed_handle_point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	//========SEGMENTATION==================

	// create segmentation object
	PointCloudSegmentation segObj;

	// performes plane detection to return plane coefficients and model inliers in a struct 
	planeInformation planeData = segObj.detectPlaneInPointCloud(pointcloud_in_pcl_format);
	pcl::ModelCoefficients::Ptr plane_coefficients = planeData.plane_coeff;

	// coeff based distance calculation
	double dist = double (plane_coefficients->values[3]);

	// noise removal from point cloud
	// removes points from the point cloud in a specific range
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_filtered = segObj.removeNoisefromPC(pointcloud_in_pcl_format,dist);

	point_cloud_scenery_ = pc_filtered;

	// the determined clusters are stores inside a  vector structure
	// the actual involves a few steps, for detailed explanation see the segmentPointCloud() function
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
	Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > cluster_vec = segObj.segmentPointCloud(pc_filtered);


	//================SUMMARY======================
	// After the door plane detection all relevant points are supposed to be clustered usind a region growing appoach. Therefor all clusters in front of the plane 
	// should be considered as a handle candidate. Using specific geometric constrains a rough preselection is performed. Remaining cluster that passed the preselection 
	// are used to perform a template matching. The goal is to align all considerable templates stored in the database to find the best fit.

	//==========================================0

	FeatureCloudGeneration featureObj;
	std::vector<double> fitness_score_vec;

	// vector of point clouds
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
	Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > template_pca_best_vec,cluster_pca_best_vec;


	double fitness_score_max = 10e-4;
	double points_ratio_min = 0.8;
	double sum_squared_err_max = 1;

	std::string best_handle_type;
	std::vector<std::string> best_handle_type_vec;

	Eigen::Matrix4f final_transformation;

	std::vector<std::string> handle_type_name_vec;

	// ==================================================ITERATION OVER CLUSTERS ===============================================


	
	if (cluster_vec.size () > 0)
	{

		// performs the iteration over the cluster vector to check each of the stored point clouds
		for (int num_cluster = 0; num_cluster < cluster_vec.size (); ++num_cluster) // 
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster = cluster_vec[num_cluster];		

			// estimate preconditio
			// get min max 


			// precondition: if the cluster does not fit geometrical constrains 
			pcl::PointXYZRGB minPt, maxPt;
			pcl::getMinMax3D (*cluster, minPt, maxPt);

			// skip cluster if the size is too large 
			double diff_x = fabs(maxPt.x-minPt.x)*100; // in cm
			double diff_y = fabs(maxPt.y-minPt.y)*100; // in cm
			double diff_z = fabs(maxPt.z-minPt.z)*100; // in cm


			// if the cluster does not fit the constrains, skip and proceed with next one
			if (diff_x > max_handle_lenght_ && diff_y > max_handle_height_ && diff_z > max_handle_width_ )
			{
				continue;
			}

			assumed_handle_point_cloud = cluster;

			// downsampling the pointcloud to reduce the number of points using a voxel grid
			// in our use case not mandatory since the cluster candidates got usually a point num < 300

			//cluster = featureObj.downSamplePointCloud(cluster);

			// PCA calculation to transform the cluster the the origin and use the eigenvectors for orientation -> highest covariance along x axis (lenght)
			pcaInformation pcaData =  segObj.calculatePCA(cluster);

			Eigen::Matrix4f cluster_pca_trafo = pcaData.pca_transformation;
			Eigen::Vector3f bb_3D_cluster = pcaData.bounding_box_3D;
			
			// calculation of the Bound Box diagonal given by the PCA
			double BB_Diag_3D =  sqrt(bb_3D_cluster[0] * bb_3D_cluster[0] + bb_3D_cluster [1] * bb_3D_cluster [1] + bb_3D_cluster [2]* bb_3D_cluster [2]);

			// ORIENTATION CHECK
			// checks if the axis with the highest covar (x) is approximate parallel to the doors plane
			bool isParallel = segObj.checkBB3DOrientation(cluster_pca_trafo,plane_coefficients);
			// if parallel proceed with given cluster 
			// else continue
			if (!isParallel)
			{
				ROS_WARN("Orientation of BB3D not parallel!");
				continue;
			}

			// size check
			if (BB_Diag_3D > diag_BB3D_lim_ || bb_3D_cluster[0] > max_handle_lenght_ || bb_3D_cluster[1] > max_handle_width_ )
			{	
				continue;
			}


			// apply transformation on assumed handle cloud to place it in the orgin and rotate based on EVs
			pcl::transformPointCloud (*cluster, *cluster_pca, cluster_pca_trafo);

			cluster_pca_best_vec.push_back(cluster_pca); // so

			// depending on the BB size decide in which directory 
			Eigen::Vector3f cam_axis;
			cam_axis << 0,0,1;

			// ========================CAMERA-DOORPLANE-DISTANCE-AND-ORIENTATION==================================================

			// This parameters describe the cameras position towards the door in respect to the distance and the angles in XY and YZ
			// based on this in information: choice in which subdirectory to go to load the templates for elignment

			double scalar_prod_xz = plane_coefficients->values[0];
			double scalar_prod_yz = plane_coefficients->values[1];
	
			double len_xz = pow(plane_coefficients->values[0],2) + pow(plane_coefficients->values[2],2);
			double len_yz = pow(plane_coefficients->values[1],2) + pow(plane_coefficients->values[2],2); 

			// get geometrical lenght
			double cos_alpha_1 = scalar_prod_xz/sqrt(len_xz);
			double cos_alpha_2 = scalar_prod_yz/sqrt(len_yz);

			int angle_XZ = 90-acos(cos_alpha_1)*180.0/M_PI;
			int angle_YZ = asin(cos_alpha_2)*180.0/M_PI;

			int dist = -plane_coefficients->values[3]*100; // to cm

			int r = 0;
			int g = 0;
			int b = 255;

			// ============================================================================================

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_pca_rgb = segObj.changePointCloudColor(cluster_pca,r,g,b);

			*published_pc+= *cluster_pca_rgb;

			// obtain path information based on orientation params
			std::string name_pcd = FeatureCloudGeneration::getFilePathFromParameter(dist,angle_XZ,angle_YZ);

			// struct containing two vectors 
			// 1. vector of template point clouds
			// 2. vector of strings storing template's names
			templateInformation template_information = featureObj.loadGeneratedTemplatePCLXYZ(filePathXYZRGB_,filePathBBInformations_,name_pcd,BB_Diag_3D);

			// storing template point cloud inside a new vector
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
			Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >  template_vec_xyz = template_information.doorhandle_template_vec;

			// check for empty entries
			handle_type_name_vec = template_information.handle_type_name_vec;

			if (template_vec_xyz.size() == 0)
			{
				continue;
			}

			// loading matrices decribing the transformation for each template
			std::vector<Eigen::Matrix4f> template_pca_trafo_vec = featureObj.loadGeneratedPCATransformations(filePathPCATransformations_,filePathBBInformations_,name_pcd,BB_Diag_3D);

			// ===================== CLASSIFICATION ==========================

			// ================================== ITERATION OVER TEMPLATES =============================================================

			for (int num_template = 0; num_template < template_vec_xyz.size(); ++num_template) // template loop
				{
					if (cluster_vec[num_cluster]->points.size() > 0)
					   {	

						    // can be used for normals and feature based matching
							// in this method the coordinated based alignment is used -> ICP

							//pcl::PointCloud<pcl::Normal>::Ptr template_normals = template_vec_normals[num_template];
							//pcl::PointCloud<pcl::FPFHSignature33>::Ptr template_features = template_vec_features[num_template];


							// temporary storage 
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_pca = template_vec_xyz[num_template];
							Eigen::Matrix4f pca_template =	 template_pca_trafo_vec[num_template]; // rotation and translatoion to origin
							std::string template_type = handle_type_name_vec[num_template];


							// Eigen::Matrix4f pca_assumed -> rotation and translation to origin
							// find rot btw PCA's coordinates systems
							//  template * R = assumed
							// R = assumed * template^T;


							
							// ============ PCA TRAFO ============
							Eigen::Matrix4f transform_hndl;
							transform_hndl.setIdentity();
							transform_hndl.block<3,3>(0,0) = pca_template.block<3,3>(0,0) * pca_template.block<3,3>(0,0).transpose();

							pcl::transformPointCloud (*template_pca,*template_pca, cluster_pca_trafo);

							// apply centroid shift
							Eigen::Vector4f pcaCentroid;
							Eigen::Matrix4f centroid_trafo;
							centroid_trafo.setIdentity();

							pcl::compute3DCentroid(*template_pca, pcaCentroid);
							centroid_trafo.block<4,1>(0,3) =  -pcaCentroid;

							pcl::transformPointCloud (*template_pca,*template_pca, centroid_trafo);

							// ============PCA TRAFO ============


							// ============ICP  ============
							// using ICP to estimate transformation
							// refinement of the PCA method

							icpInformation icp_data = featureObj.icpBasedTemplateAlignment(cluster_pca,template_pca);
							double fitness_score = icp_data.icp_fitness_score;
							Eigen::Matrix4f icp_transformation = icp_data.icp_transformation;

							pcl::transformPointCloud (*template_pca,*template_pca, icp_transformation);

							// ============ICP  ============

							int r = 255;
							int g = 0;
							int b = 0;

							pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_pca_rgb = segObj.changePointCloudColor(template_pca,r,g,b);

							// correspondence estimation btw cluster and template and calculation of the ration of registered point to all points in cluster
							double points_ratio = featureObj.estimateCorrespondences(cluster_pca, template_pca,max_dist_1_,overlap_ratio_);

							// sum squared translation error 
							double sum_squared_err =  sqrt(icp_transformation(0,3) * icp_transformation(0,3) + icp_transformation (1,3) * icp_transformation (1,3) + icp_transformation (2,3)* icp_transformation (2,3));
							
							// estimating bast fit based on fitness score and points ratio 
								if ((fitness_score < fitness_score_max) && (points_ratio > points_ratio_min) & (sum_squared_err < sum_squared_err_max))
								{
									fitness_score_max = fitness_score;
									points_ratio_min = points_ratio;
									sum_squared_err_max = sum_squared_err;

									template_pca_best = template_pca;

									// handle type
									best_handle_type = template_type;	
								}
								else
								{
					
								}
						}	//end if cluster size > 0
				} // end for over templates



			// ================================================= ITERATION OVER TEMPLATES ===============================================	

			//for each detected cluster push the fitness_score and template_pca into seperate vectors
			// size of the vectors is equal to the number of detected cluster
			fitness_score_vec.push_back(fitness_score_max);
			template_pca_best_vec.push_back(template_pca_best);
			best_handle_type_vec.push_back(best_handle_type);

		} // end for assumed handle cloud
		// find best cluster template match

		//for multiple possible clusters --> find best fitness score for cluster/template match
		if (!fitness_score_vec.empty())
		{
			int pos = 0;
			double mic = 1;

			for (int k = 0; k < fitness_score_vec.size(); k++)
			{
				if(fitness_score_vec[k] < mic)
				{
					mic = fitness_score_vec[k];
					int pos = k;
				}
			}

			// top template custer match based on prior fitness score error estimation
				cluster_pca = cluster_pca_best_vec[pos];
				template_pca = template_pca_best_vec[pos];
				std::string best_fit_name = best_handle_type_vec[pos];


				pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_filt(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::PointXYZRGB pp_filt;

			if (template_pca->points.size() > 0)
			{

				icpInformation icp_data_nest_fit = featureObj.icpBasedTemplateAlignment(cluster_pca,template_pca);

				double fitness_score = icp_data_nest_fit.icp_fitness_score;
				Eigen::Matrix4f icp_transformation_best_fit = icp_data_nest_fit.icp_transformation;

				pcl::transformPointCloud (*template_pca,*template_pca, icp_transformation_best_fit);

				// B  Distance --> paper
				ROS_WARN("Door Handle detected!");
				std::cout<<"Door Handle Type: " << best_fit_name <<std::endl;
				
				handle_point_cloud_ = assumed_handle_point_cloud;
				*published_pc+= *template_pca;


			} // end if check
			// ===============================================FIND BEST TEMPLATE ===================================================

		}
			// publish point cloud in rviz
			*published_pc+= *template_pca;
			pcl::toROSMsg(*published_pc, *point_cloud_out_msg_);
			point_cloud_out_msg_->header.frame_id = CAMERA_LINK;
			pub1_.publish(point_cloud_out_msg_);	

	}// endif
	else
	{
		handle_point_cloud_ = published_pc;
		ROS_WARN("No Cluster");
	}
		// ================================== FIND BEST TEMPLATE =================================================================
}
// end void callback




void StartHandleDetectionTM::pointcloudCallback_2(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{

	if (handle_point_cloud_->points.size() > 0)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr published_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_scenery_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);

		PointCloudSegmentation segObj;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr handle_point_cloud_rgb = segObj.changePointCloudColor(handle_point_cloud_,255,0,0);

		pcl::PointXYZRGB pp;
		for (size_t i = 0; i < point_cloud_scenery_->points.size (); ++i)
		{
			pp.x = point_cloud_scenery_->points[i].x;
			pp.y = point_cloud_scenery_->points[i].y;
			pp.z = point_cloud_scenery_->points[i].z;
			pp.r = 0;
			pp.g = 255;
			pp.b = 0;
			point_cloud_scenery_rgb->points.push_back(pp);	

		}

		*published_pc += *point_cloud_scenery_rgb;
		*published_pc += *handle_point_cloud_rgb;

		pcl::toROSMsg(*published_pc, *point_cloud_out_msg_);
		point_cloud_out_msg_->header.frame_id = CAMERA_LINK;
		pub2_.publish(point_cloud_out_msg_);
	}

}


void StartHandleDetectionTM::initCameraNode(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg)
{
	std::cout << "Initialising StartHandleDetection Constructor." << std::endl;

	// publish template & cluster
	pub1_ = nh_.advertise<sensor_msgs::PointCloud2>(TOPIC_POINT_CLOUD_OUT1,1);
	(pub1_) ? std::cout << "Pub1 is valid." << std::endl : std::cout << "Pub is not valid." << std::endl;
	ros::Subscriber point_cloud_sub1_ = nh_.subscribe<sensor_msgs::PointCloud2>(TOPIC_POINT_CLOUD_IN, 1, &StartHandleDetectionTM::pointcloudCallback_1, this);

	// publish handle into point cloud
	pub2_ = nh_.advertise<sensor_msgs::PointCloud2>(TOPIC_POINT_CLOUD_OUT2,1);
	(pub2_) ? std::cout << "Pub2 is valid." << std::endl : std::cout << "Pub is not valid." << std::endl;
	ros::Subscriber point_cloud_sub2_ = nh_.subscribe<sensor_msgs::PointCloud2>(TOPIC_POINT_CLOUD_IN, 1, &StartHandleDetectionTM::pointcloudCallback_2, this);

	ros::Duration(1).sleep();

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	if (!ros::ok()){
		std::cout << "Quit publishing" << std::endl;
	}
	std::cout << "StartHandleDetection Constructor Initialised." << std::endl;
}
