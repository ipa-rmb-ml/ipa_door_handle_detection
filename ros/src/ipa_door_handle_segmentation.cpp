
#include "ipa_door_handle_segmentation.h"
#include "ipa_door_handle_template_alignment.h"


PointCloudSegmentation::PointCloudSegmentation()
{

// filter points by distance
min_point_to_plane_dist_ = 0.03;//m
max_point_to_plane_dist_ = 0.1;

// filter points by door handle height
// height of door handle: due to DIN 85 cm to 105 cm
min_height_door_handle_ = 0.85;
max_height_door_handle_ = 1.05;

// max door-robot distance --> especially for glas doors no to caputre objects behind
max_door_robot_ = 1.5;

// cylinder params for fit
// min and max radius should be discussed
cylinder_rad_min_ = 0.01;
cylinder_rad_max_ = 0.07;
inlier_ratio_thres_ = 0.9;

// segmentation params
distance_thres_ = 0.01;
max_num_iter_ = 1000;

min_cluster_size_ = 3000;
max_cluster_size_ = 10000;

// angle between cyliders axis and door planes normals
// maximal difference for both to be orthogonal 
max_diff_norm_axis_ = 7;

angle_thres_x_ = 10.0;

}



// main segmentation process including coloring the pointcloud and the plane detection
// ================================================================================================================================
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >  PointCloudSegmentation::segmentPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
	Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >clusters_vec_pc;
	
	// PLANE DETECTION 
	planeInformation planeData = detectPlaneInPointCloud(input_cloud);

	pcl::ModelCoefficients::Ptr plane_coeff = planeData.plane_coeff;
	pcl::PointIndices::Ptr plane_pc_indices = planeData.plane_point_cloud_indices;

	// coloredPC: PC colored in blue
	// plane_pc: detected plane ccolored in red
	// plane coeff: coeffs of the detected plane

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced_pc=minimizePointCloudToObject(input_cloud,plane_pc_indices,plane_coeff);

	std::vector <pcl::PointIndices> clusters = findClustersByRegionGrowing(reduced_pc);
    clusters_vec_pc = generateAlignmentObject(clusters,reduced_pc,plane_coeff);

	// concentrate plane_pc with cluster_pc for visualizatuion
 return clusters_vec_pc;
}	
//=====================================================================================================================================


pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudSegmentation::changePointCloudColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud)
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointXYZRGB pclPoint;

	int min = 0;
	int max = 255;

	for (size_t i = 0; i < input_cloud->points.size (); ++i)
	{
		pclPoint.x = input_cloud->points[i].x;
		pclPoint.y = input_cloud->points[i].y;
		pclPoint.z = input_cloud->points[i].z;

		pclPoint.r = min + rand() % (( max + 1 ) - min);;
		pclPoint.g = min + rand() % (( max + 1 ) - min);;
		pclPoint.b = min + rand() % (( max + 1 ) - min);;

		pointcloud_xyzrgb->points.push_back(pclPoint);
	}
	//std::cout << "<<< Ending PC color change." << std::endl;
	return pointcloud_xyzrgb;
}

planeInformation PointCloudSegmentation::detectPlaneInPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	pcl::ModelCoefficients::Ptr plane_coeff (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;

	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (distance_thres_);
	seg.setInputCloud (input_cloud);
	// get inliers and plane coeff
	seg.segment (*inliers, *plane_coeff);

	planeInformation planeData;
	planeData.plane_point_cloud_indices = inliers;
 	planeData.plane_coeff = plane_coeff;

	return planeData;
};


pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudSegmentation::minimizePointCloudToObject(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointIndices::Ptr plane_pc_indices,pcl::ModelCoefficients::Ptr plane_coeff){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	// calculate point to plane distance
	pcl::PointXYZRGB pp_PC;

  	for (size_t i = 0; i < input_cloud->points.size (); ++i)
	{
		pp_PC.x = input_cloud->points[i].x;
		pp_PC.y = input_cloud->points[i].y;
		pp_PC.z = input_cloud->points[i].z;

			// storing and coloring data from only outside the plane
		double point2plane_distance =  pcl::pointToPlaneDistance (pp_PC, plane_coeff->values[0], plane_coeff->values[1], plane_coeff->values[2], plane_coeff->values[3]);

		// add DIN information to ppoint handle
		// take only values in front of the door
		if ( pp_PC.z < plane_coeff->values[3])
		{
			if ((point2plane_distance > min_point_to_plane_dist_) && (point2plane_distance < max_point_to_plane_dist_))
			{
						pp_PC.r = 0;																								
						pp_PC.b = 255;
						pp_PC.g =0;							
						reduced_pc->points.push_back(pp_PC);				
				}
		}
	}
	return reduced_pc;
}


std::vector <pcl::PointIndices> PointCloudSegmentation::findClustersByRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced_pc)
{
  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
  
  // computing normals
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);

  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;	

  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (reduced_pc);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;

  reg.setMinClusterSize (min_cluster_size_);
  reg.setMaxClusterSize (max_cluster_size_);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (reduced_pc);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);

  // most important: 
  // sets angle that will be used as the allowable range for the normals deviation
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  // curvature threshold if two points have a small normals deviation then the disparity btw their curvatures is tested
  reg.setCurvatureThreshold (0.5);
  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
 // put into new function --> visualizing/counting clusters 

  return clusters;
}

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
	Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > PointCloudSegmentation::generateAlignmentObject(std::vector <pcl::PointIndices> clusters,pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced_pc, pcl::ModelCoefficients::Ptr plane_coeff)
{
    pcl::PointXYZRGB clusteredPP;
   //write each cluster to as point cloud
   //vector to store clusters
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
	Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >clusterVec_pc;

	int min = 0;
	int max = 150;

	// avoid crashing
	if(clusters.size() > 0)
	{
	// iterate over cluster
		for (int numCluster =0; numCluster < clusters.size(); numCluster=numCluster +1)
		{
			// randomize cluster color in pc for visualization
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

				// iterate over points in cluster to color them in diferent colors
				for (size_t i = 0; i < clusters[numCluster].indices.size (); ++i)
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_pt_proj_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
					pcl::PointXYZRGB clusteredPP_proj;

					clusteredPP.x=reduced_pc->points[clusters[numCluster].indices[i]].x;
					clusteredPP.y=reduced_pc->points[clusters[numCluster].indices[i]].y;
					clusteredPP.z=reduced_pc->points[clusters[numCluster].indices[i]].z; 

					clusteredPP.r = 0;
					clusteredPP.g = 0;
					clusteredPP.b = 255;
					// adding single points to point cloud cluster, these are the object point lying outsidee the plane 
					cluster_pc->points.push_back(clusteredPP);
				} 
				clusterVec_pc.push_back(cluster_pc);
		} // end clusters
		return clusterVec_pc;
	}; //end if
   	return clusterVec_pc; 
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudSegmentation::projectPointsOnPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder_points,pcl::ModelCoefficients::Ptr plane_coeff)
{

 	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder_points_proj (new pcl::PointCloud<pcl::PointXYZRGB>);
	// plane coefficients: plane_coeff
	// creating ProjectInliners object --> using plane_coeffs to describe projection plane geometrically
	pcl::ProjectInliers<pcl::PointXYZRGB> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (cylinder_points);
	proj.setModelCoefficients (plane_coeff);
	proj.filter (*cylinder_points_proj);

	return cylinder_points_proj;
}

// fit cylinder and get parameters to check weathr parallel to plane
// project cylinder on plane and check weather all points lying inside of a rectangle --> rect with height == door handles diameter


 bool PointCloudSegmentation::alignCylinderToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_point_cloud,pcl::PointCloud<pcl::Normal>::Ptr input_point_cloud_normals, pcl::ModelCoefficients::Ptr plane_coefficients)
{
 // segmentattin object 
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg; 
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  pcl::PointCloud<pcl::Normal>::Ptr cyl_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);

  Eigen::Vector2f cylinder_geometry;

	// check if cylinders rotation axis is orthogonal to the door plane

	// ===================Definition of coefficients_cylinder ==========
	//  point_on_axis.x : the X coordinate of a point located on the cylinder axis 0
	// point_on_axis.y : the Y coordinate of a point located on the cylinder axis  1
	// point_on_axis.z : the Z coordinate of a point located on the cylinder axis 2
	// axis_direction.x : the X coordinate of the cylinder's axis direction 3 
	// axis_direction.y : the Y coordinate of the cylinder's axis direction 4
	// axis_direction.z : the Z coordinate of the cylinder's axis direction 5
	// radius : the cylinder's radius

 // actual cylinder fitting using RANSAC

  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (distance_thres_);
  seg.setMaxIterations (max_num_iter_);
  seg.setDistanceThreshold (distance_thres_);
  seg.setRadiusLimits (cylinder_rad_min_,cylinder_rad_max_);
  seg.setInputCloud (input_point_cloud);
  seg.setInputNormals (input_point_cloud_normals);
   //Obtain the cylinder inliers and geometrical coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);

double num_inliers = inliers_cylinder->indices.size();
double num_point_cloud = input_point_cloud->points.size();

	if (num_inliers > 0)
	{
		double radius =  coefficients_cylinder->values[6];
		double ratio = num_inliers/num_point_cloud;
		double ratio_thres = 0.8;
		double angle_tolerance = 10;

		double angle_cylinder_plane = checkOrientationAndGeometry(coefficients_cylinder,plane_coefficients);

		if ((radius > cylinder_rad_min_) && (radius < cylinder_rad_max_) && (ratio >ratio_thres) && (abs(angle_cylinder_plane-90.0) < angle_tolerance))
		{
			std::cout << "Is Cylinder ";
			std::cout<<" | radius: " << radius * 100 << " cm  |";
			std::cout<<"  angle: " << angle_cylinder_plane << "°" <<std::endl;
			
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	} //end if
}

double  PointCloudSegmentation::checkOrientationAndGeometry(pcl::ModelCoefficients::Ptr cylinder_coeff,pcl::ModelCoefficients::Ptr plane_coeff)
{

	int offset = 3;
	double scalar_prod = 0;

	double len_1 = 0; // cyl axis
	double len_2 = 0; // normal vec

	for (int i =0; i < 3; ++i)
	{
		 scalar_prod += cylinder_coeff->values[offset +i] * plane_coeff->values[i];

		 len_1 += pow(cylinder_coeff->values[offset +i],2);
		 len_2 += pow(plane_coeff->values[i],2);
	}
   
	// get geometrical lenght
	double cos_alpha = scalar_prod/(sqrt(len_1)*sqrt(len_2));

	double angle = acos(cos_alpha)*180.0/M_PI;

	return angle;
}

pcaInformation PointCloudSegmentation::calculatePCA(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
  {

	 // Compute principal directions
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*point_cloud, pcaCentroid);
	Eigen::Matrix3f covariance;

	// calculation of the covariance matrix
	computeCovarianceMatrixNormalized(*point_cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);

	Eigen::Matrix3f ev;
	Eigen::Vector3f vec;
	vec << 0,1,2;
	ev.setZero();

	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	ev.block<3,1>(0,0) = eigen_solver.eigenvalues();
	ev.block<3,1>(0,1) = vec;

	// sorting EV in descending order
	// min eigenvalue | num before sort
	// mid eigenvalue | num before sort
	// max eigenvalues |num before sort 
	std::sort(ev.data(),ev.data()+ev.size());

	int pos_ev_1 = ev(2,2);
	int pos_ev_2 = ev(1,2);

	// calculating Eigenvectors
	Eigen::Vector3f eigenVectorPCA_x = eigenVectorsPCA.col(pos_ev_1);
	Eigen::Vector3f eigenVectorPCA_y = eigenVectorsPCA.col(pos_ev_2);
	Eigen::Vector3f eigenVectorPCA_z = eigenVectorPCA_x.cross(eigenVectorPCA_y);

	//These eigenvectors are used to transform the point cloud to the origin point (0, 0, 0) such that the eigenvectors correspond to the axes of the space. 
	// The minimum point, maximum point, and the middle of the diagonal between these two points are calculated for the transformed cloud (also referred to as the projected cloud when using PCL's PCA interface, or reference cloud by Nicola).
	// Transform the original cloud to the origin where the principal components correspond to the axes.
	Eigen::Matrix4f pca_trafo(Eigen::Matrix4f::Identity());

	pca_trafo.block<3,1>(0,0) = eigenVectorPCA_x;
	pca_trafo.block<3,1>(0,1) = eigenVectorPCA_y;
	pca_trafo.block<3,1>(0,2) = eigenVectorPCA_z;

	pca_trafo.block<3,1>(0,3) = -1.f * (pca_trafo.block<3,3>(0,0) * pcaCentroid.head<3>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_pca (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::transformPointCloud(*point_cloud, *point_cloud_pca, pca_trafo.transpose());

	// Get the minimum and maximum points of the transformed cloud.
	pcl::PointXYZRGB minPoint, maxPoint;
	pcl::getMinMax3D(*point_cloud_pca, minPoint, maxPoint);
	const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

		// Final transform
	const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
	const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

	double rect_width = (maxPoint.x-minPoint.x)*100;
	double rect_height =  (maxPoint.y-minPoint.y)*100;
	double rect_depth = (maxPoint.z-minPoint.z)*100;

	Eigen::Vector3f bbParams;

	bbParams << rect_width, rect_height, rect_depth;


	pcaInformation pcaData;
	pcaData.pca_transformation = pca_trafo;
 	pcaData.bounding_box_3D = bbParams;

	return pcaData;

  }


  bool PointCloudSegmentation::checkBB3DOrientation(Eigen::Matrix4f PCA_trafo,pcl::ModelCoefficients::Ptr plane_coefficients)
  {
	// check orientation of the bounding box -> x axis largest variance
			Eigen::Vector3f PCA_x = PCA_trafo.block<3,1>(0,0);
			Eigen::Vector3f PCA_y = PCA_trafo.block<3,1>(0,1);
		 
			double len_1 = 0;
			double len_2 = 0;
			double scalar_prod = 0;  

			for (int i =0; i < 3; ++i)
			{
				scalar_prod +=  PCA_x(i) *plane_coefficients->values[i];
				len_1 += pow(PCA_x(i),2);
				len_2 += pow(plane_coefficients->values[i],2);
			}
		
			// get geometrical lenght
		 	double cos_alpha = scalar_prod/(sqrt(len_1)*sqrt(len_2));
			double angle = acos(cos_alpha)*180.0/M_PI - 90;

	if (abs(angle) < angle_thres_x_)
	{
		return true;
	}
	else
	{
	//	std::cout<<"Wrong orientation. Cluster main axis is not parallel to door plane"<<std::endl;
		return false;
	}



  }