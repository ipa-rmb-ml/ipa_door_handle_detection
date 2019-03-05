#include "ipa_door_handle_segmentation.h"
#include "ipa_template_generation.h"
#include "ipa_door_handle_template_alignment.h"


DoorHandleTemplateGeneration::DoorHandleTemplateGeneration(std::string file_path_to_point_clouds )
{

	TEMPLATE_PATH_ 		= "/home/rmb-ml/Desktop/PointCloudData"
	targetPathXYZRGB_  	= TEMPLATE_PATH_ + "/templateDataXYZRGB/";
	targetPathNormals_ 	= TEMPLATE_PATH_ + "/templateDataNormals/";
	targetPathFeatures_ = TEMPLATE_PATH_ + "/templateDataFeatures/";
	targetPathPCA_ 		= TEMPLATE_PATH_ + "/templateDataPCAXYZRGB/";
	targetPathEigen_	= TEMPLATE_PATH_ + "/templateDataPCATrafo/";
	targetPathBB_ 		= TEMPLATE_PATH_ + "/templateDataBB/";

	generateTemplatePCLFiles(file_path_to_point_clouds);
}

// OFFLINE PART -> TEMPLATE GENERATION	

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
	Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > DoorHandleTemplateGeneration::generateTemplateAlignmentObject(std::vector <pcl::PointIndices> clusters,pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced_pc, pcl::ModelCoefficients::Ptr plane_coeff)
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

					clusteredPP.r = 255;
					clusteredPP.g = 0;
					clusteredPP.b = 0;

					// adding single points to point cloud cluster, these are the object point lying outsidee the plane 
					cluster_pc->points.push_back(clusteredPP);
				} 

				clusterVec_pc.push_back(cluster_pc);

		} // end clusters

		return clusterVec_pc;

	}; //end if
    
	std::cout << "No cluster found"<< std::endl;
	return clusterVec_pc; 
}


void  DoorHandleTemplateGeneration::generateTemplatePCLFiles(std:: string file_path_to_point_clouds)
{

PointCloudSegmentation seg;
FeatureCloudGeneration featureObj;

pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud_reduced_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud_reduced (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr template_cloud_pca (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::Normal>::Ptr  template_cloud_normals (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::FPFHSignature33>::Ptr template_cloud_features (new pcl::PointCloud<pcl::FPFHSignature33>);


std::vector <pcl::PointIndices> door_handle_cluster;	
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >template_cluster_vec;

 DIR *pDIR;
 struct dirent *entry;
    if(pDIR=opendir(file_path_to_point_clouds.c_str()))
	{
        while(entry = readdir(pDIR)){

          if( strcmp(entry->d_name,file_path_to_point_clouds.c_str()) != 0 && strcmp(entry->d_name, "..") != 0 && strcmp(entry->d_name, ".") != 0  )				
			{
			std::string filePathPCDRead = file_path_to_point_clouds + entry->d_name;
			std::string filePathPCDWriteXYZRGB = targetPathXYZRGB_ + entry->d_name;
			std::string filePathPCDWriteNormals = targetPathNormals_ + entry->d_name;
			std::string filePathPCDWriteFeatures = targetPathFeatures_ + entry->d_name;
			std::string filePathPCDWritePCAXYZ = targetPathPCA_ + entry->d_name;

			boost::filesystem::path p(filePathPCDRead);
			std::string templateName = p.stem().c_str(); // get filename without extension

			std::string filePathTXTWriteEigen = targetPathEigen_ + templateName + ".txt";
			std::string filePathTXTWriteBB = targetPathBB_ + templateName + ".txt";

				if (pcl::io::loadPCDFile<pcl::PointXYZ> (filePathPCDRead, *template_cloud) == -1) //* load the file
					{
						PCL_ERROR ("Couldn't read PCD file. \n");
					}
			planeInformation planeData = seg.detectPlaneInPointCloud(template_cloud);

			// change color of template cloud

			pcl::ModelCoefficients::Ptr plane_coeff = planeData.plane_coeff;
			pcl::PointIndices::Ptr plane_pc_indices = planeData.plane_point_cloud_indices;

			template_cloud_reduced=seg.minimizePointCloudToObject(template_cloud,plane_pc_indices,plane_coeff);
			template_cloud_reduced_rgb = seg.changePointCloudColor(template_cloud_reduced);

				if (template_cloud_reduced->size() > 0)
					{
					door_handle_cluster=seg.findClustersByRegionGrowing(template_cloud_reduced_rgb);
					// only one object suppose to be the handle
					if (door_handle_cluster.size()== 1)
						{
						template_cluster_vec= generateTemplateAlignmentObject(door_handle_cluster,template_cloud_reduced,plane_coeff);
						// calculate xyzrgb point cloud
						*template_cloud_reduced = *template_cluster_vec[0];
						template_cloud_reduced->width = 1;
						template_cloud_reduced->height = template_cloud_reduced->points.size();

						// downsample point cloud for better performance 
						template_cloud_reduced=featureObj.downSamplePointCloud(template_cloud_reduced);
											
						// calculate normals based on template_cloud_reduced
						template_cloud_normals = featureObj.calculateSurfaceNormals(template_cloud_reduced);
											
						//calculate features based on template_cloud_reduced
						template_cloud_features = featureObj.calculate3DFeatures(template_cloud_reduced,template_cloud_normals);

						pcaInformation pcaData  = seg.calculatePCA(template_cloud_reduced);
						Eigen::Matrix4f transform_pca = pcaData.pca_transformation;
						Eigen::Vector3f bb_3D = pcaData.bounding_box_3D;

						pcl::transformPointCloud(*template_cloud_reduced, *template_cloud_pca, transform_pca);
											
						std::cout << "Writing XYZ..." << std::endl;
						pcl::io::savePCDFileASCII (filePathPCDWriteXYZRGB,*template_cloud_reduced);

						std::cout << "Writing Normals..." << std::endl;
						pcl::io::savePCDFileASCII (filePathPCDWriteNormals,*template_cloud_normals);

						std::cout << "Writing PCA Data..." << std::endl;
						pcl::io::savePCDFileASCII (filePathPCDWritePCAXYZ,*template_cloud_pca);

						std::cout << "Writing Features..." << std::endl;	
						pcl::io::savePCDFileASCII (filePathPCDWriteFeatures,*template_cloud_features);

						std::cout << "Writing PCA Transformation..." << std::endl;	
						std::ofstream fout_pca;
						fout_pca.open(filePathTXTWriteEigen.c_str());
						fout_pca <<	"\n";
							for (int r = 0; r < transform_pca.rows(); r++)
								{
								for (int c = 0; c <transform_pca.cols();c++)
									{
									fout_pca <<transform_pca(r,c) << "\n";
									}
							}

						std::cout << "Writing PCA BB Information..." << std::endl;	
						std::ofstream fout_BB;
						fout_BB.open(filePathTXTWriteBB.c_str());
						fout_BB <<	"\n";
							for (int r = 0; r < bb_3D.size(); r++)
								{
								fout_BB <<bb_3D(r) << "\n";
								}					
					}

				}
			else
				{
				std::cout << "Not sufficient points for normal estimation." << std::endl;
				}
			}
        } //end while
        closedir(pDIR);
    } //end if
}


// =================================================0
int main(int argc, char **argv)
{		
	std::string file_path_to_point_clouds = "/home/rmb-ml/Desktop/PointCloudData/unprocessed/";
	DoorHandleTemplateGeneration DoorHandleTemplateGeneration(file_path_to_point_clouds);

	return 0;
}

