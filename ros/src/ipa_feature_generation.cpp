#include "ipa_feature_generation.h"

void FeatureGeneration::StartFeatureGeneration(std::string file_path_to_point_clouds)
{

std::string handle_type;

DIR *pDIR;
 struct dirent *entry;
    if(pDIR=opendir(file_path_to_point_clouds.c_str()))
	{
        while(entry = readdir(pDIR))
		{

          if( strcmp(entry->d_name,file_path_to_point_clouds.c_str()) != 0 && strcmp(entry->d_name, "..") != 0 && strcmp(entry->d_name, ".") != 0  )				
			{
				handle_type = entry->d_name;
				std::cout<< "Database Generation for Handle Type: " << handle_type<<std::endl;
				FeatureGeneration FeatureGeneration(file_path_to_point_clouds,handle_type);
			}
		}
	}	
}

FeatureGeneration::FeatureGeneration(std::string file_path_to_point_clouds, std::string handle_type)
{

	TEMPLATE_PATH_ 		= file_path_to_point_clouds + handle_type + "/";
	BASE_PATH_ 			= "/home/rmb-ml/Desktop/TemplateDataBase" ;
	targetPathFeatures_ = BASE_PATH_ + "/DataSceneryFeatures/" + handle_type;
 
	createDirectory(targetPathFeatures_);

	generateDepthImages(TEMPLATE_PATH_,handle_type);
}


void FeatureGeneration::createDirectory(std::string path_to_dir)
{

	const char* path = path_to_dir.c_str();
	boost::filesystem::path dir(path);
	if(boost::filesystem::create_directory(dir))
	{
	}

}

void  FeatureGeneration::generateDepthImages(std:: string TEMPLATE_PATH_, std::string handle_type)
{

pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud (new pcl::PointCloud<pcl::PointXYZ>);

 DIR *pDIR;
 struct dirent *entry;
    if(pDIR=opendir(TEMPLATE_PATH_.c_str()))
	{
        while(entry = readdir(pDIR))
		{

          if( strcmp(entry->d_name,TEMPLATE_PATH_.c_str()) != 0 && strcmp(entry->d_name, "..") != 0 && strcmp(entry->d_name, ".") != 0  )				
			{
		
			std::string filePathPCDRead = TEMPLATE_PATH_ ;

			std::string full_pcd_path = filePathPCDRead + entry->d_name;

			if (pcl::io::loadPCDFile<pcl::PointXYZ> (full_pcd_path, *template_cloud) == -1) //* load the file
				{
					PCL_ERROR ("Couldn't read PCD file. \n");
				}
				
			}

            // get camera data from topic
            sensor_msgs::CameraInfo cam_info;

            float w = 352;
            float h = 288;

            cv::Mat cv_image = cv::Mat(h,w, CV_32FC1,cv::Scalar(std::numeric_limits<float>::max()));

            float focal_x = 200;
            float focal_y = 500;

            float centre_x = 550;
            float centre_y = 580;

            std::cout<<"Writing Depth image..."<<std::endl;
            
            for (int i=0; i<template_cloud->points.size();i++)
            {
                if (template_cloud->points[i].z == template_cloud->points[i].z)
                {
                    float z = template_cloud->points[i].z*1000.0;
                    float u = (template_cloud->points[i].x*1000.0*focal_x) / z;
                    float v = (template_cloud->points[i].y*1000.0*focal_y) / z;
                    int pixel_pos_x = (int)(u + centre_x);
                    int pixel_pos_y = (int)(v + centre_y);

                    if (pixel_pos_x > (w-1))
                    {
                      pixel_pos_x = w-1;
                    }
                    if (pixel_pos_y > (h-1))
                    {
                        pixel_pos_y = h-1;
                    }

                 cv_image.at<float>(pixel_pos_y,pixel_pos_x) = z;  
                }
            }

            std::cout<<cv_image<<std::endl;
            //cv::imwrite("home/rmb-ml/Desktop/TemplateDataBase/DepthImages/test.jpg",cv_image);

            // calculate features 
            // BRIEF
            // LBP
            // HOG

        } //end while
        closedir(pDIR);
    } //end if
    else
    {
    ROS_WARN("Check directory path");
    }	
}

// =================================================0
int main(int argc, char **argv)
{		
	// base path to folder containing the model type folders
	std::string file_path_to_point_clouds = "/home/rmb-ml/Desktop/TemplateDataBase/unprocessed_scenery/";

	FeatureGeneration::StartFeatureGeneration(file_path_to_point_clouds);

	return 0;
}

