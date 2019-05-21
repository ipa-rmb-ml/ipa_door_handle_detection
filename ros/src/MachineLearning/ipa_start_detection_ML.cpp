
#include "MachineLearning/ipa_start_detection_ML.h"

#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>


StartHandleDetectionML::StartHandleDetectionML(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg) :
nh_(nh), point_cloud_out_msg_(point_cloud_out_msg)
{

}




