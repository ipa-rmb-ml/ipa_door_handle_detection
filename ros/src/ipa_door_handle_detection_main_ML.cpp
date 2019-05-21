#include "MachineLearning/ipa_start_detection_ML.h"


// =================================================0
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Handle_Detection_ML");
	ros::NodeHandle nh;
	sensor_msgs::PointCloud2::Ptr point_cloud_out_msg(new sensor_msgs::PointCloud2);
	StartHandleDetectionML StartHandleDetectionML(nh, point_cloud_out_msg);
	return 0;
}
