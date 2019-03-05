#include "ipa_start_detection.h"


// =================================================0
int main(int argc, char **argv)
{
	ros::init(argc, argv, "my_pcl_image_listener");
	ros::NodeHandle nh;
	sensor_msgs::PointCloud2::Ptr point_cloud_out_msg(new sensor_msgs::PointCloud2);
	StartHandleDetection StartHandleDetection(nh, point_cloud_out_msg);
	return 0;
}
