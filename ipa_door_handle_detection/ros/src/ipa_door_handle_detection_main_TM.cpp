#include "TemplateMatching/ipa_start_detection_TM.h"
// =================================================0
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Handle_Detector_TM");
	ros::NodeHandle nh;
	sensor_msgs::PointCloud2::Ptr point_cloud_out_msg(new sensor_msgs::PointCloud2);
	StartHandleDetectionTM StartHandleDetectionTM(nh, point_cloud_out_msg);
	return 0;
}
