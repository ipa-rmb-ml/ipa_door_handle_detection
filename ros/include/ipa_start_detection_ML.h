#include <iostream>
#include <ros/ros.h>
#include <utility>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <boost/thread/mutex.hpp>


// definition of the settings for the camera --> might be adapted --> read in from TXT file

#define TOPIC_POINT_CLOUD_IN "/pico_flexx/points"
#define TOPIC_POINT_CLOUD_OUT1 "/template_matching"
#define CAMERA_LINK "/pico_flexx_optical_frame"

// creates vector with pointclouds where each represents a cluster idintified by region growing
class StartHandleDetectionML
{
public:

	StartHandleDetectionML(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg);

	void initCameraNode(ros::NodeHandle nh, sensor_msgs::PointCloud2::Ptr point_cloud_out_msg);

	void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);	

private:
	ros::Publisher pub1_;

	ros::NodeHandle nh_;
	ros::Subscriber point_cloud_sub1_;
	ros::Subscriber camera_subs_;
	sensor_msgs::PointCloud2::Ptr point_cloud_out_msg_;


	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_scenery_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr handle_point_cloud_;

};

