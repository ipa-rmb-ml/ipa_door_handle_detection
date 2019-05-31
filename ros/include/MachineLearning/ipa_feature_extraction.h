
#include <iostream>
#include <utility>
#include <math.h>
#include <ros/ros.h>

class FeatureExtraction
{
public:

FeatureExtraction();

void extractLBPFeatures();

void extractBRIEFFeatures();

private:

	std::string posDepthImagePath_;
	std::string negDepthImagePath_;

	std::string LBPFeaturesPath_;
	std::string BriefFeaturesPath_;

};

