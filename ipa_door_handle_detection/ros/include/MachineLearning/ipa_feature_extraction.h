
#include <iostream>
#include <utility>
#include <math.h>

class FeatureExtraction
{
public:

FeatureExtraction();



private:

	std::string posDepthImagePath_;
	std::string negDepthImagePath_;

	std::string LBPFeaturesPath_;
	std::string BriefFeaturesPath_;

};

