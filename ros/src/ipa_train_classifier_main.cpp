
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include "MachineLearning/AdaBooster.h"

int main( int argc, char* argv[] )
{
    
    ros::init(argc, argv, "Train_Classifier");

    return 0;
}
