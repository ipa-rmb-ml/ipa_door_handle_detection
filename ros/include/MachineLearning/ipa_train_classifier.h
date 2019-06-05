
#include <iostream>
#include <utility>
#include <math.h>
#include <ros/ros.h>
#include "opencv2/ml/ml.hpp"
#include "opencv2/core/core_c.h"
#include <opencv2/core/mat.hpp>
#include <glob.h> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>


const std::string posDepthImagePath;
const std::string negDepthImagePath;


class TrainClassifier
{
public:

TrainClassifier();

void extractFeatures(cv::Mat curr_img);

void calculateLBP(cv::Mat src) ;

std::vector<cv::Mat> readDepthImages(const std::string dir_name);

void generateTrainingData(std::vector<cv::Mat> image_seq, bool label);

cv::Mat getLabelData(void);
cv::Mat getTrainingData(void);
cv::Mat getLBPFeatures(void);

cv::Mat label_data_;
cv::Mat training_data_;
cv::Mat lbp_data_;


};

