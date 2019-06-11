
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

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>


std::string posDepthImagePathTrain;
std::string negDepthImagePathTrain;

std::string posDepthImagePathTest;
std::string negDepthImagePathTest;

std::string classifier_path;


struct DataStruct
{
    cv::Mat labels;
    cv::Mat features;
};



class TrainClassifier
{
public:

TrainClassifier();

void extractFeatures(cv::Mat curr_img);

std::vector<float> calculateLBP(cv::Mat src) ;

std::vector<cv::Mat> readDepthImages(const std::string dir_name);

void generateTrainingData(std::vector<cv::Mat> image_seq, bool label);

static void trainAllClassifier(cv::Mat traing_data,cv::Mat labels,bool trainSVM, bool trainRF,bool trainAB);

static DataStruct buildTrainingData();

static DataStruct buildTestingData();

static void calculatingPredictions(cv::Mat training_data, cv::Mat label_data,CvSVM &SVM);

cv::Mat getLabelData(void);
cv::Mat getTrainingData(void);

cv::Mat label_data_;
cv::Mat training_data_;
cv::Mat lbp_data_;


};

