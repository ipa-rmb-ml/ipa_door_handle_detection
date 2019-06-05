#include "MachineLearning/ipa_train_classifier.h"

TrainClassifier::TrainClassifier()
{
}

void TrainClassifier::calculateLBP(cv::Mat src) {

    int neighbors = 15;
    int radius = 4;

    // apply gaussian blur

    neighbors = std::max(std::min(neighbors,31),1); // set bounds...
    // Note: alternatively you can switch to the new OpenCV Mat_
    // type system to define an unsigned int matrix... I am probably
    // mistaken here, but I didn't see an unsigned int representation
    // in OpenCV's classic typesystem...
    cv::Mat dst = cv::Mat::zeros(src.rows-2*radius, src.cols-2*radius, CV_32SC1);
    for(int n=0; n<neighbors; n++) {
        // sample points
        float x = static_cast<float>(radius) * cos(2.0*M_PI*n/static_cast<float>(neighbors));
        float y = static_cast<float>(radius) * -sin(2.0*M_PI*n/static_cast<float>(neighbors));
        // relative indices
        int fx = static_cast<int>(floor(x));
        int fy = static_cast<int>(floor(y));
        int cx = static_cast<int>(ceil(x));
        int cy = static_cast<int>(ceil(y));
        // fractional part
        float ty = y - fy;
        float tx = x - fx;
        // set interpolation weights
        float w1 = (1 - tx) * (1 - ty);
        float w2 =      tx  * (1 - ty);
        float w3 = (1 - tx) *      ty;
        float w4 =      tx  *      ty;
        // iterate through your data
        for(int i=radius; i < src.rows-radius;i++) {
            for(int j=radius;j < src.cols-radius;j++) {
                float t = w1*src.at<float>(i+fy,j+fx) + w2*src.at<float>(i+fy,j+cx) + w3*src.at<float>(i+cy,j+fx) + w4*src.at<float>(i+cy,j+cx);
                // we are dealing with floating point precision, so add some little tolerance
                dst.at<unsigned int>(i-radius,j-radius) += ((t > src.at<float>(i,j)) && (abs(t-src.at<float>(i,j)) > std::numeric_limits<float>::epsilon())) << n;
            }
        }
    }

    std::cout<<dst<<std::endl;
}

void TrainClassifier::extractFeatures(cv::Mat curr_img)
{


}

std::vector<cv::Mat> TrainClassifier::readDepthImages(std::string img_dir)
{

    cv::String path(img_dir+"*.yml"); //select only jpg
    std::vector<cv::String> fn;
    std::vector<cv::Mat> img_data;

    TrainClassifier c;
    cv::Mat img;

    cv::glob(path,fn,true); // recurse
    for (size_t k=0; k<fn.size(); ++k)
    {
        cv::FileStorage storage(fn[k], cv::FileStorage::READ);
        storage["img"] >> img;
        storage.release();

        if (img.empty()) continue; //only proceed if sucsessful
        // you probably want to do some preprocessing
        c.calculateLBP(img);
     //   std::cout<<img<<std::endl;
        cv::Mat test = c.getLBPFeatures();
     //   std::cout<<test<<std::endl;
        img_data.push_back(img);

    }


    return img_data;

}

void TrainClassifier::generateTrainingData(std::vector<cv::Mat> image_seq, bool label)
{


    for (int i =0; i < image_seq.size(); i++)
    {
    cv::Mat curr_img_img = image_seq[i];

        // extract 

    }

    float labels[11] = { -1, 1, 1, -1, 1, -1, 1, -1 };
    cv::Mat labels_cv = cv::Mat(2, 4, CV_32F, labels);

    float training_data[11][2] = {
        {501, 10}, {508, 15},
        {255, 10}, {501, 255}, {10, 501}, {10, 501}, {11, 501}, {9, 501}, {10, 502}, {10, 511}, {10, 495} };
    cv::Mat training_cv = cv::Mat(11, 2, CV_32F, training_data);

    label_data_ = labels_cv;
    training_data_ = training_cv;

    }

cv::Mat TrainClassifier::getLabelData(void) 
{
    return label_data_;
}

 cv::Mat TrainClassifier::getTrainingData(void) 
{
    return training_data_;
}


 cv::Mat TrainClassifier::getLBPFeatures(void) 
{
    return lbp_data_;
}



int main(int argc, char **argv)
{

  std::string posDepthImagePath= "/home/rmb-ml/Desktop/TemplateDataBase/DepthImages/Positive/";
  std::string negDepthImagePath= "/home/rmb-ml/Desktop/TemplateDataBase/DepthImages/Negative/";

   TrainClassifier();
  
   
   TrainClassifier c1;
   TrainClassifier c2;

   std::vector<cv::Mat> posImg, negImg;

   posImg = c1.readDepthImages(posDepthImagePath);
   negImg = c2.readDepthImages(negDepthImagePath);

   // get positive and negative training data

   c1.generateTrainingData(posImg,1);
   c2.generateTrainingData(negImg,0);

   cv::Mat label_pos    = c1.getLabelData();
   cv::Mat training_pos = c1.getTrainingData();

   cv::Mat label_neg    = c2.getLabelData();
   cv::Mat training_neg = c2.getTrainingData();

    CvBoost boost;
 //   boost.train(training_data_,
  //          CV_ROW_SAMPLE,
  //          label_data_);




    return 0;
}

