#include "MachineLearning/ipa_train_classifier.h"

TrainClassifier::TrainClassifier()
{
}

//implementation of LBP
std::vector<float> TrainClassifier::calculateLBP(cv::Mat src) {

    std::vector<float> feature_vec;

    for(int i=1;i<src.rows-1;i++) {
        for(int j=1;j<src.cols-1;j++) {
            float center = src.at<float>(i,j);

            float pixel_7 = (src.at<float>(i-1,j-1) > center);
            float pixel_6 = (src.at<float>(i-1,j) > center);
            float pixel_5 = (src.at<float>(i-1,j+1) > center);
            float pixel_4 = (src.at<float>(i,j+1) > center);
            float pixel_3 = (src.at<float>(i+1,j+1) > center);
            float pixel_2 = (src.at<float>(i+1,j) > center);
            float pixel_1 = (src.at<float>(i+1,j-1) > center);
            float pixel_0 = (src.at<float>(i,j-1) > center);


            float dec_val =  (int) pixel_7 * 2^7 + (int) pixel_6 * 2^6 + (int) pixel_5 * 2^5 + (int) pixel_4 * 2^4 + (int) pixel_3 * 2^3 + (int) pixel_2 * 2^2 + (int) pixel_1 * 2 + (int) pixel_0;

            feature_vec.push_back(dec_val);

        }
    }

    return feature_vec;
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
  

        img_data.push_back(img);

    }


    return img_data;

}

void TrainClassifier::generateTrainingData(std::vector<cv::Mat> image_seq, bool label)
{

std::vector<float> feature_vec;
cv::Mat feature_mat;
cv::Mat feature;
cv::Mat labels_mat;

    for (int i =0; i < image_seq.size(); i++)
    {
        cv::Mat curr_img_img = image_seq[i];
        feature_vec = calculateLBP(curr_img_img);
        feature = cv::Mat( feature_vec ).reshape( 0, feature_vec.size() );
        feature = feature.t();

        // write the feature vector of each img into a common matrix
        feature_mat.push_back(feature); 
        
    }

   int num_features = feature_mat.rows;

    if (label)
      {
           labels_mat = cv::Mat::ones(num_features,1, CV_32SC1);
      }
      else
      {
           labels_mat = cv::Mat::zeros(num_features,1, CV_32SC1);
      }


    label_data_ = labels_mat;
    training_data_ = feature_mat;

    }

cv::Mat TrainClassifier::getLabelData(void) 
{
    return label_data_;
}

 cv::Mat TrainClassifier::getTrainingData(void) 
{
    return training_data_;
}




int main(int argc, char **argv)
{

  std::string posDepthImagePath= "/home/rmb-ml/Desktop/TemplateDataBase/DepthImages/Positive/";
  std::string negDepthImagePath= "/home/rmb-ml/Desktop/TemplateDataBase/DepthImages/Negative/";
  std::string classifier_path= "/home/rmb-ml/Desktop/TemplateDataBase/classifierModel/";

   TrainClassifier();
  
   
   TrainClassifier c1;
   TrainClassifier c2;

   std::vector<cv::Mat> posImg, negImg;

    //reading files from dir
   posImg = c1.readDepthImages(posDepthImagePath);
   negImg = c2.readDepthImages(negDepthImagePath);

   // get positive and negative training data
   c1.generateTrainingData(posImg,1);
   c2.generateTrainingData(negImg,0);

   cv::Mat label_pos    = c1.getLabelData();
   cv::Mat training_pos = c1.getTrainingData();

   cv::Mat label_neg    = c2.getLabelData();
   cv::Mat training_neg = c2.getTrainingData();


   // concentrate data
    cv::Mat training_data_all;
    training_data_all.push_back(training_pos);
    training_data_all.push_back(training_neg);

    cv::Mat label_data_all;
    label_data_all.push_back(label_pos);
    label_data_all.push_back(label_neg);


    // ==================================== SVM ================================

   CvSVMParams params;
    params.svm_type    = CvSVM::C_SVC;
    params.kernel_type = CvSVM::LINEAR;
    params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);

    CvSVM SVM;
    std::cout<<"Start Training"<<std::endl;
    std::cout<<"Num Pos Samples: " << training_pos.rows <<std::endl;
    std::cout<<"Num Neg Samples: " << training_neg.rows <<std::endl;
     std::cout<<"===============================================" << std::endl;

    std::string full_path_svm = classifier_path +"SVM_Model.xml";

    SVM.train(training_data_all, label_data_all, cv::Mat(), cv::Mat(), params);
    SVM.save(full_path_svm.c_str());

    // ==================================== SVM ================================

    // ==================================== Boost ================================

    CvBoost boost;
    
    boost.train(training_data_all, CV_ROW_SAMPLE, label_data_all);

    std::string full_path_boost = classifier_path +"Boost_Model.xml";
    boost.save(full_path_boost.c_str());

    // ==================================== Boost ================================

    // ==================================== Random Trees ================================


    // ==================================== Random Trees ================================


    std::cout<<"Training completed"<<std::endl;

    cv::Mat testSample = training_data_all.row(21);

     float responseSVM = SVM.predict(testSample);
     float responseBoost = boost.predict(testSample);

     std::cout<<"===============================================" << std::endl;

     std::cout<<"response SVM:" << responseSVM<<std::endl;
     std::cout<<"response SVM:" << responseBoost<<std::endl;

     std::cout<<"===============================================" << std::endl;

    return 0;
}

