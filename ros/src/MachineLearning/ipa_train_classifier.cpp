#include "MachineLearning/ipa_train_classifier.h"

TrainClassifier::TrainClassifier()
{

    posDepthImagePathTrain= "/home/rmb-ml/Desktop/TemplateDataBase/DepthImages/Train/Positive/";
    negDepthImagePathTrain= "/home/rmb-ml/Desktop/TemplateDataBase/DepthImages/Train/Negative/";

    posDepthImagePathTest= "/home/rmb-ml/Desktop/TemplateDataBase/DepthImages/Test/Positive/";
    negDepthImagePathTest= "/home/rmb-ml/Desktop/TemplateDataBase/DepthImages/Test/Negative/";

    classifier_path= "/home/rmb-ml/Desktop/TemplateDataBase/classifierModel/";
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


void TrainClassifier::calculatingPredictions(cv::Mat training_data,cv::Mat label_data, CvSVM &SVM)
{

            std::cout << "Calculating Predictions ...\n"<<std::endl;
            unsigned int truePositive = 0;
            unsigned int trueNegative = 0;
            unsigned int falsePositive = 0;
            unsigned int falseNegative = 0;

            for( int num_img = 0; num_img < training_data.rows; num_img++)
                {
                    cv::Mat test_sample = training_data.row(num_img);
                    int predictedLabel = SVM.predict(test_sample);
                    int trueLabel =  label_data.at<int>(num_img,0);

                    if ( predictedLabel == 1 && trueLabel == 1 ) ++truePositive;
                    else if ( predictedLabel == 1 && trueLabel == 0 ) ++falsePositive;
                    else if ( predictedLabel == 0 && trueLabel == 1) ++falseNegative;
                    else if ( predictedLabel == 0 && trueLabel == 0) ++trueNegative;
                    else std::cout << "Invalid Classification (make sure you are using binary labels)" << std::endl; //invali
                }

                double accuracy = (truePositive + trueNegative) * 100.0 / (truePositive + falsePositive + trueNegative + falseNegative);
                double precision = truePositive * 100.0 / (truePositive + falsePositive);
                double true_negative_rate = trueNegative * 100.0 / (trueNegative + falsePositive);
                double recall = truePositive * 100.0 / (truePositive + falseNegative);

                std::cout
                    << "-----------------------------------------------------------------------\n"
                    << "Training Set Classification:\n"
                    << "-----------------------------------------------------------------------\n"
                    << "True Positive     : " << truePositive << "\n"
                    << "False Positive    : " << falsePositive << "\n"
                    << "True Negative     : " << trueNegative << "\n"
                    << "False Negative    : " << falseNegative << "\n"
                    << "Accuracy          : " << accuracy << "\n"
                    << "Precision         : " << precision << "\n"
                    << "True negative rate: " << true_negative_rate << "\n"
                    << "Recall            : " << recall << "\n"
                    << "-----------------------------------------------------------------------\n";
            


}




void TrainClassifier::trainAllClassifier(cv::Mat training_data,cv::Mat label_data,bool trainSVM, bool trainRF,bool trainAB)
{

        // ==================================== SVM ================================
         if(trainSVM)
         {


            std::cout<<"TrainSVM..." << std::endl;
            CvSVMParams params;
            params.svm_type    = CvSVM::C_SVC;
            params.kernel_type = CvSVM::LINEAR;
            params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);

            CvSVM SVM;
            std::string full_path_svm = classifier_path +"SVM_Model.xml";
            SVM.train(training_data, label_data, cv::Mat(), cv::Mat(), params);
            
            TrainClassifier::calculatingPredictions(training_data,label_data,SVM);

            SVM.save(full_path_svm.c_str());
         }

            // ==================================== SVM ================================

            // ==================================== Boost ================================
        if(trainAB)
        {

            std::cout<<"TrainAdaBoost... "<< std::endl;
            CvBoost boost;
            
            boost.train(training_data, CV_ROW_SAMPLE, label_data);

           // TrainClassifier::calculatingPredictions();

            std::string full_path_boost = classifier_path +"Boost_Model.xml";
            boost.save(full_path_boost.c_str());

        }

        // ==================================== Boost ================================

        // ==================================== Random Trees ================================

        if (trainRF)
        {

             std::cout<<"TrainRandomForrest..." << std::endl;

            CvRTParams paramsRF;
            paramsRF.min_sample_count  = 10;
            paramsRF.max_depth = 5;
            paramsRF.term_crit = cvTermCriteria( CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 50, 0.1 );

            CvRTrees randomForest;
        
            std::string full_path_rf = classifier_path +"RF_Model.xml";
            randomForest.train(training_data, CV_ROW_SAMPLE, label_data, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), paramsRF );

             //TrainClassifier::calculatingPredictions();

            
            randomForest.save(full_path_rf.c_str());

          //  float err = randomForest.get_train_error();
          //  std::cout<<err<<std::endl;

        }

        std::cout<<"===================================================================="<<std::endl;

}


DataStruct TrainClassifier::buildTrainingData()
{
    TrainClassifier data_pos;
    TrainClassifier data_neg;

    std::vector<cv::Mat> posImg, negImg;

    //reading files from dir
    posImg = data_pos.readDepthImages(posDepthImagePathTrain);
    negImg = data_neg.readDepthImages(negDepthImagePathTrain);

    // get positive and negative training data
    data_pos.generateTrainingData(posImg,1);
    data_neg.generateTrainingData(negImg,0);

    cv::Mat label_pos    = data_pos.getLabelData();
    cv::Mat training_pos = data_pos.getTrainingData();

    cv::Mat label_neg    = data_neg.getLabelData();
    cv::Mat training_neg = data_neg.getTrainingData();

    // concentrate data
    cv::Mat training_data;
    training_data.push_back(training_pos);
    training_data.push_back(training_neg);

    cv::Mat training_data_labels;
    training_data_labels.push_back(label_pos);
    training_data_labels.push_back(label_neg);

    std::cout<<"===============================================" << std::endl;
    std::cout<<"Training DB"<<std::endl;
    std::cout<<"Num Pos Samples: " << training_pos.rows <<std::endl;
    std::cout<<"Num Neg Samples: " << training_neg.rows <<std::endl;
    std::cout<<"===============================================" << std::endl;

    DataStruct trainingData;
	trainingData.labels = training_data_labels;
 	trainingData.features = training_data;

    return trainingData;
}

DataStruct TrainClassifier::buildTestingData()
{
    TrainClassifier data_pos;
    TrainClassifier data_neg;

    std::vector<cv::Mat> posImg, negImg;

    //reading files from dir
    posImg = data_pos.readDepthImages(posDepthImagePathTest);
    negImg = data_neg.readDepthImages(negDepthImagePathTest);

    // et positive and negative training data
    data_pos.generateTrainingData(posImg,1);
    data_neg.generateTrainingData(negImg,0);

    cv::Mat label_pos    = data_pos.getLabelData();
    cv::Mat testing_pos = data_pos.getTrainingData();

    cv::Mat label_neg    = data_neg.getLabelData();
    cv::Mat testing_neg = data_neg.getTrainingData();

    // concentrate data
    cv::Mat testing_data;
    testing_data.push_back(testing_pos);
    testing_data.push_back(testing_neg);

    cv::Mat testing_data_labels;
    testing_data_labels.push_back(label_pos);
    testing_data_labels.push_back(label_neg);

    std::cout<<"===============================================" << std::endl;
    std::cout<<"Testing DB"<<std::endl;
    std::cout<<"Num Pos Samples: " << testing_pos.rows <<std::endl;
    std::cout<<"Num Neg Samples: " << testing_neg.rows <<std::endl;
    std::cout<<"===============================================" << std::endl;

    DataStruct testingData;
	testingData.labels = testing_data_labels;
 	testingData.features = testing_data;

}


int main(int argc, char **argv)
{

    bool trainSVM =0;
    bool trainRF = 0;
    bool trainAB = 0;

    if (argc < 2)
    {
        std::cout<<"You need at least one argument: SVM, RF or AB"<<std::endl;
    }

    else
    {

        for (int i=1; i<argc; i++) 
        {
            std::string class_str = argv[i];

            if(class_str == "SVM")
            {
                trainSVM=1;
            }
            else if(class_str == "AB")
            {
                trainAB =1;
            }
            else if(class_str == "RF")
            {
                trainRF = 1;
            }

        }
     
    }    
        TrainClassifier();

        DataStruct trainingData = TrainClassifier::buildTrainingData();

        DataStruct testingData = TrainClassifier::buildTestingData();


        cv::Mat training_data = trainingData.features;
        cv::Mat labels = trainingData.labels;

        TrainClassifier::trainAllClassifier(training_data,labels,trainSVM,trainRF,trainAB);

    return 0;
}

