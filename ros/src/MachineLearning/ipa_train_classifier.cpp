#include "MachineLearning/ipa_train_classifier.h"
#include <ctime>

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


        int height = img.rows;
        int width = img.cols;
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


void TrainClassifier::calculatingPredictionsSVM(cv::Mat data,cv::Mat labels, CvSVM &SVM)
{

            unsigned int truePositive = 0;
            unsigned int trueNegative = 0;
            unsigned int falsePositive = 0;
            unsigned int falseNegative = 0;


            std::clock_t start_SVM;
            double duration_SVM;
            start_SVM = std::clock();

            for( int num_img = 0; num_img < data.rows; num_img++)
                {
                    cv::Mat sample = data.row(num_img);
                    int predictedLabel = SVM.predict(sample);
                    int trueLabel =  labels.at<int>(num_img,0);

                    if ( predictedLabel == 1 && trueLabel == 1 ) ++truePositive;
                    else if ( predictedLabel == 1 && trueLabel == 0 ) ++falsePositive;
                    else if ( predictedLabel == 0 && trueLabel == 1) ++falseNegative;
                    else if ( predictedLabel == 0 && trueLabel == 0) ++trueNegative;
                    else std::cout << "Invalid Classification (make sure you are using binary labels)" << std::endl; //invali
                }

            duration_SVM = ( std::clock() - start_SVM ) / (double) CLOCKS_PER_SEC;
            std::cout<<"Test: time per frame SVM: " << duration_SVM/data.rows << " sec"<< std::endl; 

                double accuracy = (truePositive + trueNegative) * 100.0 / (truePositive + falsePositive + trueNegative + falseNegative);
                double precision = truePositive * 100.0 / (truePositive + falsePositive);
                double true_negative_rate = trueNegative * 100.0 / (trueNegative + falsePositive);
                double recall = truePositive * 100.0 / (truePositive + falseNegative);

                std::cout
                    << "-----------------------------------------------------------------------\n"
                    << "SVM Training Set Classification:\n"
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

void TrainClassifier::calculatingPredictionsAB(cv::Mat data,cv::Mat labels, CvBoost &boost)
{

            unsigned int truePositive = 0;
            unsigned int trueNegative = 0;
            unsigned int falsePositive = 0;
            unsigned int falseNegative = 0;

            std::clock_t start_AB;
            double duration_AB;
            start_AB = std::clock();

            for( int num_img = 0; num_img < data.rows; num_img++)
                {
                    cv::Mat sample = data.row(num_img);
                    int predictedLabel = boost.predict(sample);
                    int trueLabel =  labels.at<int>(num_img,0);

                    if ( predictedLabel == 1 && trueLabel == 1 ) ++truePositive;
                    else if ( predictedLabel == 1 && trueLabel == 0 ) ++falsePositive;
                    else if ( predictedLabel == 0 && trueLabel == 1) ++falseNegative;
                    else if ( predictedLabel == 0 && trueLabel == 0) ++trueNegative;
                    else std::cout <
                    std::cout<< "Invalid Classification (make sure you are using binary labels)" << std::endl; //invali
                }

            duration_AB = ( std::clock() - start_AB ) / (double) CLOCKS_PER_SEC;
            std::cout<<"Test: time per frame AB: " << duration_AB/data.rows << " sec"<< std::endl; 

                double accuracy = (truePositive + trueNegative) * 100.0 / (truePositive + falsePositive + trueNegative + falseNegative);
                double precision = truePositive * 100.0 / (truePositive + falsePositive);
                double true_negative_rate = trueNegative * 100.0 / (trueNegative + falsePositive);
                double recall = truePositive * 100.0 / (truePositive + falseNegative);

                std::cout
                    << "-----------------------------------------------------------------------\n"
                    << "AdaBoost Training Set Classification:\n"
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

void TrainClassifier::calculatingPredictionsRF(cv::Mat data,cv::Mat labels, CvRTrees &randomForest)
{

            unsigned int truePositive = 0;
            unsigned int trueNegative = 0;
            unsigned int falsePositive = 0;
            unsigned int falseNegative = 0;

            std::clock_t start_RF;
            double duration_RF;
            start_RF = std::clock();

            for( int num_img = 0; num_img < data.rows; num_img++)
                {
                    cv::Mat sample = data.row(num_img);
                    int predictedLabel = randomForest.predict(sample);
                    int trueLabel =  labels.at<int>(num_img,0);


                    if ( predictedLabel == 1 && trueLabel == 1 ) ++truePositive;
                    else if ( predictedLabel == 1 && trueLabel == 0 ) ++falsePositive;
                    else if ( predictedLabel == 0 && trueLabel == 1) ++falseNegative;
                    else if ( predictedLabel == 0 && trueLabel == 0) ++trueNegative;
                    else std::cout << "Invalid Classification (make sure you are using binary labels)" << std::endl; //invali
                }

              
            duration_RF = ( std::clock() - start_RF ) / (double) CLOCKS_PER_SEC;
            std::cout<<"Testing time per frame RF: " << duration_RF/data.rows << " sec"<< std::endl;  

                double accuracy = (truePositive + trueNegative) * 100.0 / (truePositive + falsePositive + trueNegative + falseNegative);
                double precision = truePositive * 100.0 / (truePositive + falsePositive);
                double true_negative_rate = trueNegative * 100.0 / (trueNegative + falsePositive);
                double recall = truePositive * 100.0 / (truePositive + falseNegative);

                std::cout
                    << "-----------------------------------------------------------------------\n"
                    << "RandomForest Training Set Classification:\n"
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

void TrainClassifier::trainAllClassifier(cv::Mat trainingData,cv::Mat testingData,cv::Mat trainingLabels,cv::Mat testingLabels, bool trainSVM, bool trainRF,bool trainAB,bool trainNB)
{



        // ==================================== SVM ================================
            if(trainNB)
         {


            std::cout<<"TrainBayes..." << std::endl;

            CvNormalBayesClassifier bayes;
            std::string full_path_bayes = classifier_path +"Bayes_Model.xml";
            bayes.train(trainingData, trainingLabels);

           // TrainClassifier::calculatingPredictionsSVM(testingData,testingLabels,SVM);

            bayes.save(full_path_bayes.c_str());

         }
         if(trainSVM)
         {


        std::cout<<"TrainSVM..." << std::endl;


        std::cout<<"Init Pixelgrid" << std::endl;
        CvParamGrid CvParamGrid_C(pow(2.0,-5), pow(2.0,15), pow(2.0,2));
        CvParamGrid CvParamGrid_gamma(pow(2.0,-15), pow(2.0,3), pow(2.0,2));
        if (!CvParamGrid_C.check() || !CvParamGrid_gamma.check())
             std::cout<<"The grid is NOT VALID."<< std::endl;
        CvSVM svm;   
        CvSVMParams paramz;
        paramz.kernel_type = CvSVM::RBF;
        paramz.svm_type = CvSVM::C_SVC;
        paramz.term_crit = cvTermCriteria(CV_TERMCRIT_ITER,100,0.000001);


         std::cout<<"Start AutoTraining" << std::endl;

        std::clock_t start_SVM;
        double duration_SVM;
        start_SVM = std::clock();

      //  svm.train_auto(trainingData, trainingLabels, cv::Mat(),  cv::Mat(), paramz,10, CvParamGrid_C, CvParamGrid_gamma, CvSVM::get_default_grid(CvSVM::P), CvSVM::get_default_grid(CvSVM::NU), CvSVM::get_default_grid(CvSVM::COEF), CvSVM::get_default_grid(CvSVM::DEGREE), true);
        
        duration_SVM = ( std::clock() - start_SVM ) / (double) CLOCKS_PER_SEC;
        std::cout<<"Training time SVM: " << duration_SVM << " sec"<< std::endl;  

        paramz = svm.get_params();
        std::cout<<"gamma:"<<paramz.gamma<< std::endl;
        std::cout<<"C:"<<paramz.C<<std::endl;


        //TrainClassifier::calculatingPredictionsSVM(testingData,testingLabels,svm);

        //std::string full_path_svm = classifier_path +"SVM_Model.xml";

       // svm.save(full_path_svm.c_str());


      
            CvSVMParams params;
            CvSVM SVM;   
            // ===== PARAMS SVM=====
            //C-Support Vector Classification. n-class classification (n \geq 2), allows imperfect separation of classes with penalty multiplier C for outliers.
            params.svm_type    = CvSVM::C_SVC; 
            //Radial basis function (RBF), a good choice in most cases.
            params.kernel_type = CvSVM::RBF;
            //Termination criteria of the iterative SVM training procedure which solves a partial case of constrained quadratic optimization problem.
            // You can specify tolerance and/or the maximum number of iterations.+
            double tol = 1e-6;
            int num_iter = 100;
            params.gamma = 3.05176e-05;
            params.C = 0.03125;

            params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, num_iter, tol);
            SVM.train(trainingData, trainingLabels, cv::Mat(), cv::Mat(), params);
            TrainClassifier::calculatingPredictionsSVM(testingData,testingLabels,SVM);

            // ==== PARAMS SVM ===========





         }

            // ==================================== SVM ================================

            // ==================================== Boost ================================
        if(trainAB)
        {

            std::cout<<"TrainAdaBoost... "<< std::endl;
            CvBoost boost;
            CvBoostParams params;

            params.boost_type = CvBoost::REAL;
            params.weak_count =100;
            params.weight_trim_rate = 0.95;
            params.cv_folds = 0;
            params.max_depth = 1;

            // ==== PARAMS BOOST ===========

            // ==== PARAMS BOOST ===========


            std::clock_t start_AB;
            double duration_AB;
            start_AB = std::clock();
            
            boost.train(trainingData, CV_ROW_SAMPLE, trainingLabels);

            duration_AB = ( std::clock() - start_AB ) / (double) CLOCKS_PER_SEC;
            std::cout<<"Train Time AB: " << duration_AB << " sec"<< std::endl; 

            TrainClassifier::calculatingPredictionsAB(testingData,testingLabels,boost);

           // TrainClassifier::calculatingPredictions();

            std::string full_path_boost = classifier_path +"Boost_Model.xml";
            boost.save(full_path_boost.c_str());

        }

        // ==================================== Boost ================================

        // ==================================== Random Trees ================================

        if (trainRF)
        {

             std::cout<<"TrainRandomForrest..." << std::endl;


            std::clock_t start_RF;
            double duration_RF;
            start_RF = std::clock();


            CvRTParams paramsRF;

            int num_samples = trainingData.rows;
            double forest_accuracy = 1e-6;
            int max_num_trees = 1000;

            // ==== PARAMS RF ==========

            paramsRF.min_sample_count  =  6; /// suggested value is 1% of all training data
            paramsRF.max_depth =30;
            paramsRF.max_categories = 2;
            paramsRF.term_crit.max_iter = 100;

            paramsRF.term_crit = cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,max_num_trees,forest_accuracy);

             // ==== PARAMS RF ===========

            CvRTrees randomForest;
        
            std::string full_path_rf = classifier_path +"RF_Model.xml";
            randomForest.train(trainingData, CV_ROW_SAMPLE, trainingLabels, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), paramsRF );


            duration_RF = ( std::clock() - start_RF ) / (double) CLOCKS_PER_SEC;
            std::cout<<"Training time RF: " << duration_RF << " sec"<< std::endl;

             //TrainClassifier::calculatingPredictions();
            TrainClassifier::calculatingPredictionsRF(testingData,testingLabels,randomForest);
            
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
    cv::Mat testing_neg  = data_neg.getTrainingData();

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

    return testingData;

}


int main(int argc, char **argv)
{

    bool trainSVM =0;
    bool trainRF = 0;
    bool trainAB = 0;  
    bool trainNB = 0;

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
            else if(class_str == "NB")
            {
                trainNB = 1;
            }
            else
            {
                std::cout<<"Please enter a valid classifier model name: SVM,AB,RF"<<std::endl;
                return 0;
            }
        }   
    }    
        TrainClassifier();

        std::clock_t start_TrainData;
        double duration_TrainData;
        start_TrainData = std::clock();

        DataStruct trainStruct = TrainClassifier::buildTrainingData();

        duration_TrainData = ( std::clock() - start_TrainData ) / (double) CLOCKS_PER_SEC;
        std::cout<<"Train Data Feature Extraction: " << duration_TrainData << " sec"<< std::endl;


        std::clock_t start_TestData;
        double duration_TestData;
        start_TestData = std::clock();

        DataStruct testStruct = TrainClassifier::buildTestingData();

        duration_TestData = ( std::clock() - start_TestData ) / (double) CLOCKS_PER_SEC;
        std::cout<<"Test Data Feature Extraction: " << duration_TestData << " sec"<< std::endl;

        //============================DEFINE TRAINING AND TESTING DATA ======

        cv::Mat trainingData = trainStruct.features;
        cv::Mat trainingLabels = trainStruct.labels;

        cv::Mat testingData =   testStruct.features;
        cv::Mat testingLabels = testStruct.labels;

        TrainClassifier::trainAllClassifier(trainingData,testingData,trainingLabels,testingLabels,trainSVM,trainRF,trainAB,trainNB);

    return 0;
}

