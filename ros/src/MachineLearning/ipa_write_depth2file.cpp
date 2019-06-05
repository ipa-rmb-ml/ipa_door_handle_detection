#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/console.h>
#include <boost/filesystem.hpp>
#include<iomanip>

static const std::string OPENCV_WINDOW = "Image window";

std::string img_path;
int img_counter;

class ImageConverter
   {
     ros::NodeHandle nh_;
     image_transport::ImageTransport it_;
     image_transport::Subscriber image_sub_;
     image_transport::Publisher image_pub_;
   
   public:
    void writeToDir(cv::Mat depth_image)
    {

    std::string file_path = img_path;
    std::ostringstream oss;

     const char* path = file_path.c_str();
     boost::filesystem::path dir(path);

     if(boost::filesystem::create_directory(dir))
        {
            ROS_WARN("Creating Directory:");
            std::cout<< file_path <<std::endl;
           
        }

    // counter to append to image name
    img_counter = img_counter + 1;

    // creating format: 0001 etc...
    oss << std::setw(4) << std::setfill('0') <<  img_counter;

    std::string img_name = "image_" + oss.str();
    std::string full_name = file_path + img_name + ".yml";   

    std::cout<<"Writing " << img_name << ".yml ..."<<std::endl;


    cv::FileStorage storage(full_name, cv::FileStorage::WRITE);
    storage << "img" << depth_image;
    storage.release();  
    }



     ImageConverter()
       : it_(nh_)
     {
       // Subscrive to input video feed and publish output video feed
       image_sub_ = it_.subscribe("pico_flexx/image_depth", 1, &ImageConverter::imageCb, this);
       image_pub_ = it_.advertise("/image_converter/output_video", 1);
   
       cv::namedWindow(OPENCV_WINDOW);
     }
   
     ~ImageConverter()
     {
       cv::destroyWindow(OPENCV_WINDOW);
     }
   
     void imageCb(const sensor_msgs::ImageConstPtr& msg)
     {
       cv_bridge::CvImagePtr cv_ptr;
       try
       {
         cv_ptr = cv_bridge::toCvCopy(msg);
       }
       catch (cv_bridge::Exception& e)
       {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
       }
   
        // mat image declaration -> convert to mm
        cv::Mat depth_img = cv_ptr->image*1000;

        // CV_<bit-depth>{U|S|F}C(<number_of_channels>)
        //depth_img.convertTo(depth_img, CV_8UC1); 

        int imgDepth = depth_img.depth();
       
       // Update GUI Window
       cv::imshow(OPENCV_WINDOW, cv_ptr->image);
       cv::waitKey(3);
   
       // Output modified video stream
       image_pub_.publish(cv_ptr->toImageMsg());

       writeToDir(depth_img);

    }
   };


int main(int argc, char **argv)
{


    if (argc > 0)
    {

     img_path = argv[1];

     img_counter = 0;
     ros::init(argc, argv, "image_converter");
     ImageConverter ic;
     ros::spin();

    }
    else
    {
      ROS_WARN("Check number entries! Must contain directory name and pos/neg folder for depth images.");
    }
    


    return 0;
}


