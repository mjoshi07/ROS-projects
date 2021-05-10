#include "common.h"

static const std::string OPENCV_WINDOW = "Image window";

void imageCallback(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_subsciber");
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_(nh_);
    image_transport::Subscriber image_sub_ = it_.subscribe("camera/image", 1, imageCallback);

    cv::namedWindow(OPENCV_WINDOW);

    ros::spin();
    return 0;
}

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
   {
      try
     {
         cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
         if(!img.empty())
         {
            cv::circle(img, cv::Point(20,20),10, cv::Scalar(102,50,255));
            cv::imshow(OPENCV_WINDOW, img);
            cv::waitKey(10);
         }
     
     }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }
