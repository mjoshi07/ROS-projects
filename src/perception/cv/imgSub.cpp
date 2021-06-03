#include "common.h"

// define a variable OPENCV_WINDOW, name of the window to be displayed
const std::string OPENCV_WINDOW = "Image window";

// declare the image callback function
void imageCallback(const sensor_msgs::ImageConstPtr& msg);


int main(int argc, char** argv)
{
    // initiate new ROS node named "image_subscriber"
    ros::init(argc, argv, "image_subsciber");
    
    // create a node Handle
    ros::NodeHandle nh_;
    
    // create image transport object and initialize with node handle
    image_transport::ImageTransport it_(nh_);
    
    // create image subscriber and subscribe to the topic "camera/image"
    image_transport::Subscriber image_sub_ = it_.subscribe("camera/image", 1, imageCallback);

    // create a window and name it OPENCV_WINDOW
    cv::namedWindow(OPENCV_WINDOW);
    
    // allow ROS to process the incoming messages
    ros::spin();
    
    return 0;
}

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
   {
      try
     {
            // convert the ImageConstPtr ROS msg to Opencv Mat
            cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
         if(!img.empty())
         {
            // draw a circle on the received image 
            cv::circle(img, cv::Point(20,20),10, cv::Scalar(102,50,255));

            //diplay the image in the named window
            cv::imshow(OPENCV_WINDOW, img);

            //wait for 10 milliseconds and then get a new image
            cv::waitKey(10);
         }
     }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }
