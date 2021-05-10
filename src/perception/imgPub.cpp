#include "common.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_(nh_);
    image_transport::Publisher image_pub = it_.advertise("camera/image", 1);

    cv::Mat img = cv::imread(argv[1]);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

    ros::Rate loop(5);
    while(nh_.ok())
    {
      image_pub.publish(msg);
      ros::spinOnce();
      loop.sleep();
    }

    return 0;
}
