#include "common.h"

int main(int argc, char** argv)
{
    // initiate new ROS node named "image_publisher"
    ros::init(argc, argv, "image_publisher");
    
    // create a node Handle
    ros::NodeHandle nh_;
    
    // create an image transport object and initialize with the node handle
    image_transport::ImageTransport it_(nh_);
    
    // create a image publisher and publish to the topic "camera/image"
    image_transport::Publisher image_pub = it_.advertise("camera/image", 1);

    // read an image
    cv::Mat img = cv::imread(argv[1]);

    // convert opencvMat image to ros ImagePtr msg 
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

    // define frequency of the loop at which messages would be published
    ros::Rate loop(5);
    while(nh_.ok())
    {
        // publish the image
        image_pub.publish(msg);
        
        // allow ROS to process the incoming messages
        ros::spinOnce();
        
        // sleep for the rest of the cycle, to enforce the loop rate
        loop.sleep();
    }

    return 0;
}
