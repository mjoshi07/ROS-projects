#include "ros/ros.h"
#include "std_msgs/String.h"

// Topic messages Callback
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("[Listener] I heard: [%s]\n", msg->data.c_str());
}

void listener_function()
{
    // create a node handle
    ros::NodeHandle node;

    // subscribe to a given topic, "chatter"
    // chatterCallback is a callback function which will be executed each time a message is received
    ros::Subscriber sub = node.subscribe("chatter", 1000, chatterCallback);

    // enter a loop for callback function 
    ros::spin();
}

int main(int argc, char** argv)
{
    // inititate a new ROS node named "Listener_node"
    ros::init(argc, argv, "Listener_node");

    listener_function();

    return 0;

}