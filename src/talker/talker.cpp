#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

void talker_fucntion(std::string messageOnLoop, double loopRate)
{
    // create a node Handle: it is reference assigned to a new node
    ros::NodeHandle n;

    //advertise is the template method which is used by NodeHandle to create publisher with a topic "chatter"
    ros::Publisher chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

    // rate is a class that is used to define frequency for a loop,
    // i.e max number of messages that can be published in a second
    ros::Rate loop_rate(loopRate);

    // count of the number of messages being published
    int count=0;
    
    while(ros::ok()) // keep spinning the loop until user presses Ctrl+C
    {

        // create a new String ROS message
        std_msgs::String msg;

        // create a string for the data
        std::stringstream ss;
        ss<<messageOnLoop<<count;

        // assign the string data to ROS message data field
        msg.data = ss.str();

        // print the content of the message in the terminal
        ROS_INFO("[TALKER] I said %s\n", msg.data.c_str());
   
        // publish the message
        chatter_publisher.publish(msg);
   
        // call this function to allow ROS to process incoming messages
        ros::spinOnce();

        // sleep for the rest of the cycle, to enfore the loop rate
        loop_rate.sleep();
        count++;
    }
}

int main(int argc, char** argv)
{
    // initiate new ROS node named "talker_node"
    ros::init(argc, argv,"talker_node");
    
    talker_fucntion("Hello World ", 0.5);

    return 0;
}