#include "Motion.h"

int main(int argc, char** argv)
{
	// initiate a new ROS node named "turtleBot_gridMotion"
	ros::init(argc, argv, "turtleBot_gridMotion");
	
	// create a new node handle
	ros::NodeHandle n;

	// create a velocity publisher and publish to the topic "/turtle1/cmd_vel"
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	
	// create a pose subscriber and subscribe to the topic "/turtle1/pose"
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	// call the function gridMotion, you may change the default values inside this function
	gridMotion();
	
	return 0;

}
