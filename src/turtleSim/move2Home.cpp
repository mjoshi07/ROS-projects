#include "Motion.h"

int main(int argc, char** argv)
{
	// initiate a new ROS node named "turtleBot_move2Home"
	ros::init(argc, argv, "turtleBot_move2Home");
	
	// create a new node handle
	ros::NodeHandle n;
	
	// create a variable of type turtlesim::Pose and store the starting location of turtleBot
	turtlesim::Pose pose;
	pose.x = 5.544445;
	pose.y = 5.544445;
	pose.theta = 0.0;

	// create a velocity publisher and publish to the topic "/turtle1/cmd_vel"
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	
	// create a pose subsciber and subscribe to the topic "/turtle1/pose"
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
	
	// call the function move2Location with starting pose location
	move2Location(pose, 0.01);
	
	// call the function setDesiredOrientation and set it to 0 
	setDesiredOrientation(0);

	return 0;
}
