#include "Motion.h"

int main(int argc, char** argv)
{

	//initiate a new ROS node named "turtleBot_move2Location"
	ros::init(argc, argv, "turtleBot_move2Location");
	
	// create a new node handle
	ros::NodeHandle n;
	
	// create a variable of type turtlesim::Pose to store the pose values
	turtlesim::Pose pose;
	
	// create a temp variable that stores values in degrees from the user
	double tempThetaInDegrees;
	
	// take pose and theta values as input from user
	std::cout<<"Enter x coordinate (type-double): \t";
	std::cin>>pose.x;
	std::cout<<"Enter y coordinate (type-double): \t";
	std::cin>>pose.y;
	std::cout<<"Enter theta in degrees (type-double): \t";
	std::cin>>tempThetaInDegrees;
	
	// convert degree to radian as pose.theta stores value in radian only
	pose.theta = degrees2radians(tempThetaInDegrees);

	// create a velocity publisher and publish to the topic "/turtle1/cmd_vel"
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	
	// create a pose subsriber and subscribe to the topic "/turtle1/pose"
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	// call the function move2Location with pose argument
	move2Location(pose, 0.01);

	return 0;
}
