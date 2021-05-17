#include "Motion.h"

int main(int argc, char** argv)
{

	ros::init(argc, argv, "turtleBot_move2Location");
	ros::NodeHandle n;
	turtlesim::Pose pose;
	
	std::cout<<"Enter x coordinate (type-double): \t";
	std::cin>>pose.x;
	std::cout<<"Enter y coordinate (type-double): \t";
	std::cin>>pose.y;
	std::cout<<"Enter theta in degrees (type-double): \t";
	double tempThetaInDegrees;
	std::cin>>tempThetaInDegrees;
	pose.theta = degrees2radians(tempThetaInDegrees);

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	move2Location(pose, 0.01);

	return 0;
}
