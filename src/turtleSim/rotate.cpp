#include "Motion.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtleBot_rotate");
	ros::NodeHandle n;
	double angularSpeedInDegrees = 10.0;
	double relativeAngleInDegrees = 90.0;
	bool isClockwise = true;

	std::cout<<"Enter angular speed (type-double): \t";
	std::cin>>angularSpeedInDegrees;
	std::cout<<"\nEnter relative angle (type-double): \t";
	std::cin>>relativeAngleInDegrees;
	std::cout<<"\nEnter 1 to rotate clockwise or 0 to rotate counter-clockwise: \t";
	int direction;
	std::cin>>direction;
	isClockwise = direction?true:false;
	std::cout<<"Rotating...\n";

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	rotate(angularSpeedInDegrees, relativeAngleInDegrees, isClockwise);
	
	return 0;
}
