#include "Motion.h"

int main(int argc, char** argv)
{
	// initiate a new ROS node named "turtleBot_rotate"
	ros::init(argc, argv, "turtleBot_rotate");
	
	// create a new node handle
	ros::NodeHandle n;
	
	// create variables to store the values
	double angularSpeedInDegrees;
	double relativeAngleInDegrees;
	bool isClockwise;

	// take angularSpeedInDegrees, relativeAngleInDegrees, isClockwise values as input from user
	std::cout<<"Enter angular speed: \t";
	std::cin>>angularSpeedInDegrees;
	std::cout<<"\nEnter relative angle (degrees): \t";
	std::cin>>relativeAngleInDegrees;
	std::cout<<"\nEnter 1 to rotate clockwise, 0 to rotate counter-clockwise: \t";
	int direction;
	std::cin>>direction;
	isClockwise = direction?true:false;
	std::cout<<"Rotating...\n";

	// create a velocity publisher and publish at the topic "/turtle1/cmd_vel"
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	
	// create a pose subscriber and subscribe to the topic "/turtle1/pose"
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	// call the function rotate with the arguments angularSpeedInDegrees, relativeAngleInDegrees, isClockwise
	rotate(angularSpeedInDegrees, relativeAngleInDegrees, isClockwise);
	
	return 0;
}
