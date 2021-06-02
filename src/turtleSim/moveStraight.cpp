#include "Motion.h"

int main(int argc, char **argv)
{
	// initiate a new ROS node named "turtleBot_moveStraight"
	ros::init(argc, argv, "turtleBot_moveStraight");
	
	// create a new node handle
	ros::NodeHandle n;
	
	// create variables to store the values
	double speed;
	double distance;
	bool isForward;

	// take speed, distance, isForward values as input from user
	std::cout<<"Enter speed: \t";
	std::cin>>speed;
	std::cout<<"Enter distance: \t";
	std::cin>>distance;
	std::cout<<"\nEnter 1 to move forward, 0 to move backwards: \t";
	int direction;
	std::cin>>direction;
	isForward = direction?true:false;
	std::cout<<"Moving...\n";

	// create a velocity publisher and publish at the topic "/turtle1/cmd_vel"
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	
	// create a pose subscriber and subsribe to the topic "/turtle1/pose"
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	// call the function moveStraight with the arguments speed, distance, isForward
	moveStraight(speed, distance, isForward);

	return 0;
}
