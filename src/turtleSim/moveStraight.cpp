#include "Motion.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtleBot_moveStraight");
	ros::NodeHandle n;
	double speed = 2;
	double distance = 10;;
	bool isForward = false;


	std::cout<<"Enter speed: \t";
	std::cin>>speed;
	std::cout<<"Enter distance: \t";
	std::cin>>distance;
	std::cout<<"\nEnter 1 to move forward, 0 to move backwards: \t";
	int direction;
	std::cin>>direction;
	isForward = direction?true:false;
	std::cout<<"Moving...\n";

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	moveStraight(speed, distance, isForward);

	return 0;
}
