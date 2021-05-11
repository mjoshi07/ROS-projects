#include "Motion.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtleBot_moveStraight");
	ros::NodeHandle n;
	double speed = 2;
	double distance = 10;;
	bool isForward = false;


	std::cout<<"Enter speed (type-double): \t";
	std::cin>>speed;
	std::cout<<"Enter distance (type-double): \t";
	std::cin>>distance;

	bool noValueReceived = false;
	int i = 5;
	while(i>0)
	{
		i--;
		std::cout<<" Enter 1 to move forward, 0 to move backwards \t";
		int direction;
		std::cin>>direction;
		if(direction ==0 || direction ==1)
		{
			isForward = (direction)?true:false;
			break;
		}
		else
		{
			std::cout<<"Enter 1 or 0 only, Tries left "<<i<<'\n';
			noValueReceived = true;
		}
	}
	if(noValueReceived)
	{
		std::cout<<"NO value received for direction, therefore moving forward\n";
		isForward = true;
	}

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	moveStraight(speed, distance, isForward);

	return 0;
}
