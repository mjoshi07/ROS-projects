#include "Motion.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtleBot_moveStraight");
	ros::NodeHandle n;
	double speed = 2;
	double distance = 10;;
	bool isForward = false;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	moveStraight(speed, distance, isForward);

	return 0;
}
