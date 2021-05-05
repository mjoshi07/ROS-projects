#include "Motion.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtleBot_rotate");
	ros::NodeHandle n;
	double angularSpeedInDegrees = 10;
	double relativeAngleInDegress = 90;
	bool isClockwise = true;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	rotate(angularSpeedInDegrees, relativeAngleInDegress, isClockwise);

	return 0;
}
