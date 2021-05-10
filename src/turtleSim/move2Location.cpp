#include "Motion.h"

int main(int argc, char** argv)
{

	ros::init(argc, argv, "turtleBot_move2Location");
	ros::NodeHandle n;
	turtlesim::Pose pose;
	pose.x = 8;
	pose.y = 4;
	pose.theta = 0;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	move2Location(pose, 0.01);

	pose.x = 5.554;
	pose.y = 5.554;
	pose.theta = 0;
	move2Location(pose, 0.01);

	return 0;
}