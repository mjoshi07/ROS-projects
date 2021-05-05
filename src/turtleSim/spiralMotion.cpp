#include "Motion.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtleBot_spiralMotion");
	ros::NodeHandle n;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

    spiralMotion();
}