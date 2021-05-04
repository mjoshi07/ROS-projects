#include "common.h"


void move2Location(turtlesim::Pose  locationPose, double distTolerance)
{
    geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(100);
	double E = 0.0;

    while(getDistance(turtlesim_pose.x, turtlesim_pose.y, locationPose.x, locationPose.y)>distTolerance)
	{
		/****** Proportional Controller ******/
		//linear velocity in the x-axis
		double Kp=1.0;
		double Ki=0.02;
		double e = getDistance(turtlesim_pose.x, turtlesim_pose.y, locationPose.x, locationPose.y);
		E += e;
		vel_msg.linear.x = (Kp*e);
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		//angular velocity in the z-axis
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 4*(atan2(locationPose.y-turtlesim_pose.y, locationPose.x-turtlesim_pose.x)-turtlesim_pose.theta);

		velocity_publisher.publish(vel_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}
	vel_msg.linear.x =0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "turtleBot_move2Location");
	ros::NodeHandle n;
 	turtlesim::Pose pose;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	pose.x=8;
	pose.y=6;
	pose.theta=0;
	move2Location(pose, 0.01);

    pose.x=0;
	pose.y=4;
	pose.theta=0;
	move2Location(pose, 0.01);

    return 0;
}