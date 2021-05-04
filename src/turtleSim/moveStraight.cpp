#include "common.h"


void moveStraight(double linearSpeed, double distance, bool isForward)
{
    geometry_msgs::Twist vel_msg;

	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;

    if(isForward)
    {
        vel_msg.linear.x = abs(linearSpeed);
    }
    else
    {
        vel_msg.linear.x = -abs(linearSpeed);
    }

    double start_time = ros::Time::now().toSec();
    double current_distance = 0.0;

    ros::Rate loop_rate(100);

    while(current_distance<distance)
    {
        velocity_publisher.publish(vel_msg);
        double end_time = ros::Time::now().toSec();
        current_distance = linearSpeed*(end_time - start_time);
        ros::spinOnce();
        loop_rate.sleep();
    }
    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "turtleBot_moveStraight");
	ros::NodeHandle n;
	double speed = 2;
    double distance = 10;;
	bool isForward = false;;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

    moveStraight(speed, distance, isForward);
 
    return 0;
}
