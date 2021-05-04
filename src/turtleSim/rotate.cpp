#include "common.h"


void rotate(double angularSpeed, double relativeAngle, bool isClockwise)
{
    geometry_msgs::Twist vel_msg;

	//set a random linear velocity in the x-axis
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

    if(isClockwise)
    {
        vel_msg.angular.z = abs(angularSpeed);
    }
    else
    {
        vel_msg.angular.z = -abs(angularSpeed);
    }

    double current_angle = 0.0;
    double start_time = ros::Time::now().toSec();
    ros::Rate loop_rate(100);

    while(current_angle<relativeAngle)
    {
        velocity_publisher.publish(vel_msg);
        double end_time = ros::Time::now().toSec();
        current_angle = angularSpeed*(end_time - start_time);
        ros::spinOnce();
        loop_rate.sleep();
    }
    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "turtleBot_rotate");
	ros::NodeHandle n;
	double angularSpeed = degrees2radians(10);
    double relativeAngle = degrees2radians(90);
	bool isClockwise = true;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

    rotate(angularSpeed, relativeAngle, isClockwise);

    return 0;
}
