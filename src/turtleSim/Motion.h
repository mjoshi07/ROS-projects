#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

const double x_min = 0.0;
const double y_min = 0.0;
const double x_max = 11.0;
const double y_max = 11.0;

const double PI = 3.14159265359;

double degrees2radians(double angle_in_degrees)
{
    return angle_in_degrees * (PI / 180.0);
}

double radians2degrees(double angle_in_radians)
{
	return angle_in_radians * (180.0 / PI);
}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

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

void rotate(double angularSpeedInDegrees, double relativeAngleInDegrees, bool isClockwise)
{
    double angularSpeed = degrees2radians(angularSpeedInDegrees);
    double relativeAngle = degrees2radians(relativeAngleInDegrees);
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

void setDesiredOrientation (double desired_angle_radians)
{
	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	bool clockwise = ((relative_angle_radians<0)?true:false);
	// rotate(10, radians2degrees(abs(relative_angle_radians)), clockwise);
	rotate(10, radians2degrees(relative_angle_radians), clockwise);
}

void move2Location(turtlesim::Pose  locationPose, double distTolerance)
{
    geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(150);
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

	setDesiredOrientation(locationPose.theta);
}

void gridMotion()
{
    ros::Rate loop(0.5);
	turtlesim::Pose pose;
	pose.x=1;
	pose.y=1;
	pose.theta=0;
	move2Location(pose, 0.01);
	loop.sleep();
	setDesiredOrientation(0);
	loop.sleep();

	moveStraight(2.0, 9.0, true);
	loop.sleep();
	rotate(10, 90, false);
	loop.sleep();
	moveStraight(2.0, 9.0, true);

	rotate(10, 90, false);
	loop.sleep();
	moveStraight(2.0, 1.0, true);
	rotate(10, 90, false);
	loop.sleep();
	moveStraight(2.0, 9.0, true);

	rotate(30, 90, true);
	loop.sleep();
	moveStraight(2.0, 1.0, true);
	rotate(30, 90, true);
	loop.sleep();
	moveStraight(2.0, 9.0, true);


	double distance = getDistance(turtlesim_pose.x, turtlesim_pose.y, x_max, y_max);
}

void spiralMotion()
{
    geometry_msgs::Twist vel_msg;
	double count =0;

	double constant_speed=4;
	double vk = 1;
	double wk = 2;
	double rk = 0.5;
	ros::Rate loop(1);

	do{
		rk=rk+1.0;
		vel_msg.linear.x =rk;
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		//set a random angular velocity in the y-axis
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =constant_speed;//((vk)/(0.5+rk));

		std::cout<<"vel_msg.linear.x = "<<vel_msg.linear.x<<std::endl;
		std::cout<<"vel_msg.angular.z = "<<vel_msg.angular.z<<std::endl;
		velocity_publisher.publish(vel_msg);
		ros::spinOnce();

		loop.sleep();
		//vk = vel_msg.linear.x;
		//wk = vel_msg.angular.z;
		//rk = vk/wk;
		std::cout<<rk<<", "<<vk <<", "<<wk<<std::endl;
	}while((turtlesim_pose.x<10.5)&&(turtlesim_pose.y<10.5));
	vel_msg.linear.x =0;
	velocity_publisher.publish(vel_msg);
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message)
{
	turtlesim_pose.x = pose_message->x;
	turtlesim_pose.y = pose_message->y;
	turtlesim_pose.theta = pose_message->theta;
}