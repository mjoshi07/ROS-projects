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

void poseCallback(turtlesim::Pose pose_message)
{
	turtlesim_pose.x = pose_message.x;
	turtlesim_pose.y = pose_message.y;
	turtlesim_pose.theta = pose_message.theta;
}

void checkForExcessRotation(double& angle_in_degrees)
{
	while(angle_in_degrees >= 360)
	{
		angle_in_degrees -= 360;
	}
}

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

	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

    if(isForward)
    {
        vel_msg.linear.x = abs(linearSpeed);
    }
    else
    {
        vel_msg.linear.x = -abs(linearSpeed);
    }

    double start_time = ros::WallTime::now().toSec();
    double current_distance = 0.0;

    while(current_distance<distance && ros::ok())
    {
        velocity_publisher.publish(vel_msg);
        double end_time = ros::WallTime::now().toSec();
        current_distance = linearSpeed*(end_time - start_time);
    }
    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
}

void rotate(double angularSpeedInDegrees, double relativeAngleInDegrees, bool isClockwise)
{
	checkForExcessRotation(relativeAngleInDegrees);

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
        vel_msg.angular.z = -abs(angularSpeed);
    }
    else
    {
        vel_msg.angular.z = abs(angularSpeed);
    }

    double current_angle_in_degrees = 0.0;
    double start_time = ros::WallTime::now().toSec();
    ros::WallRate loop_rate(100);
	
	do{        
		velocity_publisher.publish(vel_msg);
        double end_time = ros::WallTime::now().toSec();
		current_angle_in_degrees = angularSpeedInDegrees*(end_time - start_time);
		ros::spinOnce();
		loop_rate.sleep();

	}
    while(current_angle_in_degrees<relativeAngleInDegrees && ros::ok());

    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
}

void setDesiredOrientation (double desired_angle_degrees)
{	
	checkForExcessRotation(desired_angle_degrees);

	double relative_angle_radians = degrees2radians(desired_angle_degrees) - turtlesim_pose.theta;
	bool clockwise = ((relative_angle_radians<0)?true:false);
	rotate(30, radians2degrees(abs(relative_angle_radians)), clockwise);
}

void move2Location(turtlesim::Pose  locationPose, double distTolerance)
{
    geometry_msgs::Twist vel_msg;

	double E = 0.0;

	ros::WallRate loop(10);

    while(getDistance(turtlesim_pose.x, turtlesim_pose.y, locationPose.x, locationPose.y)>distTolerance && ros::ok())
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
		loop.sleep();
	}
	vel_msg.linear.x =0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

	setDesiredOrientation(radians2degrees(locationPose.theta));

}

void gridMotion()
{
    ros::WallRate loop(1);
	turtlesim::Pose pose;
	pose.x=1;
	pose.y=1;
	pose.theta=0;

	move2Location(pose, 0.01);
	loop.sleep();
	setDesiredOrientation(0.0);
	loop.sleep();

	// move 9 straight and turn left
	moveStraight(2.0, 9.0, true);
	loop.sleep();
	rotate(10, 90, false);
	loop.sleep();

	// move 1 straight and turn left
	moveStraight(2.0, 1.0, true);
	loop.sleep();
	rotate(10, 90, false);
	loop.sleep();

	// move 9 straight and turn right
	moveStraight(2.0, 9.0, true);
	loop.sleep();
	rotate(10, 90, true);
	loop.sleep();

	// move 1 straight and turn right
	moveStraight(2.0, 1.0, true);
	loop.sleep();
	rotate(10, 90, true);
	loop.sleep();

	// move 9 straight and turn left
	moveStraight(2.0, 9.0, true);
	loop.sleep();
	rotate(10, 90, false);
	loop.sleep();

	// move 1 straight and turn left
	moveStraight(2.0, 1.0, true);
	loop.sleep();
	rotate(10, 90, false);
	loop.sleep();

	// move 9 straight and turn right
	moveStraight(2.0, 9.0, true);
	loop.sleep();
	rotate(10, 90, true);
	loop.sleep();

	// move 1 straight and turn right
	moveStraight(2.0, 1.0, true);
	loop.sleep();
	rotate(10, 90, true);
	loop.sleep();

	// move 9 straight and turn left
	moveStraight(2.0, 9.0, true);
	loop.sleep();
	rotate(10, 90, false);
	loop.sleep();

	// move 1 straight and turn left
	moveStraight(2.0, 1.0, true);
	loop.sleep();
	rotate(10, 90, false);
	loop.sleep();

	// move 9 straight and turn right
	moveStraight(2.0, 9.0, true);
	loop.sleep();
	rotate(10, 90, true);
	loop.sleep();

	// move 1 straight and turn right
	moveStraight(2.0, 1.0, true);
	loop.sleep();
	rotate(10, 90, true);
	loop.sleep();

	// move 9 straight and turn left
	moveStraight(2.0, 9.0, true);
	loop.sleep();
	rotate(10, 90, false);
	loop.sleep();

	// move 1 straight and turn left
	moveStraight(2.0, 1.0, true);
	loop.sleep();
	rotate(10, 90, false);
	loop.sleep();

	// move 9 straight and turn right
	moveStraight(2.0, 9.0, true);
	loop.sleep();
	rotate(10, 90, true);
	loop.sleep();

	// move 1 straight and turn right
	moveStraight(2.0, 1.0, true);
	loop.sleep();
	rotate(10, 90, true);
	loop.sleep();

}

void spiralMotion()
{

    geometry_msgs::Twist vel_msg;
	double constant_speed=4;
	double rk = 0.5;

	ros::WallRate loop(1);

	while((turtlesim_pose.x<10.5) && (turtlesim_pose.y<10.5) && ros::ok())
	{
		rk += 1.0;
		vel_msg.linear.x =rk;
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		//set a random angular velocity in the y-axis
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =constant_speed;

		velocity_publisher.publish(vel_msg);
		loop.sleep();
		ros::spinOnce();
	}

	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

