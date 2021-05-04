#ifndef COMMON_H
#define COMMON_H

#define PI 3.14159265359

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

void moveStraight(double linearSpeed, double distance, bool isForward);
void rotate(double angularSpeed, double relativeAngle, bool isClockwise);
void move2Location(turtlesim::Pose  goal_pose, double distance_tolerance);
void gridClean();
void spiralClean();

double degrees2radians(double angle_in_degrees)
{
    return angle_in_degrees * (PI / 180.0);
}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

inline void setDesiredOrientation (double desired_angle_radians)
{
	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	bool clockwise = ((relative_angle_radians<0)?true:false);
	rotate (degrees2radians(10), abs(relative_angle_radians), clockwise);

}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}

#endif