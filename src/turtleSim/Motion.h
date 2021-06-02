#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

// declare publisher, subsriber and a turtlesim::Pose variable
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

// set the boundary limits of turtlesim map
const double x_min = 0.0;
const double y_min = 0.0;
const double x_max = 11.0;
const double y_max = 11.0;

const double PI = 3.14159265359;

// pose callback function which takes messages from the subscribed topic
void poseCallback(turtlesim::Pose pose_message)
{
	turtlesim_pose.x = pose_message.x;
	turtlesim_pose.y = pose_message.y;
	turtlesim_pose.theta = pose_message.theta;
}

// if rotation angle is more than 360, then instead of rotating all of it, we can rotate a smaller and equivalent amount to reach that angle
void checkForExcessRotation(double& angle_in_degrees)
{
	while(angle_in_degrees >= 360)
	{
		angle_in_degrees -= 360;
	}
}

// converts degrees to radians
double degrees2radians(double angle_in_degrees)
{
    return angle_in_degrees * (PI / 180.0);
}

// converts radians to degrees
double radians2degrees(double angle_in_radians)
{
	return angle_in_radians * (180.0 / PI);
}

// calculates euclidean distance between 2 points
double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

// moves the turtlebot in a straight line for a given distance at a specified linear speed in a forward/backward direction
void moveStraight(double linearSpeed, double distance, bool isForward)
{
	// create a geometry_msgs::Twist variable which will store the linear/angular speed  
    	geometry_msgs::Twist vel_msg;
	
	// set linear speed in all but x axis as 0, as we moving in a straight line only on a 2D plane
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	
	// set angular speed in all axis as 0
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

    if(isForward)
    {
	// set x component of vel_msg.linear as absolute value of linear speed
	vel_msg.linear.x = abs(linearSpeed);
    }
    else
    {
	// set x component of vel_msgs.linear as negative of absolute value of linear speed
	vel_msg.linear.x = -abs(linearSpeed);
    }
	
    // create a WallTime variable to get the start time in seconds
    double start_time = ros::WallTime::now().toSec();
	
    // set current_distance travelled as 0
    double current_distance = 0.0;

    while(current_distance<distance && ros::ok())
    {
	// publish the vel_msg
        velocity_publisher.publish(vel_msg);
	    
	// create a wallTime variable to get the end time in seconds
        double end_time = ros::WallTime::now().toSec();
	    
	// calculate the distance travelled by multiplying the linear speed and the duration in seconds
        current_distance = linearSpeed*(end_time - start_time);
    }
	
    // since we have covered the required distance, stop the turtlebot by setting its x component of linear speed to 0
    vel_msg.linear.x = 0;
	
    // publish the 0 velocity vel_msg
    velocity_publisher.publish(vel_msg);
}

void rotate(double angularSpeedInDegrees, double relativeAngleInDegrees, bool isClockwise)
{
	// before rotating through all of the angle, calculate if it can be done with a smaller degree of rotation
	checkForExcessRotation(relativeAngleInDegrees);

	// convert degrees to radians
   	double angularSpeed = degrees2radians(angularSpeedInDegrees);
    	double relativeAngle = degrees2radians(relativeAngleInDegrees);
	
	// create a geometry_msgs::Twist vel_msg to store the velocity messages
   	geometry_msgs::Twist vel_msg;

	//set all components of linear velocity to 0, as we only have to rotate and not move forward/backwards
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	
	//set all but z component of angular velocity to 0 as we have to roate in z axis only, perpendicular to x-y plane
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

    if(isClockwise)
    {
	    // set z component of angular velocity to absolute of angularSpeed
	    vel_msg.angular.z = -abs(angularSpeed);
    }
    else
    {
	    // set z component of angular velocity to negative of absolute of angularSpeed
	    vel_msg.angular.z = abs(angularSpeed);
    }

	// set current angle in degrees to 0
    	double current_angle_in_degrees = 0.0;
	
	// create a WallTime variable to get start time in seconds
    	double start_time = ros::WallTime::now().toSec();
	
	// WallRate is a class that is used to define frequency for a loop
    	ros::WallRate loop_rate(100);
	
	do{        
		// publish the velocity
		velocity_publisher.publish(vel_msg);
		
		// create a WallTime variable to get end time in seconds
		double end_time = ros::WallTime::now().toSec();
		
		// calculate the angles rotated by multiplying the angularSpeed by duration
		current_angle_in_degrees = angularSpeedInDegrees*(end_time - start_time);
		
		// call spinOnce to allow ROS to process the incoming messages, and callback function
		ros::spinOnce();
		
		// sleep for the rest of the cycle, to enforce the loop rate
		loop_rate.sleep();

	}
    while(current_angle_in_degrees<relativeAngleInDegrees && ros::ok());

	// after the desired angle of rotation is achieved, stop the turtlebot rotation by setting the z component of angular velocity to 0
    	vel_msg.angular.z = 0;
	
	// publish the vel msg to stop the turtlebot
    	velocity_publisher.publish(vel_msg);
}

void setDesiredOrientation (double desired_angle_degrees)
{	
	// before rotating through all of the angle, calculate if it can be done with a smaller degree of rotation
	checkForExcessRotation(desired_angle_degrees);

	// calculate the relative angles in radians to be rotated
	double relative_angle_radians = degrees2radians(desired_angle_degrees) - turtlesim_pose.theta;
	
	// check whether we have to rotate clockwise or counter-clockwise
	bool clockwise = ((relative_angle_radians<0)?true:false);
	
	// call the rotate function  with a default angular speed of 30
	rotate(30, radians2degrees(abs(relative_angle_radians)), clockwise);
}

void move2Location(turtlesim::Pose  locationPose, double distTolerance)
{
	// create a geometry_msgs::Twist vel_msg to store the velocity messages
    	geometry_msgs::Twist vel_msg;

	double E = 0.0;

	// WallRate is a class that is used to define frequency for a loop
	ros::WallRate loop(10);
	
	// continue the turtlebot motion till the distance between the current location and the destination is <= distanceTolerance
    	while(getDistance(turtlesim_pose.x, turtlesim_pose.y, locationPose.x, locationPose.y)>distTolerance && ros::ok())
	{
		/****** Proportional Controller ******/
		//calculate the velocity proportional to the distance between current location and destination
		double Kp=1.0;
		double Ki=0.02;
		double e = getDistance(turtlesim_pose.x, turtlesim_pose.y, locationPose.x, locationPose.y);
		E += e;
		
		// set linear speed in the x-axis
		vel_msg.linear.x = (Kp*e);
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		
		//set angular velocity in the z-axis
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 4*(atan2(locationPose.y-turtlesim_pose.y, locationPose.x-turtlesim_pose.x)-turtlesim_pose.theta);

		// publish the velocity message
		velocity_publisher.publish(vel_msg);

		// call this function to allow ROS to process incoming messages from the callback function
		ros::spinOnce();
		
		// sleep for the rest of the cycle, to enforce the loop rate
		loop.sleep();
	}
	// after reaching the destination, stop the turtlebot by setting the velocity to 0
	vel_msg.linear.x =0;
	vel_msg.angular.z = 0;
	
	// publish the vel_msg to stop the turtlebot
	velocity_publisher.publish(vel_msg);

	// rotate the turtlebot to desired orientation
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

