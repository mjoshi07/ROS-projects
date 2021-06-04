#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define PI 3.14159265359

// declare a susbscriber
ros::Subscriber scanSubscriber;

// declare radian2degree function
double radian2degree(double angle_in_radians);

// declare scan callback function
void scanCallback (sensor_msgs::LaserScan::ConstPtr LaserScanMsg);

int main(int argc, char **argv){

	//initialize the ROS node named "scan_subscribe"
	ros::init(argc, argv, "scan_subscriber");
	
	// create a new node Handle
	ros::NodeHandle n;

    	// defines the frequency of the loop
	ros::Rate loop(1);
	
	//subscribe to the laser scanner topic
	scanSubscriber = n.subscribe("/scan", 10, scanCallback);

	// allows ROS to process the incoming messages
	ros::spin();
	
	// sleeps for the rest of the cycle, enforces the loop rate
	loop.sleep();
}

double radian2degree(double angle_in_radians)
{
    return angle_in_radians * (180.0 / PI);
}

void scanCallback (sensor_msgs::LaserScan::ConstPtr LaserScanMsg)
{
	// number of measurements the scanner can measure
	std::cout << std::setw(20)<<"Size: " << LaserScanMsg->ranges.size() <<std::endl;

	// angular distance between measurements [radian]
	std::cout << std::setw(20)<< "Angle increment: "<<"[" <<LaserScanMsg->angle_increment <<" rad, " <<radian2degree(LaserScanMsg->angle_increment) <<" deg]"<<std::endl;  

	// start angle of the scan [radian]
	std::cout << std::setw(20)<< "Minimum angle: " <<"["<<LaserScanMsg->angle_min <<" rad, " <<radian2degree(LaserScanMsg->angle_min) <<" deg]"<<std::endl; 

	// end angle of the scan [radian]
	std::cout << std::setw(20)<< "Maximum angle: " <<"["<<LaserScanMsg->angle_max<<" rad, " <<radian2degree(LaserScanMsg->angle_max) <<" deg]" <<std::endl; 

	// min range value [meter]
	std::cout << std::setw(20)<< "Minumum range: " <<LaserScanMsg->range_min <<" m"<<std::endl; 

	// max range value [meter]
	std::cout << std::setw(20)<< "Maximum range: " <<LaserScanMsg->range_max <<" m"<<std::endl; 

	// time between scans [seconds]
	std::cout << std::setw(20)<< "Scan time: " <<LaserScanMsg->scan_time <<" seconds\n"<<std::endl; 

}
