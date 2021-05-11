#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define PI 3.14159265359

ros::Subscriber scanSubscriber;

double radian2degree(double angle_in_radians);
void scanCallback (sensor_msgs::LaserScan::ConstPtr LaserScanMsg);

int main(int argc, char **argv){

	//initialize the ROS node
	ros::init(argc, argv, "scan_subscriber_cpp");
	ros::NodeHandle n;

    //ros::Rate loop(0.5);
	//subscribe to the laser scanner topic
	scanSubscriber = n.subscribe("/scan", 10, scanCallback);

    //loop.sleep();
	ros::spin();
}
double radian2degree(double angle_in_radians)
{
    return angle_in_radians * (180.0 / PI);
}
void scanCallback (sensor_msgs::LaserScan::ConstPtr LaserScanMsg)
{
    std::cout << std::setw(20)<<"Size: " << LaserScanMsg->ranges.size() <<std::endl;
    std::cout << std::setw(20)<< "Angle increment: "<<"[" <<LaserScanMsg->angle_increment <<" rad, " <<radian2degree(LaserScanMsg->angle_increment) <<" deg]"<<std::endl; // angular distance between measurements [rad]
    std::cout << std::setw(20)<< "Minimum angle: " <<"["<<LaserScanMsg->angle_min <<" rad, " <<radian2degree(LaserScanMsg->angle_min) <<" deg]"<<std::endl; //start angle of the scan [rad]
    std::cout << std::setw(20)<< "Maximum angle: " <<"["<<LaserScanMsg->angle_max<<" rad, " <<radian2degree(LaserScanMsg->angle_max) <<" deg]" <<std::endl; //end angle of the scan [rad]
    std::cout << std::setw(20)<< "Minumum range: " <<LaserScanMsg->range_min <<" m"<<std::endl; 
    std::cout << std::setw(20)<< "Maximum range: " <<LaserScanMsg->range_max <<" m"<<std::endl; 
    std::cout << std::setw(20)<< "Scan time: " <<LaserScanMsg->scan_time <<" seconds\n"<<std::endl; 

}