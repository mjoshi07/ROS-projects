#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"


ros::Publisher velocity_publisher;
ros::Subscriber laser_scan_subscriber;

const double PI = 3.14159265359;
const double normal_linear_vel = 0.4;
const double obstacle_angular_vel = 1.5;
const double obstacle_linear_vel = -0.07;
const double OBSTACLE_MIN_DIST = 0.7;
const double OBSTACLE_MAX_DIST = 1.5;
double global_min_distance = 99999.99;

double radian2degree(double angle_in_radians);
void scan_callback (sensor_msgs::LaserScan scanMessage);
void start_moving();


int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_avoider");
    ros::NodeHandle nh;

    velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);

    //ros::WallRate loop(10);
    while(ros::ok())
    {  
        laser_scan_subscriber = nh.subscribe("/scan",1000, scan_callback);
        start_moving();
    }
}

double radian2degree(double angle_in_radians)
{
    return angle_in_radians * (180.0 / PI);
}

void scan_callback(sensor_msgs::LaserScan LaserScanMsg)
{
    //double degree = 120.0;
    //int start_index =(int)(LaserScanMsg.ranges.size()/2)-(int)((degree/radian2degree(LaserScanMsg.angle_increment))/2);
    //int end_index = (int)(LaserScanMsg.ranges.size()/2)+(int)((degree/radian2degree(LaserScanMsg.angle_increment))/2);
    int min_index = -1;
    int start_index = 0;
    int end_index = LaserScanMsg.ranges.size();

    for(int i = start_index; i < end_index ; i++)
    {
        if(!std::isnan(LaserScanMsg.ranges[i]))
        {
            if((LaserScanMsg.ranges[i]>= LaserScanMsg.range_min) && (LaserScanMsg.ranges[i]<= LaserScanMsg.range_max))
            {
                min_index = i;
                break;
            }
        }
    }
    global_min_distance = (min_index != -1)?LaserScanMsg.ranges[min_index]:0.1;
   
}

void start_moving()
{

    geometry_msgs::Twist vel_msg;

    vel_msg.linear.x = normal_linear_vel;
    vel_msg.angular.z = 0.0;
    while(global_min_distance > OBSTACLE_MIN_DIST)
    {
        velocity_publisher.publish(vel_msg);
        ros::spinOnce();
    }


    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = obstacle_angular_vel;
    while(global_min_distance < OBSTACLE_MAX_DIST)
    {
        velocity_publisher.publish(vel_msg);
        ros::spinOnce();
    }
    
}
