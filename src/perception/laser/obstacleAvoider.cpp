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
double global_min_distance = 99999.99;
bool rotateClockwise=true;

double radian2degree(double angle_in_radians);
void scan_callback(sensor_msgs::LaserScan scanMessage);
void start_moving();

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_avoider");
    ros::NodeHandle nh;

    velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);   
    laser_scan_subscriber = nh.subscribe("/scan",1000, scan_callback);

    start_moving();
}

double radian2degree(double angle_in_radians)
{
    return angle_in_radians * (180.0 / PI);
}

void scan_callback(sensor_msgs::LaserScan LaserScanMsg)
{
    // Since 0 is in the center, we want to scan the area sweeped between 45 degrees and 315 degrees for obstacles
    int start_index = 45;
    int end_index = 315;
    int min_index_angle = -1;

    for(int i = 0; i < LaserScanMsg.ranges.size() ; i++)
    {
        if(!std::isnan(LaserScanMsg.ranges[i]) && (i<=start_index || i>= end_index))
        {
            if(LaserScanMsg.ranges[i] < global_min_distance && (LaserScanMsg.ranges[i]>= LaserScanMsg.range_min) && (LaserScanMsg.ranges[i]<= LaserScanMsg.range_max))
            {
                global_min_distance = LaserScanMsg.ranges[i];
                min_index_angle = i;
            }
        }
    }
    if(min_index_angle != -1)
    {
        global_min_distance = 9999.9;
        if(min_index_angle<=45)
        {
            rotateClockwise = false;
        }
        else if(min_index_angle>=315)
        {
            rotateClockwise = true;
        }
    } 
}

void start_moving()
{   
    ros::WallRate loop(10);
    while(true && ros::ok())
    {
        geometry_msgs::Twist vel_msg;

        if(global_min_distance > OBSTACLE_MIN_DIST)
        {
            vel_msg.linear.x = normal_linear_vel;
            vel_msg.angular.z = 0.0;
        }
        else
        {
            vel_msg.linear.x = obstacle_linear_vel;
            vel_msg.angular.z = obstacle_angular_vel;
        }

        velocity_publisher.publish(vel_msg);

        ros::spinOnce();
        loop.sleep();
    }
}
