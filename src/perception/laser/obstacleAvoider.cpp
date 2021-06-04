#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"


// declare velocity publisher
ros::Publisher velocity_publisher;

// declare laserScanner subscriber
ros::Subscriber laser_scan_subscriber;

const double PI = 3.14159265359;

// linear velocity in x-axis when no obstacle is present or is far away
const double normal_linear_vel = 0.4;

// angular velocity in z-axis when obstacle is present or is near
const double obstacle_angular_vel = 0.5;

// linear velocity in x-axis when obstacle is present of is near
const double obstacle_linear_vel = -0.07;

// min distance threshold for an obstacle
const double OBSTACLE_MIN_DIST = 0.7;

// distance of robot from the obstacle
double global_min_distance = 99999.99;

// depending on the angle of max obstacle distance rotate clockwise or counter-clockwise
bool rotateClockwise=true;

// declare radian to degree function
double radian2degree(double angle_in_radians);

// declare scan callback function
void scan_callback(sensor_msgs::LaserScan scanMessage);

// declare start moving function
void start_moving();

int main(int argc, char** argv)
{
    // initiate a new ros node named "obstacle_avoider"
    ros::init(argc, argv, "obstacle_avoider");
    
    // create a node Handle
    ros::NodeHandle nh;

    // define publisher and publish to the topic "/cmd_vel"
    velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);   
    
    // define subscriber and subscribe to the topic "/scan"
    laser_scan_subscriber = nh.subscribe("/scan",1000, scan_callback);

    // call the function start_moving()
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
    int num_useful_readings(0);

    for(int i = 0; i < LaserScanMsg.ranges.size() ; i++)
    {
        // check if the laser scan val is not undefined and i(angle) is between 315 and 45 degrees, i.e in front of the robot covering a span of 90 degrees
        if(!std::isnan(LaserScanMsg.ranges[i]) && (i<=start_index || i>= end_index))
        {
            if(LaserScanMsg.ranges[i] < global_min_distance && (LaserScanMsg.ranges[i]>= LaserScanMsg.range_min) && (LaserScanMsg.ranges[i]<= LaserScanMsg.range_max))
            {
                global_min_distance = LaserScanMsg.ranges[i];
                min_index_angle = i;
                num_useful_readings++;
            }
        }
    }
    if(!num_useful_readings)
    {
        global_min_distance = 9999.9;
    } 
    else
    {  
         if(min_index_angle<=45) // if obstacle is found on the right hand side then rotate towards left
        {
            rotateClockwise = false;
        }
        else if(min_index_angle>=315) // if obstacle is found on the left hand side then rotate towards right
        {
            rotateClockwise = true;
        }
    }
}

void start_moving()
{   
    // define frequency of the loop
    ros::WallRate loop(10);
    
    while(true && ros::ok())
    {
        // create a Twist variable, vel_msg to store the velocity values
        geometry_msgs::Twist vel_msg;
        
        // if the distance between robot and obstacle is greater than threshold, then keep moving forward
        if(global_min_distance > OBSTACLE_MIN_DIST)
        {
            // set x component of linear velocity as "normal_linear_vel"
            vel_msg.linear.x = normal_linear_vel;
            
            // set z component of angular velocity as 0
            vel_msg.angular.z = 0.0;
        }
        else
        {
            // stop the robot from moving forward
            // set x component of linear velocity as 0
            vel_msg.linear.x = 0;
            
            // rotate the robot at its current location till the distance between robot and some obstacle is greater than threshold
            // set z component of angular velocity as "obstacle_angular_vel"
            vel_msg.angular.z = obstacle_angular_vel;
        }
        
        // publish the vel_msg
        velocity_publisher.publish(vel_msg);
    
        // allows the ROS to process the incoming messages, callback function
        ros::spinOnce();
        
        // sleep the rest of the cycle, enforces the loop rate
        loop.sleep();
    }
}
