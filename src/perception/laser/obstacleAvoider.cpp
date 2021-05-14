#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <any>
#include <unistd.h>
#include <vector>
#include <map>
#include <algorithm>


#define PI 3.14159265
#define MAX_DIST 1000000.00

ros::Publisher velocity_publisher;
ros::Subscriber laser_scan_subscriber;

const double OBSTACLE_DIST = 0.2;
const double REGIONAL_ANGLE = 30.0;
const double NORMAL_LIN_VEL = 0.5;
const double TRANS_LIN_VEL = -0.05;
const double TRANS_ANG_VEL = 1.75;

std::map<std::string, std::any> Urgency_Report { {"act",0}, {"angular_vel", 0.0}, {"sleep",0.0} };
std::vector<std::string> Regions = {
                                    "front_L", "front_C","front_R",
                                    "left_L", "left_C",  "left_R",
                                    "back_L", "back_C",  "back_R",
                                    "right_L","right_C",  "right_R"
                                    };

std::map<std::string, std::vector<double>> Regions_Report = { {"front_L",std::vector<double>{}}, {"front_C",std::vector<double>{}},{"front_R",std::vector<double>{}},
                                                            {"left_L",std::vector<double>{}}, {"left_C",std::vector<double>{}},{"left_R",std::vector<double>{}},
                                                            {"back_L",std::vector<double>{}}, {"back_C",std::vector<double>{}},{"back_R",std::vector<double>{}},
                                                            {"right_L",std::vector<double>{}}, {"right_C",std::vector<double>{}},{"right_R",std::vector<double>{}}
                                                            };

std::map<std::string, int> Regions_Distances {
                                                {"front_C", 0}, {"front_L", 1}, {"left_R", 2},
                                                {"left_C", 3}, {"left_L", 4}, {"back_R", 5},
                                                {"back_C", 6}, {"back_L", -5}, {"right_R", -4},
                                                {"right_C", -3}, {"right_L", -2}, {"front_R", -1}
                                             };

void scanCallback (sensor_msgs::LaserScan scanMessage);
void laserCallback(sensor_msgs::LaserScan scan_data);
void steer(geometry_msgs::Twist& velocity_msg);
void roamAround();

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_avoider");
    ros::NodeHandle nh;

    velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    // laser_scan_subscriber = nh.subscribe("/scan",10, scanCallback);
    
    geometry_msgs::Twist vel_msg;

    ros::WallRate loop(10);

    while(ros::ok())
    {
        laser_scan_subscriber = nh.subscribe("/scan",10, laserCallback);
        roamAround();
        bool steer_or_move = std::any_cast<int>(Urgency_Report["act"]);
        //std::cout<<"steer or move: "<<steer_or_move<<'\n';
        if(steer_or_move>0)
        {
            //std::cout<<"Hit an obstacle, need to steer away\n";
            steer(vel_msg);
        }
        else
        {
            vel_msg.linear.x = NORMAL_LIN_VEL;
            vel_msg.linear.y = 0;
            vel_msg.linear.z = 0;
            vel_msg.angular.x = 0;
            vel_msg.angular.y = 0;
            vel_msg.angular.z = 0;
            //std::cout<<"velocity "<<vel_msg.linear.x<<std::endl;
        }
        velocity_publisher.publish(vel_msg);

        // after publishing our action, we give it some time to execute the
        // needed actions before reading the data again.
        //sleep(int(std::any_cast<double>(Urgency_Report["sleep"])));
        loop.sleep();

       ros::spinOnce();
    }

    return 0;
}

void laserCallback(sensor_msgs::LaserScan scan_data)
{
    for(int i = 0; i<Regions.size(); i++)
    {
        std::string region = Regions[i];
        for(int j = i*REGIONAL_ANGLE; j<(i+1)*REGIONAL_ANGLE; j++)
        {
            if(!std::isnan(scan_data.ranges[j]) && scan_data.ranges[j]<= OBSTACLE_DIST)
            {
                Regions_Report[region].push_back(double(scan_data.ranges[j]));
            }
        }
    }
}

void steer(geometry_msgs::Twist& velocity)
{
    // since we're moving only on the plane, all we need is move in the x axis,
    // and rotate in the z (zeta) axis.
    velocity.linear.x = TRANS_LIN_VEL;
    velocity.linear.y = 0;
    velocity.linear.z = 0;
    velocity.angular.x = 0;
    velocity.angular.y = 0;
    velocity.angular.z = std::any_cast<double>(Urgency_Report["angular_vel"]);
}

void roamAround()
{
    std::string goal = "front_C";
    double closest = MAX_DIST;
    double regional_dist = 0.0;
    std::map<std::string, std::any> maxima = { {"destination", "back_C"}, {"distance",MAX_DIST }};

    if(Regions_Report.size())
    {
        for(auto& region: Regions_Report)
        {
            regional_dist = abs(Regions_Distances[region.first] - Regions_Distances[goal]);
            if(!region.second.size())
            {
                if(regional_dist < closest)
                {
                    closest = regional_dist;
                    maxima["distance"] = OBSTACLE_DIST;
                    maxima["destination"] = region.first;
                }
            }
            else
            {
                double max_element = *std::max_element(region.second.begin(), region.second.end());
                if(max_element > std::any_cast<double>(maxima["distance"]))
                {
                    maxima["distance"] = max_element;
                    maxima["destination"] = region.first;
                }

            }
        }

        regional_dist = Regions_Distances[std::any_cast<std::string>(maxima["destination"])] - Regions_Distances[goal];

        Urgency_Report["act"] = (closest!=0)?1:0;
        Urgency_Report["angular_velocity"] = double((regional_dist/std::max(1, int(abs(regional_dist))))*TRANS_ANG_VEL);
        std::cout<<"before angular velocity "<<std::any_cast<double>(Urgency_Report["angular_velocity"])<<'\n';
        Urgency_Report["sleep"] = double(((abs(regional_dist)*REGIONAL_ANGLE*PI)/(180.0*TRANS_ANG_VEL)));   
    }
}
