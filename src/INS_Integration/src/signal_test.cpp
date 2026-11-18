#include "ros/ros.h"
#include <iostream>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
using namespace std;

ros::Publisher pub1, pub2;

sensor_msgs::NavSatFix gps1;
sensor_msgs::Imu imu1;

double cnt = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sig_test");
    ros::NodeHandle nh;
    pub1 = nh.advertise<sensor_msgs::NavSatFix>("/fix", 1);
    pub2 = nh.advertise<sensor_msgs::Imu>("vectornav/IMU", 1);
    while (1)
    {
        if (cnt > 99999999)
        {
            pub1.publish(gps1);
            pub2.publish(imu1);
            cnt = 0;
        }
        cnt++;
    }
}