#ifndef PARSER_HPP
#define PARSER_HPP

#include <Global/Global.hpp>
#include <Control/Control.hpp>
#include <Planning/Planning.hpp>

// Sensro msgs
#include <Total_msgs/signalInfo.h>
#include <camera/signalInfo.h>
#include <camera/TrafficLight_Arr.h>
#include <camera/LaneInfo.h>
#include <Total_msgs/GNSSInfo.h>
#include <Total_msgs/objsInfo.h>
#include <Total_msgs/fussInfo.h>
#include <camera/fussInfo.h>
#include <Total_msgs/totalInfo.h>
// ERP msgs
#include <Total_msgs/SerialFeedBack.h>
// Morai msgs
#include <Total_msgs/GPSMessage.h>
#include <Total_msgs/EgoVehicleStatus.h>
#include <Total_msgs/GetTrafficLightStatus.h>

// temp (will be integrated within objsInfo)
#include <Total_msgs/lane_arr.h>
#include <Total_msgs/tunneloutpoint_arr.h>

/////////////////////////////////////   ROS SUBSCRIBER   /////////////////////////////////////
ros::Subscriber st_SubcriberCamera;
ros::Subscriber st_SubcriberLiDAR;
ros::Subscriber st_SubcriberFusion;
ros::Subscriber st_SubcriberGPSIMU;
ros::Subscriber st_SubcriberVehicle;
ros::Subscriber st_SubcriberTraffic;

ros::Subscriber st_SubcriberBackLane;

// temp (will be integrated within objsInfo)
ros::Subscriber st_SubcriberTunnelOutPoint;
ros::Subscriber st_SubcriberLane;

void CameraParser(ros::NodeHandle &st_NodeHandle, int &s32_CameraMode);
void LiDARParser(ros::NodeHandle &st_NodeHandle, int &s32_LiDARMode);
void FusionParser(ros::NodeHandle &st_NodeHandle, int &s32_FusionMode);
void GPSIMUParser(ros::NodeHandle &st_NodeHandle, int &s32_GPSIMUMode);
void VehicleParser(ros::NodeHandle &st_NodeHandle, int &s32_VehicleMode);
void PrintData(Camera_t &st_Camera, LiDAR_t &st_LiDAR, Fusion_t &st_Fusion, GPSIMU_t &st_GPSIMU,
               Control_t &st_Control, Vehicle_t &st_Vehicle, Planning_t &st_Planning);

#endif