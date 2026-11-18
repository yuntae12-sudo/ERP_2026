#ifndef GLOBAL_HPP
#define GLOBAL_HPP

#include <ros/ros.h>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <filesystem>
#include <time.h>

#include <yaml-cpp/yaml.h>
#include <Eigen/Eigen>

using namespace std;

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef short int16_t;
typedef float float32_t;
typedef double float64_t;

const int32_t c_EGOVEHICLE = 0;
const int32_t c_DISABLE = 0;
const int32_t c_MORAI = 0;
const int32_t c_ERP = 1;
const int32_t c_ENABLE = 1;

const int32_t c_MAIN = 0;
const int32_t c_PRELIMINARY = 1;
const int32_t c_NARROW = 2;

const int32_t c_ON = 0;
const int32_t c_TEST = 1;
const int32_t c_OFF = 2;

const int32_t c_PUREPURSUIT = 0;
const int32_t c_STANLEY = 1;
const int32_t c_LDSTANLEY = 2;
const int32_t c_LKAS = 3;
const int32_t c_MPC = 4;

/////////////////////////////////////   LiDAR     /////////////////////////////////////
const int32_t c_LIDARMAXSIZE = 20;
const float32_t c_LiDARPOSX = 0.7;
const float32_t c_LIDARPOSY = 0;
/////////////////////////////////////   Camera    /////////////////////////////////////
const int32_t c_CAMERAMAXSIZE = 5;

/////////////////////////////////////   GPSIMU    /////////////////////////////////////
const float64_t c_EarthRadius = 6378137.0;
const float64_t c_EarthFlatRatio = 1.0 / 298.257223563;
const float64_t c_N2 = (2 * c_EarthFlatRatio) - pow(c_EarthFlatRatio, 2);

/////////////////////////////////////   Planning  /////////////////////////////////////
const int32_t c_PATHMAXSIZE = 15000;
const int32_t c_MISSIONMAXSIZE = 20;
const int32_t c_TRAFFICMAXSIZE = 20;
const int32_t c_ITERMAXSIZE = 100;
const int32_t c_MAKEREFSIZE = 4;

/////////////////////////////////////   Control   /////////////////////////////////////
const int32_t c_FindNearLen = 400;
const int32_t c_FindWayLen = 200;
const float32_t c_SlowVelocity = 5.0;

typedef struct _POINT2D
{
    float32_t f32_X;
    float32_t f32_Y;

} Point2D_t;

typedef struct _POINT3D
{
    float32_t f32_X;
    float32_t f32_Y;
    float32_t f32_Z;

} Point3D_t;

typedef struct _WGSRAD
{
    float64_t f64_Latitude;
    float64_t f64_Longitude;
    float64_t f64_Altitude;

} WGSRAD_t;

typedef struct _ECEF
{
    float64_t f64_X;
    float64_t f64_Y;
    float64_t f64_Z;

} ECEF_t;

typedef struct _Path
{
    float32_t arf32_E[c_PATHMAXSIZE];
    float32_t arf32_N[c_PATHMAXSIZE];
    float32_t arf32_X[c_PATHMAXSIZE];
    float32_t arf32_Y[c_PATHMAXSIZE];
    int32_t s32_PathNum;
    int32_t s32_NearIdx;

} Path_t;

typedef struct _Camera
{
    int32_t s32_TrafficNULLCnt;

    int32_t s32_SignIndex;

    // int32_t s32_TrafficNum;
    int8_t s32_TrafficNum;
    int32_t ars32_TrafficClasses[c_CAMERAMAXSIZE];

    float32_t f32_BackLaneSlope;
    float32_t f32_BackLaneIntercept;

} Camera_t;

typedef struct _LiDAR
{
    int32_t s32_ObjectNum;
    int32_t ars32_ObjectNearIdx[c_LIDARMAXSIZE];
    Point2D_t st_XY[c_LIDARMAXSIZE];
    Point2D_t st_EN[c_LIDARMAXSIZE];
    Point2D_t st_ENyMin[c_LIDARMAXSIZE];
    Point2D_t st_ENyMax[c_LIDARMAXSIZE];
    float32_t arf32_Size[c_LIDARMAXSIZE];
    float32_t arf32_Distance[c_LIDARMAXSIZE];
    int32_t ars32_Classes[c_LIDARMAXSIZE];

    float32_t arf32_TunnelSlope[2];
    float32_t arf32_TunnelYIntercept[2];

    float32_t arf32_MiddleLaneSlope;
    float32_t arf32_MiddleLaneYIntercept;

    Point2D_t st_TunnelOutPoint[2];

    int32_t s32_LaneSize;
    float32_t f32_Slope;
    float32_t f32_Angle_deg;
    float32_t f32_Angle_rad;

} LiDAR_t;

typedef struct _Fusion
{
    int32_t s32_ObjectNum;
    Point2D_t st_XY[c_LIDARMAXSIZE];
    Point2D_t st_EN[c_LIDARMAXSIZE];
    float32_t arf32_Size[c_LIDARMAXSIZE];
    float32_t arf32_Distance[c_LIDARMAXSIZE];
    int32_t ars32_Classes[c_LIDARMAXSIZE];

} Fusion_t;

typedef struct _GPSIMU
{
    ECEF_t st_RefECEF;
    WGSRAD_t st_RefWGSRAD;
    WGSRAD_t st_WGSRAD;
    float32_t f32_E;
    float32_t f32_N;
    float32_t f32_U;
    float32_t f32_Heading_rad;
    float32_t f32_Heading_deg;

    float32_t f32_PreE;
    float32_t f32_PreN;
    float32_t f32_PreHeading_rad;
    float32_t f32_RefHeading_rad;

} GPSIMU_t;

typedef struct _Vehicle
{
    uint8_t u8_MorA;
    uint8_t u8_EStop;
    uint8_t u8_Gear;
    float64_t f64_Velocity_KPH;
    float64_t f64_Velocity_MPS;
    float64_t f64_Steer;
    int16_t s16_Brake;
    uint32_t u32_Encoder;
    uint8_t u8_Alive;

} Vehicle_t;

typedef struct _Timer
{
    std::chrono::steady_clock::time_point st_Start;
    std::chrono::steady_clock::time_point st_End;

    time_t KeepRefTimer = 0;
    time_t KeepLocalTimer = 0;

    time_t StayRefTimer = 0;
    time_t StayLocalTimer = 0;

} Timer_t;

typedef struct _IdxSet
{
    std::pair<int32_t, int32_t> arp_Idx[c_MISSIONMAXSIZE];
    std::string ars_Names[c_MISSIONMAXSIZE];
    std::string s_SelectName;
    int32_t s32_SelectNumer;
    int32_t s32_Num;
} IdxSet_t;

typedef struct _AvoidParameter
{
    float32_t f32_Gap = 0.1;
    int32_t s32_Iter = 40;
    float32_t f32_LeftThres;
    float32_t f32_RightThres;
    float32_t f32_FirstDist;
    float32_t f32_LastDist;
    float32_t f32_Thres;
    float32_t arf32_Costs[3] = {0};

} Avoid_t;

typedef struct _DeliveryParameter
{
    float32_t f32_Gap = 0.1;
    int32_t s32_Iter = 50;
    float32_t f32_LeftThres;
    float32_t f32_RightThres;
    float32_t f32_Thres;
    float32_t f32_FirstLastDist;
    float32_t arf32_Costs[3] = {0};

} Delivery_t;

typedef struct _UturnParameter
{
    float32_t f32_Gap = 2;
    int32_t s32_Iter = 40;
    float32_t f32_AngleThreshold_deg;
    int32_t s32_StartIdxMargin;
    int32_t s32_IdxMargin;
    int32_t s32_CodeSelect;

} Uturn_t;

typedef struct _TunnelParameter
{
    float32_t f32_InclineDiff = 0.05;
    float32_t f32_WayPointDist = 0.1;
    float32_t f32_WallWidth;
    float32_t f32_RoadWidth;
    float32_t f32_RoadCenter;
    float32_t f32_AvoidOffset;
    float32_t f32_AvoidMargin;

} Tunnel_t;

typedef struct _Planning
{
    Path_t st_GlobalPath;
    Path_t st_LocalPath;
    Path_t st_RefPath;
    Path_t st_InBoundary1;
    Path_t st_InBoundary2;
    Path_t st_InBoundary3;
    Path_t st_InBoundary4;
    Path_t st_OutBoundary;
    Timer_t st_Timer;

    Avoid_t st_AvoidSmall;
    Avoid_t st_AvoidLarge;
    Delivery_t st_Delivery;
    Uturn_t st_Uturn;
    Tunnel_t st_Tunnel;

    IdxSet_t st_Mission;
    IdxSet_t st_Traffic;
    IdxSet_t st_Straight;
    string s_PubMissionName;

    std::string s_PathType;
    int32_t s32_GlobalNearIdx = 6200;
    int32_t s32_LocalNearIdx;

    // Signal
    bool b_StaySignal = false;
    bool b_SlowSignal = false;
    bool b_EStopSignal = false;
    bool b_TunnelOutSignal = false;
    int32_t s32_GearSignal = 0;

    // Traffic
    int32_t s32_TrafficSignal;
    bool b_TrafficState = false;
    bool b_KeepSignal = false;
    bool b_TrafficRightNum[3] = {0};
    bool b_RightStaySignal = false;

    // MORAI Traffic & Delivery
    int16_t s16_TrafficLightStatus;
    int32_t s32_DeliveryMode;

    Point2D_t st_PreAvoidDirection[100] = {0};
    float32_t f32_PreAvoidDist[100] = {0};

    // RefYaw
    int32_t ars32_MakePathStart[c_MAKEREFSIZE];
    int32_t ars32_MakePathEnd[c_MAKEREFSIZE];
    float32_t arf32_RefYaw[c_MAKEREFSIZE];
    float32_t arf32_LaneOffSet[c_MAKEREFSIZE];
    bool b_RefPathIdx = false;
    bool b_MakeRefPathFirst = true;
    int32_t s32_RefPathStartIdx = 0;
    int32_t s32_RefPathEndIdx = 500;

} Planning_t;

typedef struct _ControlParam
{
    float32_t f32_WB = 1.04;
    float32_t f32_GPS_2_Front = 0.74;
    float32_t f32_GPS_2_Rear = 0.2;
    float32_t f32_LiDAR_2_Rear = -1.4;

} ControlParam_t;

typedef struct _PurePursuit
{
    float32_t f32_LD;
    float32_t f32_Alpha;
    int32_t s32_WayIdx;

} PurePursuit_t;

typedef struct _Stanley
{
    float32_t f32_K;
    float32_t f32_Error;
    float32_t f32_speed;

    int32_t s32_FrontNearIdx;
    float32_t f32_ReferenceYaw;

} Stanley_t;

typedef struct _PID
{
    float32_t f32_KP;
    float32_t f32_KI;
    float32_t f32_KD;

    float32_t f32_Error = 0;
    float32_t f32_PreError = 0;
    float32_t f32_I_Error = 0;

    float32_t f32_InputP = 0;
    float32_t f32_InputI = 0;
    float32_t f32_InputD = 0;

} PID_t;

typedef struct _Control
{
    // Control Setup
    int32_t s32_CanMode;
    int32_t s32_ControlMethod;
    int32_t s32_Timeflag = 0;
    Path_t *st_FinalPath;
    Timer_t st_Timer;
    bool temp_flag = false;

    // Target Input
    float32_t f32_TestVelocity_KPH;
    float32_t f32_TestVelocity_MPS;
    float32_t f32_TargetVelocity_KPH;
    float32_t f32_TargetVelocity_MPS;
    float32_t f32_ThrottleInput;
    float32_t f32_TargetBrake;
    float32_t f32_TargetSteer;
    float32_t f32_TestLD;

    float32_t arf32_MissionTargetSpeed[5];
    float32_t arf32_MissionTargetLD[5];

    // Control Method
    PurePursuit_t st_PurePursuit;
    Stanley_t st_Stanley;
    ControlParam_t st_ControlParam;
    PID_t st_PID;

    // Vehicle
    uint8_t u8_EStop;
    uint8_t u8_Gear;

} Control_t;

float64_t Deg2Rad(float64_t &f64_Deg);
float32_t Deg2Rad(float32_t &f32_Deg);
float64_t Rad2Deg(float64_t &f64_Rad);
float32_t Rad2Deg(float32_t &f32_Rad);

float32_t Scale2D(float32_t &f32_X, float32_t &f32_Y);
float32_t Scale2D(float32_t *arf_X, float32_t *arf_Y, int32_t s32_Idx);
float32_t Scale2D(Point2D_t &st_Point2D);

float32_t Scale3D(float32_t &f32_X, float32_t &f32_Y, float32_t &f32_Z);
float32_t Scale3D(float32_t *arf_X, float32_t *arf_Y, float32_t *arf_Z, int32_t s32_Idx);
float32_t Scale3D(Point3D_t &st_Point3D);

float32_t Normalize2D(float32_t &f32_X1, float32_t &f32_Y1, float32_t &f32_X2, float32_t &f32_Y2);
float32_t Normalize2D(float32_t *arf_X, float32_t *arf_Y, int32_t &s32_Idx1, int32_t &s32_Idx2);
float32_t Normalize2D(float32_t *arf_X, float32_t *arf_Y, int32_t s32_Idx1, float32_t &f32_X, float32_t &f32_Y);
float32_t Normalize2D(float32_t *arf_X, float32_t *arf_Y, int32_t s32_Idx1, Point2D_t &st_P2);
float32_t Normalize2D(Point2D_t &st_P1, Point2D_t &st_P2);
float32_t Normalize2D(std::vector<float32_t> &vf_P1, Point2D_t &st_P2);
float32_t Normalize2D(std::vector<float32_t> &vf_P1, float32_t &f32_X, float32_t &f32_Y);
float32_t Normalize2D(std::vector<float32_t> &vf_P1, std::vector<float32_t> &vf_P2);

float32_t Normalize3D(float32_t &f32_X1, float32_t &f32_Y1, float32_t &f32_Z1, float32_t &f32_X2, float32_t &f32_Y2, float32_t &f32_Z2);
float32_t Normalize3D(float32_t *arf_X, float32_t *arf_Y, float32_t *arf_Z, int32_t &s32_Idx1, int32_t &s32_Idx2);
float32_t Normalize3D(Point3D_t &st_P1, Point3D_t &st_P2);

float32_t KPH2MPS(float32_t f32_KPH);
float32_t MPS2KPH(float32_t f32_MPS);

Point2D_t UnitVector(Point2D_t &st_Point2DStart, Point2D_t &st_Point2DEnd);
Point2D_t UnitVector(float32_t f32_XStart, float32_t f32_YStart, float32_t f32_XEnd, float32_t f32_YEnd);

Point2D_t RotatePoint(Point2D_t &st_Point2D, float32_t f32_Degree);
bool CrossProduct(Point2D_t &st_Point2DStart, Point2D_t &st_Point2DEnd);
void GetGapPoints(Planning_t &st_Planning, std::vector<float> &vf_Ref, Point2D_t &st_Direction, int32_t &s32_Iter, float32_t &f32_Gap, std::vector<std::vector<float32_t>> &vf_XY);
void MakePath(Planning_t &st_Planning, int32_t &s32_Start, int32_t &s32_Last, Point2D_t &st_Direction, float32_t &f32_Gap);
void MakePath_LiDAR(Planning_t &st_Planning, int32_t &s32_Start, int32_t &s32_Last, Point2D_t &st_Direction, float32_t &f32_Gap);

void LoadData(std::ifstream &st_LoadFile, Path_t &st_Path);
void LoadIdx(std::ifstream &st_LoadFile, IdxSet_t &st_IdxSet);

void LLA2ENU(GPSIMU_t &st_GPSIMU);
float32_t ENU2NED(float32_t f32_Heading);
void XY2EN_LiDAR(std::vector<std::vector<float32_t>> &vf_LiDARObject, std::vector<std::vector<float32_t>> &vf_LiDARObjectYMinMax, std::vector<int32_t> &vs_LiDARClass, LiDAR_t &st_LiDAR, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning);
void XY2EN_Fusion(std::vector<std::vector<float32_t>> &vf_FusionObject, std::vector<int32_t> &vs_FusionClass, Fusion_t &st_Fusion, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning);
bool InOrOut(Planning_t &st_Planning, float32_t f32_PointX, float32_t f32_PointY);

float32_t pi2pi(float32_t f32_Angle);
float64_t pi2pi(float64_t f64_Angle);

void GetModData(float32_t min, float32_t max, float32_t &data);
void GetModData(float64_t min, float64_t max, float64_t &data);
void GetModData(int32_t min, int32_t max, int32_t &data);

void GetRadius(float32_t f32_X1, float32_t f32_Y1, float32_t f32_X2, float32_t f32_Y2, float32_t f32_X3, float32_t f32_Y3, float32_t &Radius);
void GetRadius(float64_t f32_X1, float64_t f32_Y1, float64_t f32_X2, float64_t f32_Y2, float64_t f32_X3, float64_t f32_Y3, float64_t &Radius);

bool InRange(float32_t min, float32_t max, float32_t data);
bool InRange(float64_t min, float64_t max, float64_t data);
bool InRange(int32_t min, int32_t max, int32_t data);

void OverShootFilter(float32_t f32_Maxtick, float32_t f32_Pre_data, float32_t &f32_Target_data);
void OverShootFilter(float64_t f64_Maxtick, float64_t f64_Pre_data, float64_t &f64_Target_data);

void removeDuplicates(std::vector<std::vector<float>> &vec);
struct VectorCompare
{
    bool operator()(const std::vector<float> &lhs, const std::vector<float> &rhs) const
    {
        if (lhs[0] != rhs[0])
            return lhs[0] < rhs[0];
        return lhs[1] < rhs[1];
    }
};
bool isObtuse(float x0, float y0, float x1, float y1, float x2, float y2);

#endif