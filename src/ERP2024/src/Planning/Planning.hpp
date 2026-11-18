#ifndef PLANNING_HPP
#define PLANNING_HPP

#include <Global/Global.hpp>
#include <Total_msgs/totalInfo.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

extern ros::Publisher st_TotalData;
extern ros::Publisher st_Visualize;

void PlanningProcess(Camera_t &st_Camera, LiDAR_t &st_LiDAR, Fusion_t &st_Fusion, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, Control_t &st_Control, Vehicle_t &st_Vehicle);

int32_t FindNear(Planning_t &st_Planning, float32_t &f32_X, float32_t &f32_Y, int32_t s32_FindNearLen, std::string PathType);
int32_t FindNear(Planning_t &st_Planning, Point2D_t &st_Point2D, int32_t s32_StartIdx, int32_t s32_FindNearLen, std::string PathType);

int32_t FindNear_LiDAR(Planning_t &st_Planning, float32_t &f32_X, float32_t &f32_Y, int32_t s32_FindNearLen, Path_t &st_Path);
int32_t FindNear_LiDAR(Planning_t &st_Planning, float32_t &f32_X, float32_t &f32_Y, int32_t s32_FindNearLen);
int32_t FindNear_LiDAR(Planning_t &st_Planning, Point2D_t &st_Point2D, int32_t s32_FindNearLen);

// MissionName
void Delivery(Fusion_t &st_Fusion, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, Delivery_t &st_Delivery);
void Delivery(LiDAR_t &st_LiDAR, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, Delivery_t &st_Delivery);
void Avoid(LiDAR_t &st_LiDAR, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, Avoid_t &st_AvoidPara);
void Avoid_LiDAR(LiDAR_t &st_LiDAR, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, Avoid_t &st_AvoidPara);
void Parking(LiDAR_t &st_LiDAR, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, Camera_t &st_Camera);
void Uturn(LiDAR_t &st_LiDAR, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, Uturn_t &st_Uturn);
void Uturn_LiDAR(LiDAR_t &st_LiDAR, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, Uturn_t &st_Uturn);
void Uturn_Dis(LiDAR_t &st_LiDAR, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, Uturn_t &st_Uturn);
void Tunnel(LiDAR_t &st_LiDAR, Planning_t &st_Planning, Tunnel_t &st_Tunnel);
void Narrow(LiDAR_t &st_LiDAR, Planning_t &st_Planning);
void MissionSelect(Planning_t &st_Planning);
void TrafficSelect(Planning_t &st_Planning);
void StraightSelect(Planning_t &st_Planning);
void Traffic(Camera_t &st_Camera, Planning_t &st_Planning);

//
void Perpendicular(Planning_t &st_Planning, int32_t *ars32_ObjectNearIdx, int32_t s32_ObjectNum,
                   int32_t &s32_Iter, float32_t &f32_Gap,
                   std::vector<std::vector<std::vector<float32_t>>> &vf_PerpendicularPoints, std::string s_PathType);
void Constraint(Planning_t &Planning, Point2D_t *st_EN, int32_t s32_ObjectNum,
                std::vector<std::vector<std::vector<float32_t>>> &vf_PerpendicularPoints,
                float32_t f32_Thres);
void Constraint(GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, LiDAR_t &st_LiDAR, Point2D_t *st_EN, int32_t *ars32_ObjectNearIdx, int32_t s32_ObjectNum,
                std::vector<bool> b_ObjectDivision, std::vector<std::vector<std::vector<float32_t>>> &vf_PerpendicularPoints,
                float32_t f32_Thres, std::string s_PathType);
void CostFunction(GPSIMU_t &st_GPSIMU, Planning_t &st_Planning,
                  int32_t *ars32_ObjectNearIdx, int32_t s32_ObjectNum, Point2D_t *st_EN,
                  std::vector<std::vector<std::vector<float32_t>>> &vf_PerpendicularPoints,
                  Point2D_t *st_OptimalPoints, bool &b_SlowSignal,
                  float32_t *arf_Costs);
void SelectFirstLastPoint(Planning_t &st_Planning, int32_t *ars32_ObjectNearIdx, int32_t s32_ObjectNum,
                          float32_t &f32_FirstDist, float32_t &f32_LastDist,
                          std::pair<int32_t, int32_t> *ps_ObjectFirstLastIdx);

//UTurn

//Avoid_LiDAR
void MakeRefPath(Planning_t &st_Planning, GPSIMU_t &st_GPSIMU, LiDAR_t &st_LiDAR);
void FindObjectNear_LiDAR(LiDAR_t &st_LiDAR, Planning_t &st_Planning);

void Perpendicular_LiDAR(Planning_t &st_Planning, int32_t *ars32_ObjectNearIdx, int32_t s32_ObjectNum,
                   int32_t &s32_Iter, float32_t &f32_Gap,
                   std::vector<std::vector<std::vector<float32_t>>> &vf_PerpendicularPoints);

void Constraint_LiDAR(Planning_t &st_Planning, Point2D_t *st_XY, int32_t *ars32_ObjectNearIdx, int32_t s32_ObjectNum,
                     std::vector<std::vector<std::vector<float32_t>>> &vf_PerpendicularPoints, float32_t f32_Thres, float32_t f32_LeftThres, float32_t f32_RightThres);
void CostFunction_LiDAR(GPSIMU_t &st_GPSIMU, Planning_t &st_Planning,
                  int32_t *ars32_ObjectNearIdx, int32_t s32_ObjectNum, Point2D_t *st_XY,
                  std::vector<std::vector<std::vector<float32_t>>> &vf_PerpendicularPoints,
                  Point2D_t *st_OptimalPoints, bool &b_OptimalExist,
                  float32_t *arf_Costs);
void SelectFirstLastPoint_LiDAR(Planning_t &st_Planning,
                          int32_t *ars32_ObjectNearIdx, int32_t s32_ObjectNum, float32_t &f32_FirstDist, float32_t &f32_LastDist,
                          std::pair<int32_t, int32_t> *ps_ObjectFirstLastIdx);
void ObjectLinked_LiDAR(LiDAR_t &st_LiDAR,
                  std::pair<int32_t, int32_t> *ps_ObjectFirstLastIdx, std::vector<std::vector<int32_t>> &vs_LinkedList, int32_t &s32_LinkedObjectNum);
void MakeAvoidPath_LiDAR(LiDAR_t &st_LiDAR, Planning_t &st_Planning,
                   Point2D_t *st_OptimalPoints, std::pair<int32_t, int32_t> *ps_ObjectFirstLastIdx,
                   std::vector<std::vector<int32_t>> &vs_LinkedList, int32_t &s32_LinkedObjectNum);


// Visualize
namespace Color
{
    const std::vector<float32_t> RED = {1.0f, 0.0f, 0.0f, 1.0f};
    const std::vector<float32_t> GREEN = {0.0f, 1.0f, 0.0f, 1.0f};
    const std::vector<float32_t> BLUE = {0.0f, 0.0f, 1.0f, 1.0f};
    const std::vector<float32_t> YELLOW = {1.0f, 1.0f, 0.0f, 1.0f};
    const std::vector<float32_t> ORANGE = {1.0f, 0.5f, 0.0f, 1.0f};
    const std::vector<float32_t> WHITE = {1.0f, 1.0f, 1.0f, 1.0f};
    const std::vector<float32_t> GRAY = {0.3f, 0.3f, 0.3f, 1.0f};
}

void VisualizeLine(std::vector<float32_t> vf32_StartPoint,
                   std::vector<float32_t> vf32_EndPoint,
                   std::vector<float32_t> vf32_RGBA,
                   float32_t f32_Scale);

void VisualizePoint(std::vector<float32_t> vf32_Point,
                    std::vector<float32_t> vf32_RGBA,
                    float32_t f32_Scale);

void VisualizeList(std::vector<std::vector<float32_t>> vf32_List,
                   std::vector<float32_t> vf32_RGBA,
                   float32_t f32_Scale);

void VisualizeLocalPath(std::vector<float32_t> vf32_RGBA,
                        float32_t f32_Scale,
                        Planning_t &st_Planning);

void VisualizeLocalPath(std::vector<float32_t> vf32_RGBA,
                        float32_t f32_Scale,
                        Planning_t &st_Planning,
                        GPSIMU_t &st_GPSIMU);

void VisualizeLocalPath_Avoid(std::vector<float32_t> vf32_RGBA,
                        float32_t f32_Scale,
                        Planning_t &st_Planning);

void VisualizeRefPath(std::vector<float32_t> vf32_RGBA,
                        float32_t f32_Scale,
                        Planning_t &st_Planning,
                        GPSIMU_t &st_GPSIMU);

void Visualize();
#endif