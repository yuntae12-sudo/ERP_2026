#include <Planning/Planning.hpp>
extern int32_t s32_TrackingMap, s32_MissionMode, s32_TrafficMode, s32_StraightMode;
std::string s_InitMissionName = "", s_FinalMissionName = "";
int32_t s32_StartIdx = 0, s32_EndIdx = 0;
bool StraightSignalFlag = false;
bool MakeRefPathFirst = true;
bool JammingEnd = false;

int32_t FindNear(Planning_t &st_Planning, float32_t &f32_X, float32_t &f32_Y, int32_t s32_FindNearLen, std::string s_PathType)
{
    int32_t s32_NearIdx = 0;
    Path_t st_Path;
    float32_t arf_Dist[s32_FindNearLen] = {0};
    if (s_PathType == "Global")
    {
        st_Path = st_Planning.st_GlobalPath;
        s32_NearIdx = st_Planning.s32_GlobalNearIdx;
    }
    else if (s_PathType == "Local")
    {
        st_Path = st_Planning.st_LocalPath;
    }

    for (int32_t s32_i = s32_NearIdx; s32_i < s32_NearIdx + s32_FindNearLen; s32_i++)
    {
        arf_Dist[s32_i - s32_NearIdx] = Normalize2D(st_Path.arf32_E, st_Path.arf32_N, s32_i % st_Path.s32_PathNum, f32_X, f32_Y);
    }

    return s32_NearIdx = (s32_NearIdx + std::distance(arf_Dist, std::min_element(arf_Dist, arf_Dist + s32_FindNearLen))) % st_Path.s32_PathNum;
}
int32_t FindNear(Planning_t &st_Planning, Point2D_t &st_Point2D, int32_t s32_StartIdx, int32_t s32_FindNearLen, std::string s_PathType)
{
    int32_t s32_NearIdx = s32_StartIdx;
    Path_t st_Path;
    float32_t arf_Dist[s32_FindNearLen] = {0};
    if (s_PathType == "Global")
    {
        st_Path = st_Planning.st_GlobalPath;
    }
    else if (s_PathType == "Local")
    {
        st_Path = st_Planning.st_LocalPath;
    }

    for (int32_t s32_i = 0; s32_i < s32_FindNearLen; s32_i++)
    {
        arf_Dist[s32_i] = Normalize2D(st_Path.arf32_E, st_Path.arf32_N, (s32_i + s32_NearIdx) % st_Path.s32_PathNum, st_Point2D);
    }

    return s32_NearIdx = (s32_NearIdx + std::distance(arf_Dist, std::min_element(arf_Dist, arf_Dist + s32_FindNearLen))) % st_Path.s32_PathNum;
}

int32_t FindNear_LiDAR(Planning_t &st_Planning, float32_t &f32_X, float32_t &f32_Y, int32_t s32_FindNearLen)
{
    int32_t s32_NearIdx = 0;
    Path_t st_Path;
    float32_t arf_Dist[s32_FindNearLen] = {0};
    st_Path = st_Planning.st_RefPath;

    for (int32_t s32_i = s32_NearIdx; s32_i < s32_NearIdx + s32_FindNearLen; s32_i++)
    {
        arf_Dist[s32_i - s32_NearIdx] = Normalize2D(st_Path.arf32_X, st_Path.arf32_Y, s32_i % st_Path.s32_PathNum, f32_X, f32_Y);
    }

    // return s32_NearIdx = (s32_NearIdx + std::distance(arf_Dist, std::min_element(arf_Dist, arf_Dist + s32_FindNearLen))) % st_Path.s32_PathNum;
    return s32_NearIdx = std::distance(arf_Dist, std::min_element(arf_Dist, arf_Dist + s32_FindNearLen)) % st_Path.s32_PathNum;
}

int32_t FindNear_LiDAR(Planning_t &st_Planning, float32_t &f32_X, float32_t &f32_Y, int32_t s32_FindNearLen, Path_t &st_Path)
{
    int32_t s32_NearIdx = 0;
    float32_t arf_Dist[s32_FindNearLen] = {0};

    for (int32_t s32_i = s32_NearIdx; s32_i < s32_NearIdx + s32_FindNearLen; s32_i++)
    {
        arf_Dist[s32_i - s32_NearIdx] = Normalize2D(st_Path.arf32_E, st_Path.arf32_N, s32_i % st_Path.s32_PathNum, f32_X, f32_Y);
    }

    // return s32_NearIdx = (s32_NearIdx + std::distance(arf_Dist, std::min_element(arf_Dist, arf_Dist + s32_FindNearLen))) % st_Path.s32_PathNum;
    return s32_NearIdx = std::distance(arf_Dist, std::min_element(arf_Dist, arf_Dist + s32_FindNearLen)) % st_Path.s32_PathNum;
}

int32_t FindNear_LiDAR(Planning_t &st_Planning, Point2D_t &st_Point2D, int32_t s32_FindNearLen)
{
    int32_t s32_NearIdx = 0;
    Path_t st_Path;
    float32_t arf_Dist[s32_FindNearLen] = {0};
    st_Path = st_Planning.st_RefPath;

    for (int32_t s32_i = s32_NearIdx; s32_i < s32_NearIdx + s32_FindNearLen; s32_i++)
    {
        arf_Dist[s32_i - s32_NearIdx] = Normalize2D(st_Path.arf32_X, st_Path.arf32_Y, s32_i % st_Path.s32_PathNum, st_Point2D);
    }

    // return s32_NearIdx = (s32_NearIdx + std::distance(arf_Dist, std::min_element(arf_Dist, arf_Dist + s32_FindNearLen))) % st_Path.s32_PathNum;
    return s32_NearIdx = std::distance(arf_Dist, std::min_element(arf_Dist, arf_Dist + s32_FindNearLen)) % st_Path.s32_PathNum;
}

void PubTotalCom(Planning_t &st_Planning, Vehicle_t &st_Vehicle)
{
    Total_msgs::totalInfo msg;
    // cout << "st_Planning.s_PubMissionName: " << st_Planning.s_PubMissionName << endl;
    if (st_Planning.s_PubMissionName == "AVOID_SMALL")
    {
        msg.State = 1;
    }
    else if (st_Planning.s_PubMissionName == "DELIVERY_A" || st_Planning.s_PubMissionName == "DELIVERY_B")
    {
        msg.State = 2;
    }
    else if (st_Planning.s_PubMissionName == "AVOID_LARGE")
    {
        msg.State = 3;
    }
    else if (st_Planning.s_PubMissionName == "TUNNEL")
    {
        msg.State = 4;
    }
    else if (st_Planning.s_PubMissionName == "NARROW")
    {
        msg.State = 8;
    }
    else if (st_Planning.s_PubMissionName == "U_TURN")
    {
        msg.State = 9;
    }
    else if (st_Planning.s_PubMissionName == "PARKING")
    {
        msg.State = -1;
    }
    else
    {
        msg.State = 0;
    }
    // cout << "msg.State: " << msg.State << endl;
    // TODO : Add data
    // msg.State = st_Planning.s32_MissionIdxNumer;
    msg.isStop = false;
    msg.Speed = st_Vehicle.f64_Velocity_KPH;
    msg.Yaw = 0;
    msg.Straight = StraightSignalFlag;

    st_TotalData.publish(msg);
}

void MissionLD(Planning_t &st_Planning, Control_t &st_Control)
{
    if (s32_MissionMode == c_ON)
    {
        if(st_Planning.s_PubMissionName != "")
        {
            st_Control.st_PurePursuit.f32_LD = st_Control.arf32_MissionTargetLD[st_Planning.st_Mission.s32_SelectNumer];

        }
        else if (st_Planning.s_PubMissionName == "")
        {
            st_Control.st_PurePursuit.f32_LD = st_Control.f32_TestLD;
        }
    }
    else if (s32_MissionMode == c_TEST)
    {
        st_Control.st_PurePursuit.f32_LD = st_Control.f32_TestLD;
    }
    else
    {
        st_Control.st_PurePursuit.f32_LD = st_Control.f32_TestLD;
    }

    // if (st_Planning.s_PubMissionName == "AVOID_SMALL")
    // {
    //     st_Control.st_PurePursuit.f32_LD = 2.5;
    // }
    // else if (st_Planning.s_PubMissionName == "DELIVERY_A" || st_Planning.s_PubMissionName == "DELIVERY_B")
    // {
    //     st_Control.st_PurePursuit.f32_LD = 3.0;
    // }
    // else if (st_Planning.s_PubMissionName == "AVOID_LARGE")
    // {
    //     st_Control.st_PurePursuit.f32_LD = 3.0;
    // }
    // else if (st_Planning.s_PubMissionName == "TUNNEL")
    // {
    //     st_Control.st_PurePursuit.f32_LD = 3.5;
    // }
    // else if (st_Planning.s_PubMissionName == "NARROW")
    // {
    //     st_Control.st_PurePursuit.f32_LD = 3.0;
    // }
    // else if (st_Planning.s_PubMissionName == "U_TURN")
    // {
    //     st_Control.st_PurePursuit.f32_LD = 2.5;
    // }
    // else if (st_Planning.s_PubMissionName == "PARKING")
    // {
    //     st_Control.st_PurePursuit.f32_LD = 3.0;
    // }
    // else
    // {
    //     st_Control.st_PurePursuit.f32_LD = 5.5;
    // }
}

void MissionSelect(Planning_t &st_Planning)
{
    s_InitMissionName = "";

    for (int32_t s32_i = 0; s32_i < st_Planning.st_Mission.s32_Num; s32_i++)
    {
        if (InRange(st_Planning.st_Mission.arp_Idx[s32_i].first, st_Planning.st_Mission.arp_Idx[s32_i].second,
                    st_Planning.s32_GlobalNearIdx))
        {
            s_InitMissionName = st_Planning.st_Mission.ars_Names[s32_i];
            st_Planning.st_Mission.s32_SelectNumer = s32_i;
            break;
        }
    }
    if (s_InitMissionName != "")
    {
        st_Planning.st_Mission.s_SelectName = s_InitMissionName;
        st_Planning.s_PubMissionName = s_InitMissionName;
    }
    else if (s_InitMissionName == "")
    {
        st_Planning.st_Mission.s_SelectName = "TRACKING";
        st_Planning.s_PubMissionName = s_InitMissionName;
    }
}

void TrafficSelect(Planning_t &st_Planning)
{
    std::string s_TrafficName = "";
    for (int32_t s32_i = 0; s32_i < st_Planning.st_Traffic.s32_Num; s32_i++)
    {
        if (InRange(st_Planning.st_Traffic.arp_Idx[s32_i].first, st_Planning.st_Traffic.arp_Idx[s32_i].second, st_Planning.s32_GlobalNearIdx))
        {
            s_TrafficName = st_Planning.st_Traffic.ars_Names[s32_i];
            st_Planning.st_Traffic.s32_SelectNumer = s32_i;
            break;
        }
    }
    st_Planning.st_Traffic.s_SelectName = s_TrafficName;
    std::cout << "st_Planning.st_Traffic.s_SelectName : " << st_Planning.st_Traffic.s_SelectName << std::endl;
}

void StraightSelect(Planning_t &st_Planning)
{
    bool inRange = false;
    StraightSignalFlag = false;
    for (int32_t s32_i = 0; s32_i < st_Planning.st_Straight.s32_Num; s32_i++)
    {
        if (st_Planning.st_Straight.ars_Names[s32_i] == "STRAIGHT" &&
            InRange(st_Planning.st_Straight.arp_Idx[s32_i].first, st_Planning.st_Straight.arp_Idx[s32_i].second, st_Planning.s32_GlobalNearIdx))
        {
            StraightSignalFlag = true;
            // inRange = true;
            // if (!StraightSignalFlag)
            // {
            //     msg.Straight = true;
            //     // cout << "!!!!!!!!!!!!!!!!!!!Publish Straight Signal" << endl;
            //     StraightSignalFlag = true;
            // }
            break;
        }
    }
    cout << "StraightSignalFlag: " << StraightSignalFlag << endl;

    // if (!inRange)
    // {
    //     StraightSignalFlag = false;
    // }

    // cout << "StraightSignalFlag : " << StraightSignalFlag << endl;
    // cout << "msg.Straight : " << msg.Straight << endl;

    // std::string s_StraightName = "";
    // for (int32_t s32_i = 0; s32_i < st_Planning.st_Straight.s32_Num; s32_i++)
    // {
    //     if (InRange(st_Planning.st_Straight.arp_Idx[s32_i].first, st_Planning.st_Straight.arp_Idx[s32_i].second, st_Planning.s32_GlobalNearIdx))
    //     {
    //         s_StraightName = st_Planning.st_Straight.ars_Names[s32_i];
    //         st_Planning.st_Straight.s32_SelectNumer = s32_i;
    //         break;
    //     }
    // }
    // st_Planning.st_Straight.s_SelectName = s_StraightName;
    // std::cout << "st_Planning.st_Straight.s_SelectName : " << st_Planning.st_Straight.s_SelectName << std::endl;
    // st_TotalData.publish(msg);
}

void Traffic(Camera_t &st_Camera, Planning_t &st_Planning)
{
    Timer_t &st_Timer = st_Planning.st_Timer;
    int32_t s32_I = 0;

    st_Planning.b_TrafficState = false;

    if (st_Planning.st_Traffic.s_SelectName == "TRAFFIC_STOP")
    {
        if (!(st_Camera.ars32_TrafficClasses[0] == 0 || st_Camera.ars32_TrafficClasses[0] == 1))
        {
            st_Planning.b_TrafficState = true;
            st_Planning.b_KeepSignal = true;
        }
    }
    else if (st_Planning.st_Traffic.s_SelectName == "TRAFFIC_LEFT")
    {
        if (!(st_Camera.ars32_TrafficClasses[0] == 3 || st_Camera.ars32_TrafficClasses[0] == 1))
        {
            st_Planning.b_TrafficState = true;
            st_Planning.b_KeepSignal = true;
        }
    }
    else if (st_Planning.st_Traffic.s_SelectName == "TRAFFIC_RIGHT1")
    {
        if (st_Planning.b_TrafficRightNum[0] == false)
        {
            st_Planning.b_RightStaySignal = true;
        }
    }
    else if (st_Planning.st_Traffic.s_SelectName == "TRAFFIC_RIGHT2")
    {
        if (st_Planning.b_TrafficRightNum[1] == false)
        {
            st_Planning.b_RightStaySignal = true;
        }
    }
    else if (st_Planning.st_Traffic.s_SelectName == "TRAFFIC_RIGHT3")
    {
        if (st_Planning.b_TrafficRightNum[2] == false)
        {
            st_Planning.b_RightStaySignal = true;
        }
    }
    else
    {
        // std::cout << "ERROR-------------Unknown Traffic Name" << std::endl;
    }

    // straight && turn left
    if (st_Planning.b_KeepSignal == true && st_Planning.b_TrafficState == true)
    {
        st_Timer.KeepRefTimer = time(NULL);
    }
    if (st_Timer.KeepRefTimer != 0)
    {
        st_Timer.KeepLocalTimer = time(NULL);
        if (st_Timer.KeepLocalTimer - st_Timer.KeepRefTimer > 3)
        {
            st_Planning.b_KeepSignal = false;
            st_Timer.KeepRefTimer = 0;
            st_Timer.KeepLocalTimer = 0;
        }
    }

    // turn right
    if (st_Planning.b_RightStaySignal == true && st_Timer.StayRefTimer == 0)
    {
        st_Timer.StayRefTimer = time(NULL);
    }
    if (st_Timer.StayRefTimer != 0)
    {
        st_Timer.StayLocalTimer = time(NULL);
        if (st_Timer.StayLocalTimer - st_Timer.StayRefTimer > 5)
        {
            st_Planning.b_RightStaySignal = false;
            st_Timer.StayRefTimer = 0;
            st_Timer.StayLocalTimer = 0;
            if (st_Planning.st_Traffic.s_SelectName == "TRAFFIC_RIGHT1")
            {
                st_Planning.b_TrafficRightNum[0] = true;
            }
            else if (st_Planning.st_Traffic.s_SelectName == "TRAFFIC_RIGHT2")
            {
                st_Planning.b_TrafficRightNum[1] = true;
            }
            else if (st_Planning.st_Traffic.s_SelectName == "TRAFFIC_RIGHT3")
            {
                st_Planning.b_TrafficRightNum[2] = true;
            }
        }
    }
}

int cnt = 0;
void PlanningProcess(Camera_t &st_Camera, LiDAR_t &st_LiDAR, Fusion_t &st_Fusion, GPSIMU_t &st_GPSIMU, Planning_t &st_Planning, Control_t &st_Control, Vehicle_t &st_Vehicle)
{
    if (s32_TrackingMap != c_NARROW && s32_MissionMode != c_OFF)
    {
        st_Planning.s32_GlobalNearIdx = FindNear(st_Planning, st_GPSIMU.f32_E, st_GPSIMU.f32_N, c_FindNearLen, "Global");
        MissionSelect(st_Planning);
        if (s32_TrafficMode != c_OFF)
        {
            TrafficSelect(st_Planning);
            Traffic(st_Camera, st_Planning);
        }
        if (s32_StraightMode != c_OFF)
        {
            StraightSelect(st_Planning);
        }
        // cout << "st_Planning.st_Mission.s_SelectName: " << st_Planning.st_Mission.s_SelectName << endl;
        // MakeRefPath(st_Planning, st_GPSIMU, st_LiDAR);
        // VisualizeRefPath(Color::GREEN, 0.5f, st_Planning, st_GPSIMU);
        if (st_Planning.st_Mission.s_SelectName == "DELIVERY_A")
        {
            st_Planning.s_PathType = "Local";
            if (st_Planning.s32_DeliveryMode == 1)
            {
                Delivery(st_Fusion, st_GPSIMU, st_Planning, st_Planning.st_Delivery);
            }
            else
            {
                Delivery(st_LiDAR, st_GPSIMU, st_Planning, st_Planning.st_Delivery);
            }
        }
        else if (st_Planning.st_Mission.s_SelectName == "AVOID_SMALL")
        {
            st_Planning.s_PathType = "Local";
            // Avoid(st_LiDAR, st_GPSIMU, st_Planning, st_Planning.st_AvoidSmall);
            Avoid_LiDAR(st_LiDAR, st_GPSIMU, st_Planning, st_Planning.st_AvoidSmall);
        }
        else if (st_Planning.st_Mission.s_SelectName == "AVOID_LARGE")
        {
            st_Planning.s_PathType = "Local";
            // Avoid(st_LiDAR, st_GPSIMU, st_Planning, st_Planning.st_AvoidLarge);
            Avoid_LiDAR(st_LiDAR, st_GPSIMU, st_Planning, st_Planning.st_AvoidLarge);
        }
        else if (st_Planning.st_Mission.s_SelectName == "DELIVERY_B")
        {
            st_Planning.s_PathType = "Local";
            if (st_Planning.s32_DeliveryMode == 1)
            {
                Delivery(st_Fusion, st_GPSIMU, st_Planning, st_Planning.st_Delivery);
            }
            else
            {
                Delivery(st_LiDAR, st_GPSIMU, st_Planning, st_Planning.st_Delivery);
            }
        }
        else if (st_Planning.st_Mission.s_SelectName == "PARKING")
        {
            st_Planning.s_PathType = "Local";
            Parking(st_LiDAR, st_GPSIMU, st_Planning, st_Camera);
        }
        else if (st_Planning.st_Mission.s_SelectName == "U_TURN")
        {
            st_Planning.s_PathType = "Local";
            if(st_Planning.st_Uturn.s32_CodeSelect == 0)
            {
                Uturn(st_LiDAR, st_GPSIMU, st_Planning, st_Planning.st_Uturn);
            }
            else if(st_Planning.st_Uturn.s32_CodeSelect == 1)
            {
                Uturn_Dis(st_LiDAR, st_GPSIMU, st_Planning, st_Planning.st_Uturn);
            }
            else if(st_Planning.st_Uturn.s32_CodeSelect == 2)
            {
                Uturn_LiDAR(st_LiDAR, st_GPSIMU, st_Planning, st_Planning.st_Uturn);
            }
        }
        else if (st_Planning.st_Mission.s_SelectName == "TUNNEL")
        {
            if (st_Planning.b_TunnelOutSignal || JammingEnd)
            {
                st_Planning.st_Mission.s_SelectName = "TRACKING";
                st_Planning.s_PubMissionName = st_Planning.st_Mission.s_SelectName;
                StraightSignalFlag = false;
                JammingEnd = true;
                st_Planning.b_SlowSignal = false;
                st_Control.st_PurePursuit.f32_LD = 4.5;
            }
            else 
            {
                st_Planning.s32_GlobalNearIdx = 10100;
                st_Planning.s_PathType = "Local";
                Tunnel(st_LiDAR, st_Planning, st_Planning.st_Tunnel);
            }

            std::cout << "st_Planning.s32_GlobalNearIdx : " << st_Planning.s32_GlobalNearIdx << std::endl;
        }
        else if (st_Planning.st_Mission.s_SelectName != "TRACKING")
        {
            std::cout << "Unknown mission name : " << st_Planning.st_Mission.s_SelectName << std::endl;
        }

        if (st_Planning.st_Mission.s_SelectName == "TRACKING")
        {
            st_Planning.s_PathType = "Global";
        }
    }
    else if (s32_TrackingMap == c_NARROW)
    {
        st_Planning.s_PathType = "Local";
        st_Planning.st_Mission.s_SelectName = "NARROW";
        Narrow(st_LiDAR, st_Planning);
    }
    else if (s32_MissionMode == c_OFF)
    {
        st_Planning.s_PathType = "Global";
        st_Planning.st_Mission.s_SelectName = "TRACKING";
        st_Planning.s32_GlobalNearIdx = FindNear(st_Planning, st_GPSIMU.f32_E, st_GPSIMU.f32_N, c_FindNearLen, "Global");
    }

    s_FinalMissionName = st_Planning.st_Mission.s_SelectName;
    // 여기서 Path Update 끝내기
    PubTotalCom(st_Planning, st_Vehicle);
    MissionLD(st_Planning, st_Control);
    Visualize();
}