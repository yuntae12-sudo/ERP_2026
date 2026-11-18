#include <Parser/Parser.hpp>
extern int32_t s32_PrintMode;
extern std::string s_InitMissionName, s_FinalMissionName;

extern Camera_t st_Camera;
extern LiDAR_t st_LiDAR;
extern Fusion_t st_Fusion;
extern GPSIMU_t st_GPSIMU;
extern Vehicle_t st_Vehicle;
extern Control_t st_Control;
extern Planning_t st_Planning;

extern bool b_HDMap;
extern float32_t f32_LookAheadDistance;

extern float32_t f32_MoraiENUYaw;
float32_t f32_MoraiENUYaw = 0;

extern bool StraightSignalFlag;

float32_t f32_MoraiHeading;

void PrintData(Camera_t &st_Camera, LiDAR_t &st_LiDAR, Fusion_t &st_Fusion, GPSIMU_t &st_GPSIMU,
               Control_t &st_Control, Vehicle_t &st_Vehicle, Planning_t &st_Planning)
{
    if (s32_PrintMode == 1)
    {
        std::string s_InOut = "Out";
        if (InOrOut(st_Planning, st_GPSIMU.f32_E, st_GPSIMU.f32_N))
            s_InOut = "In";

        int32_t s32_NearIdx = 0;
        if (st_Planning.s_PathType == "Global")
        {
            s32_NearIdx = st_Planning.s32_GlobalNearIdx;
        }
        else if (st_Planning.s_PathType == "Local")
        {
            s32_NearIdx = st_Planning.s32_LocalNearIdx;
        }
        std::cout << "-----------------------------------\n"
                  << "E: " << st_GPSIMU.f32_E << "    N: " << st_GPSIMU.f32_N << "    Heading: " << st_GPSIMU.f32_Heading_deg << "\n"
                  << "NearIdx: " << s32_NearIdx << "    WayIdx: " << st_Control.st_PurePursuit.s32_WayIdx << "     Ld : " << f32_LookAheadDistance << "\n"
                  << "InitMissionName: " << s_InitMissionName << "    FinalMissionName: " << s_FinalMissionName << "\n"
                  << "Target Steer: " << Rad2Deg(st_Control.f32_TargetSteer) << "    Target Speed: " << st_Control.f32_ThrottleInput << "\n"
                  << "Vehicle Path: " << s_InOut << "    NearDist: " << Normalize2D(st_GPSIMU.f32_E, st_GPSIMU.f32_N, st_Planning.st_GlobalPath.arf32_E[s32_NearIdx], st_Planning.st_GlobalPath.arf32_N[s32_NearIdx])
                  << "    PathType: " << st_Planning.s_PathType << "\n ";
        //   << "StraightSignalFlag: " << StraightSignalFlag << "\n"
        //   << "st_Vehicle.f64_Steer : " << st_Vehicle.f64_Steer << "\n";
        // for (int32_t s32_i = 0; s32_i < st_LiDAR.s32_ObjectNum; s32_i++)
        // {
        //     std::cout << "Obj [" << s32_i << "] : " << st_LiDAR.ars32_ObjectNearIdx[s32_i] << "  ==>  " << st_LiDAR.st_EN[s32_i].f32_X << "  ,  " << st_LiDAR.st_EN[s32_i].f32_Y << std::endl;
        // }
    }
}

void MORAICamera_rcv(const camera::TrafficLight_Arr::ConstPtr &Camera)
{
    st_Camera.s32_TrafficNum = Camera->ObjectNum;
    if (st_Camera.s32_TrafficNum == 0)
    {
        st_Camera.s32_TrafficNULLCnt++;
        if (st_Camera.s32_TrafficNULLCnt < 90 || st_Camera.s32_TrafficNULLCnt > 300)
        {
            st_Camera.ars32_TrafficClasses[0] = 1;
        }
    }
    else
    {
        for (int32_t s32_i = 0; s32_i < st_Camera.s32_TrafficNum; s32_i++)
        {
            st_Camera.ars32_TrafficClasses[s32_i] = Camera->arst_TrafficLight[s32_i].cls;
        }
        st_Camera.s32_TrafficNULLCnt = 0;
    }
}

void Camera_rcv(const camera::signalInfo::ConstPtr &Camera)
{
    st_Camera.s32_SignIndex = Camera->signIndex;
    cout << "st_Camera.s32_SignIndex: " << st_Camera.s32_SignIndex << endl;
    if (st_Camera.s32_SignIndex == -1)
    {
        st_Camera.s32_TrafficNULLCnt++;
        if (st_Camera.s32_TrafficNULLCnt < 90 || st_Camera.s32_TrafficNULLCnt > 300)
        {
            st_Camera.ars32_TrafficClasses[0] = 1;
        }
    }
    else
    {
        st_Camera.ars32_TrafficClasses[0] = st_Camera.s32_SignIndex;
        st_Camera.s32_TrafficNULLCnt = 0;
    }
}

void LiDAR_rcv(const Total_msgs::objsInfo::ConstPtr &LiDAR)
{
    int32_t s32_ObjectNum = LiDAR->objNum;
    std::vector<std::vector<float32_t>> vf_LiDARObject(s32_ObjectNum, std::vector<float32_t>(4, 0));
    std::vector<std::vector<float32_t>> vf_LiDARObjectYMinMax(s32_ObjectNum, std::vector<float32_t>(2, 0));
    std::vector<int32_t> vf_LiDARClass(s32_ObjectNum, 0);
    if (s32_ObjectNum != 0)
    {
        for (int32_t s32_i = 0; s32_i < s32_ObjectNum; s32_i++)
        {
            vf_LiDARObject[s32_i][0] = LiDAR->data[s32_i].posX;
            vf_LiDARObject[s32_i][1] = LiDAR->data[s32_i].posY;
            vf_LiDARObject[s32_i][2] = LiDAR->data[s32_i].size;
            vf_LiDARObject[s32_i][3] = LiDAR->data[s32_i].distance;
            vf_LiDARClass[s32_i] = LiDAR->data[s32_i].classes;

            vf_LiDARObjectYMinMax[s32_i][0] = LiDAR->data[s32_i].y_min;
            vf_LiDARObjectYMinMax[s32_i][1] = LiDAR->data[s32_i].y_max;
        }
        XY2EN_LiDAR(vf_LiDARObject, vf_LiDARObjectYMinMax, vf_LiDARClass, st_LiDAR, st_GPSIMU, st_Planning);
    }
    else
    {
        st_LiDAR.s32_ObjectNum = 0;
    }

    st_LiDAR.arf32_TunnelYIntercept[0] = 0.0f;
    st_LiDAR.arf32_TunnelYIntercept[1] = 0.0f;
    if (LiDAR->coeff.size() == 2)
    {
        st_LiDAR.arf32_TunnelSlope[0] = LiDAR->coeff[0].Slope;
        st_LiDAR.arf32_TunnelSlope[1] = LiDAR->coeff[1].Slope;
        st_LiDAR.arf32_TunnelYIntercept[0] = LiDAR->coeff[0].Yintercept;
        st_LiDAR.arf32_TunnelYIntercept[1] = LiDAR->coeff[1].Yintercept;
    }

    st_LiDAR.s32_LaneSize = LiDAR->lane.size();
    if (LiDAR->lane.size() != 0)
    {
        st_LiDAR.f32_Slope = LiDAR->lane[0].Slope;
        st_LiDAR.f32_Angle_deg = LiDAR->lane[0].Angle;
        // cout << "st_LiDAR.f32_Slope: " << st_LiDAR.f32_Slope << endl;
    }
}

// temp (will be integrated within objsInfo)
void TunnelOutPoint_rcv(const Total_msgs::tunneloutpoint_arr::ConstPtr &TunnelOutPoint)
{
    st_LiDAR.st_TunnelOutPoint[0].f32_X = -1.0f;
    st_LiDAR.st_TunnelOutPoint[0].f32_Y = -1.0f;
    st_LiDAR.st_TunnelOutPoint[1].f32_X = -1.0f;
    st_LiDAR.st_TunnelOutPoint[1].f32_Y = -1.0f;
    if (TunnelOutPoint->ptNum == 2)
    {
        st_LiDAR.st_TunnelOutPoint[0].f32_X = TunnelOutPoint->top_arr[0].x;
        st_LiDAR.st_TunnelOutPoint[0].f32_Y = TunnelOutPoint->top_arr[0].y;
        st_LiDAR.st_TunnelOutPoint[1].f32_X = TunnelOutPoint->top_arr[1].x;
        st_LiDAR.st_TunnelOutPoint[1].f32_Y = TunnelOutPoint->top_arr[1].y;
    }
}

// temp (will be integrated within objsInfo)
void Lane_rcv(const Total_msgs::lane_arr::ConstPtr &Lane)
{
    st_LiDAR.arf32_MiddleLaneYIntercept = 100.0f;
    if (Lane->lane_data.size() == 1)
    {
        st_LiDAR.arf32_MiddleLaneSlope = Lane->lane_data[0].Slope;
        st_LiDAR.arf32_MiddleLaneYIntercept = Lane->lane_data[0].Yintercept;
    }
}

void BackLane_rcv(const camera::LaneInfo::ConstPtr &BackLane)
{
    st_Camera.f32_BackLaneSlope = BackLane->f32_Slope;
    st_Camera.f32_BackLaneIntercept = BackLane->f32_InterceptY;
}

void Fusion_rcv(const camera::fussInfo::ConstPtr &Fusion)
{
    int32_t s32_ObjectNum = Fusion->objNum;
    std::vector<std::vector<float32_t>> vf_FusionObject(s32_ObjectNum, std::vector<float32_t>(4, 0));
    std::vector<int32_t> vf_FusionClass(s32_ObjectNum, 0);
    if (s32_ObjectNum != 0)
    {
        cout << "s32_ObjectNum: " << s32_ObjectNum << endl;
        for (int32_t s32_i = 0; s32_i < s32_ObjectNum; s32_i++)
        {
            vf_FusionObject[s32_i][0] = Fusion->data[s32_i].posX;
            vf_FusionObject[s32_i][1] = Fusion->data[s32_i].posY;
            vf_FusionObject[s32_i][2] = Fusion->data[s32_i].size;
            vf_FusionObject[s32_i][3] = Fusion->data[s32_i].distance;
            vf_FusionClass[s32_i] = Fusion->data[s32_i].classes;
        }
        XY2EN_Fusion(vf_FusionObject, vf_FusionClass, st_Fusion, st_GPSIMU, st_Planning);
    }
    else
    {
        st_Fusion.s32_ObjectNum = 0;
    }
}

void MORAIGPS_rcv(const Total_msgs::GPSMessage::ConstPtr &GPS)
{
    float64_t f64_Lat = GPS->latitude;
    float64_t f64_Lon = GPS->longitude;
    st_GPSIMU.st_WGSRAD.f64_Latitude = Deg2Rad(f64_Lat);
    st_GPSIMU.st_WGSRAD.f64_Longitude = Deg2Rad(f64_Lon);
    // st_GPSIMU.st_WGSRAD.f64_Altitude = GPS->altitude;
    st_GPSIMU.st_WGSRAD.f64_Altitude = 0;
    st_GPSIMU.f32_Heading_deg = f32_MoraiHeading;
    st_GPSIMU.f32_Heading_rad = Deg2Rad(st_GPSIMU.f32_Heading_deg);
    LLA2ENU(st_GPSIMU);
    PlanningProcess(st_Camera, st_LiDAR, st_Fusion, st_GPSIMU, st_Planning, st_Control, st_Vehicle);
    ControlProcess(st_Camera, st_GPSIMU, st_Control, st_Planning, st_Vehicle);
    PrintData(st_Camera, st_LiDAR, st_Fusion, st_GPSIMU, st_Control, st_Vehicle, st_Planning);
}

void GPS_rcv(const Total_msgs::GNSSInfo::ConstPtr &GPS)
{
    float64_t f64_Lat = GPS->latitude;
    float64_t f64_Lon = GPS->longitude;
    float64_t f64_Alt = GPS->altitude;
    float32_t f32_Yaw = GPS->heading;
    st_Planning.b_TunnelOutSignal = GPS->tunnel_out_signal;
    // std::cout << "st_Planning.b_TunnelOutSignal : " << st_Planning.b_TunnelOutSignal << std::endl;

    st_GPSIMU.st_WGSRAD.f64_Latitude = Deg2Rad(f64_Lat);
    st_GPSIMU.st_WGSRAD.f64_Longitude = Deg2Rad(f64_Lon);
    if (b_HDMap)
    {
        st_GPSIMU.st_WGSRAD.f64_Altitude = 0;
    }
    st_GPSIMU.f32_Heading_deg = f32_Yaw;
    st_GPSIMU.f32_Heading_rad = Deg2Rad(st_GPSIMU.f32_Heading_deg);
    LLA2ENU(st_GPSIMU);
    PlanningProcess(st_Camera, st_LiDAR, st_Fusion, st_GPSIMU, st_Planning, st_Control, st_Vehicle);
    ControlProcess(st_Camera, st_GPSIMU, st_Control, st_Planning, st_Vehicle);
    PrintData(st_Camera, st_LiDAR, st_Fusion, st_GPSIMU, st_Control, st_Vehicle, st_Planning);
}

void Ego_rcv(const Total_msgs::EgoVehicleStatus::ConstPtr &Ego)
{
    float32_t f32_Yaw = Ego->heading;
    f32_MoraiENUYaw = f32_Yaw;
    f32_MoraiHeading = ENU2NED(f32_Yaw);
    st_Vehicle.f64_Velocity_MPS = sqrtf(pow(Ego->velocity.x, 2) + pow(Ego->velocity.y, 2));
    st_Vehicle.f64_Velocity_KPH = MPS2KPH(st_Vehicle.f64_Velocity_MPS);
}

void ERP_rcv(const Total_msgs::SerialFeedBack::ConstPtr &ERP)
{
    st_Vehicle.u8_MorA = (uint8_t)ERP->MorA;
    st_Vehicle.u8_EStop = (uint8_t)ERP->EStop;
    st_Vehicle.u8_Gear = (uint8_t)ERP->Gear;
    st_Vehicle.f64_Velocity_KPH = (float64_t)ERP->speed;
    st_Vehicle.f64_Steer = (float64_t)ERP->steer;
    st_Vehicle.s16_Brake = (int16_t)ERP->brake;
    st_Vehicle.u32_Encoder = (uint32_t)ERP->encoder;
    st_Vehicle.u8_Alive = (uint8_t)ERP->alive;
}

void Traffic_rcv(const Total_msgs::GetTrafficLightStatus::ConstPtr &Traffic)
{
    st_Planning.s16_TrafficLightStatus = Traffic->trafficLightStatus;
}

void CameraParser(ros::NodeHandle &st_NodeHandle, int &s32_CameraMode)
{
    if (s32_CameraMode == c_DISABLE)
    {
        st_SubcriberCamera = st_NodeHandle.subscribe<camera::TrafficLight_Arr>("/YOLOv8/TrafficerLight_Arr", 1, MORAICamera_rcv);
    }
    else if (s32_CameraMode == c_ENABLE)
    {
        st_SubcriberCamera = st_NodeHandle.subscribe<camera::signalInfo>("/data_process/Camcom", 1, Camera_rcv);
    }

    st_SubcriberBackLane = st_NodeHandle.subscribe<camera::LaneInfo>("/OptimalLane", 1, BackLane_rcv);
}

void LiDARParser(ros::NodeHandle &st_NodeHandle, int &s32_LiDARMode)
{
    if (s32_LiDARMode == c_DISABLE)
    {
        ;
    }
    else if (s32_LiDARMode == c_ENABLE)
    {
        st_SubcriberLiDAR = st_NodeHandle.subscribe<Total_msgs::objsInfo>("/LiDARcom", 1, LiDAR_rcv);

        // temp (will be integrated within objsInfo)
        st_SubcriberTunnelOutPoint = st_NodeHandle.subscribe<Total_msgs::tunneloutpoint_arr>("/TOP", 1, TunnelOutPoint_rcv);
        st_SubcriberLane = st_NodeHandle.subscribe<Total_msgs::lane_arr>("/Lane_coeff", 1, Lane_rcv);
    }
}

void FusionParser(ros::NodeHandle &st_NodeHandle, int &s32_FusionMode)
{
    if (s32_FusionMode == c_DISABLE)
    {
        ;
    }
    else if (s32_FusionMode == c_ENABLE)
    {
        st_SubcriberFusion = st_NodeHandle.subscribe<camera::fussInfo>("/FusionInfo", 1, Fusion_rcv);
    }
}

void GPSIMUParser(ros::NodeHandle &st_NodeHandle, int &s32_GPSIMUMode)
{
    if (s32_GPSIMUMode == c_EGOVEHICLE)
    {
        st_SubcriberGPSIMU = st_NodeHandle.subscribe<Total_msgs::GPSMessage>("/gps", 1, MORAIGPS_rcv);
    }
    else if (s32_GPSIMUMode == c_ERP)
    {
        st_SubcriberGPSIMU = st_NodeHandle.subscribe<Total_msgs::GNSSInfo>("/GNSScom", 1, GPS_rcv);
    }
}

void VehicleParser(ros::NodeHandle &st_NodeHandle, int &s32_VehicleMode)
{
    if (s32_VehicleMode == c_EGOVEHICLE)
    {
        st_SubcriberVehicle = st_NodeHandle.subscribe<Total_msgs::EgoVehicleStatus>("/Ego_topic", 1, Ego_rcv);
        st_SubcriberTraffic = st_NodeHandle.subscribe<Total_msgs::GetTrafficLightStatus>("/GetTrafficLightStatus", 1, Traffic_rcv);
    }
    else if (s32_VehicleMode == c_ERP)
    {
        st_SubcriberVehicle = st_NodeHandle.subscribe<Total_msgs::SerialFeedBack>("/erp42_serial/feedback", 1, ERP_rcv);
    }
}