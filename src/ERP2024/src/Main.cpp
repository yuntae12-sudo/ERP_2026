#include <Control/Control.hpp>
#include <Global/Global.hpp>
#include <Parser/Parser.hpp>
#include <Planning/Planning.hpp>

int32_t s32_CameraMode, s32_LiDARMode, s32_FusionMode, s32_GPSIMUMode, s32_VehicleMode;
int32_t s32_TrackingMap, s32_MissionMode, s32_TrafficMode, s32_StraightMode;
int32_t s32_PrintMode;

bool b_SlowSignal = false, b_StaySignal = false;
bool b_HDMap;

const std::string s_YamlAdress = "./src/ERP2024/configuration.yaml";
const std::string s_LoadFileAdress = "./src/Map/";

// MORAI
ros::Publisher st_Controller;

// ERP Serial
ros::Publisher st_Mode_serial, st_Drive_serial;

// TotalInfo
ros::Publisher st_TotalData;

// Visualize
ros::Publisher st_Visualize;

Camera_t st_Camera = {0};
LiDAR_t st_LiDAR = {0};
Fusion_t st_Fusion = {0};
GPSIMU_t st_GPSIMU = {0};
Vehicle_t st_Vehicle = {0};
Control_t st_Control = {0};
Planning_t st_Planning = {0};

// Control
float32_t f32_LookAheadDistance = 0;

void InitWGSRef()
{
    YAML::Node st_Config = YAML::LoadFile(s_YamlAdress);
    std::string s_MapName = st_Config["MapName"].as<std::string>();
    std::string s_MapAdress = s_LoadFileAdress + s_MapName + "/ref.txt";
    b_HDMap = st_Config["MapMode"].as<bool>();

    std::ifstream st_ReferenceLLA(s_MapAdress);
    float64_t f64_RefLatitude, f64_RefLongitude, f64_RefAltitude;
    st_ReferenceLLA >> f64_RefLatitude >> f64_RefLongitude >> f64_RefAltitude;
    st_ReferenceLLA.close();

    st_GPSIMU.st_RefWGSRAD.f64_Latitude = Deg2Rad(f64_RefLatitude);
    st_GPSIMU.st_RefWGSRAD.f64_Longitude = Deg2Rad(f64_RefLongitude);
    st_GPSIMU.st_RefWGSRAD.f64_Altitude = f64_RefAltitude;
    if (b_HDMap)
    {
        st_GPSIMU.st_RefWGSRAD.f64_Altitude = 0;
    }

    float64_t f64_Chi = sqrt(1 - c_N2 * pow(sin(st_GPSIMU.st_RefWGSRAD.f64_Latitude), 2));
    float64_t f64_Q = (c_EarthRadius / f64_Chi + st_GPSIMU.st_RefWGSRAD.f64_Altitude) * cos(st_GPSIMU.st_RefWGSRAD.f64_Latitude);

    st_GPSIMU.st_RefECEF.f64_X = f64_Q * cos(st_GPSIMU.st_RefWGSRAD.f64_Longitude);
    st_GPSIMU.st_RefECEF.f64_Y = f64_Q * sin(st_GPSIMU.st_RefWGSRAD.f64_Longitude);
    st_GPSIMU.st_RefECEF.f64_Z = ((c_EarthRadius * (1 - c_N2) / f64_Chi) + st_GPSIMU.st_RefWGSRAD.f64_Altitude) * sin(st_GPSIMU.st_RefWGSRAD.f64_Latitude);
}

void InitGlobalPath()
{
    YAML::Node st_Config = YAML::LoadFile(s_YamlAdress);
    std::string s_PathName = "", s_FileLine = "", s_TrafficFileLine = "";
    std::string s_MapName = st_Config["MapName"].as<std::string>();
    std::string s_MapAdress = s_LoadFileAdress + s_MapName + "/Path/";
    s32_TrackingMap = st_Config["TrackingMap"].as<int32_t>();
    s32_MissionMode = st_Config["MissionMode"].as<int32_t>();
    s32_TrafficMode = st_Config["TrafficMode"].as<int32_t>();
    s32_StraightMode = st_Config["StraightMode"].as<int32_t>();
    std::string s_MissionTestName = st_Config["MissionTestName"].as<std::string>();
    std::string s_TrafficTestName = st_Config["TrafficTestName"].as<std::string>();
    std::string s_StraightTestName = st_Config["StraightTestName"].as<std::string>();

    // Select Map
    if (s32_TrackingMap == c_MAIN)
    {
        s_PathName = "Main";
    }
    else if (s32_TrackingMap == c_PRELIMINARY)
    {
        s_PathName = "Qualifer";
    }
    else if (s32_TrackingMap == c_NARROW)
    {
        return;
    }

    // Select Mission + Load Mission Index
    if (s32_MissionMode != c_OFF)
    {
        std::string s_MissionIdxAdress = s_LoadFileAdress + s_MapName + "/MissionIdx/" + s_PathName;
        std::ifstream st_MissionIdxFile(s_MissionIdxAdress + ".txt");
        LoadIdx(st_MissionIdxFile, st_Planning.st_Mission);
        if (s32_MissionMode == c_TEST)
        {
            for (int32_t s32_i = 0; s32_i < st_Planning.st_Mission.s32_Num; s32_i++)
            {
                if (st_Planning.st_Mission.ars_Names[s32_i] == s_MissionTestName)
                {
                    st_Planning.s32_GlobalNearIdx = st_Planning.st_Mission.arp_Idx[s32_i].first + 500;
                }
            }
        }
    }

    // Select Traffic + Load Traffic Index
    if (s32_TrafficMode != c_OFF)
    {
        std::string s_TrafficIdxAdress = s_LoadFileAdress + s_MapName + "/TrafficIdx/" + s_PathName;
        std::ifstream st_TrafficIdxFile(s_TrafficIdxAdress + ".txt");
        LoadIdx(st_TrafficIdxFile, st_Planning.st_Traffic);
        if (s32_TrafficMode == c_TEST)
        {
            for (int32_t s32_i = 0; s32_i < st_Planning.st_Traffic.s32_Num; s32_i++)
            {
                if (st_Planning.st_Traffic.ars_Names[s32_i] == s_TrafficTestName)
                {
                    st_Planning.s32_GlobalNearIdx = st_Planning.st_Traffic.arp_Idx[s32_i].first - 100;
                }
            }
        }
    }

    // Select Straight + Corner Index
    if (s32_StraightMode != c_OFF)
    {
        std::string s_StraightIdxAdress = s_LoadFileAdress + s_MapName + "/StraightIdx/" + s_PathName;
        std::ifstream st_StraightIdxFile(s_StraightIdxAdress + ".txt");
        LoadIdx(st_StraightIdxFile, st_Planning.st_Straight);
        if (s32_StraightMode == c_TEST)
        {
            for (int32_t s32_i = 0; s32_i < st_Planning.st_Straight.s32_Num; s32_i++)
            {
                if (st_Planning.st_Straight.ars_Names[s32_i] == s_StraightTestName)
                {
                    st_Planning.s32_GlobalNearIdx = st_Planning.st_Straight.arp_Idx[s32_i].first - 300;
                }
            }
        }
    }

    // Load Global Path
    std::ifstream st_RefPathFile(s_MapAdress + s_PathName + "Path.txt");
    LoadData(st_RefPathFile, st_Planning.st_GlobalPath);

    // Load Global Boundary
    std::string s_BoundaryAdress = s_LoadFileAdress + s_MapName + "/Boundary/" + s_PathName;
    std::ifstream st_InRefBoundary1File(s_BoundaryAdress + "/InBoundary1.txt");
    std::ifstream st_InRefBoundary2File(s_BoundaryAdress + "/InBoundary2.txt");
    std::ifstream st_InRefBoundary3File(s_BoundaryAdress + "/InBoundary3.txt");
    std::ifstream st_InRefBoundary4File(s_BoundaryAdress + "/InBoundary3.txt");
    std::ifstream st_OutRefBoundaryFile(s_BoundaryAdress + "/OutBoundary.txt");

    LoadData(st_InRefBoundary1File, st_Planning.st_InBoundary1);
    LoadData(st_InRefBoundary2File, st_Planning.st_InBoundary2);
    LoadData(st_InRefBoundary3File, st_Planning.st_InBoundary3);
    LoadData(st_InRefBoundary4File, st_Planning.st_InBoundary4);
    LoadData(st_OutRefBoundaryFile, st_Planning.st_OutBoundary);
}

void InitPlanning()
{
    YAML::Node st_Config = YAML::LoadFile(s_YamlAdress);
    st_Planning.st_AvoidSmall.f32_LeftThres = st_Config["AvoidSmallLeftThres"].as<float32_t>();
    st_Planning.st_AvoidSmall.f32_RightThres = st_Config["AvoidSmallRightThres"].as<float32_t>();
    st_Planning.st_AvoidSmall.f32_FirstDist = st_Config["AvoidSmallFirstDist"].as<float32_t>();
    st_Planning.st_AvoidSmall.f32_LastDist = st_Config["AvoidSmallLastDist"].as<float32_t>();
    st_Planning.st_AvoidSmall.f32_Thres = st_Config["AvoidSmallThres"].as<float32_t>();
    YAML::Node st_Configs = st_Config["AvoidSmallCosts"];
    for (int32_t s32_i = 0; s32_i < 3; s32_i++)
    {
        st_Planning.st_AvoidSmall.arf32_Costs[s32_i] = st_Configs[s32_i].as<float32_t>();
    }

    st_Planning.st_AvoidLarge.f32_LeftThres = st_Config["AvoidLargeLeftThres"].as<float32_t>();
    st_Planning.st_AvoidLarge.f32_RightThres = st_Config["AvoidLargeRightThres"].as<float32_t>();
    st_Planning.st_AvoidLarge.f32_FirstDist = st_Config["AvoidLargeFirstDist"].as<float32_t>();
    st_Planning.st_AvoidLarge.f32_LastDist = st_Config["AvoidLargeLastDist"].as<float32_t>();
    st_Planning.st_AvoidLarge.f32_Thres = st_Config["AvoidLargeThres"].as<float32_t>();
    st_Configs = st_Config["AvoidLargeCosts"];
    for (int32_t s32_i = 0; s32_i < 3; s32_i++)
    {
        st_Planning.st_AvoidLarge.arf32_Costs[s32_i] = st_Configs[s32_i].as<float32_t>();
    }
    st_Planning.s32_DeliveryMode = st_Config["DeliveryMode"].as<int32_t>();
    st_Planning.st_Delivery.f32_LeftThres = st_Config["DeliveryLargeLeftThres"].as<float32_t>();
    st_Planning.st_Delivery.f32_RightThres = st_Config["DeliveryLargeRightThres"].as<float32_t>();
    st_Planning.st_Delivery.f32_FirstLastDist = st_Config["DeliveryFirstLastDist"].as<float32_t>();
    st_Planning.st_Delivery.f32_Thres = st_Config["DeliveryThres"].as<float32_t>();
    st_Configs = st_Config["DeliveryCosts"];
    for (int32_t s32_i = 0; s32_i < 3; s32_i++)
    {
        st_Planning.st_Delivery.arf32_Costs[s32_i] = st_Configs[s32_i].as<float32_t>();
    }

    st_Planning.st_Uturn.s32_CodeSelect = st_Config["UturnCodeSelect"].as<int32_t>();
    st_Planning.st_Uturn.f32_AngleThreshold_deg = st_Config["UturnAngleThreshold_deg"].as<float32_t>();
    st_Planning.st_Uturn.s32_StartIdxMargin = st_Config["UturnStartIdxMargin"].as<int32_t>();
    st_Planning.st_Uturn.s32_IdxMargin = st_Config["UturnIdxMargin"].as<int32_t>();

    st_Planning.st_Tunnel.f32_WallWidth = st_Config["TunnelWallWidth"].as<float32_t>();
    st_Planning.st_Tunnel.f32_RoadWidth = st_Config["TunnelRoadWidth"].as<float32_t>();
    st_Planning.st_Tunnel.f32_RoadCenter = st_Config["TunnelRoadCenter"].as<float32_t>();
    st_Planning.st_Tunnel.f32_AvoidOffset = st_Config["TunnelAvoidOffset"].as<float32_t>();
    st_Planning.st_Tunnel.f32_AvoidMargin = st_Config["TunnelAvoidMargin"].as<float32_t>();

    st_Configs = st_Config["LaneOffSet"];
    for (int32_t s32_i = 0; s32_i < 4; s32_i++)
    {
        st_Planning.arf32_LaneOffSet[s32_i] = st_Configs[s32_i].as<float32_t>();
    }
    st_Configs = st_Config["RefYaw"];
    for (int32_t s32_i = 0; s32_i < 4; s32_i++)
    {
        st_Planning.arf32_RefYaw[s32_i] = st_Configs[s32_i].as<float32_t>();
    }
    st_Configs = st_Config["MakePathStart"];
    for (int32_t s32_i = 0; s32_i < 4; s32_i++)
    {
        st_Planning.ars32_MakePathStart[s32_i] = st_Configs[s32_i].as<int32_t>();
    }
    st_Configs = st_Config["MakePathEnd"];
    for (int32_t s32_i = 0; s32_i < 4; s32_i++)
    {
        st_Planning.ars32_MakePathEnd[s32_i] = st_Configs[s32_i].as<int32_t>();
    }
}

void InitControl()
{
    YAML::Node st_Config = YAML::LoadFile(s_YamlAdress);
    st_Control.s32_CanMode = st_Config["CanMode"].as<int32_t>();
    st_Control.s32_ControlMethod = st_Config["ControlMethod"].as<int32_t>();
    // Load Control Param
    st_Control.f32_TestVelocity_KPH = st_Config["TargetVelocity"].as<float32_t>();
    st_Control.f32_TestVelocity_MPS = KPH2MPS(st_Control.f32_TargetVelocity_KPH);
    st_Control.f32_TestLD = st_Config["LookAheadDistance"].as<float32_t>();
    st_Control.st_PID.f32_KP = st_Config["PGain"].as<float32_t>();
    st_Control.st_PID.f32_KI = st_Config["IGain"].as<float32_t>();
    st_Control.st_PID.f32_KD = st_Config["DGain"].as<float32_t>();
    st_Control.st_Stanley.f32_K = st_Config["Stanley_k"].as<float32_t>();

    if (s32_TrackingMap == c_MAIN)
    {
        YAML::Node st_Configs = st_Config["MissionMainSpeed"];
        for (int32_t s32_i = 0; s32_i < 5; s32_i++)
        {
            st_Control.arf32_MissionTargetSpeed[s32_i] = st_Configs[s32_i].as<float32_t>();
        }
    }
    else if (s32_TrackingMap == c_PRELIMINARY)
    {
        YAML::Node st_Configs = st_Config["MissionMQualSpeed"];
        for (int32_t s32_i = 0; s32_i < 2; s32_i++)
        {
            st_Control.arf32_MissionTargetSpeed[s32_i] = st_Configs[s32_i].as<float32_t>();
        }
    }

    if (s32_TrackingMap == c_MAIN)
    {
        YAML::Node st_Configs = st_Config["MissionMainLD"];
        for (int32_t s32_i = 0; s32_i < 5; s32_i++)
        {
            st_Control.arf32_MissionTargetLD[s32_i] = st_Configs[s32_i].as<float32_t>();
        }
    }
    else if (s32_TrackingMap == c_PRELIMINARY)
    {
        YAML::Node st_Configs = st_Config["MissionMQualLD"];
        for (int32_t s32_i = 0; s32_i < 2; s32_i++)
        {
            st_Control.arf32_MissionTargetLD[s32_i] = st_Configs[s32_i].as<float32_t>();
        }
    }
}

void InitSensor(ros::NodeHandle &st_NodeHandle)
{
    YAML::Node st_Config = YAML::LoadFile(s_YamlAdress);
    s32_CameraMode = st_Config["CameraMode"].as<int32_t>();
    s32_LiDARMode = st_Config["LiDARMode"].as<int32_t>();
    s32_FusionMode = st_Config["FusionMode"].as<int32_t>();
    s32_GPSIMUMode = st_Config["GPSIMUMode"].as<int32_t>();
    s32_VehicleMode = st_Config["VehicleMode"].as<int32_t>();
    s32_PrintMode = st_Config["PrintMode"].as<int32_t>();
    CameraParser(st_NodeHandle, s32_CameraMode);
    LiDARParser(st_NodeHandle, s32_LiDARMode);
    FusionParser(st_NodeHandle, s32_FusionMode);
    GPSIMUParser(st_NodeHandle, s32_GPSIMUMode);
    VehicleParser(st_NodeHandle, s32_VehicleMode);
}

void InitPublisher(ros::NodeHandle &st_NodeHandle)
{
    // controller pub
    if (st_Control.s32_CanMode == c_MORAI)
    {
        st_Controller = st_NodeHandle.advertise<morai_msgs::CtrlCmd>("ctrl_cmd", 1);
    }
    else if (st_Control.s32_CanMode == c_ERP)
    {
        st_Mode_serial = st_NodeHandle.advertise<Total_msgs::ModeCmd>("/erp42_serial/mode", 1);
        st_Drive_serial = st_NodeHandle.advertise<Total_msgs::DriveCmd>("/erp42_serial/drive", 1);
    }
    // misision number pub -> sensor
    st_TotalData = st_NodeHandle.advertise<Total_msgs::totalInfo>("/Totalcom", 1);
    // object data + path pub -> rviz
    st_Visualize = st_NodeHandle.advertise<visualization_msgs::MarkerArray>("/Visualize", 1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PlnCtrl");
    ros::NodeHandle st_NodeHandle;
    InitWGSRef();
    InitGlobalPath();
    InitPlanning();
    InitControl();
    InitSensor(st_NodeHandle);
    InitPublisher(st_NodeHandle);

    // ros::Rate loop_rate(30); // sampling time hz
    // while (ros::ok())
    // {
    // ros::spinOnce();
    // loop_rate.sleep();
    // }
    ros::spin();

    return 0;
}