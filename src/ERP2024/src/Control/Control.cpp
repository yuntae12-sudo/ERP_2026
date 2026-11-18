#include <Control/Control.hpp>
#include <Planning/Planning.hpp>

extern ros::Publisher st_Controller;
extern ros::Publisher st_Mode_serial;
extern ros::Publisher st_Drive_serial;
extern int32_t s32_TrackingMap, s32_MissionMode;
extern float32_t f32_LookAheadDistance;
int32_t s32_Tmpcount = 0;

void ControlProcess(Camera_t &st_Camera, GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning, Vehicle_t &st_Vehicle)
{
    // Data Processing
    ControlSetup(st_GPSIMU, st_Control, st_Planning);
    SpeedProfile(st_GPSIMU, st_Control, st_Planning);
    SignalProcess(st_Control, st_Planning);

    // Control
    LateralControl(st_Camera, st_GPSIMU, st_Control, st_Planning, st_Vehicle);
    PIDThrottle(st_Control, st_Vehicle);
    SignalProcess(st_Control, st_Planning);
    ControltoVehicle(st_Control, st_Vehicle, st_Planning);
}

void ControlSetup(GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning)
{
    // cout << "st_Planning.s_PathType: " << st_Planning.s_PathType << endl;
    if (st_Planning.s_PathType == "Global")
    {
        st_Control.st_FinalPath = &st_Planning.st_GlobalPath;
        st_Control.st_FinalPath->s32_NearIdx = st_Planning.s32_GlobalNearIdx;
    }
    else if (st_Planning.s_PathType == "Local")
    {
        st_Control.st_FinalPath = &st_Planning.st_LocalPath;

        st_Control.st_FinalPath->s32_NearIdx = st_Planning.s32_LocalNearIdx;
        st_Control.st_FinalPath->s32_NearIdx = FindNear(st_Planning, st_GPSIMU.f32_E, st_GPSIMU.f32_N, c_FindNearLen, "Local");
        GetModData(1, 3000, st_Control.st_FinalPath->s32_PathNum);
    }
}

void LateralControl(Camera_t &st_Camera, GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning, Vehicle_t &st_Vehicle)
{
    switch (st_Control.s32_ControlMethod)
    {
    case c_PUREPURSUIT:
        PurePursuit(st_GPSIMU, st_Control, st_Planning);
        break;
    case c_STANLEY:
        StanleyMethod(st_GPSIMU, st_Control, st_Planning, st_Vehicle);
        break;
    case c_LDSTANLEY:
        AdvancedStanleyMethod(st_GPSIMU, st_Control, st_Planning);
        break;
    case c_LKAS:
        LKAS(st_Camera, st_GPSIMU, st_Control, st_Planning);
        break;
    case c_MPC:
        MPC(st_GPSIMU, st_Control, st_Planning);
        break;
    }
}

void SignalProcess(Control_t &st_Control, Planning_t &st_Planning)
{
    Timer_t &st_Timer = st_Control.st_Timer;

    // Slow Sign
    if (st_Planning.b_SlowSignal)
    {
        st_Control.f32_TargetVelocity_KPH = c_SlowVelocity;
        st_Control.f32_TargetVelocity_MPS = c_SlowVelocity / 3.6;
    }

    // Stay & Estop
    if (st_Planning.b_StaySignal)
    {
        if (st_Control.s32_Timeflag == 0)
        {
            st_Timer.st_Start = std::chrono::steady_clock::now();
            st_Control.s32_Timeflag = 1;
        }
        else if (st_Control.s32_Timeflag == 1)
        {
            st_Timer.st_End = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = st_Timer.st_End - st_Timer.st_Start;
            
            st_Control.f32_ThrottleInput = 0;
            st_Control.f32_TargetVelocity_MPS = 0;
            st_Control.f32_TargetBrake = 200;

            if (elapsed_seconds.count() >= 5.0)
            {
                st_Planning.b_StaySignal = false;
                st_Control.s32_Timeflag = 0;
            }
        }
    }
    else if (st_Planning.b_EStopSignal)
    {
        st_Control.u8_EStop = 1;
        st_Control.f32_ThrottleInput = 0;
        st_Control.f32_TargetVelocity_MPS = 0;
        st_Control.f32_TargetBrake = 200;
    }
    else if (st_Planning.b_KeepSignal || st_Planning.b_RightStaySignal)
    {
        st_Control.f32_ThrottleInput = 0;
        st_Control.f32_TargetVelocity_MPS = 0;
        st_Control.f32_TargetBrake = 200;
    }
    else
    {
        st_Control.u8_EStop = 0;
        st_Control.f32_TargetBrake = 0;
    }

    // // Traffic KeepSignal && RightStaySignal
    // if (st_Planning.b_KeepSignal || st_Planning.b_RightStaySignal)
    // {
    //     st_Control.f32_ThrottleInput = 0;
    //     st_Control.f32_TargetVelocity_MPS = 0;
    //     st_Control.f32_TargetBrake = 200;
    // }
    // else
    // {
    //     st_Control.f32_TargetBrake = 0;
    // }
}

void ControltoVehicle(Control_t &st_Control, Vehicle_t &st_Vehicle, Planning_t &st_Planning)
{
    switch (st_Control.s32_CanMode)
    {
    case c_MORAI:
        PubMorai(st_Control);
        break;
    case c_ERP:
        PubSerial(st_Control, st_Vehicle, st_Planning);
        break;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// Control Method ///////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

void SpeedProfile(GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning)
{
    if (s32_MissionMode == c_ON)
    {
        if(st_Planning.s_PubMissionName != "")
        {
            st_Control.f32_TargetVelocity_KPH = st_Control.arf32_MissionTargetSpeed[st_Planning.st_Mission.s32_SelectNumer];
            st_Control.f32_TargetVelocity_MPS = KPH2MPS(st_Control.f32_TargetVelocity_KPH);

        }
        else if (st_Planning.s_PubMissionName == "")
        {
            st_Control.f32_TargetVelocity_KPH = st_Control.f32_TestVelocity_KPH;
            st_Control.f32_TargetVelocity_MPS = KPH2MPS(st_Control.f32_TestVelocity_KPH);
        }
    }
    else if (s32_MissionMode == c_TEST)
    {
        st_Control.f32_TargetVelocity_KPH = st_Control.f32_TestVelocity_KPH;
        st_Control.f32_TargetVelocity_MPS = st_Control.f32_TestVelocity_MPS;
    }
    else
    {
        st_Control.f32_TargetVelocity_KPH = st_Control.f32_TestVelocity_KPH;
        st_Control.f32_TargetVelocity_MPS = KPH2MPS(st_Control.f32_TestVelocity_KPH);
    }

    // if (st_Planning.st_Mission.s_SelectName == "TRACKING")
    // {
    //     // 속력 조정 밖으로 빼
    //     st_Control.f32_TargetVelocity_KPH = st_Control.f32_TestVelocity_KPH;
    //     st_Control.f32_TargetVelocity_MPS = KPH2MPS(st_Control.f32_TestVelocity_KPH);
    // }
}

void PurePursuit(GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning)
{
    PurePursuit_t &st_PurePursuit = st_Control.st_PurePursuit;
    ControlParam_t &st_ControlParam = st_Control.st_ControlParam;
    Path_t &st_Path = *st_Control.st_FinalPath;

    Point2D_t st_Rear_Pos;
    float32_t f32_Heading_rad = 0;

    if (s32_TrackingMap == c_NARROW)
    {
        st_Path.s32_NearIdx = 0;
        // st_Rear_Pos.f32_X = st_ControlParam.f32_LiDAR_2_Rear;
        st_Rear_Pos.f32_X = 0;
        st_Rear_Pos.f32_Y = 0;
    }
    else if (st_Planning.st_Mission.s_SelectName == "TUNNEL")
    {
        st_Path.s32_NearIdx = 0;
        st_Rear_Pos.f32_X = 0;
        st_Rear_Pos.f32_Y = st_ControlParam.f32_LiDAR_2_Rear;
    }
    else if(st_Planning.st_Mission.s_SelectName == "AVOID_SMALL" || st_Planning.st_Mission.s_SelectName == "AVOID_LARGE")
    {
        // st_Path.s32_NearIdx = 0;
        st_Rear_Pos.f32_X = 0;
        st_Rear_Pos.f32_Y = 0;

        st_Path.s32_NearIdx = FindNear_LiDAR(st_Planning, st_Rear_Pos.f32_X, st_Rear_Pos.f32_Y, st_Path.s32_PathNum, st_Path);
        cout << "st_Path.s32_NearIdx: " << st_Path.s32_NearIdx << endl;
    }
    else if(st_Planning.st_Mission.s_SelectName == "U_TURN" && st_Planning.st_Uturn.s32_CodeSelect == 2)
    {
        st_Rear_Pos.f32_X = 0;
        st_Rear_Pos.f32_Y = 0;
    }
    else
    {
        f32_Heading_rad = st_GPSIMU.f32_Heading_rad;
        // st_Rear_Pos.f32_X = st_GPSIMU.f32_E - st_ControlParam.f32_GPS_2_Rear * sinf(f32_Heading_rad);
        // st_Rear_Pos.f32_Y = st_GPSIMU.f32_N - st_ControlParam.f32_GPS_2_Rear * cosf(f32_Heading_rad);
        st_Rear_Pos.f32_X = st_GPSIMU.f32_E;
        st_Rear_Pos.f32_Y = st_GPSIMU.f32_N;

    }

    f32_LookAheadDistance = 0;
    // Find WayIdx
    // cout << "st_PurePursuit.f32_LD: " << st_PurePursuit.f32_LD << endl;
    for (int32_t s32_i = st_Path.s32_NearIdx; s32_i < st_Path.s32_NearIdx + c_FindWayLen; s32_i++)
    {
        f32_LookAheadDistance = Normalize2D(st_Path.arf32_E, st_Path.arf32_N, s32_i % st_Path.s32_PathNum, st_Rear_Pos);

        if (f32_LookAheadDistance > st_PurePursuit.f32_LD)
        {
            st_PurePursuit.s32_WayIdx = s32_i % st_Path.s32_PathNum;
            break;
        }
    }

    st_PurePursuit.f32_Alpha = atan2f(st_Path.arf32_E[st_PurePursuit.s32_WayIdx] - st_Rear_Pos.f32_X, st_Path.arf32_N[st_PurePursuit.s32_WayIdx] - st_Rear_Pos.f32_Y);
    st_PurePursuit.f32_Alpha -= f32_Heading_rad;

    if (st_Planning.st_Mission.s_SelectName == "TUNNEL")
    {
        st_Control.f32_TargetSteer = atan2f(2.0 * st_ControlParam.f32_WB * sin(st_PurePursuit.f32_Alpha), f32_LookAheadDistance);
    }
    else if(st_Planning.st_Mission.s_SelectName == "AVOID_SMALL" || st_Planning.st_Mission.s_SelectName == "AVOID_LARGE")
    {
        // cout << "st_PurePursuit.s32_WayIdx: " << st_PurePursuit.s32_WayIdx << endl;
        st_PurePursuit.f32_Alpha = atan2f(st_Path.arf32_N[st_PurePursuit.s32_WayIdx] - st_Rear_Pos.f32_Y, st_Path.arf32_E[st_PurePursuit.s32_WayIdx] - st_Rear_Pos.f32_X);
        st_PurePursuit.f32_Alpha += st_GPSIMU.f32_RefHeading_rad;
        st_Control.f32_TargetSteer = atan2f(2.0 * st_ControlParam.f32_WB * sin(st_PurePursuit.f32_Alpha), f32_LookAheadDistance);
        // cout << "st_Control.f32_TargetSteer : " << st_Control.f32_TargetSteer << endl;
        VisualizePoint({st_Path.arf32_E[st_Path.s32_NearIdx], st_Path.arf32_N[st_Path.s32_NearIdx]}, Color::BLUE, 1.0f);
        VisualizePoint({st_Path.arf32_E[st_PurePursuit.s32_WayIdx], st_Path.arf32_N[st_PurePursuit.s32_WayIdx]}, Color::ORANGE, 1.0f);
    }
    else if(st_Planning.st_Mission.s_SelectName == "U_TURN" && st_Planning.st_Uturn.s32_CodeSelect == 2)
    {
        VisualizePoint({st_Path.arf32_E[st_Path.s32_NearIdx], st_Path.arf32_N[st_Path.s32_NearIdx]}, Color::BLUE, 1.0f);
        VisualizePoint({st_Path.arf32_E[st_PurePursuit.s32_WayIdx], st_Path.arf32_N[st_PurePursuit.s32_WayIdx]}, Color::ORANGE, 1.0f);

        st_PurePursuit.f32_Alpha = atan2f(st_Path.arf32_N[st_PurePursuit.s32_WayIdx] - st_Rear_Pos.f32_Y, st_Path.arf32_E[st_PurePursuit.s32_WayIdx] - st_Rear_Pos.f32_X);
        st_Control.f32_TargetSteer = atan2f(2.0 * st_ControlParam.f32_WB * sin(st_PurePursuit.f32_Alpha), f32_LookAheadDistance);
        cout << "st_Control.f32_TargetSteer : " << st_Control.f32_TargetSteer << endl;
    }
    else if (s32_TrackingMap == c_NARROW)
    {
        st_PurePursuit.f32_Alpha = atan2f(st_Path.arf32_N[st_PurePursuit.s32_WayIdx] - st_Rear_Pos.f32_Y, st_Path.arf32_E[st_PurePursuit.s32_WayIdx] - st_Rear_Pos.f32_X);
        st_Control.f32_TargetSteer = atan2f(2.0 * st_ControlParam.f32_WB * sin(st_PurePursuit.f32_Alpha), f32_LookAheadDistance);
        cout << "st_Control.f32_TargetSteer : " << st_Control.f32_TargetSteer << endl;
        float32_t f32_Alpha_tmp = atan2f(st_Path.arf32_N[st_Path.s32_PathNum - 1] - st_Rear_Pos.f32_Y, st_Path.arf32_E[st_Path.s32_PathNum - 1] - st_Rear_Pos.f32_X);
        float32_t f32_Steer_tmp = -atan2f(2.0 * st_ControlParam.f32_WB * sin(f32_Alpha_tmp), f32_LookAheadDistance); 

        float32_t f32_VelocityParam = 8;

        st_Control.f32_TargetVelocity_KPH = st_Control.f32_TargetVelocity_KPH - f32_VelocityParam * abs(f32_Steer_tmp); 
        // cout << "f32_Steer_tmp : " << f32_Steer_tmp << endl;
        // cout << "st_Control.f32_TargetVelocity_KPH : " << st_Control.f32_TargetVelocity_KPH << endl;

    }
    else
    {
        st_Control.f32_TargetSteer = -atan2f(2.0 * st_ControlParam.f32_WB * sin(st_PurePursuit.f32_Alpha), f32_LookAheadDistance);
    }
}

void StanleyMethod(GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning, Vehicle_t &st_Vehicle)
{
    Stanley_t &st_Stanley = st_Control.st_Stanley;
    ControlParam_t &st_ControlParam = st_Control.st_ControlParam;
    Path_t &st_Path = *st_Control.st_FinalPath;

    float32_t arf_Dist[c_FindNearLen] = {0};
    Point2D_t st_Front_Pos;

    // Get Front Axle Position
    st_Front_Pos.f32_X = st_GPSIMU.f32_E + st_ControlParam.f32_GPS_2_Front * sinf(st_GPSIMU.f32_Heading_rad);
    st_Front_Pos.f32_Y = st_GPSIMU.f32_N + st_ControlParam.f32_GPS_2_Front * cosf(st_GPSIMU.f32_Heading_rad);

    // Find FrontNearIdx
    for (int32_t s32_i = st_Stanley.s32_FrontNearIdx; s32_i < st_Stanley.s32_FrontNearIdx + c_FindNearLen; s32_i++)
    {
        arf_Dist[s32_i - st_Stanley.s32_FrontNearIdx] = Normalize2D(st_Path.arf32_E, st_Path.arf32_N, s32_i % st_Path.s32_PathNum,
                                                                    st_Front_Pos.f32_X, st_Front_Pos.f32_Y);
    }
    st_Stanley.s32_FrontNearIdx = (st_Stanley.s32_FrontNearIdx + std::distance(arf_Dist, std::min_element(arf_Dist, arf_Dist + c_FindNearLen))) % st_Path.s32_PathNum;

    // Path Vector
    float32_t arf32_Path_Vec[2] = {st_Front_Pos.f32_X - st_Path.arf32_E[st_Stanley.s32_FrontNearIdx],
                                   st_Front_Pos.f32_Y - st_Path.arf32_N[st_Stanley.s32_FrontNearIdx]};

    // Front Axle Vector
    float32_t arf32_Axle_Vec[2] = {sin(st_GPSIMU.f32_Heading_rad),
                                   cos(st_GPSIMU.f32_Heading_rad)};

    st_Stanley.f32_Error = arf32_Path_Vec[1] * arf32_Axle_Vec[0] - arf32_Path_Vec[0] * arf32_Axle_Vec[1];

    // Get Reference Data
    st_Stanley.f32_ReferenceYaw = atan2f(st_Path.arf32_E[st_Stanley.s32_FrontNearIdx + 30] - st_Path.arf32_E[st_Stanley.s32_FrontNearIdx],
                                         st_Path.arf32_N[st_Stanley.s32_FrontNearIdx + 30] - st_Path.arf32_N[st_Stanley.s32_FrontNearIdx]);

    st_Stanley.f32_speed = st_Vehicle.f64_Velocity_MPS;
    GetModData(1.0, KPH2MPS(20.0), st_Stanley.f32_speed);

    // Get Target Steer
    st_Control.f32_TargetSteer = -(0.7 * pi2pi(st_Stanley.f32_ReferenceYaw - st_GPSIMU.f32_Heading_rad) +
                                   atan2f(st_Stanley.f32_K * st_Stanley.f32_Error, st_Stanley.f32_speed));

    // GetModData(-pst_ControlParam->f32_MAX_STEER_rad, pst_ControlParam->f32_MAX_STEER_rad, st_Control.f32_TargetSteer);
}

void AdvancedStanleyMethod(GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning)
{
}

void LKAS(Camera_t &st_Camera, GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning)
{
}

void MPC(GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning)
{
}

void PIDThrottle(Control_t &st_Control, Vehicle_t &st_Vehicle)
{
    PID_t &st_PID = st_Control.st_PID;

    if (st_Control.f32_TargetBrake == 200)
    {
        st_Control.f32_ThrottleInput = 0;
        return;
    }
    // Get Error
    float32_t Max_Speed = 24;
    st_PID.f32_Error = (st_Control.f32_TargetVelocity_KPH - st_Vehicle.f64_Velocity_KPH) / Max_Speed;
    // std::cout << "st_Vehicle.f64_Velocity_KPH : " << st_Vehicle.f64_Velocity_KPH << std::endl;

    // Calculation Input
    st_PID.f32_InputP = st_PID.f32_KP * st_PID.f32_Error;
    st_PID.f32_InputI = st_PID.f32_InputI + st_PID.f32_KI * st_PID.f32_Error * 0.02;
    st_PID.f32_InputD = st_PID.f32_KD * (st_PID.f32_Error - st_PID.f32_PreError) / 0.02;

    // Update PreError
    st_PID.f32_PreError = st_PID.f32_Error;
    // st_Control.f32_ThrottleInput = (st_PID.f32_InputP + st_PID.f32_InputI + st_PID.f32_InputD) * 24;
    st_Control.f32_ThrottleInput = st_Control.f32_TargetVelocity_KPH;
    GetModData(0.0f, 24.0f, st_Control.f32_ThrottleInput);
}

///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// Publish ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

void PubMorai(Control_t &st_Control)
{
    morai_msgs::CtrlCmd msg;

    msg.longlCmdType = 2;
    msg.steering = st_Control.f32_TargetSteer;
    msg.velocity = st_Control.f32_ThrottleInput;
    msg.brake = st_Control.f32_TargetBrake;
    // cout << "velocity: " << msg.velocity << " brake: " << msg.brake << endl;
    st_Controller.publish(msg);
}

void PubSerial(Control_t &st_Control, Vehicle_t &st_Vehicle, Planning_t &st_Planning)
{
    Total_msgs::ModeCmd::Ptr mode_msg(new Total_msgs::ModeCmd);
    Total_msgs::DriveCmd::Ptr drive_msg(new Total_msgs::DriveCmd);

    float32_t f32_TargetSteer = Rad2Deg(st_Control.f32_TargetSteer) * 74.074;

    mode_msg->alive = (uint8_t)1;
    mode_msg->EStop = (uint8_t)st_Control.u8_EStop;
    mode_msg->MorA = (uint8_t)1;

    if (st_Planning.s32_GearSignal == 0)
    {
        mode_msg->Gear = (uint8_t)0;
    }
    else if (st_Planning.s32_GearSignal == 2)
    {
        mode_msg->Gear = (uint8_t)2;
    }

    GetModData(-2000.0f, 2000.0f, f32_TargetSteer);
    GetModData(1.0f, 200.0f, st_Control.f32_TargetBrake);

    drive_msg->brake = (uint8_t)st_Control.f32_TargetBrake;
    drive_msg->Deg = (int16_t)(-f32_TargetSteer);
    // drive_msg->KPH = (uint16_t)st_Control.f32_TargetVelocity_KPH * 10;
    drive_msg->KPH = (uint16_t)st_Control.f32_ThrottleInput * 10;
    // drive_msg->KPH = (uint16_t)7 * 10;
    drive_msg->brake = 1;
    st_Mode_serial.publish(mode_msg);
    st_Drive_serial.publish(drive_msg);
}
