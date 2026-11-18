#ifndef CONTROL_HPP
#define CONTROL_HPP
#include <Global/Global.hpp>

// Morai msgs
#include <morai_msgs/CtrlCmd.h>

// ERP msgs
#include <Total_msgs/ModeCmd.h>
#include <Total_msgs/DriveCmd.h>
#include <Total_msgs/CmdControl.h>

void ControlProcess(Camera_t &st_Camera, GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning, Vehicle_t &st_Vehicle);
void LateralControl(Camera_t &st_Camera, GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning, Vehicle_t &st_Vehicle);
void ControlSetup(GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning);
void SignalProcess(Control_t &st_Control, Planning_t &st_Planning);

// Control Method
void SpeedProfile(GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning);
void PIDThrottle(Control_t &st_Control, Vehicle_t &st_Vehicle);

void PurePursuit(GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning);
void StanleyMethod(GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning, Vehicle_t &st_Vehicle);
void AdvancedStanleyMethod(GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning);
void LKAS(Camera_t &st_Camera, GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning);
void MPC(GPSIMU_t &st_GPSIMU, Control_t &st_Control, Planning_t &st_Planning);

// Control Setup
void PubMorai(Control_t &st_Control);
void PubSerial(Control_t &st_Control, Vehicle_t &st_Vehicle, Planning_t &st_Planning);
void ControltoVehicle(Control_t &st_Control, Vehicle_t &st_Vehicle, Planning_t &st_Planning);

#endif