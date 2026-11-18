#pragma once
#include "SerialPort/serial_port.h"
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <Total_msgs/DriveCmd.h>
#include <Total_msgs/ModeCmd.h>
#include <Total_msgs/CmdControl.h>
#include <Total_msgs/SerialFeedBack.h>

namespace unmansol { namespace erp42 {
class ERP42Serial {
public:
    ERP42Serial(const char* device_name, int serial_baudrate);
    bool Open();
    bool Read();
    void Write();
    void Run();
private:
    SafetyConfig LoadConfig();
    void UpdateNode();
    void DriveCallback(const Total_msgs::DriveCmd::ConstPtr& msg);
    void ModeCallback(const Total_msgs::ModeCmd::ConstPtr& msg);
    bool Rearm(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
    ros::NodeHandle nh_;
    SafetyControl safety_;
    std::unique_ptr<SerialPort> serial_port_;
    ros::Publisher feedback_pub_, command_pub_;
    ros::Subscriber drive_sub_, mode_sub_;
    ros::ServiceServer rearm_service_;
    uint8_t alive_ = 0;
    bool transmit_blocked_ = false;
};
}} // namespace unmansol::erp42
