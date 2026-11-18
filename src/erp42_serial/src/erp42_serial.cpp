#include "erp42_serial.h"
#include <algorithm>
#include <thread>

namespace unmansol { namespace erp42 {
ERP42Serial::ERP42Serial(const char* name, int baud)
    : nh_("~"), safety_(LoadConfig()) {
    int timeout = 20;
    bool allow_hardware = false;
    nh_.param("io_timeout_ms", timeout, timeout);
    nh_.param("allow_hardware_serial", allow_hardware, false);
    if (timeout <= 0 || timeout > 100) {
        ROS_ERROR("io_timeout_ms must be in [1,100]; refusing transport");
        return;
    }
    serial_port_.reset(new SerialPort(name, baud, timeout, allow_hardware));
    UpdateNode();
}
SafetyConfig ERP42Serial::LoadConfig() {
    SafetyConfig c;
    nh_.param("stationary_only", c.stationary_only, true);
    nh_.param("test_deadline_sec", c.test_deadline_sec, -1.0);
    nh_.param("protocol_confirmed", c.protocol_confirmed, false);
    nh_.param("stop_brake", c.stop_brake, -1);
    nh_.param("brake_min", c.brake_min, -1);
    nh_.param("brake_max", c.brake_max, -1);
    nh_.param("brake_release", c.brake_release, -1);
    nh_.param("estop_assert", c.estop_assert, -1);
    nh_.param("estop_release", c.estop_release, -1);
    nh_.param("speed_limit_raw", c.speed_limit_raw, -1);
    nh_.param("initial_gear", c.initial_gear, -1);
    nh_.param("steer_limit_raw", c.steer_limit_raw, 0);
    nh_.getParam("allowed_gears", c.allowed_gears);
    nh_.param("drive_timeout", c.drive_timeout, c.drive_timeout);
    nh_.param("mode_timeout", c.mode_timeout, c.mode_timeout);
    nh_.param("feedback_timeout", c.feedback_timeout, c.feedback_timeout);
    nh_.param("stationary_hold", c.stationary_hold, c.stationary_hold);
    nh_.param("manual_transition_confirmed", c.manual_transition_confirmed, false);
    nh_.param("manual_transition_timeout", c.manual_transition_timeout, c.manual_transition_timeout);
    return c;
}
void ERP42Serial::UpdateNode() {
    const auto ns = ros::this_node::getNamespace();
    feedback_pub_ = nh_.advertise<Total_msgs::SerialFeedBack>(ns + "/feedback", 1);
    command_pub_ = nh_.advertise<Total_msgs::CmdControl>(ns + "/command", 1);
    mode_sub_ = nh_.subscribe(ns + "/mode", 1, &ERP42Serial::ModeCallback, this);
    drive_sub_ = nh_.subscribe(ns + "/drive", 1, &ERP42Serial::DriveCallback, this);
    rearm_service_ = nh_.advertiseService("rearm", &ERP42Serial::Rearm, this);
}
bool ERP42Serial::Open() {
    if (!safety_.configured()) {
        ROS_ERROR("Unconfirmed ERP42 protocol/stop policy: Serial will not be opened");
        return false;
    }
    if (!serial_port_ || !serial_port_->Open()) {
        safety_.ReadFailure();
        ROS_ERROR("Serial open refused/failed (PTY-only unless explicitly enabled)");
        return false;
    }
    return true;
}
bool ERP42Serial::Read() {
    std::array<uint8_t, NUM_READ> bytes{};
    const auto status = serial_port_->ReadFrame(bytes.data(), bytes.size());
    if (status == SerialPort::ReadStatus::NoData) return false;
    Feedback f;
    if (status == SerialPort::ReadStatus::Error || !DecodeFeedback(bytes.data(), bytes.size(), f)) {
        safety_.ReadFailure();
        ROS_ERROR_THROTTLE(1, "Invalid/missing Serial frame; motion latched off");
        return false;
    }
    safety_.ReceiveFeedback(f, SafetyClock::now());
    Total_msgs::SerialFeedBack msg;
    msg.MorA = f.mode.mode; msg.EStop = f.mode.estop; msg.Gear = f.mode.gear;
    msg.speed = f.speed; msg.steer = f.steer; msg.brake = f.brake;
    msg.encoder = f.encoder; msg.alive = f.alive;
    feedback_pub_.publish(msg);
    return true;
}
void ERP42Serial::Write() {
    if (transmit_blocked_) return;
    auto bytes = safety_.Packet(SafetyClock::now(), alive_);
    const bool success = serial_port_->Write(bytes.data(), bytes.size());
    safety_.WriteResult(success);
    if (!success) {
        transmit_blocked_ = true;
        ROS_ERROR("Serial write failed; framing unknown. No automatic packet replay/reopen");
        return;
    }
    Total_msgs::CmdControl msg;
    // Publish the exact fully written packet, including watchdog overrides/alive.
    msg.MorA = bytes[3]; msg.EStop = bytes[4]; msg.Gear = bytes[5];
    msg.KPH = uint16_t(bytes[6]) * 256 + bytes[7];
    const int steer = int(bytes[8]) * 256 + bytes[9];
    msg.Deg = static_cast<int16_t>(steer >= 32768 ? steer - 65536 : steer);
    msg.brake = bytes[10]; msg.alive = bytes[11];
    ++alive_;
    command_pub_.publish(msg);
}
void ERP42Serial::Run() {
    if (!Open()) return;
    const auto period = std::chrono::milliseconds(20);
    int previous_state = -1;
    std::string previous_reason;
    while (ros::ok()) {
        const auto start = SafetyClock::now();
        ros::spinOnce();
        Read();
        safety_.Tick(SafetyClock::now());
        Write(); // A missing feedback frame does not skip the stop request.
        if (previous_state != int(safety_.state()) || previous_reason != safety_.reason()) {
            previous_state = int(safety_.state());
            previous_reason = safety_.reason();
            ROS_INFO_STREAM("ERP42 safety state " << previous_state << ": " << safety_.reason());
        }
        ROS_WARN_STREAM_THROTTLE(1, "ERP42 safety state " << int(safety_.state()) << ": " << safety_.reason());
        std::this_thread::sleep_until(start + period);
    }
    // Graceful exit only: bounded best effort, no guarantee on SIGKILL/link loss.
    safety_.Stop("process shutdown");
    Write();
    serial_port_->Close();
}
void ERP42Serial::ModeCallback(const Total_msgs::ModeCmd::ConstPtr& msg) {
    ModeCommand m;
    m.mode = msg->MorA; m.estop = msg->EStop; m.gear = msg->Gear;
    safety_.Mode(m, SafetyClock::now()); // ModeCmd.alive remains unused by contract.
}
void ERP42Serial::DriveCallback(const Total_msgs::DriveCmd::ConstPtr& msg) {
    DriveCommand d;
    d.speed = msg->KPH; d.steer = msg->Deg; d.brake = msg->brake;
    safety_.Drive(d, SafetyClock::now()); // Existing raw units, no added scaling.
    if (!safety_.drive_diagnostic().empty())
        ROS_WARN_STREAM_THROTTLE(1, safety_.drive_diagnostic());
}
bool ERP42Serial::Rearm(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    response.success = !transmit_blocked_ && safety_.Rearm(SafetyClock::now(), response.message);
    if (transmit_blocked_) response.message = "transport recovery required; framing unknown";
    return true;
}
}} // namespace unmansol::erp42
