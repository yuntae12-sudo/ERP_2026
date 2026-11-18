#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace unmansol { namespace erp42 {

using SafetyClock = std::chrono::steady_clock;
using SafetyTime = SafetyClock::time_point;
constexpr std::size_t NUM_WRITE = 14;
constexpr std::size_t NUM_READ = 18;
constexpr double SPEED_FACTOR = 10.0;
constexpr double STEER_FACTOR = 71.0;

struct DriveCommand {
    uint16_t speed = 0;
    int16_t steer = 0;
    uint8_t brake = 0;
};
struct ModeCommand {
    uint8_t mode = 0;  // Existing code contract: Manual=0, Auto=1.
    uint8_t estop = 0;
    uint8_t gear = 1;
};
struct Feedback {
    ModeCommand mode;
    uint16_t speed_raw = 0;
    int16_t steer_raw = 0;
    double speed = 0;
    double steer = 0;
    uint8_t brake = 0;
    int32_t encoder = 0;
    uint8_t alive = 0;
};
struct SafetyConfig {
    bool stationary_only = true;
    double test_deadline_sec = -1; // Mandatory session limit; no implicit activation.
    bool protocol_confirmed = false;
    int stop_brake = -1;
    int brake_min = -1;
    int brake_max = -1;
    int brake_release = -1;
    int estop_assert = -1;
    int estop_release = -1;
    int speed_limit_raw = -1;
    int initial_gear = -1;
    int steer_limit_raw = 0; // Steering disabled until a verified bound is supplied.
    std::vector<int> allowed_gears;
    double drive_timeout = 0.25;
    double mode_timeout = 0.25;
    double feedback_timeout = 0.25;
    double stationary_hold = 0.3;
    bool manual_transition_confirmed = false; // Vehicle-specific interlock gate; Mock opt-in only until verified.
    double manual_transition_timeout = 1.0; // Software policy, not a vehicle specification.
    bool Valid() const;
};

std::array<uint8_t, NUM_WRITE> EncodeCommand(const ModeCommand&, const DriveCommand&, uint8_t alive);
bool DecodeFeedback(const uint8_t* bytes, std::size_t size, Feedback& result);

// No ROS, device access or wall clock dependency. Tests supply monotonic time.
class SafetyControl {
public:
    enum class State { Locked, Releasing, Armed, ManualTransition, ManualLocked };
    explicit SafetyControl(const SafetyConfig& config, SafetyTime session_start = SafetyClock::now());
    void Drive(const DriveCommand&, SafetyTime now);
    void Mode(const ModeCommand&, SafetyTime now);
    void ReceiveFeedback(const Feedback&, SafetyTime now);
    void WriteResult(bool success);
    void ReadFailure();
    void Stop(const std::string& reason);
    void Tick(SafetyTime now);
    bool Rearm(SafetyTime now, std::string& reason);
    std::array<uint8_t, NUM_WRITE> Packet(SafetyTime now, uint8_t alive);
    DriveCommand OutputDrive() const;
    ModeCommand OutputMode() const;
    State state() const { return state_; }
    const std::string& reason() const { return reason_; }
    const std::string& drive_diagnostic() const { return drive_diagnostic_; }
    bool configured() const { return config_.Valid(); }
    bool deadline_expired() const { return deadline_expired_; }

private:
    bool Fresh(bool seen, SafetyTime last, double timeout, SafetyTime now) const;
    bool GearAllowed(uint8_t gear) const;
    bool Stationary(SafetyTime now) const;
    SafetyConfig config_;
    SafetyTime session_start_;
    bool deadline_expired_ = false;
    State state_ = State::Locked;
    std::string reason_ = "startup: explicit rearm required";
    std::string drive_diagnostic_;
    DriveCommand drive_;
    ModeCommand requested_mode_, applied_mode_;
    Feedback feedback_;
    bool have_drive_ = false, have_mode_ = false, have_feedback_ = false;
    bool zero_drive_ = false, write_healthy_ = false, stopped_ = false;
    SafetyTime last_drive_{}, last_mode_{}, last_feedback_{}, stopped_since_{};
    uint64_t feedback_sequence_ = 0, release_feedback_sequence_ = 0;
    unsigned stopped_samples_ = 0;
    SafetyTime manual_started_{};
    bool manual_requested_ = false, manual_written_ = false;
    uint64_t manual_feedback_sequence_ = 0;
};

// A scenario limit is independent of command/feedback watchdogs.
class MotionWindow {
public:
    explicit MotionWindow(double seconds);
    void Start(SafetyTime now);
    bool Expired(SafetyTime now) const;
private:
    double seconds_;
    bool started_ = false;
    SafetyTime start_{};
};

}} // namespace unmansol::erp42
