#include "safety_control.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace unmansol { namespace erp42 {
namespace {
double Elapsed(SafetyTime now, SafetyTime then) {
    return std::chrono::duration<double>(now - then).count();
}
bool Byte(int n) { return n >= 0 && n <= 255; }
bool Positive(double n) { return std::isfinite(n) && n > 0; }
}

bool SafetyConfig::Valid() const {
    return protocol_confirmed && Byte(stop_brake) && Byte(brake_min) && Byte(brake_max) &&
        brake_min <= brake_max && stop_brake >= brake_min && stop_brake <= brake_max &&
        Byte(brake_release) && brake_release >= brake_min && brake_release <= brake_max &&
        stop_brake != brake_release &&
        steer_limit_raw >= 0 && steer_limit_raw <= 32767 && Byte(estop_assert) &&
        Byte(estop_release) && estop_assert != estop_release &&
        speed_limit_raw >= (stationary_only ? 0 : 1) && speed_limit_raw <= 65535 && Byte(initial_gear) &&
        std::find(allowed_gears.begin(), allowed_gears.end(), initial_gear) != allowed_gears.end() && !allowed_gears.empty() &&
        std::all_of(allowed_gears.begin(), allowed_gears.end(), Byte) &&
        Positive(drive_timeout) && Positive(mode_timeout) &&
        Positive(feedback_timeout) && Positive(stationary_hold) && Positive(manual_transition_timeout) &&
        Positive(test_deadline_sec);
}

std::array<uint8_t, NUM_WRITE> EncodeCommand(const ModeCommand& m, const DriveCommand& d, uint8_t alive) {
    const uint16_t steer = static_cast<uint16_t>(d.steer);
    return {{0x53, 0x54, 0x58, m.mode, m.estop, m.gear,
        static_cast<uint8_t>(d.speed >> 8), static_cast<uint8_t>(d.speed),
        static_cast<uint8_t>(steer >> 8), static_cast<uint8_t>(steer),
        d.brake, alive, 0x0d, 0x0a}};
}

bool DecodeFeedback(const uint8_t* b, std::size_t size, Feedback& result) {
    if (!b || size != NUM_READ || b[0] != 0x53 || b[1] != 0x54 || b[2] != 0x58 ||
        b[16] != 0x0d || b[17] != 0x0a) return false;
    Feedback f;
    f.mode.mode = b[3]; f.mode.estop = b[4]; f.mode.gear = b[5];
    f.speed_raw = static_cast<uint16_t>(uint16_t(b[6]) | (uint16_t(b[7]) << 8));
    int32_t steer = int32_t(b[8]) | (int32_t(b[9]) << 8);
    if (steer >= 32768) steer -= 65536;
    f.steer_raw = static_cast<int16_t>(steer);
    f.speed = f.speed_raw / SPEED_FACTOR;
    f.steer = f.steer_raw / STEER_FACTOR;
    f.brake = b[10];
    uint32_t encoder = uint32_t(b[11]) | (uint32_t(b[12]) << 8) |
        (uint32_t(b[13]) << 16) | (uint32_t(b[14]) << 24);
    int64_t signed_encoder = encoder;
    if (encoder >= 0x80000000u) signed_encoder -= 0x100000000LL;
    f.encoder = static_cast<int32_t>(signed_encoder);
    f.alive = b[15];
    result = f;
    return true;
}

SafetyControl::SafetyControl(const SafetyConfig& c, SafetyTime start) : config_(c), session_start_(start) {
    if (config_.Valid()) applied_mode_.gear = static_cast<uint8_t>(config_.initial_gear);
}
bool SafetyControl::Fresh(bool seen, SafetyTime last, double timeout, SafetyTime now) const {
    return seen && now >= last && Elapsed(now, last) < timeout;
}
bool SafetyControl::GearAllowed(uint8_t gear) const {
    return std::find(config_.allowed_gears.begin(), config_.allowed_gears.end(), gear) != config_.allowed_gears.end();
}
void SafetyControl::Stop(const std::string& why) {
    state_ = State::Locked;
    reason_ = deadline_expired_ ? "test deadline expired; new session requires restart and approval" : why;
    drive_ = DriveCommand{};
    zero_drive_ = false; // A fresh zero DriveCmd after the fault is required.
    manual_requested_ = manual_written_ = false;
}
void SafetyControl::Drive(const DriveCommand& d, SafetyTime now) {
    Tick(now); // Detect expiry before a late command can refresh the timestamp.
    drive_diagnostic_.clear();
    if (deadline_expired_) {
        drive_diagnostic_ = "Test deadline expired: DriveCmd rejected";
        return;
    }
    if (config_.stationary_only && (d.speed != 0 || d.steer != 0)) {
        drive_diagnostic_ = "Stationary-only violation: speed=" + std::to_string(d.speed) +
            " steer=" + std::to_string(d.steer) + "; command rejected and motion discarded";
        have_drive_ = false;
        Stop(drive_diagnostic_);
        return;
    }
    if (!config_.Valid() || d.speed > config_.speed_limit_raw ||
        d.brake < config_.brake_min || d.brake > config_.brake_max ||
        (d.brake == config_.brake_release && std::abs(int(d.steer)) > config_.steer_limit_raw)) {
        have_drive_ = false;
        Stop("invalid/unconfirmed drive command");
        return;
    }
    last_drive_ = now; have_drive_ = true;
    zero_drive_ = d.speed == 0 && d.steer == 0;
    if (d.brake != config_.brake_release && (d.speed != 0 || d.steer != 0))
        drive_diagnostic_ = "Brake contract conflict: brake=" + std::to_string(d.brake) +
            " differs from brake_release=" + std::to_string(config_.brake_release) +
            "; speed/steer suppressed (no hardware meaning inferred)";
    if (state_ != State::Armed) return; // No cached motion survives rearm.
    drive_ = d;
    // Brake takes precedence. No claim about physical brake range or force.
    if (d.brake != config_.brake_release) { drive_.speed = 0; drive_.steer = 0; }
}
void SafetyControl::Mode(const ModeCommand& m, SafetyTime now) {
    Tick(now);
    if (deadline_expired_) return;
    if (!config_.Valid() || m.mode > 1 || !GearAllowed(m.gear) ||
        (m.estop != config_.estop_release && m.estop != config_.estop_assert)) {
        have_mode_ = false; Stop("invalid/unconfirmed mode command"); return;
    }
    last_mode_ = now; have_mode_ = true; requested_mode_ = m;
    if (m.estop != config_.estop_release) { Stop("E-Stop requested"); return; }
    if (state_ == State::Armed && m.mode == 0) {
        if (!config_.manual_transition_confirmed) { Stop("Manual interlock policy not confirmed"); return; }
        if (m.gear != applied_mode_.gear || !write_healthy_ ||
            !Fresh(have_feedback_, last_feedback_, config_.feedback_timeout, now) ||
            feedback_.mode.estop != config_.estop_release) {
            Stop("Manual request requires same gear, healthy link and released feedback E-Stop"); return;
        }
        drive_ = DriveCommand{}; zero_drive_ = false;
        stopped_ = false; stopped_samples_ = 0; // Observe standstill AFTER this request.
        manual_started_ = now; manual_requested_ = manual_written_ = false;
        state_ = State::ManualTransition;
        reason_ = "Manual transition: stopping in applied Auto mode";
        return;
    }
    if (state_ == State::ManualTransition) {
        if (m.mode != 0 || m.gear != applied_mode_.gear)
            Stop("Manual transition interrupted by Auto/gear request");
        return; // Identical requests do not extend the transition deadline.
    }
    if (state_ == State::ManualLocked) {
        if (m.mode != 0 || m.gear != applied_mode_.gear)
            Stop("Manual exit requires explicit Auto rearm");
        return;
    }
    if (state_ != State::Locked && (m.gear != applied_mode_.gear || m.mode != applied_mode_.mode))
        Stop("mode/gear change requires stationary rearm");
}
void SafetyControl::ReceiveFeedback(const Feedback& f, SafetyTime now) {
    Tick(now);
    if (deadline_expired_) return;
    if (!config_.Valid() || f.mode.mode > 1 || !GearAllowed(f.mode.gear) ||
        (f.mode.estop != config_.estop_release && f.mode.estop != config_.estop_assert)) {
        ReadFailure(); return;
    }
    // An asserted E-Stop/mode mismatch is actionable even on a repeated counter.
    if (state_ == State::Armed && (f.mode.estop != config_.estop_release ||
        f.mode.mode != applied_mode_.mode || f.mode.gear != applied_mode_.gear))
        Stop("feedback mode/gear/E-Stop mismatch");
    if ((state_ == State::ManualTransition || state_ == State::ManualLocked) &&
        (f.mode.estop != config_.estop_release || f.mode.gear != applied_mode_.gear ||
         (state_ == State::ManualLocked && f.mode.mode != 0)))
        Stop("Manual feedback mode/gear/E-Stop mismatch");
    if (have_feedback_ && f.alive == feedback_.alive) return; // A repeated frame cannot refresh liveness.
    const bool unchanged = have_feedback_ && f.encoder == feedback_.encoder;
    const bool was_fresh = Fresh(have_feedback_, last_feedback_, config_.feedback_timeout, now);
    if (f.speed_raw == 0 && unchanged && stopped_ && was_fresh) ++stopped_samples_;
    else { stopped_ = f.speed_raw == 0; stopped_since_ = now; stopped_samples_ = stopped_ ? 1 : 0; }
    feedback_ = f; have_feedback_ = true; last_feedback_ = now; ++feedback_sequence_;
    Tick(now);
}
void SafetyControl::WriteResult(bool success) {
    write_healthy_ = success;
    if (!success) Stop("Serial write failure");
    else if (state_ == State::ManualTransition && manual_requested_ && !manual_written_) {
        manual_written_ = true;
        manual_feedback_sequence_ = feedback_sequence_; // Require feedback AFTER a full Manual packet write.
    }
}
void SafetyControl::ReadFailure() {
    stopped_ = false; stopped_samples_ = 0; have_feedback_ = false;
    Stop("Serial read/feedback frame failure");
}
bool SafetyControl::Stationary(SafetyTime now) const {
    return stopped_ && stopped_samples_ >= 2 && now >= stopped_since_ &&
        Elapsed(now, stopped_since_) >= config_.stationary_hold &&
        Fresh(have_feedback_, last_feedback_, config_.feedback_timeout, now);
}
void SafetyControl::Tick(SafetyTime now) {
    // This session clock is independent of input freshness, faults and rearm.
    // Check even while Locked; no public method can reset the session epoch.
    if (deadline_expired_ || (config_.Valid() &&
        (now < session_start_ || Elapsed(now, session_start_) >= config_.test_deadline_sec))) {
        deadline_expired_ = true;
        have_drive_ = have_mode_ = false;
        Stop("test deadline expired");
        return;
    }
    if (state_ == State::Locked) return;
    if (!Fresh(have_drive_, last_drive_, config_.drive_timeout, now)) { Stop("DriveCmd timeout"); return; }
    if (!Fresh(have_mode_, last_mode_, config_.mode_timeout, now)) { Stop("ModeCmd timeout"); return; }
    if (!Fresh(have_feedback_, last_feedback_, config_.feedback_timeout, now)) { Stop("feedback stale"); return; }
    if (!write_healthy_) { Stop("Serial write unhealthy"); return; }
    if (state_ == State::ManualTransition) {
        if (Elapsed(now, manual_started_) >= config_.manual_transition_timeout) {
            Stop("Manual transition timeout"); return;
        }
        if (!manual_requested_ && Stationary(now)) {
            applied_mode_.mode = 0;
            manual_requested_ = true;
            reason_ = "Manual transition: waiting for post-write Manual feedback";
        }
        if (manual_written_ && feedback_sequence_ > manual_feedback_sequence_ &&
            feedback_.mode.mode == 0 && feedback_.mode.estop == config_.estop_release && Stationary(now)) {
            state_ = State::ManualLocked;
            drive_ = DriveCommand{}; zero_drive_ = false;
            reason_ = "Manual confirmed: ROS motion locked; Auto requires fresh rearm";
        }
        return;
    }
    if (state_ == State::Releasing && feedback_sequence_ > release_feedback_sequence_ &&
        feedback_.mode.estop == config_.estop_release && feedback_.mode.mode == applied_mode_.mode &&
        feedback_.mode.gear == applied_mode_.gear && Stationary(now)) {
        state_ = State::Armed;
        drive_ = DriveCommand{};
        drive_.brake = static_cast<uint8_t>(config_.stop_brake);
        reason_ = "armed: waiting for a new DriveCmd";
    }
}
bool SafetyControl::Rearm(SafetyTime now, std::string& reason) {
    Tick(now);
    if (deadline_expired_) reason = "test deadline expired; restart and new safety approval required";
    else if (state_ != State::Locked) reason = "rearm requires Locked state";
    else if (!config_.Valid()) reason = "hardware protocol and stop policy not confirmed";
    else if (!write_healthy_) reason = "Serial write not healthy";
    else if (!Fresh(have_mode_, last_mode_, config_.mode_timeout, now) ||
        requested_mode_.mode != 1 || requested_mode_.estop != config_.estop_release) reason = "fresh Auto/release ModeCmd required";
    else if (!Fresh(have_drive_, last_drive_, config_.drive_timeout, now) || !zero_drive_) reason = "fresh zero speed/steer DriveCmd required";
    else if (!Stationary(now)) reason = "fresh stationary feedback/encoder hold required";
    else {
        applied_mode_ = requested_mode_;
        release_feedback_sequence_ = feedback_sequence_;
        state_ = State::Releasing;
        drive_ = DriveCommand{};
        reason_ = "explicit rearm: waiting for release/mode/gear feedback";
        reason = reason_;
        return true;
    }
    return false;
}
ModeCommand SafetyControl::OutputMode() const {
    ModeCommand m = applied_mode_;
    if (state_ == State::Locked && config_.Valid()) m.estop = static_cast<uint8_t>(config_.estop_assert);
    return m;
}
DriveCommand SafetyControl::OutputDrive() const {
    if (state_ == State::Armed && !deadline_expired_) {
        DriveCommand d = drive_;
        if (config_.stationary_only) { d.speed = 0; d.steer = 0; }
        return d;
    }
    DriveCommand d;
    if (config_.Valid()) d.brake = static_cast<uint8_t>(config_.stop_brake);
    return d;
}
std::array<uint8_t, NUM_WRITE> SafetyControl::Packet(SafetyTime now, uint8_t alive) {
    Tick(now);
    DriveCommand d = OutputDrive();
    // Last boundary before encoding, including startup/fault/shutdown packets.
    if (config_.stationary_only || deadline_expired_) { d.speed = 0; d.steer = 0; }
    return EncodeCommand(OutputMode(), d, alive);
}
MotionWindow::MotionWindow(double seconds) : seconds_(seconds) {
    if (!Positive(seconds)) throw std::invalid_argument("scenario duration must be positive and finite");
}
void MotionWindow::Start(SafetyTime now) { start_ = now; started_ = true; }
bool MotionWindow::Expired(SafetyTime now) const {
    return started_ && now >= start_ && Elapsed(now, start_) >= seconds_;
}
}} // namespace unmansol::erp42
