#include "safety_control.h"
#include "SerialPort/serial_port.h"
#include <gtest/gtest.h>
#include <cerrno>
#include <cmath>
#include <deque>
#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

using namespace unmansol::erp42;
namespace {
SafetyTime At(double seconds) {
    return SafetyTime{} + std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(seconds));
}
// Synthetic mock values ONLY; these are not ERP42 hardware protocol claims.
SafetyConfig Config() {
    SafetyConfig c;
    c.protocol_confirmed = true;
    c.stationary_only = false; // Explicit opt-in for isolated Mock motion regressions.
    c.test_deadline_sec = 60;
    c.stop_brake = 37; c.estop_assert = 1; c.estop_release = 0;
    c.brake_min = 0; c.brake_max = 255; c.brake_release = 0;
    c.initial_gear = 1; c.allowed_gears = {0, 1, 2}; c.speed_limit_raw = 10;
    c.drive_timeout = c.mode_timeout = c.feedback_timeout = 0.25;
    c.stationary_hold = 0.04;
    return c;
}
ModeCommand Auto(uint8_t gear = 0) { ModeCommand m; m.mode = 1; m.gear = gear; return m; }
DriveCommand Motion() { DriveCommand d; d.speed = 1; return d; }
std::array<uint8_t, NUM_READ> Frame(uint16_t speed, int16_t steer, int32_t encoder, uint8_t alive = 1) {
    const uint16_t s = static_cast<uint16_t>(steer);
    const uint32_t e = static_cast<uint32_t>(encoder);
    return {{0x53, 0x54, 0x58, 1, 0, 0, uint8_t(speed), uint8_t(speed >> 8),
        uint8_t(s), uint8_t(s >> 8), 17, uint8_t(e), uint8_t(e >> 8),
        uint8_t(e >> 16), uint8_t(e >> 24), alive, 0x0d, 0x0a}};
}
class ControlTest : public ::testing::Test {
protected:
    SafetyControl c{Config(), At(0)};
    uint8_t alive = 0;
    double now = 0;
    void FeedbackAt(double time, uint16_t speed = 0, int32_t encoder = 10, ModeCommand mode = Auto()) {
        now = time; Feedback f; f.mode = mode; f.speed_raw = speed; f.speed = speed / 10.0;
        f.encoder = encoder; f.alive = ++alive; c.ReceiveFeedback(f, At(now));
    }
    void Ready() {
        c.WriteResult(true);
        c.Mode(Auto(), At(0)); c.Drive(DriveCommand{}, At(0));
        FeedbackAt(0); FeedbackAt(0.06);
        std::string reason;
        ASSERT_TRUE(c.Rearm(At(now), reason)) << reason;
        ASSERT_EQ(c.state(), SafetyControl::State::Releasing);
        FeedbackAt(0.07);
        ASSERT_EQ(c.state(), SafetyControl::State::Armed);
    }
    void IsStopped() {
        const auto p = c.Packet(At(now), 5);
        EXPECT_EQ(p[6], 0); EXPECT_EQ(p[7], 0); EXPECT_EQ(p[8], 0); EXPECT_EQ(p[9], 0);
        EXPECT_EQ(p[10], 37);
    }
};
TEST_F(ControlTest, SAFE_001_StartupBlocked) {
    c.Drive(Motion(), At(0)); IsStopped(); EXPECT_EQ(c.state(), SafetyControl::State::Locked);
    SafetyControl unknown{SafetyConfig{}, At(0)}; std::string why;
    EXPECT_FALSE(unknown.configured()); EXPECT_FALSE(unknown.Rearm(At(0), why));
}

TEST_F(ControlTest, CONTRACT_001_ReleaseMismatchBlocksMotion) {
    Ready(); auto d = Motion(); d.brake = 2; c.Drive(d, At(now));
    const auto p = c.Packet(At(now), 0);
    EXPECT_EQ(p[7], 0); EXPECT_EQ(p[10], 2);
    EXPECT_NE(c.drive_diagnostic().find("brake_release=0"), std::string::npos);
}
TEST(ContractTest, CONTRACT_002_MissingBrakePolicyBlocksConfiguration) {
    for (int which = 0; which < 4; ++which) {
        auto config = Config();
        if (which == 0) config.brake_release = -1;
        if (which == 1) config.stop_brake = -1;
        if (which == 2) config.brake_min = -1;
        if (which == 3) config.brake_max = -1;
        SafetyControl s{config, At(0)}; EXPECT_FALSE(s.configured());
        std::string why; EXPECT_FALSE(s.Rearm(At(0), why));
    }
}
TEST_F(ControlTest, CONTRACT_003_LegacyBrakeOneIsBrakeNotRelease) {
    Ready(); auto d = Motion(); d.steer = 71; d.brake = 1; c.Drive(d, At(now));
    const auto p = c.Packet(At(now), 0);
    EXPECT_EQ(p[7], 0); EXPECT_EQ(p[8], 0); EXPECT_EQ(p[9], 0);
    EXPECT_EQ(p[10], 1); EXPECT_EQ(p[4], 0); // Brake and E-Stop are independent.
    EXPECT_NE(c.drive_diagnostic().find("brake=1"), std::string::npos);
}
TEST_F(ControlTest, CONTRACT_004_BrakeAndEStopStop) {
    Ready(); auto d = Motion(); d.brake = 200; c.Drive(d, At(now));
    auto m = Auto(); m.estop = 1; c.Mode(m, At(now));
    IsStopped(); EXPECT_EQ(c.Packet(At(now), 0)[4], 1);
    EXPECT_EQ(c.state(), SafetyControl::State::Locked);
}

class ManualTest : public ControlTest {
protected:
    void SetUp() override {
        auto config = Config(); config.manual_transition_confirmed = true;
        c = SafetyControl{config, At(0)};
        Ready(); c.Drive(Motion(), At(now)); FeedbackAt(.08, 1, 11);
    }
    ModeCommand Manual() { auto m = Auto(); m.mode = 0; return m; }
    void Request() { c.Mode(Manual(), At(now)); }
    void Complete() {
        Request(); FeedbackAt(.10, 0, 12); FeedbackAt(.16, 0, 12);
        ASSERT_EQ(c.state(), SafetyControl::State::ManualTransition);
        ASSERT_EQ(c.Packet(At(now), 0)[3], 0);
        c.WriteResult(true); // A full Manual packet, then new feedback, is required.
        FeedbackAt(.17, 0, 12, Manual());
        ASSERT_EQ(c.state(), SafetyControl::State::ManualLocked);
    }
};
TEST_F(ManualTest, MANUAL_001_RequestDiscardsMotion) {
    Request(); IsStopped(); EXPECT_EQ(c.state(), SafetyControl::State::ManualTransition);
    const auto p = c.Packet(At(now), 0); EXPECT_EQ(p[3], 1); EXPECT_EQ(p[4], 0);
}
TEST_F(ManualTest, MANUAL_002_MovingCannotComplete) {
    Request(); FeedbackAt(.12, 1, 12); FeedbackAt(.18, 1, 13);
    EXPECT_EQ(c.state(), SafetyControl::State::ManualTransition);
    EXPECT_EQ(c.Packet(At(now), 0)[3], 1); IsStopped();
}
TEST_F(ManualTest, MANUAL_003_StationaryNewManualFeedbackCompletes) {
    Complete(); IsStopped(); EXPECT_EQ(c.Packet(At(now), 0)[3], 0);
}
TEST_F(ManualTest, MANUAL_004_DriveDuringTransitionDiscarded) {
    Request(); c.Drive(Motion(), At(.09)); FeedbackAt(.10, 0, 12); FeedbackAt(.16, 0, 12);
    c.Drive(Motion(), At(now)); c.WriteResult(true); FeedbackAt(.17, 0, 12, Manual());
    EXPECT_EQ(c.state(), SafetyControl::State::ManualLocked); IsStopped();
}
TEST_F(ManualTest, MANUAL_005_TimeoutNotExtendedByRepeatedRequests) {
    Request();
    for (int i=1; i<=11; ++i) {
        const double t=.08+i*.1;
        c.Drive(DriveCommand{}, At(t)); c.Mode(Manual(), At(t));
        FeedbackAt(t, 1, i+11);
    }
    EXPECT_EQ(c.state(), SafetyControl::State::Locked);
    EXPECT_EQ(c.reason(), "Manual transition timeout"); IsStopped();
}
TEST_F(ManualTest, MANUAL_006_AutoRequiresNewRearmAndDrive) {
    Complete(); now=.18; c.Mode(Auto(), At(now)); c.Drive(Motion(), At(now)); IsStopped();
    std::string why; EXPECT_FALSE(c.Rearm(At(.18), why));
    c.Drive(DriveCommand{}, At(.19)); ASSERT_TRUE(c.Rearm(At(.19), why));
    FeedbackAt(.20, 0, 12); EXPECT_EQ(c.state(), SafetyControl::State::Armed); IsStopped();
    c.Drive(Motion(), At(.21)); EXPECT_EQ(c.Packet(At(.21), 0)[7], 1);
}
TEST_F(ManualTest, MANUAL_007_EStopCannotInitiateManualRecovery) {
    auto m=Auto(); m.estop=1; c.Mode(m, At(now)); c.Mode(Manual(), At(.09));
    EXPECT_EQ(c.state(), SafetyControl::State::Locked); IsStopped();
    EXPECT_EQ(c.Packet(At(.09), 0)[4], 1);
}
TEST_F(ManualTest, MANUAL_008_StaleFeedbackCannotComplete) {
    Request(); c.Drive(DriveCommand{}, At(.30)); c.Mode(Manual(), At(.30)); c.Tick(At(.34));
    EXPECT_EQ(c.state(), SafetyControl::State::Locked);
    EXPECT_EQ(c.reason(), "feedback stale"); IsStopped();
}
TEST_F(ManualTest, MANUAL_009_PreviousMotionNeverRestored) {
    Complete(); now=.18; c.Drive(Motion(), At(now)); IsStopped();
    EXPECT_EQ(c.state(), SafetyControl::State::ManualLocked);
    c.Stop("injected fault"); now=.19; c.Drive(Motion(), At(now)); IsStopped();
}
TEST_F(ManualTest, MANUAL_010_RepeatedManualIsIdempotent) {
    Complete(); c.Mode(Manual(), At(.18)); now=.19; c.Mode(Manual(), At(now));
    EXPECT_EQ(c.state(), SafetyControl::State::ManualLocked); IsStopped();
}
TEST_F(ControlTest, RegressionManualUnconfirmedPolicyBlocks) {
    Ready(); auto m=Auto(); m.mode=0; c.Mode(m, At(now));
    EXPECT_EQ(c.state(), SafetyControl::State::Locked);
    EXPECT_EQ(c.reason(), "Manual interlock policy not confirmed"); IsStopped();
}
TEST_F(ManualTest, RegressionManualFeedbackBeforeWriteCannotComplete) {
    Request(); FeedbackAt(.10, 0, 12); FeedbackAt(.16, 0, 12, Manual());
    EXPECT_EQ(c.state(), SafetyControl::State::ManualTransition);
    FeedbackAt(.17, 0, 12, Manual()); EXPECT_EQ(c.state(), SafetyControl::State::ManualTransition);
    c.WriteResult(true); c.Tick(At(.18)); EXPECT_EQ(c.state(), SafetyControl::State::ManualTransition);
    FeedbackAt(.19, 0, 12, Manual()); EXPECT_EQ(c.state(), SafetyControl::State::ManualLocked);
}
TEST_F(ManualTest, RegressionSerialFailureCannotCompleteManual) {
    Request(); FeedbackAt(.10, 0, 12); FeedbackAt(.16, 0, 12);
    c.WriteResult(false); FeedbackAt(.17, 0, 12, Manual());
    EXPECT_EQ(c.state(), SafetyControl::State::Locked); IsStopped();
}
TEST_F(ManualTest, RegressionManualCallbackOrdersCannotRestoreMotion) {
    for (bool drive_first : {false,true}) {
        SetUp();
        if (drive_first) { c.Drive(Motion(), At(now)); Request(); }
        else { Request(); c.Drive(Motion(), At(now)); }
        EXPECT_EQ(c.state(), SafetyControl::State::ManualTransition); IsStopped();
    }
}
TEST_F(ManualTest, RegressionReadFailureCannotCompleteManual) {
    Request(); FeedbackAt(.10, 0, 12); FeedbackAt(.16, 0, 12); c.WriteResult(true);
    c.ReadFailure(); FeedbackAt(.17, 0, 12, Manual());
    EXPECT_EQ(c.state(), SafetyControl::State::Locked); IsStopped();
}
TEST_F(ManualTest, RegressionDuplicateAliveCannotConfirmManual) {
    Request(); FeedbackAt(.10, 0, 12); FeedbackAt(.16, 0, 12); c.WriteResult(true);
    Feedback f; f.mode=Manual(); f.encoder=12; f.alive=alive;
    c.ReceiveFeedback(f, At(.17));
    EXPECT_EQ(c.state(), SafetyControl::State::ManualTransition);
    f.mode.estop=1; c.ReceiveFeedback(f, At(.18));
    EXPECT_EQ(c.state(), SafetyControl::State::Locked); IsStopped();
}
TEST_F(ManualTest, RegressionManualDriveWatchdog) {
    Request(); c.Mode(Manual(), At(.30)); FeedbackAt(.30, 1, 12); c.Tick(At(.33));
    EXPECT_EQ(c.state(), SafetyControl::State::Locked); EXPECT_EQ(c.reason(), "DriveCmd timeout");
}
TEST_F(ManualTest, RegressionManualModeWatchdog) {
    Request(); c.Drive(DriveCommand{}, At(.30)); FeedbackAt(.30, 1, 12); c.Tick(At(.34));
    EXPECT_EQ(c.state(), SafetyControl::State::Locked); EXPECT_EQ(c.reason(), "ModeCmd timeout");
}
TEST(ContractTest, RegressionManualInvalidDeadlineBlocksConfig) {
    for (double timeout : {0.0, -1.0, std::nan("")}) {
        auto config=Config(); config.manual_transition_timeout=timeout;
        EXPECT_FALSE(config.Valid());
    }
}
TEST_F(ControlTest, SAFE_002_DriveOnlyBlocked) {
    c.Drive(Motion(), At(0)); c.WriteResult(true); FeedbackAt(0); FeedbackAt(0.06);
    std::string why; EXPECT_FALSE(c.Rearm(At(now), why)); IsStopped();
}
TEST_F(ControlTest, SAFE_003_ModeOnlyBlocked) {
    c.Mode(Auto(), At(0)); c.WriteResult(true); FeedbackAt(0); FeedbackAt(0.06);
    std::string why; EXPECT_FALSE(c.Rearm(At(now), why)); IsStopped();
}
TEST_F(ControlTest, SAFE_004_CallbackOrderBlocked) {
    for (bool drive_first : {false, true}) {
        SafetyControl s{Config(), At(0)};
        if (drive_first) { s.Drive(Motion(), At(0)); s.Mode(Auto(), At(0)); }
        else { s.Mode(Auto(), At(0)); s.Drive(Motion(), At(0)); }
        EXPECT_EQ(s.Packet(At(0), 0)[7], 0); EXPECT_EQ(s.state(), SafetyControl::State::Locked);
    }
}
TEST_F(ControlTest, SAFE_005_FreshLowSpeedPacket) {
    Ready(); c.Drive(Motion(), At(now));
    auto p = c.Packet(At(now), 255);
    EXPECT_EQ(p.size(), 14u); EXPECT_EQ(p[3], 1); EXPECT_EQ(p[5], 0);
    EXPECT_EQ(p[6], 0); EXPECT_EQ(p[7], 1); EXPECT_EQ(p[10], 0); EXPECT_EQ(p[11], 255);
    EXPECT_EQ(p[12], 0x0d); EXPECT_EQ(p[13], 0x0a);
}
TEST_F(ControlTest, SAFE_006_OneSecondScenarioLimit) {
    Ready(); MotionWindow window(1.0); window.Start(At(now)); const double start = now;
    for (int i = 0; i <= 100; ++i) {
        now = start + i / 100.0;
        c.Mode(Auto(), At(now)); c.Drive(Motion(), At(now)); FeedbackAt(now, 1, i + 10);
        if (window.Expired(At(now))) c.Stop("one-second scenario completed");
        if (i < 100) { EXPECT_EQ(c.Packet(At(now), 0)[7], 1); }
    }
    EXPECT_TRUE(window.Expired(At(now))); IsStopped(); EXPECT_EQ(c.state(), SafetyControl::State::Locked);
    RecordProperty("scenario_duration_seconds", "1.0 (virtual monotonic time)");
    RecordProperty("scenario_stop", "speed_raw=0 steer_raw=0 brake_raw=37 (mock only)");
}
TEST_F(ControlTest, SAFE_007_DriveTimeoutLatched) {
    Ready(); c.Drive(Motion(), At(now)); c.Mode(Auto(), At(0.2)); FeedbackAt(0.2, 1);
    now = 0.33; c.Tick(At(now)); IsStopped();
    EXPECT_EQ(c.reason(), "DriveCmd timeout"); c.Drive(Motion(), At(now)); IsStopped();
}
TEST_F(ControlTest, SAFE_008_ModeTimeoutLatched) {
    Ready(); c.Drive(Motion(), At(0.2)); FeedbackAt(0.2, 1);
    now = 0.26; c.Tick(At(now)); IsStopped(); EXPECT_EQ(c.reason(), "ModeCmd timeout");
}
TEST_F(ControlTest, SAFE_009_EStopDiscardsMotion) {
    Ready(); c.Drive(Motion(), At(now)); auto m = Auto(); m.estop = 1; c.Mode(m, At(now));
    IsStopped(); EXPECT_EQ(c.Packet(At(now), 0)[4], 1);
}
TEST_F(ControlTest, SAFE_010_EStopReleaseDoesNotRestart) {
    Ready(); c.Drive(Motion(), At(now)); auto m = Auto(); m.estop = 1; c.Mode(m, At(now));
    c.Mode(Auto(), At(now)); c.Drive(Motion(), At(now));
    IsStopped(); EXPECT_EQ(c.state(), SafetyControl::State::Locked);
    EXPECT_EQ(c.Packet(At(now), 0)[4], 1);
}
TEST_F(ControlTest, SAFE_011_WriteFailureLatches) {
    Ready(); c.Drive(Motion(), At(now)); c.WriteResult(false); IsStopped();
    c.WriteResult(true); c.Drive(Motion(), At(now)); IsStopped();
}
TEST_F(ControlTest, SAFE_012_ReadFailureLatches) {
    Ready(); c.ReadFailure(); c.Drive(Motion(), At(now)); IsStopped();
    std::string why; EXPECT_FALSE(c.Rearm(At(now), why));
}
TEST_F(ControlTest, SAFE_013_FeedbackStale) {
    Ready(); c.Drive(Motion(), At(0.2)); c.Mode(Auto(), At(0.2)); now = 0.33;
    c.Tick(At(now)); IsStopped(); EXPECT_EQ(c.reason(), "feedback stale");
}
TEST_F(ControlTest, SAFE_014_BrakeByteAndConflict) {
    Ready(); auto d = Motion(); d.steer = -71; d.brake = 19;
    c.Drive(d, At(now)); auto p = c.Packet(At(now), 0);
    EXPECT_EQ(p[10], 19); EXPECT_EQ(p[7], 0); EXPECT_EQ(p[8], 0); EXPECT_EQ(p[9], 0);
    d.brake = 255; c.Drive(d, At(now)); EXPECT_EQ(c.Packet(At(now), 0)[10], 255);
}
TEST(ProtocolTest, SAFE_015_DecimalFeedbackAndEncoder) {
    auto b = Frame(1, 0, -217); Feedback f;
    ASSERT_TRUE(DecodeFeedback(b.data(), b.size(), f));
    EXPECT_DOUBLE_EQ(f.speed, 0.1); EXPECT_EQ(f.encoder, -217);
    RecordProperty("feedback_sample_1", "speed_raw=" + std::to_string(f.speed_raw) +
        " speed_kph=" + std::to_string(f.speed) + " encoder=" + std::to_string(f.encoder));
    b = Frame(0, 0, -216); ASSERT_TRUE(DecodeFeedback(b.data(), b.size(), f));
    EXPECT_DOUBLE_EQ(f.speed, 0); EXPECT_EQ(f.encoder, -216);
    RecordProperty("feedback_sample_2", "speed_raw=" + std::to_string(f.speed_raw) +
        " speed_kph=" + std::to_string(f.speed) + " encoder=" + std::to_string(f.encoder));
}
TEST(ProtocolTest, SAFE_016_NegativeSteer) {
    auto b = Frame(0, -71, INT32_MIN); Feedback f;
    ASSERT_TRUE(DecodeFeedback(b.data(), b.size(), f)); EXPECT_DOUBLE_EQ(f.steer, -1.0);
    EXPECT_EQ(f.encoder, INT32_MIN);
    b = Frame(0, INT16_MIN, INT32_MAX); ASSERT_TRUE(DecodeFeedback(b.data(), b.size(), f));
    EXPECT_EQ(f.steer_raw, INT16_MIN); EXPECT_DOUBLE_EQ(f.steer, -32768 / 71.0);
    EXPECT_EQ(f.encoder, INT32_MAX);
}
class MockIo : public SerialIo {
public:
    std::deque<int> writes, reads;
    std::vector<uint8_t> sent, input;
    SafetyTime time{};
    bool poll_error = false, writable = true;
    int wait_step_ms = 1;
    ssize_t Read(int, void* data, size_t n) override {
        int result = reads.empty() ? int(std::min(n, input.size())) : reads.front();
        if (!reads.empty()) reads.pop_front();
        if (result < 0) { errno = -result; return -1; }
        if (size_t(result) > n || size_t(result) > input.size()) return -1;
        std::copy(input.begin(), input.begin() + result, static_cast<uint8_t*>(data));
        input.erase(input.begin(), input.begin() + result); return result;
    }
    ssize_t Write(int, const void* data, size_t n) override {
        int result = writes.empty() ? int(n) : writes.front();
        if (!writes.empty()) writes.pop_front();
        if (result < 0) { errno = -result; return -1; }
        if (size_t(result) > n) { errno = EIO; return -1; }
        const auto p = static_cast<const uint8_t*>(data);
        sent.insert(sent.end(), p, p + result); return result;
    }
    int Wait(int, short events, int) override {
        time += std::chrono::milliseconds(wait_step_ms);
        if (poll_error) { errno = EIO; return -1; }
        if (events == POLLIN) return !input.empty() || !reads.empty();
        return writable;
    }
    SafetyTime Now() const override { return time; }
};
class TransportTest : public ::testing::Test {
protected:
    int master = -1, slave = -1; char name[128]{};
    std::shared_ptr<MockIo> io{new MockIo};
    void SetUp() override { ASSERT_EQ(openpty(&master, &slave, name, nullptr, nullptr), 0); }
    void TearDown() override { if (master >= 0) close(master); if (slave >= 0) close(slave); }
};
TEST_F(TransportTest, SAFE_017_PartialWriteNoFrameReplay) {
    SerialPort port(name, 115200, 20, false, io); ASSERT_TRUE(port.Open());
    auto packet = EncodeCommand(Auto(), Motion(), 9);
    io->writes = {5, -EAGAIN, -EINTR, 9};
    ASSERT_TRUE(port.Write(packet.data(), packet.size()));
    EXPECT_EQ(io->sent, std::vector<uint8_t>(packet.begin(), packet.end()));
    io->sent.clear(); io->writes = {5, -EIO};
    EXPECT_FALSE(port.Write(packet.data(), packet.size())); EXPECT_TRUE(port.poisoned());
    EXPECT_EQ(io->sent.size(), 5u);
    EXPECT_FALSE(port.Write(packet.data(), packet.size())); EXPECT_EQ(io->sent.size(), 5u);
}
TEST_F(TransportTest, SAFE_018_GracefulStopAndForcedExitLimit) {
    SafetyControl c{Config(), At(0)}; c.Stop("graceful shutdown");
    EXPECT_EQ(c.Packet(At(0), 0)[7], 0); EXPECT_EQ(c.Packet(At(0), 0)[10], 37);
    // A real child writes to a PTY, then is SIGKILLed: no shutdown frame can run.
    pid_t child = fork(); ASSERT_GE(child, 0);
    if (child == 0) {
        SerialPort port(name, 115200);
        auto p = EncodeCommand(Auto(), Motion(), 0);
        if (!port.Open() || !port.Write(p.data(), p.size())) _exit(2);
        while (true) pause();
    }
    pollfd fd{master, POLLIN, 0};
    int ready = poll(&fd, 1, 1000);
    if (ready != 1) { kill(child, SIGKILL); waitpid(child, nullptr, 0); FAIL() << "child did not emit mock motion frame"; }
    std::array<uint8_t, NUM_WRITE> data{}; ssize_t n = read(master, data.data(), data.size());
    EXPECT_EQ(n, 14); EXPECT_EQ(data[7], 1);
    ASSERT_EQ(kill(child, SIGKILL), 0); int status = 0; ASSERT_EQ(waitpid(child, &status, 0), child);
    EXPECT_TRUE(WIFSIGNALED(status)); EXPECT_EQ(WTERMSIG(status), SIGKILL);
    fd.revents = 0; EXPECT_EQ(poll(&fd, 1, 20), 0); // No stop packet; NOT physical-stop PASS.
}
TEST_F(ControlTest, SAFE_019_GearChangeDiscardsOldSpeed) {
    Ready(); c.Drive(Motion(), At(now)); c.Mode(Auto(2), At(now));
    IsStopped(); EXPECT_EQ(c.Packet(At(now), 0)[5], 0); // Hold previously applied gear.
    EXPECT_EQ(c.state(), SafetyControl::State::Locked);
}
TEST_F(ControlTest, SAFE_020_NewCommandsNeedExplicitRearm) {
    Ready(); auto m = Auto(); m.estop = 1; c.Mode(m, At(now));
    c.Mode(Auto(), At(now)); c.Drive(Motion(), At(now));
    std::string why; EXPECT_FALSE(c.Rearm(At(now), why)); IsStopped();
    c.Drive(DriveCommand{}, At(now)); ASSERT_TRUE(c.Rearm(At(now), why));
    c.Drive(Motion(), At(now)); // Before release acknowledgement: discarded.
    FeedbackAt(0.08); EXPECT_EQ(c.state(), SafetyControl::State::Armed);
    EXPECT_EQ(c.Packet(At(now), 0)[7], 0);
    c.Drive(Motion(), At(now)); EXPECT_EQ(c.Packet(At(now), 0)[7], 1);
}
TEST_F(ControlTest, RegressionLateCommandCannotHideTimeout) {
    Ready(); c.Drive(Motion(), At(0.4)); IsStopped(); EXPECT_EQ(c.state(), SafetyControl::State::Locked);
}
TEST_F(ControlTest, RegressionRepeatedAliveDoesNotRefreshFeedback) {
    Ready(); Feedback f; f.mode = Auto(); f.encoder = 10; f.alive = alive;
    c.Mode(Auto(), At(0.2)); c.Drive(Motion(), At(0.2)); c.ReceiveFeedback(f, At(0.2));
    now = 0.33; c.Tick(At(now)); IsStopped(); EXPECT_EQ(c.reason(), "feedback stale");
}
TEST_F(ControlTest, RegressionEncoderMotionBlocksRearm) {
    c.Mode(Auto(), At(0)); c.Drive(DriveCommand{}, At(0)); c.WriteResult(true);
    FeedbackAt(0, 0, 10); FeedbackAt(0.06, 0, 11); std::string why;
    EXPECT_FALSE(c.Rearm(At(now), why));
}
TEST_F(ControlTest, RegressionFeedbackModeMismatchStops) {
    Ready(); c.Drive(Motion(), At(now)); FeedbackAt(0.08, 1, 11, Auto(2)); IsStopped();
}
TEST_F(ControlTest, RegressionDuplicateCounterEStopStillStops) {
    Ready(); c.Drive(Motion(), At(now)); Feedback f; f.mode = Auto(); f.mode.estop = 1;
    f.alive = alive; c.ReceiveFeedback(f, At(now)); IsStopped();
    EXPECT_EQ(c.state(), SafetyControl::State::Locked);
}
TEST_F(ControlTest, RegressionReleaseNeedsNewMatchingFeedback) {
    c.WriteResult(true); c.Mode(Auto(), At(0)); c.Drive(DriveCommand{}, At(0));
    FeedbackAt(0); FeedbackAt(0.06); std::string why; ASSERT_TRUE(c.Rearm(At(now), why));
    c.Drive(Motion(), At(now)); c.Tick(At(now));
    EXPECT_EQ(c.state(), SafetyControl::State::Releasing); EXPECT_EQ(c.Packet(At(now), 0)[7], 0);
    auto m = Auto(); m.estop = 1; FeedbackAt(0.07, 0, 10, m);
    EXPECT_EQ(c.state(), SafetyControl::State::Releasing);
    FeedbackAt(0.08); EXPECT_EQ(c.state(), SafetyControl::State::Armed);
    EXPECT_EQ(c.Packet(At(now), 0)[7], 0);
}
TEST_F(ControlTest, RegressionUnverifiedSteeringAndInvalidDriveStop) {
    Ready(); auto d = Motion(); d.steer = 1; c.Drive(d, At(now)); IsStopped();
    EXPECT_EQ(c.state(), SafetyControl::State::Locked);
    SafetyConfig config = Config(); config.brake_max = 40; SafetyControl s{config, At(0)};
    d = DriveCommand{}; d.brake = 41; s.Drive(d, At(0));
    EXPECT_EQ(s.state(), SafetyControl::State::Locked); EXPECT_EQ(s.reason(), "invalid/unconfirmed drive command");
}
TEST(ProtocolTest, RegressionMalformedAndShortFrames) {
    auto b = Frame(1, -71, -217); Feedback f;
    EXPECT_FALSE(DecodeFeedback(b.data(), 17, f)); EXPECT_FALSE(DecodeFeedback(nullptr, 18, f));
    b[0] = 0; EXPECT_FALSE(DecodeFeedback(b.data(), b.size(), f));
    b = Frame(1, 0, 0); b[17] = 0; EXPECT_FALSE(DecodeFeedback(b.data(), b.size(), f));
}
TEST(ProtocolTest, RegressionOutgoingEndianAndAliveWrap) {
    DriveCommand d; d.speed = 0x1234; d.steer = -71; d.brake = 19;
    auto b = EncodeCommand(Auto(2), d, 255);
    EXPECT_EQ(b[6], 0x12); EXPECT_EQ(b[7], 0x34); EXPECT_EQ(b[8], 0xff); EXPECT_EQ(b[9], 0xb9);
    uint8_t alive = 255; ++alive; EXPECT_EQ(EncodeCommand(Auto(), d, alive)[11], 0);
}
TEST_F(TransportTest, RegressionReadFragmentsAndPartialTimeout) {
    SerialPort port(name, 115200, 20, false, io); ASSERT_TRUE(port.Open());
    const auto f = Frame(1, -71, -217); io->input.assign(f.begin(), f.end()); io->reads = {8, 10};
    std::array<uint8_t, NUM_READ> packet{};
    EXPECT_EQ(port.ReadFrame(packet.data(), packet.size()), SerialPort::ReadStatus::NoData);
    EXPECT_EQ(port.ReadFrame(packet.data(), packet.size()), SerialPort::ReadStatus::Frame); EXPECT_EQ(packet, f);
    io->input = {0x53}; EXPECT_EQ(port.ReadFrame(packet.data(), packet.size()), SerialPort::ReadStatus::NoData);
    io->time += std::chrono::milliseconds(21);
    EXPECT_EQ(port.ReadFrame(packet.data(), packet.size()), SerialPort::ReadStatus::Error);
}
TEST_F(TransportTest, RegressionReadErrorAndWriteDeadline) {
    SerialPort port(name, 115200, 20, false, io); ASSERT_TRUE(port.Open());
    std::array<uint8_t, NUM_READ> packet{}; io->reads = {-EIO};
    EXPECT_EQ(port.ReadFrame(packet.data(), packet.size()), SerialPort::ReadStatus::Error);
    auto out = EncodeCommand(Auto(), Motion(), 0); io->writes = {3}; io->wait_step_ms = 11;
    EXPECT_FALSE(port.Write(out.data(), out.size())); EXPECT_TRUE(port.poisoned());
}
TEST_F(TransportTest, RegressionActualPtyRoundTrip) {
    SerialPort port(name, 115200); ASSERT_TRUE(port.Open());
    auto out = EncodeCommand(Auto(), Motion(), 7); ASSERT_TRUE(port.Write(out.data(), out.size()));
    std::array<uint8_t, NUM_WRITE> got{}; ASSERT_EQ(read(master, got.data(), got.size()), 14); EXPECT_EQ(got, out);
    auto f = Frame(1, -71, -217); ASSERT_EQ(write(master, f.data(), f.size()), 18);
    std::array<uint8_t, NUM_READ> received{}; auto status = SerialPort::ReadStatus::NoData;
    for (int i = 0; i < 20 && status == SerialPort::ReadStatus::NoData; ++i) {
        status = port.ReadFrame(received.data(), received.size()); usleep(1000);
    }
    ASSERT_EQ(status, SerialPort::ReadStatus::Frame); EXPECT_EQ(received, f);
}
TEST_F(TransportTest, RegressionClosedPortNoUndefinedBufferRead) {
    SerialPort port(name, 115200, 20, false, io);
    std::array<uint8_t, NUM_READ> packet{}; EXPECT_EQ(port.ReadFrame(packet.data(), packet.size()), SerialPort::ReadStatus::Error);
    auto out = EncodeCommand(Auto(), Motion(), 0); EXPECT_FALSE(port.Write(out.data(), out.size()));
}
TEST_F(TransportTest, RegressionPtyControllerOneSecondCycle) {
    SerialPort port(name, 115200); ASSERT_TRUE(port.Open());
    SafetyControl c{Config(), At(0)}; MotionWindow window(1.0);
    uint8_t receive_alive = 0, send_alive = 0;
    auto cycle = [&](double time, uint16_t speed, int32_t encoder, std::array<uint8_t, NUM_WRITE>& captured) {
        auto frame = Frame(speed, 0, encoder, ++receive_alive);
        if (write(master, frame.data(), frame.size()) != 18) return false;
        std::array<uint8_t, NUM_READ> received{};
        auto status = SerialPort::ReadStatus::NoData;
        for (int i = 0; i < 20 && status == SerialPort::ReadStatus::NoData; ++i) {
            status = port.ReadFrame(received.data(), received.size());
            if (status == SerialPort::ReadStatus::NoData) usleep(1000);
        }
        Feedback f;
        if (status != SerialPort::ReadStatus::Frame || !DecodeFeedback(received.data(), received.size(), f)) return false;
        c.ReceiveFeedback(f, At(time));
        auto out = c.Packet(At(time), send_alive++);
        bool success = port.Write(out.data(), out.size()); c.WriteResult(success);
        if (!success) return false;
        pollfd ready{master, POLLIN, 0};
        if (poll(&ready, 1, 100) != 1 || read(master, captured.data(), captured.size()) != 14) return false;
        return captured == out;
    };
    std::array<uint8_t, NUM_WRITE> got{};
    c.Mode(Auto(), At(0)); c.Drive(DriveCommand{}, At(0));
    ASSERT_TRUE(cycle(0, 0, 10, got)); ASSERT_TRUE(cycle(0.06, 0, 10, got));
    std::string why; ASSERT_TRUE(c.Rearm(At(0.06), why)) << why;
    ASSERT_TRUE(cycle(0.07, 0, 10, got)); ASSERT_EQ(c.state(), SafetyControl::State::Armed);
    window.Start(At(0.07));
    for (int i = 0; i <= 100; ++i) {
        const double time = 0.07 + i / 100.0;
        c.Mode(Auto(), At(time)); c.Drive(Motion(), At(time));
        if (window.Expired(At(time))) c.Stop("PTY one-second scenario completed");
        ASSERT_TRUE(cycle(time, 1, 10 + i, got));
        if (i < 100) { EXPECT_EQ(got[7], 1); }
        else { EXPECT_EQ(got[7], 0); EXPECT_EQ(got[10], 37); EXPECT_EQ(got[4], 1); }
    }
    EXPECT_EQ(c.state(), SafetyControl::State::Locked);
    RecordProperty("pty_scenario", "101 mock motion feedback/command cycles; final stop packet captured");
}
TEST(TransportGateTest, RegressionDefaultRejectsNonPtyWithoutOpeningIt) {
    // /dev/null is not a vehicle port; the path gate rejects it before open().
    SerialPort port("/dev/null", 115200);
    EXPECT_FALSE(port.Open());
}
TEST(ProtocolTest, RegressionInvalidConfigAndScenario) {
    auto c = Config(); c.estop_assert = c.estop_release; EXPECT_FALSE(c.Valid());
    c = Config(); c.stop_brake = -1; EXPECT_FALSE(c.Valid());
    c = Config(); c.drive_timeout = NAN; EXPECT_FALSE(c.Valid());
    c = Config(); c.initial_gear = 9; EXPECT_FALSE(c.Valid());
    EXPECT_THROW(MotionWindow(0), std::invalid_argument);
}
} // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// Session constraints use injected steady time; all values remain Mock-only.
class StationaryTest : public ControlTest {
protected:
    void SetUp() override {
        auto config = Config(); config.stationary_only = true; config.speed_limit_raw = 0;
        config.manual_transition_confirmed = true;
        c = SafetyControl{config, At(0)};
    }
    void ZeroPacket() {
        const auto p = c.Packet(At(now), 255);
        EXPECT_EQ(p.size(), 14u);
        for (int i=6; i<10; ++i) EXPECT_EQ(p[i], 0);
        EXPECT_EQ(c.OutputDrive().speed, 0); EXPECT_EQ(c.OutputDrive().steer, 0);
    }
    void ManualComplete() {
        Ready(); auto m=Auto(); m.mode=0; c.Mode(m, At(.08));
        FeedbackAt(.10); FeedbackAt(.16); c.WriteResult(true);
        FeedbackAt(.17, 0, 10, m);
        ASSERT_EQ(c.state(), SafetyControl::State::ManualLocked); ZeroPacket();
    }
};
TEST_F(StationaryTest, STATIONARY_001_DefaultEnabled) {
    EXPECT_TRUE(SafetyConfig{}.stationary_only); EXPECT_FALSE(SafetyConfig{}.Valid()); ZeroPacket();
}
TEST_F(StationaryTest, STATIONARY_002_PositiveSpeedRejected) {
    Ready(); c.Drive(Motion(), At(now));
    EXPECT_EQ(c.state(), SafetyControl::State::Locked);
    EXPECT_NE(c.drive_diagnostic().find("Stationary-only"), std::string::npos); ZeroPacket();
}
TEST_F(StationaryTest, STATIONARY_003_NonzeroSteeringRejected) {
    for (int16_t steer : std::vector<int16_t>{1, -1, INT16_MIN}) {
        SetUp(); Ready(); DriveCommand d; d.steer=steer; d.brake=200;
        c.Drive(d, At(now)); EXPECT_EQ(c.state(), SafetyControl::State::Locked);
        EXPECT_NE(c.drive_diagnostic().find("steer="), std::string::npos); ZeroPacket();
    }
}
TEST_F(StationaryTest, STATIONARY_004_ZeroInputAccepted) {
    Ready(); DriveCommand d; d.brake=73; c.Drive(d, At(now));
    EXPECT_EQ(c.state(), SafetyControl::State::Armed); EXPECT_EQ(c.Packet(At(now), 0)[10], 73); ZeroPacket();
}
TEST_F(StationaryTest, STATIONARY_005_ArmedZero) { Ready(); ZeroPacket(); }
TEST_F(StationaryTest, STATIONARY_006_ReleasingZero) {
    c.WriteResult(true); c.Mode(Auto(), At(0)); c.Drive(DriveCommand{}, At(0));
    FeedbackAt(0); FeedbackAt(.06); std::string why;
    ASSERT_TRUE(c.Rearm(At(now), why)); EXPECT_EQ(c.state(), SafetyControl::State::Releasing); ZeroPacket();
}
TEST_F(StationaryTest, STATIONARY_007_ManualTransitionAndLockedZero) {
    Ready(); auto m=Auto(); m.mode=0; c.Mode(m, At(.08)); now=.08;
    EXPECT_EQ(c.state(), SafetyControl::State::ManualTransition); ZeroPacket();
    FeedbackAt(.10); FeedbackAt(.16); EXPECT_EQ(c.Packet(At(now), 0)[3], 0); ZeroPacket();
    c.WriteResult(true); FeedbackAt(.17, 0, 10, m);
    EXPECT_EQ(c.state(), SafetyControl::State::ManualLocked); ZeroPacket();
}
TEST_F(StationaryTest, STATIONARY_008_EStopZero) {
    Ready(); auto m=Auto(); m.estop=1; c.Mode(m, At(now)); ZeroPacket();
    EXPECT_EQ(c.Packet(At(now), 0)[4], 1);
}
TEST_F(StationaryTest, STATIONARY_009_GearInterlock) {
    Ready(); c.Mode(Auto(2), At(now)); EXPECT_EQ(c.state(), SafetyControl::State::Locked);
    EXPECT_EQ(c.Packet(At(now), 0)[5], 0); ZeroPacket();
    auto m=Auto(255); c.Mode(m, At(now)); std::string why;
    EXPECT_FALSE(c.Rearm(At(now), why)); EXPECT_EQ(c.Packet(At(now), 0)[5], 0);
}
TEST_F(StationaryTest, STATIONARY_010_RejectedMotionNeverRestored) {
    Ready(); c.Drive(Motion(), At(now)); std::string why;
    EXPECT_FALSE(c.Rearm(At(now), why)); c.Drive(DriveCommand{}, At(now));
    ASSERT_TRUE(c.Rearm(At(now), why)); FeedbackAt(.08);
    EXPECT_EQ(c.state(), SafetyControl::State::Armed); ZeroPacket();
}
TEST_F(StationaryTest, STATIONARY_011_PacketBoundaryAndShutdown) {
    ZeroPacket(); Ready(); ZeroPacket(); c.ReadFailure(); ZeroPacket();
    c.WriteResult(false); ZeroPacket(); c.Stop("process shutdown"); ZeroPacket();
    auto p=c.Packet(At(now), 255); EXPECT_EQ(p[11], 255); EXPECT_EQ(p[12], 13); EXPECT_EQ(p[13], 10);
}
TEST(StationaryConfigTest, STATIONARY_012_DisabledUsesPositiveBound) {
    auto config=Config(); config.speed_limit_raw=0; EXPECT_FALSE(config.Valid());
    config.speed_limit_raw=1; ASSERT_TRUE(config.Valid()); SafetyControl c{config, At(0)};
    DriveCommand d; d.speed=2; c.Drive(d, At(0)); EXPECT_EQ(c.reason(), "invalid/unconfirmed drive command");
}
TEST(StationaryConfigTest, STATIONARY_013_MissingOrContradictoryPolicyBlocked) {
    for (int i=0;i<5;++i) {
        auto config=Config(); config.stationary_only=true; config.speed_limit_raw=0;
        if(i==0) config.protocol_confirmed=false;
        if(i==1) config.stop_brake=-1;
        if(i==2) config.estop_release=-1;
        if(i==3) config.initial_gear=255;
        if(i==4) config.speed_limit_raw=-1;
        EXPECT_FALSE(config.Valid());
    }
}
class DeadlineTest : public ControlTest {
protected:
    void SetUp() override {
        auto config=Config(); config.test_deadline_sec=1.0;
        c=SafetyControl{config, At(0)};
    }
    void KeepFresh(double t) {
        c.Mode(Auto(), At(t)); c.Drive(DriveCommand{}, At(t)); FeedbackAt(t);
    }
    void Expire() { for(int i=1;i<=10;++i) KeepFresh(i*.1); }
};
TEST_F(DeadlineTest, DEADLINE_001_SessionStartsAtConstruction) {
    c.Tick(At(.999)); EXPECT_FALSE(c.deadline_expired()); c.Tick(At(1)); EXPECT_TRUE(c.deadline_expired());
}
TEST_F(DeadlineTest, DEADLINE_002_ExpirationLocksMotion) {
    Ready(); c.Drive(Motion(), At(now)); EXPECT_EQ(c.Packet(At(now), 0)[7], 1);
    Expire(); EXPECT_TRUE(c.deadline_expired()); EXPECT_EQ(c.state(), SafetyControl::State::Locked); IsStopped();
}
TEST_F(DeadlineTest, DEADLINE_003_PostDeadlineDriveRejected) {
    Ready(); Expire(); c.Drive(Motion(), At(1.01)); now=1.01; IsStopped();
    EXPECT_NE(c.drive_diagnostic().find("deadline"), std::string::npos);
}
TEST_F(DeadlineTest, DEADLINE_004_PostDeadlineModeRejected) {
    Ready(); Expire(); c.Mode(Auto(2), At(1.01)); now=1.01; IsStopped(); EXPECT_EQ(c.Packet(At(now), 0)[5], 0);
}
TEST_F(DeadlineTest, DEADLINE_005_PostDeadlineRearmDenied) {
    Ready(); Expire(); KeepFresh(1.01); std::string why;
    EXPECT_FALSE(c.Rearm(At(now), why)); EXPECT_NE(why.find("deadline"), std::string::npos); IsStopped();
}
TEST_F(DeadlineTest, DEADLINE_006_RepeatedRearmNeverExtendsSession) {
    Ready();
    for(int i=1;i<10;++i) {
        double t=i*.1; KeepFresh(t); c.Stop("injected fault"); c.Drive(DriveCommand{}, At(t));
        std::string why; ASSERT_TRUE(c.Rearm(At(t), why)) << why;
    }
    KeepFresh(1); EXPECT_TRUE(c.deadline_expired()); IsStopped();
}
TEST_F(DeadlineTest, DEADLINE_007_WatchdogSimultaneousExpiration) {
    Ready(); now=1; c.Tick(At(now)); EXPECT_TRUE(c.deadline_expired());
    EXPECT_NE(c.reason().find("deadline"), std::string::npos); IsStopped();
}
TEST_F(DeadlineTest, DEADLINE_008_EStopSimultaneousExpiration) {
    Ready(); auto m=Auto(); m.estop=1; now=1; c.Mode(m, At(now));
    EXPECT_TRUE(c.deadline_expired()); IsStopped(); EXPECT_EQ(c.Packet(At(now), 0)[4], 1);
}
TEST_F(DeadlineTest, DEADLINE_009_NoRosOrWallClockDependency) {
    EXPECT_TRUE(SafetyClock::is_steady);
    // Pure core accepts only steady time; ROS /clock changes are exercised by ROS-037.
    Expire(); c.Tick(At(.1)); EXPECT_TRUE(c.deadline_expired()); std::string why;
    EXPECT_FALSE(c.Rearm(At(.1), why));
    RecordProperty("ros_clock_evidence", "Actual ROS time jumps: integration ROS-037; core has only steady_clock input");
}
TEST_F(DeadlineTest, DEADLINE_010_NewSessionStartsLocked) {
    Ready(); Expire(); auto config=Config(); config.test_deadline_sec=1;
    SafetyControl restarted{config, At(2)}; restarted.Drive(Motion(), At(2));
    EXPECT_EQ(restarted.state(), SafetyControl::State::Locked); EXPECT_EQ(restarted.Packet(At(2), 0)[7], 0);
    std::string why; EXPECT_FALSE(restarted.Rearm(At(2), why));
}
TEST(DeadlineConfigTest, DEADLINE_011_InvalidLimitsFailClosed) {
    for(double t : std::vector<double>{0.0,-1.0,std::nan(""),INFINITY,-INFINITY}) {
        auto config=Config(); config.test_deadline_sec=t; EXPECT_FALSE(config.Valid());
    }
    EXPECT_EQ(SafetyConfig{}.test_deadline_sec, -1);
}
TEST_F(DeadlineTest, DEADLINE_012_MotionWindowIndependent) {
    auto config=Config(); config.test_deadline_sec=3; c=SafetyControl{config, At(0)};
    MotionWindow window(1); window.Start(At(0));
    for(int i=0;i<=10;++i) KeepFresh(i*.1);
    EXPECT_TRUE(window.Expired(At(1))); EXPECT_FALSE(c.deadline_expired());
    c.Stop("one-second motion window"); c.Tick(At(3)); EXPECT_TRUE(c.deadline_expired());
}
TEST_F(DeadlineTest, RegressionDeadlinePrecedesManualAndTransportFaults) {
    Ready(); Expire(); c.ReadFailure(); c.WriteResult(false); c.Stop("shutdown");
    EXPECT_TRUE(c.deadline_expired()); EXPECT_NE(c.reason().find("deadline"), std::string::npos); IsStopped();
}
