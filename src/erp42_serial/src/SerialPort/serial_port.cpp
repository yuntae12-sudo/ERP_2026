#include "SerialPort/serial_port.h"
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <cstdlib>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace unmansol { namespace erp42 {
namespace {
class PosixIo : public SerialIo {
public:
    ssize_t Read(int fd, void* p, size_t n) override { return ::read(fd, p, n); }
    ssize_t Write(int fd, const void* p, size_t n) override { return ::write(fd, p, n); }
    int Wait(int fd, short events, int ms) override {
        pollfd p{fd, events, 0};
        int result = ::poll(&p, 1, ms);
        if (result > 0 && (p.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            errno = EIO; return -1;
        }
        return result;
    }
    SafetyTime Now() const override { return SafetyClock::now(); }
};
}
SerialPort::SerialPort(const char* name, int baud, int timeout, bool allow, std::shared_ptr<SerialIo> io)
    : device_name_(name ? name : ""), baudrate_(baud), timeout_ms_(timeout),
      allow_hardware_(allow), io_(io ? io : std::make_shared<PosixIo>()) {}
SerialPort::~SerialPort() { Close(); }

bool SerialPort::Open() {
    if (fd_ >= 0 || timeout_ms_ <= 0 || device_name_.empty()) return false;
    // Default is PTY-only. Resolve aliases so a symlink cannot bypass this gate.
    char path[4096];
    if (!::realpath(device_name_.c_str(), path)) return false;
    if (!allow_hardware_ && std::string(path).compare(0, 9, "/dev/pts/") != 0) return false;
    fd_ = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) return false;
    if (!Configure()) { Close(); return false; }
    write_poisoned_ = false; receive_size_ = 0;
    return true;
}
void SerialPort::Close() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1; receive_size_ = 0;
}
bool SerialPort::Configure() {
    termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) return false;
    ::cfmakeraw(&tty);
    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tty.c_cflag |= CS8 | CREAD | CLOCAL;
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 0;
    speed_t baud;
    if (baudrate_ == 115200) baud = B115200;
    else if (baudrate_ == 9600) baud = B9600;
    else return false;
    return ::cfsetispeed(&tty, baud) == 0 && ::cfsetospeed(&tty, baud) == 0 &&
        ::tcsetattr(fd_, TCSANOW, &tty) == 0;
}
SerialPort::ReadStatus SerialPort::ReadFrame(uint8_t* packet, size_t size) {
    if (fd_ < 0 || !packet || size != NUM_READ) return ReadStatus::Error;
    if (receive_size_ && io_->Now() - partial_since_ >= std::chrono::milliseconds(timeout_ms_)) {
        receive_size_ = 0; return ReadStatus::Error;
    }
    // One nonblocking read per loop, so callbacks/watchdogs cannot be starved.
    int ready = io_->Wait(fd_, POLLIN, 0);
    if (ready < 0) {
        if (errno == EINTR) return ReadStatus::NoData;
        receive_size_ = 0; return ReadStatus::Error;
    }
    if (!ready) return ReadStatus::NoData;
    ssize_t n = io_->Read(fd_, receive_.data() + receive_size_, NUM_READ - receive_size_);
    if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) return ReadStatus::NoData;
    if (n <= 0 || size_t(n) > NUM_READ - receive_size_) {
        receive_size_ = 0; return ReadStatus::Error;
    }
    if (!receive_size_) partial_since_ = io_->Now();
    receive_size_ += size_t(n);
    if (receive_size_ < NUM_READ) return ReadStatus::NoData;
    std::memcpy(packet, receive_.data(), NUM_READ);
    receive_size_ = 0;
    return ReadStatus::Frame;
}
bool SerialPort::Read(unsigned char* packet, const size_t& size) {
    return ReadFrame(packet, size) == ReadStatus::Frame;
}
bool SerialPort::Write(unsigned char* packet, const size_t& size) {
    if (fd_ < 0 || !packet || size != NUM_WRITE || write_poisoned_) return false;
    size_t offset = 0;
    const auto deadline = io_->Now() + std::chrono::milliseconds(timeout_ms_);
    while (offset < size && io_->Now() < deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - io_->Now()).count();
        int ready = io_->Wait(fd_, POLLOUT, std::max(1, int(remaining)));
        if (ready < 0 && errno == EINTR) continue;
        if (ready <= 0) break;
        if (io_->Now() >= deadline) break;
        ssize_t n = io_->Write(fd_, packet + offset, size - offset);
        if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
        if (n <= 0 || size_t(n) > size - offset) break;
        offset += size_t(n);
    }
    if (offset == size && io_->Now() <= deadline) return true;
    // Never replay an incomplete frame or guess a framing recovery procedure.
    // Even a zero-byte failure requires explicit transport recovery.
    write_poisoned_ = true;
    return false;
}
}} // namespace unmansol::erp42
