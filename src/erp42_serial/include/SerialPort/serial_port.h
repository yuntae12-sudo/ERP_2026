#pragma once

#include "safety_control.h"
#include <memory>
#include <string>
#include <sys/types.h>

namespace unmansol { namespace erp42 {

// Injectable system calls; production uses bounded, nonblocking POSIX I/O.
class SerialIo {
public:
    virtual ~SerialIo() = default;
    virtual ssize_t Read(int fd, void* data, size_t size) = 0;
    virtual ssize_t Write(int fd, const void* data, size_t size) = 0;
    virtual int Wait(int fd, short events, int timeout_ms) = 0;
    virtual SafetyTime Now() const = 0;
};

class SerialPort {
public:
    enum class ReadStatus { NoData, Frame, Error };
    SerialPort(const char* device_name, int baudrate, int io_timeout_ms = 20,
               bool allow_hardware = false, std::shared_ptr<SerialIo> io = nullptr);
    ~SerialPort();
    bool Open();
    void Close();
    ReadStatus ReadFrame(uint8_t* packet, size_t size);
    bool Read(unsigned char* packet, const size_t& size);
    bool Write(unsigned char* packet, const size_t& size);
    bool poisoned() const { return write_poisoned_; }
private:
    bool Configure();
    std::string device_name_;
    int baudrate_, timeout_ms_, fd_ = -1;
    bool allow_hardware_, write_poisoned_ = false;
    std::shared_ptr<SerialIo> io_;
    std::array<uint8_t, NUM_READ> receive_{};
    size_t receive_size_ = 0;
    SafetyTime partial_since_{};
};
}} // namespace unmansol::erp42
