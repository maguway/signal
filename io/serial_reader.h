#pragma once
#include "core/types.h"
#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <fstream>

class SerialReader {
public:
    using PacketCallback = std::function<void(const DataPacket&)>;

    SerialReader();
    ~SerialReader();

    bool start(const std::string& portName, uint32_t baudRate, PacketCallback callback);
    void stop();
    bool isRunning() const { return running_; }
    uint32_t getBaudRate() const { return baudRate_; }

private:
    void runLoop();
    std::thread worker_;
    std::atomic<bool> running_ = false;
    PacketCallback onPacket_;
    std::string portName_;
    uint32_t baudRate_;
};