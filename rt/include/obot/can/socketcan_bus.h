// Linux SocketCAN transport: one non-blocking raw socket per interface.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "obot/can/bus.h"

namespace obot::can {

class SocketCanBus : public Bus {
 public:
    // Opens and binds the interface. Throws std::runtime_error with the reason on failure.
    // rx_ids: identifiers to receive; empty means everything on the bus.
    SocketCanBus(std::string name, const std::string& interface, bool fd_frames,
                 const std::vector<uint32_t>& rx_ids = {});
    ~SocketCanBus() override;
    SocketCanBus(const SocketCanBus&) = delete;
    SocketCanBus& operator=(const SocketCanBus&) = delete;

    bool send(const Frame& frame) override;
    int recv(Frame* out, int max) override;
    void append_fds(std::vector<int>& out) const override { out.push_back(fd_); }
    const std::string& name() const override { return name_; }

 private:
    std::string name_;
    int fd_ = -1;
};

}  // namespace obot::can
