// In-process CAN bus for tests and for the built-in simulator. Two endpoints share a pair
// of fixed-size queues: what one side sends the other side receives. Single threaded by
// design: the caller alternates between the two sides.
#pragma once

#include <array>
#include <memory>
#include <string>

#include "obot/can/bus.h"

namespace obot::can {

class FakeBusEndpoint;

class FakeBusPair {
 public:
    explicit FakeBusPair(std::string name);
    ~FakeBusPair();
    Bus& host();    // the side the robot runtime uses
    Bus& device();  // the side simulated devices use

    // Test hooks: drop every frame sent by the device side / by the host side.
    void set_drop_device_tx(bool drop) { drop_device_tx_ = drop; }
    void set_drop_host_tx(bool drop) { drop_host_tx_ = drop; }

 private:
    friend class FakeBusEndpoint;
    static constexpr int kCapacity = 256;
    struct Queue {
        std::array<Frame, kCapacity> frames;
        int head = 0;
        int count = 0;
        bool push(const Frame& f);
        bool pop(Frame& f);
    };
    Queue to_device_;
    Queue to_host_;
    bool drop_device_tx_ = false;
    bool drop_host_tx_ = false;
    std::unique_ptr<FakeBusEndpoint> host_;
    std::unique_ptr<FakeBusEndpoint> device_;
};

}  // namespace obot::can
