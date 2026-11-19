// Transport interface for one CAN bus. Implementations never block and never allocate
// in send() / recv(), so they can be called from the realtime thread.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "obot/can/frame.h"

namespace obot::can {

struct BusCounters {
    uint64_t tx = 0;
    uint64_t tx_dropped = 0;  // send() could not queue the frame
    uint64_t rx = 0;
    uint64_t rx_error = 0;  // error frames from the controller (bus off, error passive, ...)
};

class Bus {
 public:
    virtual ~Bus() = default;

    // Queue one frame. false if the frame could not be queued (counted in tx_dropped).
    virtual bool send(const Frame& frame) = 0;

    // Read up to max frames without blocking. Returns the number read.
    virtual int recv(Frame* out, int max) = 0;

    // Appends the file descriptors that become readable when frames are waiting. In-process
    // buses have none and are simply polled by calling recv().
    virtual void append_fds(std::vector<int>& out) const { (void)out; }

    virtual const std::string& name() const = 0;
    const BusCounters& counters() const { return counters_; }

 protected:
    BusCounters counters_;
};

}  // namespace obot::can
