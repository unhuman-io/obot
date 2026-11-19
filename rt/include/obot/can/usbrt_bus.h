// obot-controller boards connected over USB through the usb_rt kernel driver
// (/dev/usbrtN or /dev/mtrN), presented as a bus so that the same driver and control
// cycle serve USB and CAN robots. Frames use the obot CAN identifiers: a command frame
// for address A is written to the device registered as A, and each status packet read
// from that device comes back as a status frame from A.
//
// Statuses are pipelined: a read is always in flight on every device, so the status the
// control cycle sees was captured within the previous cycle and never has to be waited for.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "obot/can/bus.h"

namespace obot::can {

struct UsbRtDevice {
    uint8_t address = 0;  // the joint's id in the robot configuration
    std::string name;     // the board's name (its USB interface string), e.g. "wl"
};

class UsbRtBus : public Bus {
 public:
    // Finds each named board under /sys/class/usbmisc and opens it. Throws
    // std::runtime_error naming the boards that are missing and the ones that are present.
    UsbRtBus(std::string name, const std::vector<UsbRtDevice>& devices);

    // For tests: descriptors that are already open and deliver one packet per read.
    UsbRtBus(std::string name, const std::vector<std::pair<uint8_t, int>>& open_fds);

    ~UsbRtBus() override;
    UsbRtBus(const UsbRtBus&) = delete;
    UsbRtBus& operator=(const UsbRtBus&) = delete;

    bool send(const Frame& frame) override;
    int recv(Frame* out, int max) override;
    void append_fds(std::vector<int>& out) const override;
    const std::string& name() const override { return name_; }

    // Names of the boards the usb_rt driver currently exposes, with their device nodes.
    static std::vector<std::pair<std::string, std::string>> enumerate();

 private:
    struct Open {
        uint8_t address;
        int fd;
    };
    std::string name_;
    std::vector<Open> devices_;
};

}  // namespace obot::can
