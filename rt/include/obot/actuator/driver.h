// One actuator on a CAN bus, seen in joint space (radians, rad/s, N m at the joint).
// A driver turns a joint command into the one frame sent each cycle and turns the
// actuator's reply back into a joint status. Drivers do no I/O themselves.
#pragma once

#include <cstdint>
#include <limits>

#include "obot/can/frame.h"
#include "obot/ipc/layout.h"

namespace obot::actuator {

struct DriveCommand {
    ipc::JointMode mode = ipc::JointMode::Idle;
    float q = 0;                // position target, rad
    float qd = 0;               // velocity target, rad/s
    float tau = 0;              // feed-forward torque, N m
    float kp = 0;               // N m / rad
    float kd = 0;               // N m s / rad
    bool clear_faults = false;  // ask the drive to clear its latched faults this cycle
};

struct DriveStatus {
    float q = 0;
    float qd = 0;
    float tau = 0;
    float temperature = std::numeric_limits<float>::quiet_NaN();
    float bus_voltage = std::numeric_limits<float>::quiet_NaN();
    uint32_t error = 0;      // the drive's own error word
    bool fault = false;      // the drive has stopped following commands
    int64_t rx_time_ns = 0;  // when the last status arrived, 0 if none yet
    uint64_t rx_count = 0;
};

// What a joint needs to know about its transmission, common to every driver.
struct Transmission {
    float gear_ratio = 1;       // motor turns per joint turn
    float torque_constant = 1;  // N m per A at the motor
    float direction = 1;        // +1 or -1: joint positive direction relative to the drive's
    float zero_offset = 0;      // joint angle, in the drive's frame, that the robot calls 0, rad
};

class Driver {
 public:
    virtual ~Driver() = default;

    // The frame to send this cycle. `poll_only` asks for a status without commanding
    // anything (used before the robot has taken control of the actuator).
    virtual void make_frame(const DriveCommand& command, bool poll_only, uint32_t cycle, can::Frame& out) = 0;

    // The frame from the last make_frame() could not be queued on the bus.
    virtual void send_failed() {}

    // Identifier of the status frames this driver consumes.
    virtual uint32_t rx_id() const = 0;

    // Parse a frame with id rx_id(). false if it was not a valid status from this actuator.
    virtual bool on_frame(const can::Frame& frame) = 0;

    // Worst-case payload sizes, for the bus budget.
    virtual unsigned tx_len() const = 0;
    virtual unsigned rx_len() const = 0;
    virtual bool uses_fd() const = 0;

    const DriveStatus& status() const { return status_; }

 protected:
    DriveStatus status_;
};

}  // namespace obot::actuator
