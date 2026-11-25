// One actuator seen in joint space (radians, rad/s, N m at the joint): what the control
// cycle asks of it and what it reports. obot_driver.h turns these into the firmware's
// MotorCommand and back from its MotorStatus; motor-realtime carries those to the actuators.
#pragma once

#include <cstdint>
#include <limits>

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

// What a joint needs to know about its transmission.
struct Transmission {
    float gear_ratio = 1;       // motor turns per joint turn
    float torque_constant = 1;  // N m per A at the motor
    float direction = 1;        // +1 or -1: joint positive direction relative to the drive's
    float zero_offset = 0;      // joint angle, in the drive's frame, that the robot calls 0, rad
};

}  // namespace obot::actuator
