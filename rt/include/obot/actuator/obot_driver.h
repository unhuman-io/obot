// Joint space to the obot-controller firmware's messages and back: a DriveCommand becomes
// a MotorCommand, a MotorStatus becomes a DriveStatus. No I/O happens here; how the
// messages reach the actuator (CAN FD, USB, ...) is motor-realtime's business.
#pragma once

#include <motor_messages.h>

#include "obot/actuator/driver.h"

namespace obot::actuator {

struct ObotDriverConfig {
    Transmission transmission;
    bool idle_damped = true;      // Idle: DAMPED (true) or OPEN (false)
    bool output_encoder = false;  // joint position from the output encoder instead of motor / ratio
    bool torque_sensor = false;   // torque from the drive's torque sensor instead of iq * kt * ratio
    float damped_kd = 0;          // Damped: 0 uses the firmware DAMPED mode (shorted windings),
                                  // otherwise velocity damping with this gain, N m s / rad
    float wrap = 0;               // the drive's position jumps by this much when it wraps, rad at
                                  // the joint; 0: it does not wrap. The firmware keeps its
                                  // position within +-rollover, so this is twice its rollover.
};

class ObotDriver {
 public:
    ObotDriver() = default;
    explicit ObotDriver(const ObotDriverConfig& config) : config_(config) {}

    // The command to send this cycle. `cycle` goes out as the host timestamp, which the
    // drive echoes in its status.
    void fill(const DriveCommand& command, uint32_t cycle, MotorCommand& out) const;

    // A status that has just arrived.
    void on_status(const MotorStatus& status, int64_t rx_time_ns);

    const DriveStatus& status() const { return status_; }

 private:
    ObotDriverConfig config_;
    DriveStatus status_;
    double unwrapped_ = 0;  // position with the drive's wraps taken out, joint frame before the zero offset
    float last_raw_ = 0;
    bool have_raw_ = false;
};

}  // namespace obot::actuator
