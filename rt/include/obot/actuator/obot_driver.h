// Driver for actuators running the obot-controller firmware (motorlib CANCommunication):
// CAN FD with bit rate switch, 11 bit id = message << 7 | address, MotorCommand in,
// MotorStatus out. See motor_messages.h and motorlib/communication/can_communication.h.
#pragma once

#include "obot/actuator/driver.h"

namespace obot::actuator {

namespace obot_can {
inline constexpr uint32_t kCmd = 0x1;          // command, no reply
inline constexpr uint32_t kCmdStatus = 0x2;    // command, reply with status
inline constexpr uint32_t kStatus = 0x3;       // status; an empty frame from the host requests one
inline constexpr uint32_t kMaxAddress = 0x7e;  // 0x7f is the enumeration broadcast
constexpr uint32_t can_id(uint32_t message, uint32_t address) {
    return message << 7 | address;
}
}  // namespace obot_can

struct ObotDriverConfig {
    uint8_t address = 1;
    Transmission transmission;
    bool idle_damped = true;      // Idle: DAMPED (true) or OPEN (false)
    bool output_encoder = false;  // joint position from the output encoder instead of motor / ratio
    bool torque_sensor = false;   // torque from the drive's torque sensor instead of iq * kt * ratio
    float damped_kd = 0;          // Damped: 0 uses the firmware DAMPED mode (shorted windings),
                                  // otherwise velocity damping with this gain, N m s / rad
};

class ObotDriver : public Driver {
 public:
    explicit ObotDriver(const ObotDriverConfig& config);

    void make_frame(const DriveCommand& command, bool poll_only, uint32_t cycle, can::Frame& out) override;
    uint32_t rx_id() const override { return obot_can::can_id(obot_can::kStatus, config_.address); }
    bool on_frame(const can::Frame& frame) override;
    unsigned tx_len() const override { return 48; }
    unsigned rx_len() const override { return 64; }
    bool uses_fd() const override { return true; }

 private:
    ObotDriverConfig config_;
    float last_q_ = 0;
    int64_t last_rx_ns_ = 0;
};

}  // namespace obot::actuator
