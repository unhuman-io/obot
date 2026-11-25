// The motors a configuration names, opened through motor-realtime in joint order.
#pragma once

#include <memory>
#include <vector>

#include "obot/robot/config.h"
#include "obot/sim/devices.h"

namespace obot::robot {

// A simulated motor for a joint: its plant lives in the drive's frame.
std::shared_ptr<sim::SimMotor> make_sim_motor(const JointConfig& joint, const RobotConfig& config);

struct OpenedMotors {
    std::vector<std::shared_ptr<Motor>> motors;  // one per joint
    std::vector<sim::SimMotor*> simulated;       // those of them that are simulated
};

// Joints on a "sim" bus get simulated motors, on a "usbrt" bus the board with their name
// (locked for this process: motor_util honours the same lock), otherwise the node with
// their id on that CAN interface. Throws std::runtime_error when a motor is not there,
// naming it and, for USB, the boards that are.
OpenedMotors open_motors(const RobotConfig& config);

}  // namespace obot::robot
