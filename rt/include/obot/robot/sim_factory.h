// Builds the simulated actuator that corresponds to a configured joint.
#pragma once

#include <memory>

#include "obot/robot/config.h"
#include "obot/sim/devices.h"

namespace obot::robot {

std::unique_ptr<sim::Device> make_sim_device(const JointConfig& joint, const RobotConfig& config);

}  // namespace obot::robot
