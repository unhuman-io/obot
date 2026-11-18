#include "obot/robot/sim_factory.h"

#include <algorithm>

namespace obot::robot {

std::unique_ptr<sim::Device> make_sim_device(const JointConfig& joint, const RobotConfig& config) {
    // The simulated plant lives in the drive's frame: undo the joint's direction and zero.
    sim::PlantParams plant = joint.sim_plant;
    const float dir = joint.transmission.direction;
    const float zero = joint.transmission.zero_offset;
    if (!joint.continuous) {
        const float a = dir * (joint.sim_plant.q_min + zero);
        const float b = dir * (joint.sim_plant.q_max + zero);
        plant.q_min = std::min(a, b);
        plant.q_max = std::max(a, b);
    }
    if (joint.driver == DriverKind::Obot) {
        sim::ObotDeviceParams p;
        p.address = static_cast<uint8_t>(joint.id);
        p.gear_ratio = joint.transmission.gear_ratio;
        p.torque_constant = joint.transmission.torque_constant;
        p.current_limit = joint.sim_current_limit;
        // Like a drive configured with a host timeout a little longer than the comm watchdog.
        p.host_timeout_s =
            static_cast<float>(config.period_us) * 1e-6f * static_cast<float>(config.comm_timeout_cycles * 10);
        p.plant = plant;
        auto device = std::make_unique<sim::ObotDeviceSim>(p);
        device->plant().q = dir * zero;
        return device;
    }
    sim::CubeMarsDeviceParams p;
    p.id = static_cast<uint8_t>(joint.id);
    p.reply_id = joint.reply_id;
    p.ranges = joint.cubemars;
    p.plant = plant;
    auto device = std::make_unique<sim::CubeMarsDeviceSim>(p);
    device->plant().q = dir * zero;
    return device;
}

}  // namespace obot::robot
