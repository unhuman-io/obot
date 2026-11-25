#include "obot/robot/motor_factory.h"

#include <algorithm>

#include "obot/actuator/motors.h"

namespace obot::robot {

std::shared_ptr<sim::SimMotor> make_sim_motor(const JointConfig& joint, const RobotConfig& config) {
    // The simulated plant lives in the drive's frame: undo the joint's direction and zero.
    sim::ObotDeviceParams p;
    p.plant = joint.sim_plant;
    const float dir = joint.transmission.direction;
    const float zero = joint.transmission.zero_offset;
    if (!joint.continuous) {
        const float a = dir * (joint.sim_plant.q_min + zero);
        const float b = dir * (joint.sim_plant.q_max + zero);
        p.plant.q_min = std::min(a, b);
        p.plant.q_max = std::max(a, b);
    }
    p.gear_ratio = joint.transmission.gear_ratio;
    p.torque_constant = joint.transmission.torque_constant;
    p.current_limit = joint.sim_current_limit;
    // Like a drive configured with a host timeout a little longer than the comm watchdog.
    p.host_timeout_s =
        static_cast<float>(config.period_us) * 1e-6f * static_cast<float>(config.comm_timeout_cycles * 10);
    auto motor = std::make_shared<sim::SimMotor>(joint.name, p);
    motor->device().plant().q = dir * zero;
    return motor;
}

OpenedMotors open_motors(const RobotConfig& config) {
    OpenedMotors out;
    for (const JointConfig& joint : config.joints) {
        const BusConfig& bus = config.buses[static_cast<size_t>(joint.bus_index)];
        if (bus.interface == "sim") {
            auto motor = make_sim_motor(joint, config);
            out.simulated.push_back(motor.get());
            out.motors.push_back(std::move(motor));
        } else if (bus.interface == "usbrt") {
            out.motors.push_back(actuator::open_usb_motor(joint.usb_name));
        } else {
            out.motors.push_back(actuator::open_can_motor(bus.interface, joint.id));
        }
    }
    return out;
}

}  // namespace obot::robot
