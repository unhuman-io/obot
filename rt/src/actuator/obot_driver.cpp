#include "obot/actuator/obot_driver.h"

namespace obot::actuator {

void ObotDriver::fill(const DriveCommand& command, uint32_t cycle, MotorCommand& mc) const {
    const Transmission& t = config_.transmission;
    // Joint quantities to motor quantities. The firmware's STATE controller works on motor
    // position in rad with gains in A/rad and A s/rad, and takes a current feed-forward.
    const float motor_per_joint = t.direction * t.gear_ratio;
    const float amp_per_nm = 1.0f / (t.torque_constant * t.gear_ratio);

    mc = MotorCommand{};
    mc.host_timestamp = cycle;
    switch (command.mode) {
        case ipc::JointMode::Idle:
            mc.mode_desired = config_.idle_damped ? DAMPED : OPEN;
            break;
        case ipc::JointMode::Damped:
            if (config_.damped_kd > 0) {
                mc.mode_desired = STATE;
                mc.state.kd = config_.damped_kd * amp_per_nm / t.gear_ratio;
            } else {
                mc.mode_desired = DAMPED;
            }
            break;
        case ipc::JointMode::Position:
        case ipc::JointMode::Impedance:
            mc.mode_desired = STATE;
            mc.state.position_desired = motor_per_joint * (command.q + t.zero_offset);
            mc.state.velocity_desired = motor_per_joint * command.qd;
            mc.state.kp = command.kp * amp_per_nm / t.gear_ratio;
            mc.state.kd = command.kd * amp_per_nm / t.gear_ratio;
            mc.state.current_desired = t.direction * command.tau * amp_per_nm;
            break;
        case ipc::JointMode::Velocity:
            mc.mode_desired = VELOCITY;
            mc.velocity_desired = motor_per_joint * command.qd;
            mc.current_desired = t.direction * command.tau * amp_per_nm;
            break;
        case ipc::JointMode::Torque:
            mc.mode_desired = CURRENT;
            mc.current_desired = t.direction * command.tau * amp_per_nm;
            break;
    }
    if (command.clear_faults) mc.mode_desired = CLEAR_FAULTS;
}

void ObotDriver::on_status(const MotorStatus& ms, int64_t rx_time_ns) {
    const Transmission& t = config_.transmission;
    status_.q = config_.output_encoder ? t.direction * ms.joint_position - t.zero_offset
                                       : t.direction * ms.motor_position / t.gear_ratio - t.zero_offset;
    status_.qd =
        config_.output_encoder ? t.direction * ms.joint_velocity : t.direction * ms.motor_velocity / t.gear_ratio;
    status_.tau =
        config_.torque_sensor ? t.direction * ms.torque : t.direction * ms.iq * t.torque_constant * t.gear_ratio;
    status_.error = ms.flags.error.all;
    // FAULT mode, or the firmware's summary fault bit.
    status_.fault = ms.flags.mode == FAULT || ms.flags.error.fault;
    if (ms.rr_data.type == FLOAT) {
        if (ms.rr_data.index == MOTOR_TEMPERATURE_INDEX) status_.temperature = ms.rr_data.data;
        if (ms.rr_data.index == BUS_VOLTAGE_INDEX) status_.bus_voltage = ms.rr_data.data;
    }
    status_.rx_time_ns = rx_time_ns;
    ++status_.rx_count;
}

}  // namespace obot::actuator
