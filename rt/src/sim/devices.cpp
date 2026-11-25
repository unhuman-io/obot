#include "obot/sim/devices.h"

#include <fcntl.h>

#include <algorithm>
#include <cerrno>
#include <cmath>

namespace obot::sim {

void JointPlant::step(float tau, float dt) {
    const float acc = (tau - p_.damping * qd - p_.gravity * std::sin(q)) / p_.inertia;
    qd += acc * dt;
    q += qd * dt;
    if (q < p_.q_min) {
        q = p_.q_min;
        qd = std::max(qd, 0.0f);
    } else if (q > p_.q_max) {
        q = p_.q_max;
        qd = std::min(qd, 0.0f);
    }
}

// ---------------------------------------------------------------------------------------

ObotDeviceSim::ObotDeviceSim(const ObotDeviceParams& params) : p_(params), plant_(params.plant) {
    // J s^2 + kt kp s + kt ki with both poles at the bandwidth, in motor units.
    const float torque_per_amp = p_.torque_constant * p_.gear_ratio;
    const float w = p_.velocity_bandwidth;
    velocity_kp_ = 2.0f * w * p_.plant.inertia / torque_per_amp / p_.gear_ratio;
    velocity_ki_ = w * w * p_.plant.inertia / torque_per_amp / p_.gear_ratio;
}

void ObotDeviceSim::on_command(const MotorCommand& mc) {
    since_command_s_ = 0;
    first_command_ = true;
    host_timestamp_ = mc.host_timestamp;
    if (!safe_mode_) {
        command_ = mc;
        if (mode_ != mc.mode_desired) {
            velocity_integral_ = 0;
            mode_ = mc.mode_desired;
        }
    } else if (mc.mode_desired == CLEAR_FAULTS) {
        command_ = mc;
        first_command_ = false;
        mode_ = CLEAR_FAULTS;
    }
    if (mode_ == CLEAR_FAULTS) {
        error_ = 0;
        safe_mode_ = false;
    }
}

void ObotDeviceSim::step(float dt) {
    mcu_timestamp_ += static_cast<uint32_t>(dt * 170e6f);  // a 170 MHz timer, as on the G474
    since_command_s_ += dt;
    if (p_.host_timeout_s > 0 && first_command_ && since_command_s_ > p_.host_timeout_s) {
        error_ |= ERROR_MASK_SEQUENCE;
    }
    if ((error_ & 0x00FFFFFFu) != 0 && mode_ != CLEAR_FAULTS) {
        error_ |= 0x80000000u;
        safe_mode_ = true;
        mode_ = DAMPED;
    }

    const MotorCommand& mc = command_;
    const float motor_position = plant_.q * p_.gear_ratio;
    const float motor_velocity = plant_.qd * p_.gear_ratio;
    float iq = 0;
    switch (mode_) {
        case DAMPED:
            iq = -p_.damped_gain * motor_velocity;
            break;
        case CURRENT:
            iq = mc.current_desired;
            break;
        case STATE:
            iq = mc.state.kp * (mc.state.position_desired - motor_position) +
                 mc.state.kd * (mc.state.velocity_desired - motor_velocity) + mc.state.current_desired;
            break;
        case VELOCITY: {
            const float err = mc.velocity_desired - motor_velocity;
            velocity_integral_ =
                std::clamp(velocity_integral_ + velocity_ki_ * err * dt, -p_.current_limit, p_.current_limit);
            iq = velocity_kp_ * err + velocity_integral_ + mc.current_desired;
            break;
        }
        default:  // OPEN, CLEAR_FAULTS and everything this model does not implement
            iq = 0;
            break;
    }
    iq_desired_ = iq;
    iq_ = std::clamp(iq, -p_.current_limit, p_.current_limit);
    plant_.step(iq_ * p_.torque_constant * p_.gear_ratio, dt);
}

bool ObotDeviceSim::make_status(MotorStatus& ms) {
    if (silent_) return false;
    ms = MotorStatus{};
    ms.mcu_timestamp = mcu_timestamp_;
    ms.host_timestamp_received = host_timestamp_;
    ms.motor_position = plant_.q * p_.gear_ratio;
    if (p_.position_wrap > 0) {  // as the firmware's rollover: within half a wrap either side of zero
        ms.motor_position -= p_.position_wrap * std::round(ms.motor_position / p_.position_wrap);
    }
    ms.joint_position = plant_.q;
    ms.iq = iq_;
    ms.torque = iq_ * p_.torque_constant * p_.gear_ratio;
    ms.flags.mode = mode_;
    ms.flags.error.all = error_;
    ms.iq_desired = iq_desired_;
    ms.motor_velocity = plant_.qd * p_.gear_ratio;
    ms.joint_velocity = plant_.qd;
    // Cycle through the round robin values the host uses.
    static constexpr uint8_t kIndices[] = {MOTOR_TEMPERATURE_INDEX, BOARD_TEMPERATURE_INDEX, BUS_VOLTAGE_INDEX};
    ms.rr_data.index = kIndices[rr_index_];
    ms.rr_data.type = FLOAT;
    ms.rr_data.data = ms.rr_data.index == BUS_VOLTAGE_INDEX ? p_.bus_voltage : p_.temperature;
    rr_index_ = static_cast<uint8_t>((rr_index_ + 1) % 3);
    return true;
}

// ---------------------------------------------------------------------------------------

SimMotor::SimMotor(const std::string& name, const ObotDeviceParams& params) : device_(params) {
    name_ = name;
    dev_path_ = "sim";
    board_name_ = "simulated";
    messages_version_ = MOTOR_MESSAGES_VERSION;
    motor_txt_ = std::make_unique<SimulatedTextFile>();  // what the library asks a motor at start-up
    // Something the library can poll: always readable, like a drive that has answered.
    fd_ = ::open("/dev/zero", O_RDONLY | O_CLOEXEC);
    fd_flags_ = 0;
}

ssize_t SimMotor::write() {
    if (fail_writes_ > 0) {
        --fail_writes_;
        errno = ENOBUFS;
        return -1;
    }
    if (powered_) {
        device_.on_command(command_);
        pending_ = device_.make_status(reply_);
    }
    return sizeof(command_);
}

ssize_t SimMotor::read() {
    if (!pending_) {
        errno = EAGAIN;
        return -1;
    }
    status_ = reply_;
    pending_ = false;
    return sizeof(status_);
}

}  // namespace obot::sim
