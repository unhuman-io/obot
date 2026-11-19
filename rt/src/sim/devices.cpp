#include "obot/sim/devices.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "obot/wire/obot_messages.h"
#include "obot/actuator/obot_driver.h"

namespace obot::sim {

using actuator::CubeMarsDriver;
namespace oc = actuator::obot_can;

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

bool ObotDeviceSim::accepts(uint32_t can_id) const {
    const uint32_t address = can_id & 0x7f;
    const uint32_t message = can_id >> 7;
    return address == p_.address && (message == oc::kCmd || message == oc::kCmdStatus || message == oc::kStatus);
}

void ObotDeviceSim::on_frame(const can::Frame& frame, can::Bus& bus) {
    const uint32_t message = frame.id >> 7;
    if (message == oc::kStatus) {
        if (frame.len == 0) send_status(bus);
        return;
    }
    MotorCommand mc{};
    std::memcpy(&mc, frame.data, std::min<size_t>(frame.len, sizeof(mc)));
    since_command_s_ = 0;
    first_command_ = true;
    host_timestamp_ = mc.host_timestamp;
    if (!safe_mode_) {
        std::memcpy(command_, &mc, sizeof(mc));
        if (mode_ != mc.mode_desired) {
            velocity_integral_ = 0;
            mode_ = mc.mode_desired;
        }
    } else if (mc.mode_desired == CLEAR_FAULTS) {
        std::memcpy(command_, &mc, sizeof(mc));
        first_command_ = false;
        mode_ = CLEAR_FAULTS;
    }
    if (mode_ == CLEAR_FAULTS) {
        error_ = 0;
        safe_mode_ = false;
    }
    if (message == oc::kCmdStatus) send_status(bus);
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

    MotorCommand mc;
    std::memcpy(&mc, command_, sizeof(mc));
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

void ObotDeviceSim::send_status(can::Bus& bus) {
    if (silent_) return;
    MotorStatusRegular ms{};
    ms.mcu_timestamp = mcu_timestamp_;
    ms.host_timestamp_received = host_timestamp_;
    ms.motor_position = plant_.q * p_.gear_ratio;
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

    can::Frame out;
    out.id = oc::can_id(oc::kStatus, p_.address);
    out.fd = true;
    out.brs = true;
    if (p_.lite_status) {
        out.set_payload(&ms, sizeof(MotorStatusLite), 48);
    } else {
        out.set_payload(&ms, sizeof(ms), 64);
    }
    bus.send(out);
}

// ---------------------------------------------------------------------------------------

CubeMarsDeviceSim::CubeMarsDeviceSim(const CubeMarsDeviceParams& params) : p_(params), plant_(params.plant) {}

void CubeMarsDeviceSim::on_frame(const can::Frame& frame, can::Bus& bus) {
    if (frame.len != 8) return;
    if (CubeMarsDriver::is_special(frame.data, CubeMarsDriver::kEnterMotorMode)) {
        motor_mode_ = true;
        p_des_ = v_des_ = kp_ = kd_ = t_ff_ = 0;
    } else if (CubeMarsDriver::is_special(frame.data, CubeMarsDriver::kExitMotorMode)) {
        motor_mode_ = false;
    } else if (motor_mode_) {
        CubeMarsDriver::unpack_mit(frame.data, p_.ranges, p_des_, v_des_, kp_, kd_, t_ff_);
    }
    const auto& r = p_.ranges;
    const uint32_t pi = CubeMarsDriver::to_uint(plant_.q, -r.p_max, r.p_max, 16);
    const uint32_t vi = CubeMarsDriver::to_uint(plant_.qd, -r.v_max, r.v_max, 12);
    const uint32_t ti = CubeMarsDriver::to_uint(tau_, -r.t_max, r.t_max, 12);
    can::Frame out;
    out.id = p_.reply_id ? p_.reply_id : p_.id;
    out.len = 8;
    out.data[0] = p_.id;
    out.data[1] = static_cast<uint8_t>(pi >> 8);
    out.data[2] = static_cast<uint8_t>(pi & 0xff);
    out.data[3] = static_cast<uint8_t>(vi >> 4);
    out.data[4] = static_cast<uint8_t>(((vi & 0xf) << 4) | (ti >> 8));
    out.data[5] = static_cast<uint8_t>(ti & 0xff);
    out.data[6] = static_cast<uint8_t>(static_cast<int8_t>(p_.temperature));
    out.data[7] = error_;
    bus.send(out);
}

void CubeMarsDeviceSim::step(float dt) {
    tau_ = 0;
    if (motor_mode_) {
        tau_ = kp_ * (p_des_ - plant_.q) + kd_ * (v_des_ - plant_.qd) + t_ff_;
        tau_ = std::clamp(tau_, -p_.ranges.t_max, p_.ranges.t_max);
    }
    plant_.step(tau_, dt);
}

// ---------------------------------------------------------------------------------------

int DeviceBus::pump() {
    can::Frame frames[32];
    int handled = 0;
    for (;;) {
        const int n = bus_.recv(frames, 32);
        if (n == 0) break;
        for (int i = 0; i < n; ++i) {
            for (auto& d : devices_) {
                if (d->accepts(frames[i].id)) {
                    d->on_frame(frames[i], bus_);
                    ++handled;
                }
            }
        }
    }
    return handled;
}

void DeviceBus::step(float dt) {
    for (auto& d : devices_) d->step(dt);
}

}  // namespace obot::sim
