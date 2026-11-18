#include "obot/actuator/cubemars_driver.h"

#include <algorithm>
#include <cstring>

namespace obot::actuator {

CubeMarsDriver::CubeMarsDriver(const CubeMarsDriverConfig& config) : config_(config) {}

uint32_t CubeMarsDriver::to_uint(float x, float lo, float hi, int bits) {
    const float clamped = std::clamp(x, lo, hi);
    const float scale = static_cast<float>((1u << bits) - 1);
    return static_cast<uint32_t>((clamped - lo) / (hi - lo) * scale + 0.5f);
}

float CubeMarsDriver::to_float(uint32_t x, float lo, float hi, int bits) {
    const float scale = static_cast<float>((1u << bits) - 1);
    return static_cast<float>(x) * (hi - lo) / scale + lo;
}

void CubeMarsDriver::pack_mit(float p, float v, float kp, float kd, float t, const CubeMarsRanges& r, uint8_t out[8]) {
    const uint32_t pi = to_uint(p, -r.p_max, r.p_max, 16);
    const uint32_t vi = to_uint(v, -r.v_max, r.v_max, 12);
    const uint32_t kpi = to_uint(kp, 0, r.kp_max, 12);
    const uint32_t kdi = to_uint(kd, 0, r.kd_max, 12);
    const uint32_t ti = to_uint(t, -r.t_max, r.t_max, 12);
    out[0] = static_cast<uint8_t>(pi >> 8);
    out[1] = static_cast<uint8_t>(pi & 0xff);
    out[2] = static_cast<uint8_t>(vi >> 4);
    out[3] = static_cast<uint8_t>(((vi & 0xf) << 4) | (kpi >> 8));
    out[4] = static_cast<uint8_t>(kpi & 0xff);
    out[5] = static_cast<uint8_t>(kdi >> 4);
    out[6] = static_cast<uint8_t>(((kdi & 0xf) << 4) | (ti >> 8));
    out[7] = static_cast<uint8_t>(ti & 0xff);
}

void CubeMarsDriver::unpack_mit(const uint8_t in[8], const CubeMarsRanges& r, float& p, float& v, float& kp, float& kd,
                                float& t) {
    const uint32_t pi = static_cast<uint32_t>(in[0]) << 8 | in[1];
    const uint32_t vi = static_cast<uint32_t>(in[2]) << 4 | in[3] >> 4;
    const uint32_t kpi = static_cast<uint32_t>(in[3] & 0xf) << 8 | in[4];
    const uint32_t kdi = static_cast<uint32_t>(in[5]) << 4 | in[6] >> 4;
    const uint32_t ti = static_cast<uint32_t>(in[6] & 0xf) << 8 | in[7];
    p = to_float(pi, -r.p_max, r.p_max, 16);
    v = to_float(vi, -r.v_max, r.v_max, 12);
    kp = to_float(kpi, 0, r.kp_max, 12);
    kd = to_float(kdi, 0, r.kd_max, 12);
    t = to_float(ti, -r.t_max, r.t_max, 12);
}

bool CubeMarsDriver::is_special(const uint8_t data[8], uint8_t which) {
    for (int i = 0; i < 7; ++i) {
        if (data[i] != 0xFF) return false;
    }
    return data[7] == which;
}

void CubeMarsDriver::make_frame(const DriveCommand& command, bool poll_only, uint32_t /*cycle*/, can::Frame& out) {
    out = can::Frame{};
    out.id = config_.id;
    out.len = 8;
    const bool idle = poll_only || command.mode == ipc::JointMode::Idle;
    if (idle) {
        // The exit frame leaves the motor unpowered and is answered with a status, so it
        // doubles as the status poll.
        std::memset(out.data, 0xFF, 7);
        out.data[7] = kExitMotorMode;
        motor_mode_ = false;
        return;
    }
    if (!motor_mode_) {
        std::memset(out.data, 0xFF, 7);
        out.data[7] = kEnterMotorMode;
        motor_mode_ = true;
        return;
    }
    const Transmission& t = config_.transmission;
    float p = 0, v = 0, kp = 0, kd = 0, tau = 0;
    switch (command.mode) {
        case ipc::JointMode::Idle:
            break;
        case ipc::JointMode::Damped:
            kd = config_.damped_kd;
            break;
        case ipc::JointMode::Position:
        case ipc::JointMode::Impedance:
            p = t.direction * (command.q + t.zero_offset);
            v = t.direction * command.qd;
            kp = command.kp;
            kd = command.kd;
            tau = t.direction * command.tau;
            break;
        case ipc::JointMode::Velocity:
            v = t.direction * command.qd;
            kd = command.kd > 0 ? command.kd : config_.damped_kd;
            tau = t.direction * command.tau;
            break;
        case ipc::JointMode::Torque:
            tau = t.direction * command.tau;
            break;
    }
    pack_mit(p, v, kp, kd, tau, config_.ranges, out.data);
}

bool CubeMarsDriver::on_frame(const can::Frame& frame) {
    if (frame.len < 8 || frame.data[0] != config_.id) return false;
    const uint8_t* d = frame.data;
    const uint32_t pi = static_cast<uint32_t>(d[1]) << 8 | d[2];
    const uint32_t vi = static_cast<uint32_t>(d[3]) << 4 | d[4] >> 4;
    const uint32_t ti = static_cast<uint32_t>(d[4] & 0xf) << 8 | d[5];
    const CubeMarsRanges& r = config_.ranges;
    const Transmission& t = config_.transmission;
    status_.q = t.direction * to_float(pi, -r.p_max, r.p_max, 16) - t.zero_offset;
    status_.qd = t.direction * to_float(vi, -r.v_max, r.v_max, 12);
    status_.tau = t.direction * to_float(ti, -r.t_max, r.t_max, 12);
    status_.temperature = static_cast<float>(static_cast<int8_t>(d[6]));
    status_.error = d[7];
    status_.fault = d[7] != 0;
    status_.rx_time_ns = frame.rx_time_ns;
    ++status_.rx_count;
    return true;
}

}  // namespace obot::actuator
