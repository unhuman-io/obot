#include "obot/robot/runtime.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include "obot/actuator/obot_driver.h"

namespace obot::robot {

using ipc::JointMode;
using ipc::Request;
using ipc::RobotMode;

namespace {
constexpr int kClearBurstCycles = 5;
constexpr float kVelocityTripFactor = 1.5f;
constexpr int64_t kFutureStampToleranceNs = 2'000'000;

bool is_position_mode(JointMode m) {
    return m == JointMode::Position || m == JointMode::Impedance;
}

// The joint's velocity as a starting point for a setpoint: within what the filter may command.
template <typename J>
float measured_velocity(const J& j) {
    return std::clamp(j.driver.status().qd, -j.cfg.qd_max, j.cfg.qd_max);
}
}  // namespace

Runtime::Runtime(const RobotConfig& config) : config_(config) {
    joints_.reserve(config_.joints.size());
    for (const JointConfig& jc : config_.joints) {
        Joint j;
        j.cfg = jc;
        actuator::ObotDriverConfig dc;
        dc.transmission = jc.transmission;
        dc.idle_damped = jc.idle_damped;
        dc.output_encoder = jc.output_encoder;
        dc.torque_sensor = jc.torque_sensor;
        dc.damped_kd = jc.damped_kd;
        j.driver = actuator::ObotDriver(dc);
        j.position_filter.configure(jc.qd_max, jc.qdd_max);
        joints_.push_back(std::move(j));
    }
}

ipc::RobotInfo Runtime::info() const {
    ipc::RobotInfo info{};
    info.num_joints = static_cast<uint32_t>(joints_.size());
    info.period_ns = static_cast<uint32_t>(config_.period_us) * 1000u;
    std::strncpy(info.robot_name, config_.name.c_str(), ipc::kNameLen - 1);
    for (size_t i = 0; i < joints_.size(); ++i) {
        const JointConfig& c = joints_[i].cfg;
        std::strncpy(info.joint_name[i], c.name.c_str(), ipc::kNameLen - 1);
        info.q_min[i] = c.q_min;
        info.q_max[i] = c.q_max;
        info.qd_max[i] = c.qd_max;
        info.tau_max[i] = c.tau_max;
        info.continuous[i] = c.continuous ? 1 : 0;
    }
    info.left_wheel = config_.base.present ? config_.base.left_index : -1;
    info.right_wheel = config_.base.present ? config_.base.right_index : -1;
    info.wheel_radius = config_.base.wheel_radius;
    info.track_width = config_.base.track_width;
    return info;
}

bool Runtime::accept_command(const ipc::RobotCommand& c) const {
    if (c.request > static_cast<uint32_t>(Request::Estop)) return false;
    if (c.base_mode > static_cast<uint32_t>(ipc::BaseMode::Twist)) return false;
    if (!std::isfinite(c.base_v) || !std::isfinite(c.base_omega)) return false;
    for (size_t i = 0; i < joints_.size(); ++i) {
        if (c.joint_mode[i] >= ipc::kJointModeCount) return false;
        if (!std::isfinite(c.q[i]) || !std::isfinite(c.qd[i]) || !std::isfinite(c.tau[i]) || !std::isfinite(c.kp[i]) ||
            !std::isfinite(c.kd[i])) {
            return false;
        }
    }
    return true;
}

void Runtime::enter_active() {
    for (Joint& j : joints_) {
        const actuator::DriveStatus& st = j.driver.status();
        j.position_filter.reset(st.q, measured_velocity(j));
        j.velocity_limiter.reset(0);
        j.holding = false;
        j.sent = actuator::DriveCommand{};
    }
    mode_ = RobotMode::Active;
}

void Runtime::handle_request(Request request) {
    switch (request) {
        case Request::None:
            break;
        case Request::Estop:
            fault_ |= ipc::robot_fault::kEstop;
            mode_ = RobotMode::Estop;
            break;
        case Request::ClearFaults:
            if (mode_ == RobotMode::Init) break;
            for (Joint& j : joints_) j.fault = 0;
            fault_ = 0;
            clear_burst_ = kClearBurstCycles;
            mode_ = RobotMode::Idle;
            break;
        case Request::Enable:
            if (mode_ == RobotMode::Idle && fault_ == 0 && clear_burst_ == 0 && !shutdown_) enter_active();
            break;
        case Request::Disable:
            if (mode_ == RobotMode::Active) mode_ = RobotMode::Idle;
            break;
    }
}

void Runtime::command_joint(Joint& j, size_t index, bool fresh, float dt) {
    const JointConfig& c = j.cfg;
    const actuator::DriveStatus& st = j.driver.status();
    const JointMode previous = j.sent.mode;
    actuator::DriveCommand dc;

    JointMode wanted = static_cast<JointMode>(command_.joint_mode[index]);
    float q_target = command_.q[index];
    float qd_target = command_.qd[index];
    float tau = command_.tau[index];
    float kp = wanted == JointMode::Impedance ? std::clamp(command_.kp[index], 0.0f, c.kp_max) : c.kp;
    float kd = wanted == JointMode::Impedance ? std::clamp(command_.kd[index], 0.0f, c.kd_max) : c.kd;

    const BaseConfig& base = config_.base;
    if (base.present && command_.base_mode == static_cast<uint32_t>(ipc::BaseMode::Twist) &&
        (static_cast<int>(index) == base.left_index || static_cast<int>(index) == base.right_index)) {
        const float v = std::clamp(command_.base_v, -base.v_max, base.v_max);
        const float w = std::clamp(command_.base_omega, -base.omega_max, base.omega_max);
        const float side = static_cast<int>(index) == base.left_index ? -1.0f : 1.0f;
        wanted = JointMode::Velocity;
        qd_target = (v + side * w * base.track_width * 0.5f) / base.wheel_radius;
        tau = 0;
    }

    if (!fresh) {
        // No fresh command: keep doing the safe version of what the joint was doing.
        if (is_position_mode(previous)) {
            if (!j.holding) {
                j.holding = true;
                // With the client's own (possibly zero) impedance gains the setpoint says
                // little about where the joint is: hold from the measured position instead.
                if (previous == JointMode::Impedance) j.position_filter.reset(st.q, measured_velocity(j));
                const float stop = j.position_filter.stopping_point();
                j.hold_q = c.continuous ? stop : std::clamp(stop, c.q_min, c.q_max);
            }
            dc.mode = JointMode::Position;
            dc.q = j.position_filter.update(j.hold_q, dt);
            dc.qd = j.position_filter.velocity();
            dc.kp = c.kp;
            dc.kd = c.kd;
            dc.tau = 0;
        } else if (previous == JointMode::Velocity) {
            dc.mode = JointMode::Velocity;
            dc.qd = j.velocity_limiter.update(0, c.qdd_max, dt);
            dc.kd = c.kd;
            j.position_filter.reset(st.q, measured_velocity(j));
        } else if (previous == JointMode::Torque || previous == JointMode::Damped) {
            dc.mode = JointMode::Damped;
            j.position_filter.reset(st.q, measured_velocity(j));
        } else {
            dc.mode = JointMode::Idle;
            j.position_filter.reset(st.q, measured_velocity(j));
        }
        j.sent = dc;
        return;
    }
    j.holding = false;

    switch (wanted) {
        case JointMode::Idle:
        case JointMode::Damped:
            dc.mode = wanted;
            j.position_filter.reset(st.q, measured_velocity(j));
            j.velocity_limiter.reset(0);
            break;
        case JointMode::Position:
        case JointMode::Impedance: {
            if (!is_position_mode(previous)) j.position_filter.reset(st.q, measured_velocity(j));
            const float target = c.continuous ? q_target : std::clamp(q_target, c.q_min, c.q_max);
            dc.mode = wanted;
            dc.q = j.position_filter.update(target, dt);
            dc.qd = j.position_filter.velocity();
            dc.kp = kp;
            dc.kd = kd;
            dc.tau = std::clamp(tau, -c.tau_max, c.tau_max);
            j.velocity_limiter.reset(dc.qd);
            break;
        }
        case JointMode::Velocity: {
            if (previous != JointMode::Velocity) j.velocity_limiter.reset(is_position_mode(previous) ? j.sent.qd : 0);
            float target = std::clamp(qd_target, -c.qd_max, c.qd_max);
            if (!c.continuous) {
                // Never faster towards a limit than can be braked before reaching it.
                const float up = std::max(0.0f, c.q_max - st.q);
                const float down = std::max(0.0f, st.q - c.q_min);
                target = std::min(target, std::sqrt(2.0f * c.qdd_max * up));
                target = std::max(target, -std::sqrt(2.0f * c.qdd_max * down));
            }
            dc.mode = JointMode::Velocity;
            dc.qd = j.velocity_limiter.update(target, c.qdd_max, dt);
            dc.kd = c.kd;
            dc.tau = std::clamp(tau, -c.tau_max, c.tau_max);
            j.position_filter.reset(st.q, measured_velocity(j));
            break;
        }
        case JointMode::Torque: {
            float t = std::clamp(tau, -c.tau_max, c.tau_max);
            if (!c.continuous) {
                if (st.q >= c.q_max) t = std::min(t, 0.0f);  // no pushing further out of range
                if (st.q <= c.q_min) t = std::max(t, 0.0f);
            }
            dc.mode = JointMode::Torque;
            dc.tau = t;
            j.position_filter.reset(st.q, measured_velocity(j));
            j.velocity_limiter.reset(0);
            break;
        }
    }
    j.sent = dc;
}

void Runtime::update(const CycleTiming& timing, const ipc::RobotCommand* command, const MotorIo& io,
                     ipc::RobotState& out) {
    timing_ = timing;
    ++cycle_;
    warning_ = 0;
    if (timing.overrun) {
        ++overruns_;
        warning_ |= ipc::robot_warning::kOverrun;
    }
    take_statuses(io);
    // Faults first: what is found in this cycle's statuses decides this cycle's commands.
    evaluate();
    command_motors(command, io);
    publish(out);
}

void Runtime::take_statuses(const MotorIo& io) {
    for (size_t i = 0; i < joints_.size(); ++i) {
        Joint& j = joints_[i];
        j.replied = io.fresh[i];
        if (j.replied) {
            j.driver.on_status(io.statuses[i], timing_.now_mono_ns);
            j.ever_replied = true;
        }
        if (io.write_failed[i]) warning_ |= ipc::robot_warning::kTxDropped;
    }
}

void Runtime::command_motors(const ipc::RobotCommand* command, const MotorIo& io) {
    const CycleTiming& timing = timing_;
    if (command && (!seen_command_ || command->seq != seen_seq_)) {
        seen_command_ = true;
        seen_seq_ = command->seq;
        // A stamp ahead of the clock would turn "fresh" later, with no live client behind it.
        const bool valid =
            accept_command(*command) && command->stamp_ns <= timing.now_mono_ns + kFutureStampToleranceNs;
        const bool estop = command->estop_id != last_estop_id_;
        const bool request = command->request_id != last_request_id_;
        last_estop_id_ = command->estop_id;
        last_request_id_ = command->request_id;
        if (valid) {
            command_ = *command;
            have_command_ = true;
        } else {
            warning_ |= ipc::robot_warning::kCommandInvalid;
            invalid_pending_ = true;
        }
        if (valid) invalid_pending_ = false;
        // Stopping never depends on the rest of the command being acceptable.
        if (estop) {
            handle_request(Request::Estop);  // and whatever else was asked for is dropped
        } else if (request) {
            const auto wanted = static_cast<Request>(command->request);
            if (valid || wanted == Request::Disable) handle_request(wanted);
        }
    } else if (invalid_pending_) {
        warning_ |= ipc::robot_warning::kCommandInvalid;  // the newest command is still the refused one
    }

    bool fresh = false;
    command_age_us_ = -1;
    if (have_command_) {
        const int64_t age = timing.now_mono_ns - command_.stamp_ns;
        command_age_us_ = static_cast<int32_t>(std::clamp<int64_t>(age / 1000, INT32_MIN, INT32_MAX));
        fresh = age >= -kFutureStampToleranceNs && age <= static_cast<int64_t>(config_.command_timeout_ms) * 1'000'000;
    }
    if (shutdown_ && mode_ == RobotMode::Active) mode_ = RobotMode::Idle;
    if (mode_ == RobotMode::Active && !fresh) warning_ |= ipc::robot_warning::kCommandStale;

    const float dt = static_cast<float>(config_.period_us) * 1e-6f;
    const bool clearing = clear_burst_ > 0;
    for (size_t i = 0; i < joints_.size(); ++i) {
        Joint& j = joints_[i];
        if (mode_ == RobotMode::Active) {
            command_joint(j, i, fresh, dt);
        } else {
            j.sent = actuator::DriveCommand{};  // idle; also what asks a drive for its first status
        }
        j.sent.clear_faults = clearing;
        j.driver.fill(j.sent, static_cast<uint32_t>(cycle_), io.commands[i]);
    }
    if (clearing) --clear_burst_;
}

void Runtime::update_odometry() {
    const BaseConfig& base = config_.base;
    Joint& left = joints_[static_cast<size_t>(base.left_index)];
    Joint& right = joints_[static_cast<size_t>(base.right_index)];
    const float dt = static_cast<float>(config_.period_us) * 1e-6f;
    const auto wheel_delta = [dt](Joint& j) {
        const actuator::DriveStatus& st = j.driver.status();
        float d = st.qd * dt;
        if (j.have_last_q) {
            const float measured = st.q - j.last_q;
            // A jump larger than the wheel can turn is the drive's position rolling over.
            if (std::fabs(measured) < 4.0f * j.cfg.qd_max * dt + 0.01f) d = measured;
        }
        j.last_q = st.q;
        j.have_last_q = true;
        return d;
    };
    if (!left.replied || !right.replied) return;
    const double dl = static_cast<double>(wheel_delta(left)) * base.wheel_radius;
    const double dr = static_cast<double>(wheel_delta(right)) * base.wheel_radius;
    const double d = 0.5 * (dl + dr);
    const double dyaw = (dr - dl) / base.track_width;
    odom_x_ += d * std::cos(odom_yaw_ + 0.5 * dyaw);
    odom_y_ += d * std::sin(odom_yaw_ + 0.5 * dyaw);
    odom_yaw_ += dyaw;
    const float vl = left.driver.status().qd * base.wheel_radius;
    const float vr = right.driver.status().qd * base.wheel_radius;
    base_v_ = 0.5f * (vl + vr);
    base_omega_ = (vr - vl) / base.track_width;
}

void Runtime::evaluate() {
    bool all_replied = true;
    bool any_drive_fault = false;
    for (Joint& j : joints_) {
        const actuator::DriveStatus& st = j.driver.status();
        if (j.replied) {
            j.consecutive_miss = 0;
        } else {
            all_replied = false;
            ++j.consecutive_miss;
            if (mode_ != RobotMode::Init) ++j.rx_miss;
        }
        any_drive_fault = any_drive_fault || st.fault;
        if (mode_ == RobotMode::Init) continue;

        if (j.consecutive_miss > config_.comm_timeout_cycles) j.fault |= ipc::joint_fault::kCommTimeout;
        if (st.fault && clear_burst_ == 0) j.fault |= ipc::joint_fault::kDrive;
        if (st.temperature > j.cfg.temperature_max) j.fault |= ipc::joint_fault::kTemperature;
        if (mode_ == RobotMode::Active) {
            if (!j.cfg.continuous &&
                (st.q < j.cfg.q_min - j.cfg.limit_margin || st.q > j.cfg.q_max + j.cfg.limit_margin)) {
                j.fault |= ipc::joint_fault::kPositionLimit;
            }
            if (std::fabs(st.qd) > kVelocityTripFactor * j.cfg.qd_max) j.fault |= ipc::joint_fault::kVelocityLimit;
        }
        if (j.fault) fault_ |= ipc::robot_fault::kJoint;
    }

    if (mode_ == RobotMode::Init) {
        if (all_replied && clear_burst_ == 0) {
            if (config_.clear_drive_faults_on_start && !start_clear_done_ && any_drive_fault) {
                // Drives that timed out while no host was talking to them are faulted; clear once.
                start_clear_done_ = true;
                clear_burst_ = kClearBurstCycles;
            } else {
                mode_ = RobotMode::Idle;
            }
        }
    } else if (fault_ != 0 && (mode_ == RobotMode::Idle || mode_ == RobotMode::Active)) {
        mode_ = (fault_ & ipc::robot_fault::kEstop) ? RobotMode::Estop : RobotMode::Fault;
    }

    if (config_.base.present) update_odometry();
}

void Runtime::publish(ipc::RobotState& out) const {
    const int64_t now_mono_ns = timing_.now_mono_ns;
    out = ipc::RobotState{};
    out.cycle = cycle_;
    out.t_mono_ns = timing_.now_mono_ns;
    out.t_real_ns = timing_.now_real_ns;
    out.mode = static_cast<uint32_t>(mode_);
    out.fault = fault_;
    out.warning = warning_;
    out.num_joints = static_cast<uint32_t>(joints_.size());
    out.command_seq = have_command_ ? command_.seq : 0;
    out.command_source = have_command_ ? command_.source : 0;
    out.command_age_us = command_age_us_;
    out.wake_latency_ns = timing_.wake_latency_ns;
    out.exec_ns = timing_.exec_ns;
    out.rx_wait_ns = timing_.rx_wait_ns;
    out.overruns = overruns_;
    out.odom_x = odom_x_;
    out.odom_y = odom_y_;
    out.odom_yaw = odom_yaw_;
    out.base_v = base_v_;
    out.base_omega = base_omega_;
    for (size_t i = 0; i < joints_.size(); ++i) {
        const Joint& j = joints_[i];
        const actuator::DriveStatus& st = j.driver.status();
        out.joint_mode[i] = static_cast<uint8_t>(j.sent.mode);
        out.joint_fault[i] = j.fault;
        out.drive_error[i] = st.error;
        out.rx_miss[i] = j.rx_miss;
        out.rx_age_us[i] = st.rx_time_ns ? static_cast<int32_t>((now_mono_ns - st.rx_time_ns) / 1000) : -1;
        out.q[i] = st.q;
        out.qd[i] = st.qd;
        out.tau[i] = st.tau;
        out.q_cmd[i] = j.sent.q;
        out.qd_cmd[i] = j.sent.qd;
        out.tau_cmd[i] = j.sent.tau;
        out.kp_cmd[i] = j.sent.kp;
        out.kd_cmd[i] = j.sent.kd;
        out.temperature[i] = st.temperature;
        out.bus_voltage[i] = st.bus_voltage;
    }
}

}  // namespace obot::robot
