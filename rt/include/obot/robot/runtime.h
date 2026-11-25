// The control cycle: the motors' statuses and the client's command in, the motors'
// commands and the robot's state out. Owns the robot's mode, the watchdogs and the limits.
// Does no I/O and no timing itself, so the same code runs inside the realtime thread
// (motor-realtime's MotorThread, which reads the statuses, calls update() and writes the
// commands) and in deterministic tests.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "obot/actuator/obot_driver.h"
#include "obot/control/setpoint_filter.h"
#include "obot/ipc/layout.h"
#include "obot/robot/config.h"

namespace obot::robot {

struct CycleTiming {
    int64_t now_mono_ns = 0;
    int64_t now_real_ns = 0;
    int32_t wake_latency_ns = 0;
    bool overrun = false;     // the previous cycle ran past its period
    int32_t exec_ns = 0;      // how long the previous cycle took
    int32_t rx_wait_ns = -1;  // how long this cycle waited for the statuses, -1 if unknown
};

// What a cycle exchanges with the motors, one entry per joint in configuration order.
struct MotorIo {
    const MotorStatus* statuses;  // the last status of each motor
    const bool* fresh;            // it arrived since the cycle before
    const bool* write_failed;     // the command of the cycle before could not be written
    MotorCommand* commands;       // filled in: what to send now
};

class Runtime {
 public:
    explicit Runtime(const RobotConfig& config);

    ipc::RobotInfo info() const;
    size_t num_joints() const { return joints_.size(); }

    // One cycle. command: the newest command from the mailbox, or nullptr if nothing has
    // ever been written. A status answers the command of the cycle before, so a motor
    // that is not fresh has missed one.
    void update(const CycleTiming& timing, const ipc::RobotCommand* command, const MotorIo& io, ipc::RobotState& out);

    ipc::RobotMode mode() const { return mode_; }
    // Ask for the drives to be idled from outside the command path (daemon shutdown).
    void request_shutdown() { shutdown_ = true; }

 private:
    struct Joint {
        JointConfig cfg;
        actuator::ObotDriver driver;
        control::SetpointFilter position_filter;
        control::RateLimiter velocity_limiter;
        actuator::DriveCommand sent;
        bool replied = false;  // in this cycle
        bool ever_replied = false;
        int consecutive_miss = 0;
        uint32_t rx_miss = 0;
        uint32_t fault = 0;
        bool holding = false;  // following the stale-command hold target
        float hold_q = 0;
        float last_q = 0;  // previous measured position, for odometry
        bool have_last_q = false;
    };

    bool accept_command(const ipc::RobotCommand& c) const;
    void handle_request(ipc::Request request);
    void enter_active();
    void command_joint(Joint& j, size_t index, bool fresh, float dt);
    void update_odometry();
    void take_statuses(const MotorIo& io);
    void evaluate();
    void command_motors(const ipc::RobotCommand* command, const MotorIo& io);
    void publish(ipc::RobotState& out) const;

    RobotConfig config_;
    std::vector<Joint> joints_;

    ipc::RobotMode mode_ = ipc::RobotMode::Init;
    uint32_t fault_ = 0;
    uint32_t warning_ = 0;
    uint64_t cycle_ = 0;
    uint32_t overruns_ = 0;
    CycleTiming timing_;

    ipc::RobotCommand command_{};  // last accepted command
    bool have_command_ = false;
    uint32_t last_request_id_ = 0;
    uint32_t last_estop_id_ = 0;
    bool seen_command_ = false;  // newest mailbox content examined, accepted or not
    uint64_t seen_seq_ = 0;
    bool invalid_pending_ = false;  // that content was refused
    int32_t command_age_us_ = -1;

    int clear_burst_ = 0;  // cycles left of sending clear-faults to the drives
    bool start_clear_done_ = false;
    bool shutdown_ = false;

    double odom_x_ = 0, odom_y_ = 0, odom_yaw_ = 0;
    float base_v_ = 0, base_omega_ = 0;
};

}  // namespace obot::robot
