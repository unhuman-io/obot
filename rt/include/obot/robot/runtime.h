// The control cycle: commands in, frames out, statuses in, state out. Owns the robot's
// mode, the watchdogs and the limits. Does no timing and no blocking itself, so the same
// code runs under the realtime loop and under deterministic tests.
//
// A cycle is:  begin_cycle()  ->  poll_rx() one or more times  ->  end_cycle()
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "obot/actuator/driver.h"
#include "obot/can/bus.h"
#include "obot/control/setpoint_filter.h"
#include "obot/ipc/layout.h"
#include "obot/robot/config.h"

namespace obot::robot {

struct CycleTiming {
    int64_t now_mono_ns = 0;
    int64_t now_real_ns = 0;
    int32_t wake_latency_ns = 0;
    bool overrun = false;  // the previous cycle ran past its period
};

class Runtime {
 public:
    // buses[i] carries config.buses[i]. The buses must outlive the runtime.
    Runtime(const RobotConfig& config, std::vector<can::Bus*> buses);

    ipc::RobotInfo info() const;

    // command: the newest command from the mailbox, or nullptr if nothing has ever been written.
    void begin_cycle(const CycleTiming& timing, const ipc::RobotCommand* command);

    // Drain the buses. Returns the number of joints that have not replied in this cycle.
    int poll_rx(int64_t now_mono_ns);

    void end_cycle(int64_t now_mono_ns, ipc::RobotState& out);

    ipc::RobotMode mode() const { return mode_; }
    // Ask for the drives to be idled from outside the command path (daemon shutdown).
    void request_shutdown() { shutdown_ = true; }

 private:
    struct Joint {
        JointConfig cfg;
        std::unique_ptr<actuator::Driver> driver;
        can::Bus* bus = nullptr;
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

    RobotConfig config_;
    std::vector<can::Bus*> buses_;
    std::vector<Joint> joints_;
    std::vector<uint64_t> bus_rx_errors_;

    ipc::RobotMode mode_ = ipc::RobotMode::Init;
    uint32_t fault_ = 0;
    uint32_t warning_ = 0;
    uint64_t cycle_ = 0;
    uint32_t overruns_ = 0;
    CycleTiming timing_;
    int64_t last_rx_ns_ = 0;

    ipc::RobotCommand command_{};  // last accepted command
    bool have_command_ = false;
    uint32_t last_request_id_ = 0;
    uint32_t last_estop_id_ = 0;
    int32_t command_age_us_ = -1;

    int clear_burst_ = 0;  // cycles left of sending clear-faults to the drives
    bool start_clear_done_ = false;
    bool shutdown_ = false;

    double odom_x_ = 0, odom_y_ = 0, odom_yaw_ = 0;
    float base_v_ = 0, base_omega_ = 0;
};

}  // namespace obot::robot
