// The control cycle against simulated motors, in the order motor-realtime's MotorThread
// runs it (read the statuses, update, write the commands) and through its MotorManager,
// but with a simulated clock: every test is deterministic and runs faster than real time.
#include "doctest.h"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <motor_manager.h>

#include "obot/ipc/shm_region.h"
#include "obot/robot/config.h"
#include "obot/robot/motor_factory.h"
#include "obot/robot/runtime.h"
#include "obot/sim/devices.h"

using namespace obot;
using ipc::JointMode;
using ipc::Request;
using ipc::RobotMode;

namespace {

// Two arm joints and a two wheel base.
const char* kRobot = R"(
[robot]
name = "rig"
period_us = 1000
command_timeout_ms = 50
comm_timeout_cycles = 5
max_bus_load = 2.0

[[bus]]
name = "fd"
interface = "sim"

[[joint]]
name = "shoulder"
bus = "fd"
id = 1
gear_ratio = 9
torque_constant = 0.1
q_min = -1.0
q_max = 1.5
qd_max = 2.0
qdd_max = 10.0
tau_max = 20.0
kp = 200.0
kd = 8.0
sim = { inertia = 0.2, damping = 0.2 }

[[joint]]
name = "wrist"
bus = "fd"
id = 4
direction = -1
q_min = -1.5
q_max = 1.5
qd_max = 3.0
qdd_max = 20.0
tau_max = 5.0
kp = 30.0
kd = 1.0
sim = { inertia = 0.01, damping = 0.05 }

[[joint]]
name = "left_wheel"
bus = "fd"
id = 2
torque_constant = 0.5
continuous = true
wrap = 12.5663706
qd_max = 10.0
qdd_max = 20.0
tau_max = 10.0
sim = { inertia = 0.05, damping = 0.02 }

[[joint]]
name = "right_wheel"
bus = "fd"
id = 3
torque_constant = 0.5
direction = -1
continuous = true
wrap = 12.5663706
qd_max = 10.0
qdd_max = 20.0
tau_max = 10.0
sim = { inertia = 0.05, damping = 0.02 }

[base]
left = "left_wheel"
right = "right_wheel"
wheel_radius = 0.1
track_width = 0.4
v_max = 0.8
omega_max = 2.0
)";

enum { kShoulder = 0, kWrist = 1, kLeft = 2, kRight = 3 };

struct Rig {
    robot::RobotConfig config;
    std::vector<sim::SimMotor*> sims;  // one per joint
    MotorManager manager;
    std::unique_ptr<robot::Runtime> runtime;
    ipc::RobotCommand command{};
    bool have_command = false;
    ipc::RobotState state{};
    int64_t now = 1'000'000'000;
    bool devices_powered = true;
    bool fresh[ipc::kMaxJoints] = {};
    bool write_failed[ipc::kMaxJoints] = {};

    explicit Rig(const char* toml = kRobot) : config(robot::parse_config(toml)) {
        robot::OpenedMotors opened = robot::open_motors(config);
        sims = opened.simulated;
        REQUIRE(sims.size() == config.joints.size());
        for (sim::SimMotor* m : sims) m->step(0.001f);  // the drives have been running for a while
        // What MotorThread's fault tolerant mode sets up.
        manager.set_motors(opened.motors);
        manager.set_throw_on_error(false);
        for (auto& m : manager.motors()) {
            m->set_cmd_status_req();
            m->set_nonblock();
        }
        runtime = std::make_unique<robot::Runtime>(config);
    }

    // The simulated drive of a joint.
    sim::ObotDeviceSim& device(size_t joint) { return sims[joint]->device(); }

    // What a client's write_command() does.
    void send(Request request = Request::None) {
        ipc::RobotCommand next = command;
        next.request = static_cast<uint32_t>(request);
        ipc::apply_request(command, next);
        next.seq = command.seq + 1;
        next.stamp_ns = now;
        command = next;
        have_command = true;
    }

    void cycle(bool overrun = false) {
        now += static_cast<int64_t>(config.period_us) * 1000;
        robot::CycleTiming timing;
        timing.now_mono_ns = now;
        timing.now_real_ns = now;
        timing.overrun = overrun;
        for (sim::SimMotor* m : sims) m->set_powered(devices_powered);
        manager.read();
        for (size_t i = 0; i < sims.size(); ++i) {
            fresh[i] = manager.get_status_fresh()[i];
            write_failed[i] = manager.get_write_error_count()[i] > 0;
        }
        const robot::MotorIo io{manager.get_statuses().data(), fresh, write_failed, manager.commands().data()};
        runtime->update(timing, have_command ? &command : nullptr, io, state);
        manager.write_saved_commands();
        const float dt = static_cast<float>(config.period_us) * 1e-6f;
        for (sim::SimMotor* m : sims) m->step(dt);
    }

    // Run cycles, re-sending the current command every `resend_every` cycles as a client would.
    void run(int cycles, int resend_every = 10) {
        for (int i = 0; i < cycles; ++i) {
            if (resend_every > 0 && have_command && i % resend_every == 0) send();
            cycle();
        }
    }

    void set_all(JointMode mode) {
        for (auto& m : command.joint_mode) m = static_cast<uint8_t>(mode);
    }

    void enable() {
        run(20, 0);
        REQUIRE(state.mode == static_cast<uint32_t>(RobotMode::Idle));
        set_all(JointMode::Idle);
        send(Request::Enable);
        cycle();
        REQUIRE(state.mode == static_cast<uint32_t>(RobotMode::Active));
    }
};

uint32_t mode_of(const Rig& r) {
    return r.state.mode;
}
constexpr uint32_t kInit = static_cast<uint32_t>(RobotMode::Init);
constexpr uint32_t kIdle = static_cast<uint32_t>(RobotMode::Idle);
constexpr uint32_t kActive = static_cast<uint32_t>(RobotMode::Active);
constexpr uint32_t kFault = static_cast<uint32_t>(RobotMode::Fault);
constexpr uint32_t kEstop = static_cast<uint32_t>(RobotMode::Estop);

}  // namespace

TEST_CASE("start-up: init until every actuator reports, then idle") {
    Rig r;
    r.devices_powered = false;
    r.run(50, 0);
    CHECK(mode_of(r) == kInit);
    CHECK(r.state.fault == 0);  // waiting is not a fault
    CHECK(r.state.rx_age_us[kShoulder] == -1);

    r.devices_powered = true;
    r.run(5, 0);
    CHECK(mode_of(r) == kIdle);
    CHECK(r.state.num_joints == 4);
    CHECK(r.state.rx_age_us[kShoulder] >= 0);
    CHECK(r.state.rx_age_us[kShoulder] < 1000);
    CHECK(r.state.command_age_us == -1);
    // Idle commands the drives' DAMPED mode.
    r.run(100, 0);
    CHECK(r.device(kShoulder).mode() == DAMPED);
    CHECK(r.device(kWrist).mode() == DAMPED);
    CHECK(r.state.temperature[kShoulder] == doctest::Approx(30.0f));
    CHECK(r.state.bus_voltage[kShoulder] == doctest::Approx(48.0f));
    CHECK(r.state.bus_voltage[kWrist] == doctest::Approx(48.0f));
}

TEST_CASE("enable is refused before idle and when not requested") {
    Rig r;
    r.set_all(JointMode::Position);
    r.send(Request::Enable);  // arrives while still in init
    r.cycle();
    CHECK(mode_of(r) != kActive);
    r.run(20);
    CHECK(mode_of(r) == kIdle);  // the old request is not replayed
    CHECK(r.state.joint_mode[kShoulder] == static_cast<uint8_t>(JointMode::Idle));
    CHECK(std::fabs(r.state.q[kShoulder]) < 1e-3f);
}

TEST_CASE("position commands are tracked through the velocity and acceleration limits") {
    Rig r;
    r.enable();
    r.command.joint_mode[kShoulder] = static_cast<uint8_t>(JointMode::Position);
    r.command.joint_mode[kWrist] = static_cast<uint8_t>(JointMode::Position);
    r.command.q[kShoulder] = 1.0f;
    r.command.q[kWrist] = -0.7f;
    r.send();

    float max_qd_cmd = 0, max_qdd_cmd = 0, last = 0;
    for (int i = 0; i < 2500; ++i) {
        if (i % 10 == 0) r.send();
        r.cycle();
        max_qd_cmd = std::max(max_qd_cmd, std::fabs(r.state.qd_cmd[kShoulder]));
        max_qdd_cmd = std::max(max_qdd_cmd, std::fabs(r.state.qd_cmd[kShoulder] - last) / 0.001f);
        last = r.state.qd_cmd[kShoulder];
    }
    CHECK(mode_of(r) == kActive);
    CHECK(r.state.fault == 0);
    CHECK(r.state.warning == 0);
    CHECK(max_qd_cmd <= 2.0f * 1.001f);
    CHECK(max_qdd_cmd <= 10.0f * 1.01f);
    CHECK(r.state.q_cmd[kShoulder] == 1.0f);
    CHECK(r.state.q[kShoulder] == doctest::Approx(1.0f).epsilon(0.01));
    CHECK(r.state.q[kWrist] == doctest::Approx(-0.7f).epsilon(0.01));
    CHECK(r.state.kp_cmd[kShoulder] == 200.0f);
    // The joint really moved in the drive's own frame, with the sign flipped for the wrist.
    CHECK(r.device(kShoulder).plant().q == doctest::Approx(1.0f).epsilon(0.01));
    CHECK(r.device(kWrist).plant().q == doctest::Approx(0.7f).epsilon(0.01));
    CHECK(r.state.command_age_us >= 0);
    CHECK(r.state.command_age_us < 11000);
}

TEST_CASE("targets outside the joint range are clamped, impedance gains are limited") {
    Rig r;
    r.enable();
    r.command.joint_mode[kShoulder] = static_cast<uint8_t>(JointMode::Impedance);
    r.command.q[kShoulder] = 9.0f;
    r.command.kp[kShoulder] = 1e6f;
    r.command.kd[kShoulder] = 8.0f;
    r.command.tau[kShoulder] = 500.0f;
    r.run(3000);
    CHECK(mode_of(r) == kActive);
    CHECK(r.state.q_cmd[kShoulder] == 1.5f);
    CHECK(r.state.kp_cmd[kShoulder] == 800.0f);  // 4 * kp
    CHECK(r.state.tau_cmd[kShoulder] == 20.0f);
    CHECK(r.state.q[kShoulder] <= 1.5f + r.config.joints[kShoulder].limit_margin);
}

TEST_CASE("a stale command makes the arm hold and the base stop, and is not a fault") {
    Rig r;
    r.enable();
    r.command.joint_mode[kShoulder] = static_cast<uint8_t>(JointMode::Position);
    r.command.q[kShoulder] = 1.2f;
    r.command.base_mode = static_cast<uint32_t>(ipc::BaseMode::Twist);
    r.command.base_v = 0.5f;
    r.run(400);  // moving at full speed, part of the way there
    const float q_when_lost = r.state.q_cmd[kShoulder];
    CHECK(q_when_lost > 0.2f);
    CHECK(q_when_lost < 1.0f);
    CHECK(r.state.base_v > 0.3f);
    CHECK((r.state.warning & ipc::robot_warning::kCommandStale) == 0);

    r.run(1500, 0);  // the client goes away
    CHECK(mode_of(r) == kActive);
    CHECK(r.state.fault == 0);
    CHECK((r.state.warning & ipc::robot_warning::kCommandStale) != 0);
    CHECK(r.state.command_age_us > 50000);
    // The arm braked to a stop a little past where it was, far short of the target.
    CHECK(r.state.q_cmd[kShoulder] > q_when_lost);
    CHECK(r.state.q_cmd[kShoulder] < q_when_lost + 0.35f);  // 50 ms to notice, then v^2 / 2a
    CHECK(std::fabs(r.state.qd[kShoulder]) < 0.01f);
    CHECK(std::fabs(r.state.base_v) < 0.01f);
    CHECK(r.state.joint_mode[kLeft] == static_cast<uint8_t>(JointMode::Velocity));

    r.run(1500);  // the client comes back: carry on
    CHECK((r.state.warning & ipc::robot_warning::kCommandStale) == 0);
    CHECK(r.state.q[kShoulder] == doctest::Approx(1.2f).epsilon(0.01));
}

TEST_CASE("an actuator that stops answering faults the robot and everything goes idle") {
    Rig r;
    r.enable();
    r.set_all(JointMode::Position);
    r.command.q[kShoulder] = 0.5f;
    r.run(1000);
    CHECK(r.device(kShoulder).mode() == STATE);

    r.device(kShoulder).set_silent(true);
    r.run(5);
    CHECK(mode_of(r) == kActive);  // a few lost frames are tolerated
    r.run(3);
    CHECK(mode_of(r) == kFault);
    CHECK((r.state.joint_fault[kShoulder] & ipc::joint_fault::kCommTimeout) != 0);
    CHECK(r.state.joint_fault[kWrist] == 0);
    CHECK((r.state.fault & ipc::robot_fault::kJoint) != 0);
    CHECK(r.state.rx_miss[kShoulder] >= 6);
    r.run(5);
    CHECK(r.state.joint_mode[kWrist] == static_cast<uint8_t>(JointMode::Idle));
    CHECK(r.device(kWrist).mode() == DAMPED);
    CHECK(r.device(kLeft).mode() == DAMPED);

    // Enable is refused while faulted; the fault stays latched after the actuator returns.
    r.device(kShoulder).set_silent(false);
    r.send(Request::Enable);
    r.run(20);
    CHECK(mode_of(r) == kFault);

    r.send(Request::ClearFaults);
    r.run(20);
    CHECK(mode_of(r) == kIdle);
    CHECK(r.state.fault == 0);
    CHECK(r.state.joint_fault[kShoulder] == 0);
    r.send(Request::Enable);
    r.cycle();
    CHECK(mode_of(r) == kActive);
}

TEST_CASE("clearing faults while the cause persists faults again") {
    Rig r;
    r.enable();
    r.device(kShoulder).set_silent(true);
    r.run(10);
    CHECK(mode_of(r) == kFault);
    r.send(Request::ClearFaults);
    r.run(3);
    CHECK(mode_of(r) == kFault);
}

TEST_CASE("a drive fault is reported with the drive's error word and cleared on request") {
    Rig r;
    r.enable();
    r.device(kShoulder).inject_error(ERROR_MASK_MOTOR_TEMPERATURE);
    r.run(3);
    CHECK(mode_of(r) == kFault);
    CHECK((r.state.joint_fault[kShoulder] & ipc::joint_fault::kDrive) != 0);
    CHECK((r.state.drive_error[kShoulder] & ERROR_MASK_MOTOR_TEMPERATURE) != 0);

    r.send(Request::ClearFaults);
    r.run(20);
    CHECK(mode_of(r) == kIdle);
    CHECK(r.state.drive_error[kShoulder] == 0);
    CHECK_FALSE(r.device(kShoulder).safe_mode());
}

TEST_CASE("drives left faulted by an earlier host are cleared once at start-up") {
    Rig r;
    r.device(kShoulder).inject_error(ERROR_MASK_SEQUENCE);  // what a host timeout leaves behind
    r.sims[kShoulder]->step(0.001f);
    REQUIRE(r.device(kShoulder).safe_mode());
    r.run(30, 0);
    CHECK(mode_of(r) == kIdle);
    CHECK(r.state.fault == 0);
    CHECK_FALSE(r.device(kShoulder).safe_mode());
}

TEST_CASE("estop wins from any mode and needs an explicit clear") {
    Rig r;
    r.enable();
    r.set_all(JointMode::Position);
    r.run(100);
    r.send(Request::Estop);
    r.run(3);
    CHECK(mode_of(r) == kEstop);
    CHECK((r.state.fault & ipc::robot_fault::kEstop) != 0);
    CHECK(r.state.joint_mode[kShoulder] == static_cast<uint8_t>(JointMode::Idle));
    r.send(Request::Enable);
    r.run(3);
    CHECK(mode_of(r) == kEstop);
    r.send(Request::ClearFaults);
    r.run(10);
    CHECK(mode_of(r) == kIdle);
}

TEST_CASE("commands with non-finite numbers or unknown modes are ignored") {
    Rig r;
    r.enable();
    r.command.joint_mode[kShoulder] = static_cast<uint8_t>(JointMode::Position);
    r.command.q[kShoulder] = 0.3f;
    r.run(1000);
    CHECK(r.state.q[kShoulder] == doctest::Approx(0.3f).epsilon(0.02));
    const uint64_t good_seq = r.state.command_seq;

    r.command.q[kShoulder] = std::numeric_limits<float>::quiet_NaN();
    r.send();
    r.cycle();
    CHECK((r.state.warning & ipc::robot_warning::kCommandInvalid) != 0);
    CHECK(r.state.command_seq == good_seq);
    CHECK(r.state.q_cmd[kShoulder] == 0.3f);

    r.command.q[kShoulder] = 0.3f;
    r.command.joint_mode[kWrist] = 99;
    r.send();
    r.cycle();
    CHECK((r.state.warning & ipc::robot_warning::kCommandInvalid) != 0);
    CHECK(mode_of(r) == kActive);
}

TEST_CASE("a wheel drive that wraps its position every revolution: position and odometry stay continuous") {
    Rig r;
    r.enable();
    r.command.base_mode = static_cast<uint32_t>(ipc::BaseMode::Twist);
    r.command.base_v = 0.8f;  // 8 rad/s at the wheels: a wrap every 0.8 s
    float last_q = 0, last_x = 0, biggest_q_step = 0, biggest_x_step = 0;
    bool drive_wrapped = false;
    float last_drive = 0;
    for (int i = 0; i < 6000; ++i) {
        if (i % 10 == 0) r.send();
        r.cycle();
        if (i > 0) {
            biggest_q_step = std::max(biggest_q_step, std::fabs(r.state.q[kLeft] - last_q));
            biggest_x_step = std::max(biggest_x_step, std::fabs(static_cast<float>(r.state.odom_x) - last_x));
        }
        last_q = r.state.q[kLeft];
        last_x = static_cast<float>(r.state.odom_x);
        MotorStatus raw;
        r.device(kLeft).make_status(raw);
        if (std::fabs(raw.motor_position - last_drive) > 3.0f) drive_wrapped = true;  // the drive did jump
        last_drive = raw.motor_position;
    }
    CHECK(drive_wrapped);
    CHECK(mode_of(r) == kActive);
    CHECK(r.state.fault == 0);
    CHECK(biggest_q_step < 0.02f);  // 8 rad/s for 1 ms, and no jump of a revolution
    CHECK(biggest_x_step < 0.002f);
    CHECK(r.state.q[kLeft] > 40.0f);             // several revolutions, counted up
    CHECK(r.state.q[kRight] > 40.0f);            // forward is positive for both joints...
    CHECK(r.device(kRight).plant().q < -40.0f);  // ...whichever way the drive itself counts (direction = -1 here)
    CHECK(r.state.odom_x == doctest::Approx(0.1 * r.state.q[kLeft]).epsilon(0.02));
    CHECK(std::fabs(r.state.odom_y) < 0.01);
}

TEST_CASE("base twist: wheel speeds, direction signs and odometry") {
    Rig r;
    r.enable();
    r.command.base_mode = static_cast<uint32_t>(ipc::BaseMode::Twist);
    r.command.base_v = 0.4f;
    r.run(3000);
    // Both wheels turn forward in joint terms; the right motor itself turns the other way.
    CHECK(r.state.qd[kLeft] == doctest::Approx(4.0f).epsilon(0.02));
    CHECK(r.state.qd[kRight] == doctest::Approx(4.0f).epsilon(0.02));
    CHECK(r.device(kLeft).plant().qd > 0);
    CHECK(r.device(kRight).plant().qd < 0);
    CHECK(r.state.base_v == doctest::Approx(0.4f).epsilon(0.02));
    CHECK(std::fabs(r.state.base_omega) < 0.01f);
    // 3 s at 0.4 m/s less the ramp-up (0.4 m/s at 2 m/s^2 loses 0.04 m).
    CHECK(r.state.odom_x == doctest::Approx(1.16).epsilon(0.03));
    CHECK(std::fabs(r.state.odom_y) < 0.01);

    // Stop, then turn on the spot by a quarter turn.
    r.command.base_v = 0;
    r.run(1000);
    const double x0 = r.state.odom_x;
    const double yaw0 = r.state.odom_yaw;
    r.command.base_omega = 1.0f;
    r.run(1571);
    r.command.base_omega = 0;
    r.run(1000);
    CHECK(r.state.odom_yaw - yaw0 == doctest::Approx(1.571).epsilon(0.03));
    CHECK(r.state.odom_x == doctest::Approx(x0).epsilon(0.01));

    // Requests beyond the base limits are clamped.
    r.command.base_v = 5.0f;
    r.run(2000);
    CHECK(r.state.base_v == doctest::Approx(0.8f).epsilon(0.03));
    CHECK(mode_of(r) == kActive);
}

TEST_CASE("velocity mode slows to a stop at a joint limit instead of running into it") {
    Rig r;
    r.enable();
    r.command.joint_mode[kWrist] = static_cast<uint8_t>(JointMode::Velocity);
    r.command.qd[kWrist] = 3.0f;
    r.run(4000);
    CHECK(mode_of(r) == kActive);
    CHECK(r.state.q[kWrist] <= 1.5f + 0.02f);
    CHECK(r.state.q[kWrist] > 1.3f);
    CHECK(std::fabs(r.state.qd[kWrist]) < 0.05f);
}

TEST_CASE("torque mode is limited and a joint pushed past its hard limit faults") {
    Rig r;
    r.enable();
    r.command.joint_mode[kWrist] = static_cast<uint8_t>(JointMode::Torque);
    r.command.tau[kWrist] = 0.15f;  // settles at 3 rad/s against the plant's damping
    r.run(3000);
    // Constant torque drives the joint to its end stop: the host stops pushing at the soft
    // limit, and trips if the joint is found beyond the margin.
    CHECK(r.state.q[kWrist] > 1.4f);
    if (mode_of(r) == kFault) {
        CHECK((r.state.joint_fault[kWrist] & ipc::joint_fault::kPositionLimit) != 0);
    } else {
        CHECK(r.state.tau_cmd[kWrist] == 0.0f);
    }
}

TEST_CASE("a runaway joint trips on velocity") {
    Rig r;
    r.enable();
    r.command.joint_mode[kWrist] = static_cast<uint8_t>(JointMode::Torque);
    r.command.tau[kWrist] = 50.0f;
    r.send();
    r.cycle();
    CHECK(r.state.tau_cmd[kWrist] == 5.0f);  // limited to tau_max
    r.run(200);
    CHECK(mode_of(r) == kFault);
    CHECK((r.state.joint_fault[kWrist] & ipc::joint_fault::kVelocityLimit) != 0);
    CHECK(r.state.q[kWrist] < 1.5f);  // caught long before the end stop
}

TEST_CASE("a request survives being followed at once by an ordinary command; estop is never displaced") {
    Rig r;
    r.run(20, 0);
    r.send(Request::Enable);
    r.send();  // same cycle, no request
    r.cycle();
    CHECK(mode_of(r) == kActive);
    r.send(Request::Estop);
    r.send(Request::ClearFaults);  // same cycle: must not cancel the estop
    r.cycle();
    CHECK(mode_of(r) == kEstop);
    r.run(20);
    CHECK(mode_of(r) == kEstop);  // nor is the dropped request replayed later
}

TEST_CASE("disable and shutdown return the drives to idle") {
    Rig r;
    r.enable();
    r.set_all(JointMode::Position);
    r.run(200);
    CHECK(r.device(kWrist).mode() == STATE);
    r.send(Request::Disable);
    r.run(5);
    CHECK(mode_of(r) == kIdle);
    CHECK(r.device(kWrist).mode() == DAMPED);

    r.send(Request::Enable);
    r.run(50);
    CHECK(mode_of(r) == kActive);
    r.runtime->request_shutdown();
    r.run(5);
    CHECK(mode_of(r) == kIdle);
    r.send(Request::Enable);
    r.run(5);
    CHECK(mode_of(r) == kIdle);  // no re-enabling on the way down
}

TEST_CASE("overruns are counted and flagged") {
    Rig r;
    r.run(10, 0);
    r.now += 5'000'000;  // the cycle before this one took 5 ms too long
    r.cycle(true);
    CHECK(r.state.overruns == 1);
    CHECK((r.state.warning & ipc::robot_warning::kOverrun) != 0);
}

TEST_CASE("stop and disable are taken even from a command that is otherwise refused") {
    Rig r;
    r.enable();
    r.set_all(JointMode::Position);
    r.run(100);
    // A client streaming garbage, and the operator's stop lands in the same mailbox write.
    r.command.q[kShoulder] = std::numeric_limits<float>::quiet_NaN();
    r.send(Request::Estop);
    r.cycle();
    CHECK(mode_of(r) == kEstop);
    CHECK((r.state.warning & ipc::robot_warning::kCommandInvalid) != 0);

    Rig d;
    d.enable();
    d.command.q[kShoulder] = std::numeric_limits<float>::infinity();
    d.send(Request::Disable);
    d.cycle();
    CHECK(mode_of(d) == kIdle);
    // ...but nothing that starts motion is taken from a refused command.
    d.send(Request::Enable);
    d.run(5, 0);
    CHECK(mode_of(d) == kIdle);
    CHECK((d.state.warning & ipc::robot_warning::kCommandInvalid) != 0);  // still reported
}

TEST_CASE("a command stamped in the future is refused, also once the clock catches up") {
    Rig r;
    r.enable();
    r.command.joint_mode[kShoulder] = static_cast<uint8_t>(JointMode::Position);
    r.command.q[kShoulder] = 0.8f;
    r.send();
    r.command.stamp_ns += 500'000'000;  // half a second ahead
    r.run(1000, 0);
    CHECK((r.state.warning & ipc::robot_warning::kCommandInvalid) != 0);
    CHECK(std::fabs(r.state.q[kShoulder]) < 0.01f);
    CHECK(r.state.joint_mode[kShoulder] == static_cast<uint8_t>(JointMode::Idle));
}

TEST_CASE("a client with soft impedance gains goes away: the hold is stiff and starts where the joint is") {
    Rig r;
    r.enable();
    r.command.joint_mode[kShoulder] = static_cast<uint8_t>(JointMode::Impedance);
    r.command.q[kShoulder] = 0.9f;  // setpoint far from the joint, which zero gains never move
    r.command.kp[kShoulder] = 0;
    r.command.kd[kShoulder] = 0;
    r.run(1500);
    CHECK(std::fabs(r.state.q[kShoulder]) < 0.01f);
    CHECK(r.state.q_cmd[kShoulder] == doctest::Approx(0.9f));
    r.run(500, 0);  // the client is gone
    CHECK((r.state.warning & ipc::robot_warning::kCommandStale) != 0);
    CHECK(r.state.kp_cmd[kShoulder] == 200.0f);  // the joint's own gains
    CHECK(r.state.kd_cmd[kShoulder] == 8.0f);
    CHECK(std::fabs(r.state.q_cmd[kShoulder]) < 0.02f);  // held where it was, not yanked to 0.9
    CHECK(std::fabs(r.state.q[kShoulder]) < 0.02f);
    CHECK(mode_of(r) == kActive);
}

TEST_CASE("switching a moving joint from velocity to position keeps the setpoint within the acceleration limit") {
    Rig r;
    r.enable();
    r.command.joint_mode[kWrist] = static_cast<uint8_t>(JointMode::Velocity);
    r.command.qd[kWrist] = 2.0f;
    r.run(300);
    CHECK(r.state.qd[kWrist] == doctest::Approx(2.0f).epsilon(0.1));
    r.command.joint_mode[kWrist] = static_cast<uint8_t>(JointMode::Position);
    r.command.q[kWrist] = r.state.q[kWrist];  // "stop here"
    // The setpoint has to pick up from the velocity the joint really has (a little under
    // the commanded one) and brake from there, not start from a standstill.
    float last = r.state.qd[kWrist];
    float worst = 0;
    for (int i = 0; i < 400; ++i) {
        if (i % 10 == 0) r.send();
        r.cycle();
        worst = std::max(worst, std::fabs(r.state.qd_cmd[kWrist] - last) / 0.001f);
        last = r.state.qd_cmd[kWrist];
    }
    CHECK(worst <= 20.0f * 1.5f);  // qdd_max, plus the resolution of the measured velocity
    CHECK(std::fabs(r.state.qd[kWrist]) < 0.05f);
    CHECK(mode_of(r) == kActive);
}

TEST_CASE("a command that cannot be sent is flagged, and the other drives still get theirs") {
    Rig r;
    r.enable();
    r.set_all(JointMode::Position);
    r.run(50);
    CHECK(r.state.warning == 0);
    const uint32_t shoulder_before = r.device(kShoulder).last_host_timestamp();
    r.sims[kShoulder]->fail_writes(1);
    r.cycle();  // the write at the end of this cycle fails...
    CHECK(r.state.warning == 0);
    r.cycle();  // ...which the next cycle knows, and it has no status from that drive
    CHECK((r.state.warning & ipc::robot_warning::kTxDropped) != 0);
    CHECK(r.device(kWrist).last_host_timestamp() == shoulder_before + 2);
    CHECK(r.device(kShoulder).last_host_timestamp() == shoulder_before + 2);  // and is back in step
    CHECK(r.state.rx_miss[kShoulder] == 1);
    CHECK(r.state.rx_miss[kWrist] == 0);
    r.run(10);
    CHECK(mode_of(r) == kActive);  // one lost cycle is tolerated
    CHECK((r.state.warning & ipc::robot_warning::kTxDropped) == 0);

    // A drive that cannot be written to at all ends as a comm fault, like one that is silent.
    r.sims[kShoulder]->fail_writes(1000);
    r.run(10);
    CHECK(mode_of(r) == kFault);
    CHECK((r.state.joint_fault[kShoulder] & ipc::joint_fault::kCommTimeout) != 0);
    CHECK(r.device(kWrist).mode() == DAMPED);
}
