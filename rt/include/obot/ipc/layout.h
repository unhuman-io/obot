// Shared memory interface between the realtime daemon and everything else (logging,
// visualization, teleoperation, policies). The daemon is the only process that touches the
// actuators; other processes read RobotState records and write RobotCommand.
//
// All structs are plain data with fixed-width fields and no implicit padding, so that the
// same layout can be described as a numpy dtype (python/src/obot/shm/layout.py). A test
// compares the two through `obot-rtd --dump-layout`. Bump kLayoutVersion on any change.
#pragma once

#include <cstddef>
#include <cstdint>

namespace obot::ipc {

inline constexpr uint32_t kMagic = 0x544F424F;  // "OBOT" little endian
inline constexpr uint32_t kLayoutVersion = 1;
inline constexpr int kMaxJoints = 16;
inline constexpr int kNameLen = 24;
inline constexpr uint32_t kRingCapacity = 4096;  // state records kept, power of two

enum class RobotMode : uint32_t {
    Init = 0,    // waiting for every actuator to report
    Idle = 1,    // actuators reporting, drives idle
    Active = 2,  // following commands
    Fault = 3,   // latched until a clear-faults request; drives idle
    Estop = 4,   // latched until a clear-faults request; drives idle
};

enum class JointMode : uint8_t {
    Idle = 0,       // the joint's configured idle behaviour (damped or open)
    Damped = 1,     // velocity damping only
    Position = 2,   // q target with the joint's configured gains, plus tau feed-forward
    Impedance = 3,  // q, qd targets with kp, kd from the command, plus tau feed-forward
    Velocity = 4,   // qd target
    Torque = 5,     // tau only
};
inline constexpr uint8_t kJointModeCount = 6;

enum class Request : uint32_t {
    None = 0,
    Enable = 1,       // Idle -> Active
    Disable = 2,      // Active -> Idle
    ClearFaults = 3,  // Fault / Estop -> Idle
    Estop = 4,        // anything -> Estop
};
// A client passes one Request with a write; the write that carries it advances request_id
// (or estop_id for Estop), and later writes without a request repeat the same ids, so a
// request is not lost when another command is written before the daemon's next cycle.
// An estop is never displaced by another request.

enum class BaseMode : uint32_t {
    Joints = 0,  // the wheel joints follow their per-joint commands
    Twist = 1,   // the wheel joints follow base_v / base_omega
};

// RobotState::fault, robot-level.
namespace robot_fault {
inline constexpr uint32_t kJoint = 1u << 0;  // a joint fault, see joint_fault[]
inline constexpr uint32_t kBus = 1u << 1;    // CAN controller error (bus off, ...)
inline constexpr uint32_t kEstop = 1u << 2;  // estop requested
}  // namespace robot_fault

// RobotState::warning, robot-level, not latched.
namespace robot_warning {
inline constexpr uint32_t kCommandStale = 1u << 0;    // no fresh command: holding
inline constexpr uint32_t kCommandInvalid = 1u << 1;  // command rejected (non-finite, bad mode)
inline constexpr uint32_t kOverrun = 1u << 2;         // the previous cycle missed its period
inline constexpr uint32_t kTxDropped = 1u << 3;       // a command frame could not be queued
}  // namespace robot_warning

// RobotState::joint_fault[], latched.
namespace joint_fault {
inline constexpr uint32_t kCommTimeout = 1u << 0;    // no status for too many cycles
inline constexpr uint32_t kDrive = 1u << 1;          // the drive reports a fault
inline constexpr uint32_t kPositionLimit = 1u << 2;  // outside the hard position limits
inline constexpr uint32_t kVelocityLimit = 1u << 3;  // above the velocity trip level
inline constexpr uint32_t kTemperature = 1u << 4;    // above the temperature trip level
}  // namespace joint_fault

struct RobotInfo {
    uint32_t num_joints;
    uint32_t period_ns;
    char robot_name[kNameLen];
    char joint_name[kMaxJoints][kNameLen];
    float q_min[kMaxJoints];
    float q_max[kMaxJoints];
    float qd_max[kMaxJoints];
    float tau_max[kMaxJoints];
    uint8_t continuous[kMaxJoints];  // 1: no position limits (wheels)
    int32_t left_wheel;              // joint index, -1 if the robot has no base
    int32_t right_wheel;
    float wheel_radius;
    float track_width;
};

struct RobotState {
    uint64_t cycle;     // control cycle counter
    int64_t t_mono_ns;  // CLOCK_MONOTONIC at the start of the cycle
    int64_t t_real_ns;  // CLOCK_REALTIME at the start of the cycle
    uint32_t mode;      // RobotMode
    uint32_t fault;     // robot_fault bits, latched
    uint32_t warning;   // robot_warning bits for this cycle
    uint32_t num_joints;
    uint64_t command_seq;  // RobotCommand::seq applied in this cycle
    uint32_t command_source;
    int32_t command_age_us;   // age of that command, -1 if there has been none
    int32_t wake_latency_ns;  // how late the cycle started
    int32_t exec_ns;          // start of cycle to state published
    int32_t rx_wait_ns;       // start of cycle to last status received, -1 if none
    uint32_t overruns;        // cycles that missed their period, total
    double odom_x;            // base odometry in the frame the daemon started in, m
    double odom_y;
    double odom_yaw;                   // rad
    float base_v;                      // measured, m/s
    float base_omega;                  // measured, rad/s
    uint8_t joint_mode[kMaxJoints];    // JointMode actually commanded
    uint32_t joint_fault[kMaxJoints];  // joint_fault bits, latched
    uint32_t drive_error[kMaxJoints];  // the drive's own error word
    uint32_t rx_miss[kMaxJoints];      // cycles without a status from this joint, total
    int32_t rx_age_us[kMaxJoints];     // age of the joint's last status, -1 if none yet
    float q[kMaxJoints];               // measured position, rad
    float qd[kMaxJoints];              // measured velocity, rad/s
    float tau[kMaxJoints];             // measured or estimated torque, N m
    float q_cmd[kMaxJoints];           // position sent to the drive
    float qd_cmd[kMaxJoints];          // velocity sent to the drive
    float tau_cmd[kMaxJoints];         // feed-forward torque sent to the drive
    float kp_cmd[kMaxJoints];
    float kd_cmd[kMaxJoints];
    float temperature[kMaxJoints];  // deg C, NaN if not reported
    float bus_voltage[kMaxJoints];  // V, NaN if not reported
};

struct RobotCommand {
    int64_t stamp_ns;  // CLOCK_MONOTONIC when written; stale commands are not followed
    uint64_t seq;      // incremented by the writer
    uint32_t source;   // writer's id, free form
    uint32_t request;  // last Request other than Estop; acted on when request_id changes
    uint32_t request_id;
    uint32_t estop_id;   // an estop is taken whenever this changes
    uint32_t base_mode;  // BaseMode
    uint32_t reserved;
    float base_v;                    // m/s
    float base_omega;                // rad/s
    uint8_t joint_mode[kMaxJoints];  // JointMode
    float q[kMaxJoints];
    float qd[kMaxJoints];
    float tau[kMaxJoints];
    float kp[kMaxJoints];
    float kd[kMaxJoints];
};

// The shared memory file starts with this header. A reader maps the file, waits for
// magic == kMagic, and checks layout_version and the struct sizes before touching anything.
struct ShmHeader {
    uint32_t magic;  // written last by the daemon
    uint32_t layout_version;
    uint32_t header_size;
    uint32_t info_size;
    uint32_t state_size;
    uint32_t command_size;
    uint32_t ring_capacity;
    uint32_t ring_slot_size;  // 8 byte sequence + state, padded to 8
    uint64_t info_offset;
    uint64_t command_offset;  // -> CommandSlot
    uint64_t ring_offset;     // -> StateRing
    uint64_t total_size;
    int64_t daemon_pid;
    int64_t start_time_real_ns;
};

static_assert(sizeof(RobotInfo) == 8 + kNameLen + kMaxJoints * kNameLen + 4 * 4 * kMaxJoints + kMaxJoints + 16);
static_assert(sizeof(RobotState) % 8 == 0);
static_assert(sizeof(RobotCommand) % 8 == 0);
static_assert(sizeof(ShmHeader) == 80);
static_assert(offsetof(RobotState, odom_x) % 8 == 0);
static_assert(offsetof(RobotState, joint_fault) % 4 == 0);

}  // namespace obot::ipc
