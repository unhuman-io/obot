#include "obot/ipc/layout_json.h"

#include <cstddef>
#include <sstream>
#include <type_traits>

#include "obot/ipc/channels.h"
#include "obot/ipc/layout.h"

namespace obot::ipc {

namespace {

template <typename T>
const char* type_name() {
    using B = std::remove_all_extents_t<T>;
    if (std::is_same_v<B, char>) return "char";
    if (std::is_same_v<B, uint8_t>) return "u8";
    if (std::is_same_v<B, uint32_t>) return "u32";
    if (std::is_same_v<B, int32_t>) return "i32";
    if (std::is_same_v<B, uint64_t>) return "u64";
    if (std::is_same_v<B, int64_t>) return "i64";
    if (std::is_same_v<B, float>) return "f32";
    if (std::is_same_v<B, double>) return "f64";
    return "?";
}

struct Writer {
    std::ostringstream os;
    bool first_field = true;
    void begin(const char* name, size_t size, bool first) {
        if (!first) os << ",\n";
        os << "  \"" << name << "\": {\"size\": " << size << ", \"fields\": [";
        first_field = true;
    }
    template <typename T>
    void field(const char* name, size_t offset) {
        using B = std::remove_all_extents_t<T>;
        if (!first_field) os << ", ";
        first_field = false;
        os << "[\"" << name << "\", " << offset << ", \"" << type_name<T>() << "\", [";
        // Array shape, outermost dimension first.
        if constexpr (std::rank_v<T> == 1) {
            os << std::extent_v<T, 0>;
        } else if constexpr (std::rank_v<T> == 2) {
            os << std::extent_v<T, 0> << ", " << std::extent_v<T, 1>;
        }
        os << "]]";
        (void)sizeof(B);
    }
    void end() { os << "]}"; }
};

#define OBOT_FIELD(S, f) w.field<decltype(S::f)>(#f, offsetof(S, f))

}  // namespace

std::string layout_json() {
    Writer w;
    w.os << "{\n  \"layout_version\": " << kLayoutVersion << ",\n  \"magic\": " << kMagic
         << ",\n  \"max_joints\": " << kMaxJoints << ",\n  \"name_len\": " << kNameLen
         << ",\n  \"ring_capacity\": " << kRingCapacity << ",\n  \"ring_slot_size\": " << sizeof(StateSlot)
         << ",\n  \"info_offset\": " << kInfoOffset << ",\n  \"command_offset\": " << kCommandOffset
         << ",\n  \"ring_offset\": " << kRingOffset << ",\n  \"total_size\": " << kTotalSize << ",\n";

    w.begin("ShmHeader", sizeof(ShmHeader), true);
    OBOT_FIELD(ShmHeader, magic);
    OBOT_FIELD(ShmHeader, layout_version);
    OBOT_FIELD(ShmHeader, header_size);
    OBOT_FIELD(ShmHeader, info_size);
    OBOT_FIELD(ShmHeader, state_size);
    OBOT_FIELD(ShmHeader, command_size);
    OBOT_FIELD(ShmHeader, ring_capacity);
    OBOT_FIELD(ShmHeader, ring_slot_size);
    OBOT_FIELD(ShmHeader, info_offset);
    OBOT_FIELD(ShmHeader, command_offset);
    OBOT_FIELD(ShmHeader, ring_offset);
    OBOT_FIELD(ShmHeader, total_size);
    OBOT_FIELD(ShmHeader, daemon_pid);
    OBOT_FIELD(ShmHeader, start_time_real_ns);
    w.end();

    w.begin("RobotInfo", sizeof(RobotInfo), false);
    OBOT_FIELD(RobotInfo, num_joints);
    OBOT_FIELD(RobotInfo, period_ns);
    OBOT_FIELD(RobotInfo, robot_name);
    OBOT_FIELD(RobotInfo, joint_name);
    OBOT_FIELD(RobotInfo, q_min);
    OBOT_FIELD(RobotInfo, q_max);
    OBOT_FIELD(RobotInfo, qd_max);
    OBOT_FIELD(RobotInfo, tau_max);
    OBOT_FIELD(RobotInfo, continuous);
    OBOT_FIELD(RobotInfo, left_wheel);
    OBOT_FIELD(RobotInfo, right_wheel);
    OBOT_FIELD(RobotInfo, wheel_radius);
    OBOT_FIELD(RobotInfo, track_width);
    w.end();

    w.begin("RobotState", sizeof(RobotState), false);
    OBOT_FIELD(RobotState, cycle);
    OBOT_FIELD(RobotState, t_mono_ns);
    OBOT_FIELD(RobotState, t_real_ns);
    OBOT_FIELD(RobotState, mode);
    OBOT_FIELD(RobotState, fault);
    OBOT_FIELD(RobotState, warning);
    OBOT_FIELD(RobotState, num_joints);
    OBOT_FIELD(RobotState, command_seq);
    OBOT_FIELD(RobotState, command_source);
    OBOT_FIELD(RobotState, command_age_us);
    OBOT_FIELD(RobotState, wake_latency_ns);
    OBOT_FIELD(RobotState, exec_ns);
    OBOT_FIELD(RobotState, rx_wait_ns);
    OBOT_FIELD(RobotState, overruns);
    OBOT_FIELD(RobotState, odom_x);
    OBOT_FIELD(RobotState, odom_y);
    OBOT_FIELD(RobotState, odom_yaw);
    OBOT_FIELD(RobotState, base_v);
    OBOT_FIELD(RobotState, base_omega);
    OBOT_FIELD(RobotState, joint_mode);
    OBOT_FIELD(RobotState, joint_fault);
    OBOT_FIELD(RobotState, drive_error);
    OBOT_FIELD(RobotState, rx_miss);
    OBOT_FIELD(RobotState, rx_age_us);
    OBOT_FIELD(RobotState, q);
    OBOT_FIELD(RobotState, qd);
    OBOT_FIELD(RobotState, tau);
    OBOT_FIELD(RobotState, q_cmd);
    OBOT_FIELD(RobotState, qd_cmd);
    OBOT_FIELD(RobotState, tau_cmd);
    OBOT_FIELD(RobotState, kp_cmd);
    OBOT_FIELD(RobotState, kd_cmd);
    OBOT_FIELD(RobotState, temperature);
    OBOT_FIELD(RobotState, bus_voltage);
    w.end();

    w.begin("RobotCommand", sizeof(RobotCommand), false);
    OBOT_FIELD(RobotCommand, stamp_ns);
    OBOT_FIELD(RobotCommand, seq);
    OBOT_FIELD(RobotCommand, source);
    OBOT_FIELD(RobotCommand, request);
    OBOT_FIELD(RobotCommand, request_id);
    OBOT_FIELD(RobotCommand, estop_id);
    OBOT_FIELD(RobotCommand, base_mode);
    OBOT_FIELD(RobotCommand, reserved);
    OBOT_FIELD(RobotCommand, base_v);
    OBOT_FIELD(RobotCommand, base_omega);
    OBOT_FIELD(RobotCommand, joint_mode);
    OBOT_FIELD(RobotCommand, q);
    OBOT_FIELD(RobotCommand, qd);
    OBOT_FIELD(RobotCommand, tau);
    OBOT_FIELD(RobotCommand, kp);
    OBOT_FIELD(RobotCommand, kd);
    w.end();

    w.os << "\n}\n";
    return w.os.str();
}

}  // namespace obot::ipc
