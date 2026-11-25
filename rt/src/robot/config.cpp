#include "obot/robot/config.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>

#define TOML_EXCEPTIONS 1
#define TOML_ENABLE_FORMATTERS 0
#include "toml.hpp"

#include "obot/actuator/obot_driver.h"
#include "obot/can/bus_budget.h"
#include "obot/ipc/layout.h"

namespace obot::robot {

namespace {

struct Ctx {
    std::string source;
    [[noreturn]] void fail(const std::string& where, const std::string& what) const {
        throw std::runtime_error(source + ": " + where + ": " + what);
    }
};

// Reads table[key] into out if present; complains if it has the wrong type.
template <typename T>
void get(const Ctx& ctx, const toml::table& table, const std::string& where, const char* key, T& out) {
    const toml::node* node = table.get(key);
    if (!node) return;
    if constexpr (std::is_same_v<T, bool>) {
        if (auto v = node->value_exact<bool>()) {
            out = *v;
            return;
        }
    } else if constexpr (std::is_same_v<T, std::string>) {
        if (auto v = node->value_exact<std::string>()) {
            out = *v;
            return;
        }
    } else if constexpr (std::is_integral_v<T>) {
        if (auto v = node->value_exact<int64_t>()) {
            out = static_cast<T>(*v);
            return;
        }
    } else {
        if (node->is_number()) {  // integers are accepted where a real is expected
            out = static_cast<T>(*node->value<double>());
            return;
        }
    }
    ctx.fail(where, std::string("'") + key + "' has the wrong type");
}

void check_keys(const Ctx& ctx, const toml::table& table, const std::string& where,
                const std::set<std::string>& known) {
    for (const auto& [key, value] : table) {
        (void)value;
        const std::string k(key.str());
        if (!known.count(k)) ctx.fail(where, "unknown key '" + k + "'");
    }
}

const toml::table* subtable(const Ctx& ctx, const toml::table& table, const std::string& where, const char* key) {
    const toml::node* node = table.get(key);
    if (!node) return nullptr;
    if (!node->is_table()) ctx.fail(where, std::string("'") + key + "' must be a table");
    return node->as_table();
}

RobotConfig from_table(const Ctx& ctx, const toml::table& root) {
    RobotConfig c;
    // [[camera]] and [teleop] belong to the operator tools (python/); they are not read here.
    check_keys(ctx, root, "top level",
               {"robot", "rt", "bus", "joint", "base", "camera", "teleop", "arm", "model", "world"});

    if (const auto* t = subtable(ctx, root, "top level", "robot")) {
        check_keys(ctx, *t, "[robot]",
                   {"name", "shm_name", "period_us", "command_timeout_ms", "comm_timeout_cycles",
                    "clear_drive_faults_on_start", "max_bus_load"});
        get(ctx, *t, "[robot]", "name", c.name);
        get(ctx, *t, "[robot]", "shm_name", c.shm_name);
        get(ctx, *t, "[robot]", "period_us", c.period_us);
        get(ctx, *t, "[robot]", "command_timeout_ms", c.command_timeout_ms);
        get(ctx, *t, "[robot]", "comm_timeout_cycles", c.comm_timeout_cycles);
        get(ctx, *t, "[robot]", "clear_drive_faults_on_start", c.clear_drive_faults_on_start);
        get(ctx, *t, "[robot]", "max_bus_load", c.max_bus_load);
    }
    if (c.period_us < 100 || c.period_us > 100000) ctx.fail("[robot]", "period_us must be 100..100000");
    if (c.command_timeout_ms < 1) ctx.fail("[robot]", "command_timeout_ms must be at least 1");
    if (c.comm_timeout_cycles < 1) ctx.fail("[robot]", "comm_timeout_cycles must be at least 1");
    if (c.name.size() >= ipc::kNameLen) ctx.fail("[robot]", "name is too long");

    if (const auto* t = subtable(ctx, root, "top level", "rt")) {
        check_keys(ctx, *t, "[rt]", {"cpu", "priority", "lock_memory", "rx_deadline_us"});
        get(ctx, *t, "[rt]", "cpu", c.rt.cpu);
        get(ctx, *t, "[rt]", "priority", c.rt.priority);
        get(ctx, *t, "[rt]", "lock_memory", c.rt.lock_memory);
        get(ctx, *t, "[rt]", "rx_deadline_us", c.rt.rx_deadline_us);
    }
    if (c.rt.priority < 0 || c.rt.priority > 98) ctx.fail("[rt]", "priority must be 0..98");
    if (c.rt.rx_deadline_us == 0) c.rt.rx_deadline_us = c.period_us * 6 / 10;
    if (c.rt.rx_deadline_us < 0 || c.rt.rx_deadline_us >= c.period_us) {
        ctx.fail("[rt]", "rx_deadline_us must be shorter than the period");
    }

    const toml::array* buses = root["bus"].as_array();
    if (!buses || buses->empty()) ctx.fail("top level", "at least one [[bus]] is required");
    for (const auto& node : *buses) {
        const toml::table* t = node.as_table();
        if (!t) ctx.fail("[[bus]]", "must be a table");
        BusConfig b;
        get(ctx, *t, "[[bus]]", "name", b.name);
        const std::string where = "[[bus]] " + b.name;
        check_keys(ctx, *t, where, {"name", "interface", "fd", "bitrate", "data_bitrate"});
        get(ctx, *t, where, "interface", b.interface);
        get(ctx, *t, where, "fd", b.fd);
        get(ctx, *t, where, "bitrate", b.bitrate);
        get(ctx, *t, where, "data_bitrate", b.data_bitrate);
        b.usb = b.interface == "usbrt";
        if (b.name.empty()) ctx.fail("[[bus]]", "'name' is required");
        if (b.interface.empty()) ctx.fail(where, "'interface' is required");
        if (b.bitrate <= 0 || b.data_bitrate <= 0) ctx.fail(where, "bit rates must be positive");
        for (const auto& other : c.buses) {
            if (other.name == b.name) ctx.fail(where, "duplicate bus name");
            if (other.interface == b.interface && b.interface != "sim" && b.interface != "usbrt") {
                ctx.fail(where, "interface " + b.interface + " is used by two buses");
            }
        }
        c.buses.push_back(b);
    }

    const toml::array* joints = root["joint"].as_array();
    if (!joints || joints->empty()) ctx.fail("top level", "at least one [[joint]] is required");
    if (joints->size() > static_cast<size_t>(ipc::kMaxJoints)) {
        ctx.fail("top level", "at most " + std::to_string(ipc::kMaxJoints) + " joints");
    }
    for (const auto& node : *joints) {
        const toml::table* t = node.as_table();
        if (!t) ctx.fail("[[joint]]", "must be a table");
        JointConfig j;
        get(ctx, *t, "[[joint]]", "name", j.name);
        if (j.name.empty()) ctx.fail("[[joint]]", "'name' is required");
        if (j.name.size() >= ipc::kNameLen) ctx.fail("[[joint]] " + j.name, "name is too long");
        const std::string where = "[[joint]] " + j.name;
        check_keys(ctx, *t, where,
                   {"name",
                    "bus",
                    "driver",
                    "id",
                    "gear_ratio",
                    "torque_constant",
                    "direction",
                    "zero_offset",
                    "idle",
                    "output_encoder",
                    "torque_sensor",
                    "damped_kd",
                    "continuous",
                    "q_min",
                    "q_max",
                    "limit_margin",
                    "qd_max",
                    "qdd_max",
                    "tau_max",
                    "kp",
                    "kd",
                    "kp_max",
                    "kd_max",
                    "temperature_max",
                    "sim",
                    "usb_name"});
        get(ctx, *t, where, "bus", j.bus);
        std::string driver = "obot";
        get(ctx, *t, where, "driver", driver);
        // Every actuator runs the obot-controller firmware; the key is accepted so that
        // existing files keep loading.
        if (driver != "obot") ctx.fail(where, "unknown driver '" + driver + "' (only \"obot\" exists)");
        get(ctx, *t, where, "id", j.id);
        j.usb_name = j.name;
        get(ctx, *t, where, "usb_name", j.usb_name);
        get(ctx, *t, where, "gear_ratio", j.transmission.gear_ratio);
        get(ctx, *t, where, "torque_constant", j.transmission.torque_constant);
        get(ctx, *t, where, "direction", j.transmission.direction);
        get(ctx, *t, where, "zero_offset", j.transmission.zero_offset);
        std::string idle = "damped";
        get(ctx, *t, where, "idle", idle);
        if (idle != "damped" && idle != "open") ctx.fail(where, "'idle' must be \"damped\" or \"open\"");
        j.idle_damped = idle == "damped";
        get(ctx, *t, where, "output_encoder", j.output_encoder);
        get(ctx, *t, where, "torque_sensor", j.torque_sensor);
        get(ctx, *t, where, "damped_kd", j.damped_kd);
        get(ctx, *t, where, "continuous", j.continuous);
        get(ctx, *t, where, "q_min", j.q_min);
        get(ctx, *t, where, "q_max", j.q_max);
        get(ctx, *t, where, "limit_margin", j.limit_margin);
        get(ctx, *t, where, "qd_max", j.qd_max);
        get(ctx, *t, where, "qdd_max", j.qdd_max);
        get(ctx, *t, where, "tau_max", j.tau_max);
        get(ctx, *t, where, "kp", j.kp);
        get(ctx, *t, where, "kd", j.kd);
        get(ctx, *t, where, "kp_max", j.kp_max);
        get(ctx, *t, where, "kd_max", j.kd_max);
        get(ctx, *t, where, "temperature_max", j.temperature_max);
        if (const auto* s = subtable(ctx, *t, where, "sim")) {
            check_keys(ctx, *s, where + " sim", {"inertia", "damping", "gravity", "current_limit"});
            get(ctx, *s, where, "inertia", j.sim_plant.inertia);
            get(ctx, *s, where, "damping", j.sim_plant.damping);
            get(ctx, *s, where, "gravity", j.sim_plant.gravity);
            get(ctx, *s, where, "current_limit", j.sim_current_limit);
        }

        // Validation.
        j.bus_index = -1;
        for (size_t i = 0; i < c.buses.size(); ++i) {
            if (c.buses[i].name == j.bus) j.bus_index = static_cast<int>(i);
        }
        if (j.bus_index < 0) ctx.fail(where, "bus '" + j.bus + "' is not defined");
        const BusConfig& bus = c.buses[static_cast<size_t>(j.bus_index)];
        if (j.id < 1 || j.id > 126) ctx.fail(where, "id must be 1..126");  // 127 is the enumeration broadcast
        if (!bus.fd && !bus.usb) ctx.fail(where, "the actuators need a CAN FD bus (fd = true)");
        if (j.output_encoder && !j.continuous) {
            // Position commands go to the drive in the motor encoder's frame, which is not
            // related to the output encoder's until the two are aligned.
            ctx.fail(where, "output_encoder is not supported yet for position controlled joints");
        }
        const float dir = j.transmission.direction;
        if (dir != 1.0f && dir != -1.0f) ctx.fail(where, "direction must be 1 or -1");
        if (!(j.transmission.gear_ratio > 0)) ctx.fail(where, "gear_ratio must be positive");
        if (!(j.transmission.torque_constant > 0)) ctx.fail(where, "torque_constant must be positive");
        if (!j.continuous && !(j.q_max > j.q_min)) ctx.fail(where, "q_max must be greater than q_min");
        if (!(j.qd_max > 0) || !(j.qdd_max > 0) || !(j.tau_max > 0)) {
            ctx.fail(where, "qd_max, qdd_max and tau_max must be positive");
        }
        if (j.kp < 0 || j.kd < 0 || j.kp_max < 0 || j.kd_max < 0) ctx.fail(where, "gains must not be negative");
        if (j.kp_max == 0) {
            j.kp_max = 4 * j.kp;
        }
        if (j.kd_max == 0) {
            j.kd_max = 4 * j.kd;
        }
        if (j.kp > j.kp_max || j.kd > j.kd_max) ctx.fail(where, "kp / kd exceed kp_max / kd_max");
        j.sim_plant.q_min = j.continuous ? -INFINITY : j.q_min - 2 * j.limit_margin;
        j.sim_plant.q_max = j.continuous ? INFINITY : j.q_max + 2 * j.limit_margin;

        for (const auto& other : c.joints) {
            if (other.name == j.name) ctx.fail(where, "duplicate joint name");
            if (other.bus_index == j.bus_index && other.id == j.id) {
                ctx.fail(where,
                         "id " + std::to_string(j.id) + " is already used on bus " + j.bus + " by " + other.name);
            }
        }
        c.joints.push_back(j);
    }

    if (const auto* t = subtable(ctx, root, "top level", "base")) {
        check_keys(ctx, *t, "[base]", {"left", "right", "wheel_radius", "track_width", "v_max", "omega_max"});
        c.base.present = true;
        get(ctx, *t, "[base]", "left", c.base.left);
        get(ctx, *t, "[base]", "right", c.base.right);
        get(ctx, *t, "[base]", "wheel_radius", c.base.wheel_radius);
        get(ctx, *t, "[base]", "track_width", c.base.track_width);
        get(ctx, *t, "[base]", "v_max", c.base.v_max);
        get(ctx, *t, "[base]", "omega_max", c.base.omega_max);
        for (size_t i = 0; i < c.joints.size(); ++i) {
            if (c.joints[i].name == c.base.left) c.base.left_index = static_cast<int>(i);
            if (c.joints[i].name == c.base.right) c.base.right_index = static_cast<int>(i);
        }
        if (c.base.left_index < 0 || c.base.right_index < 0 || c.base.left_index == c.base.right_index) {
            ctx.fail("[base]", "'left' and 'right' must name two different joints");
        }
        if (!c.joints[static_cast<size_t>(c.base.left_index)].continuous ||
            !c.joints[static_cast<size_t>(c.base.right_index)].continuous) {
            ctx.fail("[base]", "wheel joints must be continuous");
        }
        if (!(c.base.wheel_radius > 0) || !(c.base.track_width > 0) || !(c.base.v_max > 0) || !(c.base.omega_max > 0)) {
            ctx.fail("[base]", "wheel_radius, track_width, v_max and omega_max must be positive");
        }
    }
    return c;
}

}  // namespace

RobotConfig parse_config(const std::string& toml_text, const std::string& source_name) {
    Ctx ctx{source_name};
    toml::table root;
    try {
        root = toml::parse(toml_text, source_name);
    } catch (const toml::parse_error& e) {
        std::ostringstream os;
        os << source_name << ":" << e.source().begin.line << ": " << e.description();
        throw std::runtime_error(os.str());
    }
    return from_table(ctx, root);
}

RobotConfig load_config(const std::string& path) {
    Ctx ctx{path};
    toml::table root;
    try {
        root = toml::parse_file(path);
    } catch (const toml::parse_error& e) {
        std::ostringstream os;
        os << path << ":" << e.source().begin.line << ": " << e.description();
        throw std::runtime_error(os.str());
    }
    return from_table(ctx, root);
}

void override_interfaces(RobotConfig& config, bool all_sim, const std::vector<std::string>& assignments) {
    if (all_sim) {
        for (auto& b : config.buses) b.interface = "sim";
    }
    for (const std::string& a : assignments) {
        const size_t eq = a.find('=');
        if (eq == std::string::npos || eq == 0 || eq + 1 == a.size()) {
            throw std::runtime_error("bus override '" + a + "' is not NAME=INTERFACE");
        }
        const std::string name = a.substr(0, eq);
        bool found = false;
        for (auto& b : config.buses) {
            if (b.name == name) {
                b.interface = a.substr(eq + 1);
                found = true;
            }
        }
        if (!found) throw std::runtime_error("bus override '" + a + "': no bus named " + name);
    }
}

std::vector<BusLoad> bus_loads(const RobotConfig& config) {
    std::vector<BusLoad> loads;
    for (const auto& b : config.buses) loads.push_back(BusLoad{b.name, 0, 0, 0});
    for (const auto& j : config.joints) {
        const BusConfig& b = config.buses[static_cast<size_t>(j.bus_index)];
        const can::BitTiming timing{b.bitrate, b.data_bitrate};
        // Per cycle and joint: a 48 byte command and a 64 byte status, CAN FD with bit rate switch.
        BusLoad& l = loads[static_cast<size_t>(j.bus_index)];
        l.busy_s += can::frame_time_s(48, true, true, timing) + can::frame_time_s(64, true, true, timing);
        l.frames += 2;
    }
    for (size_t i = 0; i < loads.size(); ++i) {
        // USB is not limited by bit time the way CAN is; only CAN buses carry a budget.
        if (config.buses[i].usb) loads[i].busy_s = 0;
        loads[i].load = loads[i].busy_s / (config.period_us * 1e-6);
    }
    return loads;
}

}  // namespace obot::robot
