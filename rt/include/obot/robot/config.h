// Robot description loaded from a TOML file (see config/obot3.toml for every key).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "obot/actuator/driver.h"
#include "obot/sim/devices.h"

namespace obot::robot {

struct BusConfig {
    std::string name;
    // CAN FD interface ("can0"); "usbrt" for boards on the usb_rt driver; "sim" for
    // built-in simulated actuators. In every case the motors are reached through
    // motor-realtime.
    std::string interface;
    bool usb = false;  // configured as "usbrt" (kept when the interface is overridden for simulation)
    bool fd = true;
    double bitrate = 1'000'000;
    double data_bitrate = 5'000'000;
};

struct JointConfig {
    std::string name;
    std::string bus;
    int id = 1;
    int bus_index = 0;     // filled in by the loader
    std::string usb_name;  // on a "usbrt" bus: the board's name, defaults to the joint name

    actuator::Transmission transmission;
    bool idle_damped = true;
    bool output_encoder = false;
    bool torque_sensor = false;
    float damped_kd = 0;

    bool continuous = false;  // no position limits (wheels)
    float wrap = 0;           // continuous joints: the drive's position wraps by this much, rad
    float q_min = 0;          // commands are clamped to q_min..q_max
    float q_max = 0;
    float limit_margin = 0.05f;  // measured position this far outside the limits is a fault, rad
    float qd_max = 1;            // rad/s
    float qdd_max = 10;          // rad/s^2
    float tau_max = 1;           // N m
    float kp = 0;                // Position mode gains
    float kd = 0;
    float kp_max = 0;  // Impedance mode gain limits; 0 means 4 * kp, 4 * kd
    float kd_max = 0;
    float temperature_max = 90;  // deg C

    sim::PlantParams sim_plant;  // only used with simulated actuators
    float sim_current_limit = 40;
};

struct BaseConfig {
    bool present = false;
    std::string left;
    std::string right;
    int left_index = -1;
    int right_index = -1;
    float wheel_radius = 0.1f;  // m
    float track_width = 0.3f;   // m, between wheel centres
    float v_max = 0.5f;         // m/s
    float omega_max = 1.5f;     // rad/s
};

struct RtConfig {
    int cpu = -1;       // CPU to pin the control thread to, -1 leaves it unpinned
    int priority = 80;  // SCHED_FIFO priority, 0 runs without a realtime policy
    bool lock_memory = true;
    int rx_deadline_us = 0;  // how long to wait for replies each cycle; 0 means 60 % of the period
};

struct RobotConfig {
    std::string name = "obot";
    std::string shm_name = "obot";
    int period_us = 1000;
    int command_timeout_ms = 100;  // commands older than this are not followed
    int comm_timeout_cycles = 5;   // consecutive cycles without a status before a joint faults
    bool clear_drive_faults_on_start = true;
    double max_bus_load = 0.7;  // refuse to start if a bus would be busier than this
    RtConfig rt;
    std::vector<BusConfig> buses;
    std::vector<JointConfig> joints;
    BaseConfig base;
};

// Throws std::runtime_error naming the file and the offending key.
RobotConfig load_config(const std::string& path);
RobotConfig parse_config(const std::string& toml_text, const std::string& source_name = "<string>");

// Command line overrides: run every bus on built-in simulated actuators, or move named
// buses to other interfaces ("arm_a=vcan0"). Throws std::runtime_error on an unknown bus.
void override_interfaces(RobotConfig& config, bool all_sim, const std::vector<std::string>& assignments);

struct BusLoad {
    std::string bus;
    double busy_s = 0;  // worst-case time on the wire per cycle
    double load = 0;    // busy_s / period
    int frames = 0;
};
std::vector<BusLoad> bus_loads(const RobotConfig& config);

}  // namespace obot::robot
