// Simulated actuators that speak the real CAN protocols, so the whole stack above the
// bus can run without hardware: in-process on a FakeBus (unit tests, `interface = "sim"`)
// or on a SocketCAN interface such as vcan0 (obot-sim-node).
#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include "obot/actuator/cubemars_driver.h"
#include "obot/can/bus.h"

namespace obot::sim {

// A single rigid joint: inertia, viscous damping, a gravity term and optional hard stops.
struct PlantParams {
    float inertia = 0.05f;  // kg m^2 at the joint
    float damping = 0.05f;  // N m s / rad
    float gravity = 0;      // N m: torque = -gravity * sin(q)
    float q_min = -std::numeric_limits<float>::infinity();
    float q_max = std::numeric_limits<float>::infinity();
};

class JointPlant {
 public:
    explicit JointPlant(const PlantParams& p) : p_(p) {}
    void step(float tau, float dt);
    float q = 0;
    float qd = 0;

 private:
    PlantParams p_;
};

class Device {
 public:
    virtual ~Device() = default;
    virtual bool accepts(uint32_t can_id) const = 0;
    virtual void on_frame(const can::Frame& frame, can::Bus& bus) = 0;
    virtual void step(float dt) = 0;
};

struct ObotDeviceParams {
    uint8_t address = 1;
    float gear_ratio = 1;
    float torque_constant = 0.1f;   // N m / A at the motor
    float current_limit = 40;       // A
    float host_timeout_s = 0.05f;   // 0 disables; otherwise no command for this long faults the drive
    float damped_gain = 0.02f;      // A s / rad (motor), behaviour of DAMPED
    float velocity_bandwidth = 30;  // rad/s, VELOCITY mode: a critically damped PI tuned for the plant
    float temperature = 30;         // deg C, reported through round robin
    float bus_voltage = 48;         // V
    bool lite_status = false;       // reply with the 48 byte status instead of the 60 byte one
    PlantParams plant;
};

// Behaves like obot-controller firmware as far as the host can see it.
class ObotDeviceSim : public Device {
 public:
    explicit ObotDeviceSim(const ObotDeviceParams& params);
    bool accepts(uint32_t can_id) const override;
    void on_frame(const can::Frame& frame, can::Bus& bus) override;
    void step(float dt) override;

    JointPlant& plant() { return plant_; }
    // Test hooks.
    void inject_error(uint32_t error_bits) { error_ |= error_bits; }
    void set_silent(bool silent) { silent_ = silent; }
    uint8_t mode() const { return mode_; }
    bool safe_mode() const { return safe_mode_; }
    uint32_t last_host_timestamp() const { return host_timestamp_; }

 private:
    void send_status(can::Bus& bus);
    ObotDeviceParams p_;
    JointPlant plant_;
    uint8_t command_[64] = {};
    uint8_t mode_ = 0;
    uint32_t error_ = 0;
    bool safe_mode_ = false;
    bool first_command_ = false;
    float since_command_s_ = 0;
    float iq_ = 0;
    float iq_desired_ = 0;
    float velocity_integral_ = 0;
    float velocity_kp_ = 0;  // A s / rad (motor)
    float velocity_ki_ = 0;  // A / rad (motor)
    uint32_t host_timestamp_ = 0;
    uint32_t mcu_timestamp_ = 0;
    uint8_t rr_index_ = 0;
    bool silent_ = false;
};

struct CubeMarsDeviceParams {
    uint8_t id = 1;
    uint32_t reply_id = 0;  // 0: reply with the same id
    actuator::CubeMarsRanges ranges;
    float temperature = 30;
    PlantParams plant;
};

class CubeMarsDeviceSim : public Device {
 public:
    explicit CubeMarsDeviceSim(const CubeMarsDeviceParams& params);
    bool accepts(uint32_t can_id) const override { return can_id == p_.id; }
    void on_frame(const can::Frame& frame, can::Bus& bus) override;
    void step(float dt) override;

    JointPlant& plant() { return plant_; }
    bool motor_mode() const { return motor_mode_; }
    void inject_error(uint8_t code) { error_ = code; }

 private:
    CubeMarsDeviceParams p_;
    JointPlant plant_;
    bool motor_mode_ = false;
    float p_des_ = 0, v_des_ = 0, kp_ = 0, kd_ = 0, t_ff_ = 0;
    float tau_ = 0;
    uint8_t error_ = 0;
};

// The devices on one bus.
class DeviceBus {
 public:
    explicit DeviceBus(can::Bus& bus) : bus_(bus) {}
    void add(std::unique_ptr<Device> device) { devices_.push_back(std::move(device)); }
    // Deliver pending frames to the devices (which may reply). Returns frames handled.
    int pump();
    void step(float dt);
    Device& device(size_t i) { return *devices_[i]; }
    size_t size() const { return devices_.size(); }

 private:
    can::Bus& bus_;
    std::vector<std::unique_ptr<Device>> devices_;
};

}  // namespace obot::sim
