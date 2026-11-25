// Simulated actuators, so the whole stack runs without hardware. ObotDeviceSim behaves
// like a drive running the obot-controller firmware as far as its messages show. It is
// used two ways:
//   - SimMotor: a motor-realtime Motor backed by one, in this process (unit tests,
//     `interface = "sim"`, `obot-rtd --sim`). The daemon talks to it exactly as it talks
//     to a real motor.
//   - obot-sim-node: nodes on a CAN interface such as vcan0, for testing against the
//     library's real CAN transport.
#pragma once

#include <cstdint>
#include <limits>
#include <string>

#include <motor.h>

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

struct ObotDeviceParams {
    float gear_ratio = 1;
    float torque_constant = 0.1f;   // N m / A at the motor
    float current_limit = 40;       // A
    float host_timeout_s = 0.05f;   // 0 disables; otherwise no command for this long faults the drive
    float damped_gain = 1.0f;       // A s / rad (motor), behaviour of DAMPED: windings shorted
    float velocity_bandwidth = 30;  // rad/s, VELOCITY mode: a critically damped PI tuned for the plant
    float temperature = 30;         // deg C, reported through round robin
    float bus_voltage = 48;         // V
    PlantParams plant;
};

// Behaves like obot-controller firmware as far as the host can see it.
class ObotDeviceSim {
 public:
    explicit ObotDeviceSim(const ObotDeviceParams& params);
    void on_command(const MotorCommand& command);
    // The status the drive would send now. false if it is not answering.
    bool make_status(MotorStatus& out);
    void step(float dt);

    JointPlant& plant() { return plant_; }
    // Test hooks.
    void inject_error(uint32_t error_bits) { error_ |= error_bits; }
    void set_silent(bool silent) { silent_ = silent; }
    uint8_t mode() const { return mode_; }
    bool safe_mode() const { return safe_mode_; }
    uint32_t last_host_timestamp() const { return host_timestamp_; }

 private:
    ObotDeviceParams p_;
    JointPlant plant_;
    MotorCommand command_{};
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

// A motor-realtime Motor whose other end is an ObotDeviceSim. A command written to it is
// answered with a status, as a real drive answers when asked to.
class SimMotor : public Motor {
 public:
    SimMotor(const std::string& name, const ObotDeviceParams& params);

    ssize_t write() override;
    ssize_t read() override;
    ssize_t aread() override { return 0; }
    int set_nonblock() override {
        nonblock_ = true;
        return 0;
    }
    int clear_nonblock() override {
        nonblock_ = false;
        return 0;
    }
    int lock() override { return 0; }
    void set_timeout_ms(int) override {}
    int get_timeout_ms() const override { return 0; }

    ObotDeviceSim& device() { return device_; }
    void step(float dt) { device_.step(dt); }
    // Test hooks: a drive without power neither hears nor answers; a write that fails.
    void set_powered(bool powered) { powered_ = powered; }
    void fail_writes(int count) { fail_writes_ = count; }

 private:
    ObotDeviceSim device_;
    MotorStatus reply_{};
    bool pending_ = false;
    bool powered_ = true;
    int fail_writes_ = 0;
};

}  // namespace obot::sim
