// Driver for CubeMars AK series actuators with their stock driver board in "MIT mode":
// classic CAN at 1 Mbit/s, one 8 byte command and one 8 byte reply per cycle.
//
// NOT YET CHECKED AGAINST HARDWARE. The packing follows the published AK series manual
// (and the mini-cheetah convention it derives from); the value ranges differ per model and
// must be copied from the manual of the exact actuator into the robot configuration.
#pragma once

#include "obot/actuator/driver.h"

namespace obot::actuator {

struct CubeMarsRanges {
    float p_max = 12.5f;    // rad, position range is -p_max..p_max
    float v_max = 45.0f;    // rad/s
    float t_max = 15.0f;    // N m
    float kp_max = 500.0f;  // N m / rad
    float kd_max = 5.0f;    // N m s / rad
};

struct CubeMarsDriverConfig {
    uint8_t id = 1;
    uint32_t reply_id = 0;      // CAN id of the reply frame; 0 means the same as id
    Transmission transmission;  // gear_ratio and torque_constant are not used: the drive works in joint units
    CubeMarsRanges ranges;
    float damped_kd = 1.0f;  // gain used for Damped, N m s / rad
};

class CubeMarsDriver : public Driver {
 public:
    explicit CubeMarsDriver(const CubeMarsDriverConfig& config);

    void make_frame(const DriveCommand& command, bool poll_only, uint32_t cycle, can::Frame& out) override;
    uint32_t rx_id() const override { return config_.reply_id ? config_.reply_id : config_.id; }
    bool on_frame(const can::Frame& frame) override;
    unsigned tx_len() const override { return 8; }
    unsigned rx_len() const override { return 8; }
    bool uses_fd() const override { return false; }

    // Packing helpers, shared with the simulated device.
    static uint32_t to_uint(float x, float lo, float hi, int bits);
    static float to_float(uint32_t x, float lo, float hi, int bits);
    static void pack_mit(float p, float v, float kp, float kd, float t, const CubeMarsRanges& r, uint8_t out[8]);
    static void unpack_mit(const uint8_t in[8], const CubeMarsRanges& r, float& p, float& v, float& kp, float& kd,
                           float& t);
    static constexpr uint8_t kEnterMotorMode = 0xFC;
    static constexpr uint8_t kExitMotorMode = 0xFD;
    static bool is_special(const uint8_t data[8], uint8_t which);

 private:
    CubeMarsDriverConfig config_;
    bool motor_mode_ = false;  // the drive has been told to enter motor mode
};

}  // namespace obot::actuator
