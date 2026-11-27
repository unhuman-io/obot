// Turns a position target that may jump (a policy at 30 Hz, a teleop device, a typed
// command) into a setpoint that respects velocity and acceleration limits at the control
// rate. No allocation, no branches that depend on history length.
#pragma once

#include <algorithm>
#include <cmath>

namespace obot::control {

class SetpointFilter {
 public:
    void configure(float v_max, float a_max) {
        v_max_ = v_max;
        a_max_ = a_max;
    }

    void reset(float x, float v = 0) {
        x_ = x;
        v_ = v;
    }

    // Advance one step towards target. Returns the new position; velocity() goes with it.
    float update(float target, float dt) {
        const float err = target - x_;
        const float step_dv = a_max_ * dt;
        // Close enough to land exactly on the target within this step without exceeding
        // the acceleration limit.
        if (std::fabs(err) <= 0.5f * step_dv * dt + std::fabs(v_) * dt && std::fabs(v_) <= step_dv) {
            x_ = target;
            v_ = 0;
            return x_;
        }
        // Fastest approach speed from which a constant deceleration still stops at the
        // target, evaluated one step ahead so the discrete loop does not overshoot.
        const float v_stop = std::sqrt(2.0f * a_max_ * std::fabs(err));
        const float v_cap = std::max(0.0f, v_stop - 0.5f * step_dv);
        const float v_des = std::copysign(std::min(v_max_, v_cap), err);
        v_ += std::clamp(v_des - v_, -step_dv, step_dv);
        x_ += v_ * dt;
        return x_;
    }

    // Where the setpoint comes to rest if it starts braking now.
    float stopping_point() const { return x_ + std::copysign(v_ * v_ / (2.0f * a_max_), v_); }

    float position() const { return x_; }
    float velocity() const { return v_; }

 private:
    float v_max_ = 1;
    float a_max_ = 1;
    float x_ = 0;
    float v_ = 0;
};

// Acceleration-limited velocity setpoint.
class RateLimiter {
 public:
    void reset(float v = 0) { v_ = v; }
    float update(float target, float a_max, float dt) {
        v_ += std::clamp(target - v_, -a_max * dt, a_max * dt);
        return v_;
    }
    float value() const { return v_; }

 private:
    float v_ = 0;
};

}  // namespace obot::control
