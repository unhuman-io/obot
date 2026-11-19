#include "doctest.h"

#include <cmath>

#include "obot/control/setpoint_filter.h"

using obot::control::RateLimiter;
using obot::control::SetpointFilter;

namespace {
struct Trace {
    float max_v = 0;
    float max_a = 0;
    float overshoot = 0;
    int steps_to_settle = -1;
};

Trace run(SetpointFilter& f, float target, float dt, int steps) {
    Trace t;
    const float start = f.position();
    const float direction = target >= start ? 1.0f : -1.0f;
    float last_v = f.velocity();
    for (int i = 0; i < steps; ++i) {
        const float x = f.update(target, dt);
        const float v = f.velocity();
        t.max_v = std::max(t.max_v, std::fabs(v));
        t.max_a = std::max(t.max_a, std::fabs(v - last_v) / dt);
        t.overshoot = std::max(t.overshoot, direction * (x - target));
        last_v = v;
        if (t.steps_to_settle < 0 && x == target && v == 0) t.steps_to_settle = i + 1;
    }
    return t;
}
}  // namespace

TEST_CASE("setpoint filter reaches a step target within its limits and without overshoot") {
    const float dt = 0.001f;
    for (float target : {1.0f, -2.5f, 0.003f, 0.2f}) {
        SetpointFilter f;
        f.configure(2.0f, 10.0f);
        f.reset(0);
        const Trace t = run(f, target, dt, 5000);
        CAPTURE(target);
        CHECK(t.steps_to_settle > 0);
        CHECK(f.position() == target);
        CHECK(f.velocity() == 0);
        CHECK(t.max_v <= 2.0f * 1.001f);
        CHECK(t.max_a <= 10.0f * 1.01f);
        CHECK(t.overshoot <= 1e-4f);
        // Not much slower than the time-optimal profile.
        const float d = std::fabs(target);
        const float t_opt = d > 0.4f ? d / 2.0f + 0.2f : 2.0f * std::sqrt(d / 10.0f);
        CHECK(static_cast<float>(t.steps_to_settle) * dt < t_opt * 1.15f + 0.02f);
    }
}

TEST_CASE("setpoint filter follows a moving target and survives a reversal") {
    const float dt = 0.001f;
    SetpointFilter f;
    f.configure(1.0f, 5.0f);
    f.reset(0);
    // A target that jumps every 33 ms, like a 30 Hz policy.
    float target = 0;
    float last_v = 0;
    for (int i = 0; i < 3000; ++i) {
        if (i % 33 == 0) target = 0.2f * std::sin(static_cast<float>(i) * 0.004f);
        f.update(target, dt);
        CHECK(std::fabs(f.velocity()) <= 1.0f * 1.001f);
        CHECK(std::fabs(f.velocity() - last_v) / dt <= 5.0f * 1.01f);
        last_v = f.velocity();
    }
    // Tracking a target moving at v lags by about v^2 / 2a: here at most 0.8^2 / 10.
    CHECK(std::fabs(f.position() - target) < 0.08f);

    // Moving fast one way, then asked to go the other way: it has to brake first.
    f.reset(0, 1.0f);
    const Trace t = run(f, -0.5f, dt, 4000);
    CHECK(f.position() == -0.5f);
    CHECK(t.max_a <= 5.0f * 1.01f);
}

TEST_CASE("stopping point is where the setpoint ends up when braking") {
    SetpointFilter f;
    f.configure(2.0f, 4.0f);
    f.reset(1.0f, 2.0f);
    const float stop = f.stopping_point();
    CHECK(stop == doctest::Approx(1.5f));
    run(f, stop, 0.001f, 2000);
    CHECK(f.position() == stop);
    f.reset(1.0f, -2.0f);
    CHECK(f.stopping_point() == doctest::Approx(0.5f));
}

TEST_CASE("rate limiter") {
    RateLimiter r;
    r.reset(0);
    for (int i = 0; i < 100; ++i) r.update(1.0f, 2.0f, 0.001f);
    CHECK(r.value() == doctest::Approx(0.2f));
    for (int i = 0; i < 1000; ++i) r.update(1.0f, 2.0f, 0.001f);
    CHECK(r.value() == doctest::Approx(1.0f));
}
