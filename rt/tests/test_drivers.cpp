#include "doctest.h"

#include <cmath>
#include <cstring>

#include "obot/wire/obot_messages.h"
#include "obot/actuator/cubemars_driver.h"
#include "obot/actuator/obot_driver.h"
#include "obot/can/fake_bus.h"
#include "obot/sim/devices.h"

using namespace obot;
using actuator::CubeMarsDriver;
using actuator::DriveCommand;
using actuator::ObotDriver;
using ipc::JointMode;

namespace {
MotorCommand decode(const can::Frame& f) {
    MotorCommand mc{};
    std::memcpy(&mc, f.data, sizeof(mc));
    return mc;
}
}  // namespace

TEST_CASE("obot driver: frame ids and sizes follow the firmware's CAN protocol") {
    actuator::ObotDriverConfig cfg;
    cfg.address = 5;
    ObotDriver d(cfg);
    can::Frame f;

    d.make_frame(DriveCommand{}, true, 1, f);  // status request
    CHECK(f.id == ((3u << 7) | 5u));
    CHECK(f.len == 0);
    CHECK(f.fd);

    d.make_frame(DriveCommand{}, false, 77, f);  // command, asks for a status back
    CHECK(f.id == ((2u << 7) | 5u));
    CHECK(f.len == 48);
    CHECK(f.fd);
    CHECK(f.brs);
    CHECK(decode(f).host_timestamp == 77);
    CHECK(decode(f).mode_desired == DAMPED);
    CHECK(d.rx_id() == ((3u << 7) | 5u));
}

TEST_CASE("obot driver: joint units are converted to motor units") {
    actuator::ObotDriverConfig cfg;
    cfg.transmission.gear_ratio = 10;
    cfg.transmission.torque_constant = 0.1f;
    cfg.transmission.direction = -1;
    cfg.transmission.zero_offset = 0.5f;
    ObotDriver d(cfg);
    can::Frame f;

    DriveCommand c;
    c.mode = JointMode::Impedance;
    c.q = 1.0f;
    c.qd = 2.0f;
    c.kp = 100.0f;  // N m / rad at the joint
    c.kd = 4.0f;
    c.tau = 3.0f;
    d.make_frame(c, false, 0, f);
    const MotorCommand mc = decode(f);
    CHECK(mc.mode_desired == STATE);
    CHECK(mc.state.position_desired == doctest::Approx(-10.0f * 1.5f));
    CHECK(mc.state.velocity_desired == doctest::Approx(-20.0f));
    // 1 rad of joint error is 10 rad of motor error and must produce 100 N m at the joint
    // = 10 N m at the motor = 100 A: kp = 10 A per motor rad.
    CHECK(mc.state.kp == doctest::Approx(10.0f));
    CHECK(mc.state.kd == doctest::Approx(0.4f));
    CHECK(mc.state.current_desired == doctest::Approx(-3.0f));

    c.mode = JointMode::Torque;
    d.make_frame(c, false, 0, f);
    CHECK(decode(f).mode_desired == CURRENT);
    CHECK(decode(f).current_desired == doctest::Approx(-3.0f));

    c.mode = JointMode::Velocity;
    d.make_frame(c, false, 0, f);
    CHECK(decode(f).mode_desired == VELOCITY);
    CHECK(decode(f).velocity_desired == doctest::Approx(-20.0f));

    c.clear_faults = true;
    d.make_frame(c, false, 0, f);
    CHECK(decode(f).mode_desired == CLEAR_FAULTS);
}

TEST_CASE("obot driver: status is converted back to joint units") {
    actuator::ObotDriverConfig cfg;
    cfg.address = 2;
    cfg.transmission.gear_ratio = 10;
    cfg.transmission.torque_constant = 0.1f;
    cfg.transmission.direction = -1;
    cfg.transmission.zero_offset = 0.5f;
    ObotDriver d(cfg);

    MotorStatusRegular ms{};
    ms.motor_position = -15.0f;
    ms.motor_velocity = -20.0f;
    ms.iq = -3.0f;
    ms.flags.mode = STATE;
    ms.rr_data.index = BUS_VOLTAGE_INDEX;
    ms.rr_data.type = FLOAT;
    ms.rr_data.data = 47.5f;
    can::Frame f;
    f.id = d.rx_id();
    f.fd = true;
    f.set_payload(&ms, sizeof(ms), 64);
    f.rx_time_ns = 1000;
    REQUIRE(d.on_frame(f));
    CHECK(d.status().q == doctest::Approx(1.0f));
    CHECK(d.status().qd == doctest::Approx(2.0f));
    CHECK(d.status().tau == doctest::Approx(3.0f));
    CHECK(d.status().bus_voltage == doctest::Approx(47.5f));
    CHECK(std::isnan(d.status().temperature));
    CHECK_FALSE(d.status().fault);
    CHECK(d.status().rx_count == 1);

    ms.flags.error.all = ERROR_MASK_MOTOR_ENCODER | 0x80000000u;
    ms.rr_data.index = MOTOR_TEMPERATURE_INDEX;
    ms.rr_data.data = 55.0f;
    f.set_payload(&ms, sizeof(ms), 64);
    REQUIRE(d.on_frame(f));
    CHECK(d.status().fault);
    CHECK(d.status().error == (ERROR_MASK_MOTOR_ENCODER | 0x80000000u));
    CHECK(d.status().temperature == doctest::Approx(55.0f));
    CHECK(d.status().bus_voltage == doctest::Approx(47.5f));  // kept from the earlier frame

    f.len = 8;  // too short to be a status
    CHECK_FALSE(d.on_frame(f));
}

TEST_CASE("obot driver: the 48 byte status has no velocity, so it is differenced") {
    ObotDriver d(actuator::ObotDriverConfig{});
    MotorStatusLite ms{};
    can::Frame f;
    f.id = d.rx_id();
    float q = 0;
    for (int i = 1; i <= 200; ++i) {
        q += 0.002f;  // 2 rad/s at 1 kHz
        ms.motor_position = q;
        f.set_payload(&ms, sizeof(ms), 48);
        f.rx_time_ns = static_cast<int64_t>(i) * 1'000'000;
        REQUIRE(d.on_frame(f));
    }
    CHECK(d.status().qd == doctest::Approx(2.0f).epsilon(0.02));
}

TEST_CASE("cubemars: MIT packing round trips within the resolution of each field") {
    const actuator::CubeMarsRanges r;
    uint8_t data[8];
    CubeMarsDriver::pack_mit(1.234f, -3.21f, 123.0f, 2.5f, -4.5f, r, data);
    float p, v, kp, kd, t;
    CubeMarsDriver::unpack_mit(data, r, p, v, kp, kd, t);
    CHECK(p == doctest::Approx(1.234f).epsilon(0.001));
    CHECK(std::fabs(v + 3.21f) < 2 * r.v_max / 4095);
    CHECK(std::fabs(kp - 123.0f) < r.kp_max / 4095);
    CHECK(std::fabs(kd - 2.5f) < r.kd_max / 4095);
    CHECK(std::fabs(t + 4.5f) < 2 * r.t_max / 4095);

    // Out of range values saturate rather than wrap.
    CubeMarsDriver::pack_mit(100.0f, 0, 0, 0, 1000.0f, r, data);
    CubeMarsDriver::unpack_mit(data, r, p, v, kp, kd, t);
    CHECK(p == doctest::Approx(r.p_max));
    CHECK(t == doctest::Approx(r.t_max));
}

TEST_CASE("cubemars: enter motor mode before the first command, exit when idle") {
    actuator::CubeMarsDriverConfig cfg;
    cfg.id = 3;
    CubeMarsDriver d(cfg);
    can::Frame f;

    d.make_frame(DriveCommand{}, true, 0, f);
    CHECK(f.id == 3);
    CHECK(f.len == 8);
    CHECK_FALSE(f.fd);
    CHECK(CubeMarsDriver::is_special(f.data, CubeMarsDriver::kExitMotorMode));

    DriveCommand c;
    c.mode = JointMode::Position;
    c.q = 0.5f;
    c.kp = 50;
    c.kd = 1;
    d.make_frame(c, false, 0, f);
    CHECK(CubeMarsDriver::is_special(f.data, CubeMarsDriver::kEnterMotorMode));
    d.make_frame(c, false, 0, f);
    float p, v, kp, kd, t;
    CubeMarsDriver::unpack_mit(f.data, cfg.ranges, p, v, kp, kd, t);
    CHECK(p == doctest::Approx(0.5f).epsilon(0.001));
    CHECK(kp == doctest::Approx(50.0f).epsilon(0.01));

    c.mode = JointMode::Idle;
    d.make_frame(c, false, 0, f);
    CHECK(CubeMarsDriver::is_special(f.data, CubeMarsDriver::kExitMotorMode));
    c.mode = JointMode::Position;
    d.make_frame(c, false, 0, f);
    CHECK(CubeMarsDriver::is_special(f.data, CubeMarsDriver::kEnterMotorMode));
}

TEST_CASE("cubemars: driver against the simulated actuator") {
    actuator::CubeMarsDriverConfig cfg;
    cfg.id = 4;
    cfg.transmission.direction = -1;
    CubeMarsDriver d(cfg);
    sim::CubeMarsDeviceParams p;
    p.id = 4;
    p.plant.inertia = 0.01f;
    p.plant.damping = 0.02f;
    can::FakeBusPair pair("b");
    sim::DeviceBus devices(pair.device());
    devices.add(std::make_unique<sim::CubeMarsDeviceSim>(p));

    DriveCommand c;
    c.mode = JointMode::Position;
    c.q = 0.8f;
    c.kp = 40;
    c.kd = 1.5f;
    can::Frame f, rx[4];
    for (int i = 0; i < 3000; ++i) {
        d.make_frame(c, false, 0, f);
        pair.host().send(f);
        devices.pump();
        devices.step(0.001f);
        const int n = pair.host().recv(rx, 4);
        REQUIRE(n == 1);
        REQUIRE(rx[0].id == d.rx_id());
        REQUIRE(d.on_frame(rx[0]));
    }
    CHECK(d.status().q == doctest::Approx(0.8f).epsilon(0.01));
    CHECK(std::fabs(d.status().qd) < 0.05f);
    // direction = -1: the actuator itself went the other way.
    auto& device = static_cast<sim::CubeMarsDeviceSim&>(devices.device(0));
    CHECK(device.plant().q == doctest::Approx(-0.8f).epsilon(0.01));
    CHECK(device.motor_mode());
    CHECK(d.status().temperature == doctest::Approx(30.0f));

    // A reply carrying another actuator's id is not ours.
    rx[0].data[0] = 9;
    CHECK_FALSE(d.on_frame(rx[0]));
}

TEST_CASE("obot simulated actuator: host timeout faults it, clear faults recovers") {
    sim::ObotDeviceParams p;
    p.address = 1;
    p.host_timeout_s = 0.02f;
    can::FakeBusPair pair("b");
    sim::DeviceBus devices(pair.device());
    devices.add(std::make_unique<sim::ObotDeviceSim>(p));
    auto& device = static_cast<sim::ObotDeviceSim&>(devices.device(0));
    ObotDriver d(actuator::ObotDriverConfig{});

    const auto cycle = [&](const DriveCommand& c, bool send) {
        can::Frame f, rx[4];
        if (send) {
            d.make_frame(c, false, 0, f);
            pair.host().send(f);
        }
        devices.pump();
        devices.step(0.001f);
        if (pair.host().recv(rx, 4) == 1) d.on_frame(rx[0]);
    };

    DriveCommand c;
    c.mode = JointMode::Torque;
    c.tau = 0.5f;
    for (int i = 0; i < 10; ++i) cycle(c, true);
    CHECK(device.mode() == CURRENT);
    CHECK_FALSE(d.status().fault);

    for (int i = 0; i < 40; ++i) cycle(c, false);  // host goes quiet
    CHECK(device.safe_mode());
    cycle(c, true);  // commands are ignored while faulted
    CHECK(d.status().fault);
    CHECK((d.status().error & ERROR_MASK_SEQUENCE) != 0);
    CHECK(device.mode() == DAMPED);

    c.clear_faults = true;
    cycle(c, true);
    c.clear_faults = false;
    for (int i = 0; i < 3; ++i) cycle(c, true);
    CHECK_FALSE(device.safe_mode());
    CHECK_FALSE(d.status().fault);
    CHECK(device.mode() == CURRENT);
}
