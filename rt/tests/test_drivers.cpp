// Joint space <-> the firmware's messages, and the path those messages take: through
// motor-realtime's Motor and MotorManager, here with simulated motors on the other end.
#include "doctest.h"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include "obot/actuator/motors.h"
#include "obot/actuator/obot_driver.h"
#include "obot/sim/devices.h"

using namespace obot;
using actuator::DriveCommand;
using actuator::ObotDriver;
using ipc::JointMode;

TEST_CASE("obot driver: joint units are converted to motor units") {
    actuator::ObotDriverConfig cfg;
    cfg.transmission.gear_ratio = 10;
    cfg.transmission.torque_constant = 0.1f;
    cfg.transmission.direction = -1;
    cfg.transmission.zero_offset = 0.5f;
    ObotDriver d(cfg);
    MotorCommand mc;

    d.fill(DriveCommand{}, 77, mc);  // nothing asked: idle, which is DAMPED unless configured open
    CHECK(mc.host_timestamp == 77);
    CHECK(mc.mode_desired == DAMPED);

    DriveCommand c;
    c.mode = JointMode::Impedance;
    c.q = 1.0f;
    c.qd = 2.0f;
    c.kp = 100.0f;  // N m / rad at the joint
    c.kd = 4.0f;
    c.tau = 3.0f;
    d.fill(c, 0, mc);
    CHECK(mc.mode_desired == STATE);
    CHECK(mc.state.position_desired == doctest::Approx(-10.0f * 1.5f));
    CHECK(mc.state.velocity_desired == doctest::Approx(-20.0f));
    // 1 rad of joint error is 10 rad of motor error and must produce 100 N m at the joint
    // = 10 N m at the motor = 100 A: kp = 10 A per motor rad.
    CHECK(mc.state.kp == doctest::Approx(10.0f));
    CHECK(mc.state.kd == doctest::Approx(0.4f));
    CHECK(mc.state.current_desired == doctest::Approx(-3.0f));

    c.mode = JointMode::Torque;
    d.fill(c, 0, mc);
    CHECK(mc.mode_desired == CURRENT);
    CHECK(mc.current_desired == doctest::Approx(-3.0f));

    c.mode = JointMode::Velocity;
    d.fill(c, 0, mc);
    CHECK(mc.mode_desired == VELOCITY);
    CHECK(mc.velocity_desired == doctest::Approx(-20.0f));

    c.clear_faults = true;
    d.fill(c, 0, mc);
    CHECK(mc.mode_desired == CLEAR_FAULTS);

    cfg.idle_damped = false;
    ObotDriver(cfg).fill(DriveCommand{}, 0, mc);
    CHECK(mc.mode_desired == OPEN);
}

TEST_CASE("obot driver: status is converted back to joint units") {
    actuator::ObotDriverConfig cfg;
    cfg.transmission.gear_ratio = 10;
    cfg.transmission.torque_constant = 0.1f;
    cfg.transmission.direction = -1;
    cfg.transmission.zero_offset = 0.5f;
    ObotDriver d(cfg);

    MotorStatus ms{};
    ms.motor_position = -15.0f;
    ms.motor_velocity = -20.0f;
    ms.iq = -3.0f;
    ms.flags.mode = STATE;
    ms.rr_data.index = BUS_VOLTAGE_INDEX;
    ms.rr_data.type = FLOAT;
    ms.rr_data.data = 47.5f;
    d.on_status(ms, 1000);
    CHECK(d.status().q == doctest::Approx(1.0f));
    CHECK(d.status().qd == doctest::Approx(2.0f));
    CHECK(d.status().tau == doctest::Approx(3.0f));
    CHECK(d.status().bus_voltage == doctest::Approx(47.5f));
    CHECK(std::isnan(d.status().temperature));
    CHECK_FALSE(d.status().fault);
    CHECK(d.status().rx_count == 1);
    CHECK(d.status().rx_time_ns == 1000);

    ms.flags.error.all = ERROR_MASK_MOTOR_ENCODER | 0x80000000u;
    ms.rr_data.index = MOTOR_TEMPERATURE_INDEX;
    ms.rr_data.data = 55.0f;
    d.on_status(ms, 2000);
    CHECK(d.status().fault);
    CHECK(d.status().error == (ERROR_MASK_MOTOR_ENCODER | 0x80000000u));
    CHECK(d.status().temperature == doctest::Approx(55.0f));
    CHECK(d.status().bus_voltage == doctest::Approx(47.5f));  // kept from the earlier status
}

TEST_CASE("motors: commands go out and statuses come back through motor-realtime") {
    sim::ObotDeviceParams p;
    auto a = std::make_shared<sim::SimMotor>("a", p);
    auto b = std::make_shared<sim::SimMotor>("b", p);
    actuator::Motors motors({a, b});
    REQUIRE(motors.size() == 2);
    CHECK(motors.motor(1).name() == "b");

    motors.read();  // nothing was asked yet
    CHECK_FALSE(motors.fresh(0));
    CHECK_FALSE(motors.fresh(1));

    motors.command(0).mode_desired = CURRENT;
    motors.command(0).current_desired = 2.0f;
    motors.command(0).host_timestamp = 1;
    motors.command(1).mode_desired = DAMPED;
    motors.command(1).host_timestamp = 1;
    motors.write();
    CHECK_FALSE(motors.tx_failed(0));
    a->step(0.001f);
    b->step(0.001f);
    motors.wait(100'000);  // simulated motors have answered already: does not wait
    motors.read();
    CHECK(motors.fresh(0));
    CHECK(motors.fresh(1));
    CHECK(motors.status(0).host_timestamp_received == 1);
    CHECK(a->device().mode() == CURRENT);
    CHECK(b->device().mode() == DAMPED);

    // Reading again without a new answer: the old status is still there, but not fresh.
    motors.read();
    CHECK_FALSE(motors.fresh(0));
    CHECK(motors.status(0).host_timestamp_received == 1);

    // A motor that cannot be written to does not keep the other from being commanded.
    a->fail_writes(1);
    motors.command(0).host_timestamp = 2;
    motors.command(1).host_timestamp = 2;
    motors.write();
    CHECK(motors.tx_failed(0));
    CHECK_FALSE(motors.tx_failed(1));
    motors.read();
    CHECK_FALSE(motors.fresh(0));
    CHECK(motors.fresh(1));
    CHECK(b->device().last_host_timestamp() == 2);
    CHECK(a->device().last_host_timestamp() == 1);

    // A drive without power: the command is "sent", nothing comes back.
    b->set_powered(false);
    motors.command(1).host_timestamp = 3;
    motors.write();
    CHECK_FALSE(motors.tx_failed(1));
    motors.read();
    CHECK_FALSE(motors.fresh(1));
}

namespace {
// As a board with firmware built for another major version of the messages reports itself.
class OldFirmware : public sim::SimMotor {
 public:
    OldFirmware() : SimMotor("old", sim::ObotDeviceParams{}) { messages_version_ = "3.1"; }
};
}  // namespace

TEST_CASE("motors: firmware that speaks another messages version is refused") {
    CHECK_THROWS_WITH_AS(actuator::Motors({std::make_shared<OldFirmware>()}), doctest::Contains("version"),
                         std::runtime_error);
}

TEST_CASE("simulated actuator: host timeout faults it, clear faults recovers") {
    sim::ObotDeviceParams p;
    p.host_timeout_s = 0.02f;
    auto motor = std::make_shared<sim::SimMotor>("m", p);
    sim::ObotDeviceSim& device = motor->device();
    actuator::Motors motors({motor});
    ObotDriver d(actuator::ObotDriverConfig{});

    uint32_t count = 0;
    const auto cycle = [&](const DriveCommand& c, bool send) {
        if (send) {
            d.fill(c, ++count, motors.command(0));
            motors.write();
        }
        motor->step(0.001f);
        motors.read();
        if (motors.fresh(0)) d.on_status(motors.status(0), 0);
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
