#include "doctest.h"

#include <string>

#include "obot/robot/config.h"

using namespace obot::robot;

namespace {

const char* kGood = R"(
[robot]
name = "t"
period_us = 1000

[[bus]]
name = "a"
interface = "sim"

[[bus]]
name = "b"
interface = "can7"

[[joint]]
name = "j1"
bus = "a"
id = 1
gear_ratio = 9
torque_constant = 0.1
q_min = -1
q_max = 1
qd_max = 2
qdd_max = 10
tau_max = 5
kp = 20
kd = 1

[[joint]]
name = "j2"
bus = "b"
id = 2
q_min = -1
q_max = 1
kp = 20
kd = 1

[[joint]]
name = "wl"
bus = "a"
id = 2
continuous = true

[[joint]]
name = "wr"
bus = "a"
id = 3
continuous = true
direction = -1

[base]
left = "wl"
right = "wr"
wheel_radius = 0.1
track_width = 0.4
)";

std::string error_of(const std::string& text) {
    try {
        parse_config(text, "cfg");
    } catch (const std::runtime_error& e) {
        return e.what();
    }
    return "";
}

std::string replaced(std::string text, const std::string& from, const std::string& to) {
    const size_t at = text.find(from);
    REQUIRE(at != std::string::npos);
    text.replace(at, from.size(), to);
    return text;
}

}  // namespace

TEST_CASE("a valid configuration loads with defaults filled in") {
    const RobotConfig c = parse_config(kGood);
    CHECK(c.name == "t");
    CHECK(c.buses.size() == 2);
    CHECK(c.joints.size() == 4);
    CHECK(c.joints[0].transmission.gear_ratio == 9);
    CHECK(c.joints[0].bus_index == 0);
    CHECK(c.joints[1].bus_index == 1);
    CHECK(c.joints[0].kp_max == 80);    // 4 * kp when not given
    CHECK(c.rt.rx_deadline_us == 600);  // 60 % of the period when not given
    CHECK(c.base.present);
    CHECK(c.base.left_index == 2);
    CHECK(c.base.right_index == 3);
    CHECK(c.joints[3].transmission.direction == -1);
}

TEST_CASE("mistakes are reported with the place they were made") {
    CHECK(error_of("[robot]\nname = \"x\"\n").find("[[bus]]") != std::string::npos);
    CHECK(error_of(replaced(kGood, "period_us = 1000", "period_us = 1000\nperiod = 3")).find("unknown key 'period'") !=
          std::string::npos);
    CHECK(error_of(replaced(kGood, "gear_ratio = 9", "gear_ratio = \"9\"")).find("[[joint]] j1") != std::string::npos);
    CHECK(error_of(replaced(kGood, "bus = \"b\"", "bus = \"c\"")).find("bus 'c' is not defined") != std::string::npos);
    // Two actuators with the same id on one bus.
    CHECK(error_of(replaced(kGood, "name = \"wl\"\nbus = \"a\"\nid = 2", "name = \"wl\"\nbus = \"a\"\nid = 1"))
              .find("already used") != std::string::npos);
    // The actuators' protocol is CAN FD only.
    CHECK(error_of(replaced(kGood, "interface = \"can7\"", "interface = \"can7\"\nfd = false")).find("CAN FD") !=
          std::string::npos);
    CHECK(error_of(replaced(kGood, "q_max = 1\nqd_max = 2", "q_max = -2\nqd_max = 2")).find("q_max") !=
          std::string::npos);
    CHECK(error_of(replaced(kGood, "direction = -1", "direction = 0.5")).find("direction") != std::string::npos);
    // A position that wraps is a wheel's, not an arm joint's.
    CHECK(error_of(replaced(kGood, "gear_ratio = 9", "gear_ratio = 9\nwrap = 6.28")).find("wrap") != std::string::npos);
    CHECK(parse_config(replaced(kGood, "id = 3\ncontinuous = true", "id = 3\ncontinuous = true\nwrap = 6.2832"))
              .joints[3]
              .wrap == doctest::Approx(6.2832f));
    CHECK(error_of(replaced(kGood, "left = \"wl\"", "left = \"j1\"")).find("continuous") != std::string::npos);
    // There is one kind of actuator; the MIT-mode CubeMars driver is gone.
    CHECK(error_of(replaced(kGood, "bus = \"b\"\nid = 2", "bus = \"b\"\ndriver = \"cubemars_mit\"\nid = 2"))
              .find("unknown driver") != std::string::npos);
    CHECK(
        parse_config(replaced(kGood, "bus = \"b\"\nid = 2", "bus = \"b\"\ndriver = \"obot\"\nid = 2")).joints.size() ==
        4);
    CHECK(error_of(replaced(kGood, "gear_ratio = 9", "gear_ratio = 9\noutput_encoder = true")).find("output_encoder") !=
          std::string::npos);
    CHECK(error_of("this is not toml").find("cfg:1") != std::string::npos);
}

TEST_CASE("bus load") {
    RobotConfig c = parse_config(kGood);
    const auto loads = bus_loads(c);
    REQUIRE(loads.size() == 2);
    CHECK(loads[0].frames == 6);  // three obot joints on bus a
    CHECK(loads[1].frames == 2);
    CHECK(loads[1].busy_s == doctest::Approx(307e-6).epsilon(0.01));  // a 48 byte command and a 64 byte status
    CHECK(loads[0].load > 0.8);                                       // three joints do not fit in 1 ms with margin
    CHECK(loads[1].load == doctest::Approx(0.307).epsilon(0.01));

    c.period_us = 2000;
    CHECK(bus_loads(c)[0].load < 0.5);
}

TEST_CASE("interface overrides") {
    RobotConfig c = parse_config(kGood);
    override_interfaces(c, false, {"b=vcan3"});
    CHECK(c.buses[1].interface == "vcan3");
    CHECK_THROWS_AS(override_interfaces(c, false, {"nope=vcan0"}), std::runtime_error);
    CHECK_THROWS_AS(override_interfaces(c, false, {"b"}), std::runtime_error);
    override_interfaces(c, true, {});
    CHECK(c.buses[1].interface == "sim");
}
