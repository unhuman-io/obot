#include "doctest.h"

#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>
#include <string>

#include "obot/actuator/obot_driver.h"
#include "obot/can/usbrt_bus.h"
#include "obot/wire/obot_messages.h"

using namespace obot;

TEST_CASE("usb bus: commands go to the addressed board, statuses come back as status frames") {
    // Each board is a packet socket pair: [0] is what the bus opens, [1] plays the board.
    int a[2], b[2];
    REQUIRE(socketpair(AF_UNIX, SOCK_SEQPACKET, 0, a) == 0);
    REQUIRE(socketpair(AF_UNIX, SOCK_SEQPACKET, 0, b) == 0);
    can::UsbRtBus bus("usb", {{uint8_t{1}, a[0]}, {uint8_t{2}, b[0]}});
    std::vector<int> fds;
    bus.append_fds(fds);
    CHECK(fds.size() == 2);

    actuator::ObotDriverConfig cfg;
    cfg.address = 2;
    actuator::ObotDriver driver(cfg);
    actuator::DriveCommand command;
    command.mode = ipc::JointMode::Torque;
    command.tau = 1.5f;
    can::Frame frame;
    driver.make_frame(command, false, 7, frame);
    REQUIRE(bus.send(frame));

    // Board 2 receives exactly one MotorCommand; board 1 receives nothing.
    MotorCommand mc{};
    CHECK(::recv(b[1], &mc, sizeof(mc), MSG_DONTWAIT) == static_cast<ssize_t>(sizeof(MotorCommand)));
    CHECK(mc.mode_desired == CURRENT);
    CHECK(mc.host_timestamp == 7);
    char byte;
    CHECK(::recv(a[1], &byte, 1, MSG_DONTWAIT) == -1);

    // A status request sends nothing: the board streams.
    driver.make_frame(command, true, 8, frame);
    CHECK(bus.send(frame));
    CHECK(::recv(b[1], &byte, 1, MSG_DONTWAIT) == -1);

    // Nothing to read yet.
    can::Frame rx[4];
    CHECK(bus.recv(rx, 4) == 0);

    // The board sends two statuses before the host looks: only the newest is delivered.
    MotorStatusLite status{};
    status.motor_position = 1.0f;
    REQUIRE(::send(b[1], &status, sizeof(status), 0) == static_cast<ssize_t>(sizeof(status)));
    status.motor_position = 2.0f;
    REQUIRE(::send(b[1], &status, sizeof(status), 0) == static_cast<ssize_t>(sizeof(status)));
    REQUIRE(bus.recv(rx, 4) == 1);
    CHECK(rx[0].id == driver.rx_id());
    CHECK(rx[0].len == sizeof(MotorStatusLite));
    CHECK(rx[0].rx_time_ns > 0);
    REQUIRE(driver.on_frame(rx[0]));
    CHECK(driver.status().q == doctest::Approx(2.0f));

    // A command for a board that is not on this bus is refused.
    cfg.address = 9;
    actuator::ObotDriver other(cfg);
    other.make_frame(command, false, 0, frame);
    CHECK_FALSE(bus.send(frame));
    CHECK(bus.counters().tx_dropped == 1);

    ::close(a[1]);
    ::close(b[1]);
}

TEST_CASE("usb bus: a missing board is reported by name") {
    try {
        can::UsbRtBus bus("usb", std::vector<can::UsbRtDevice>{{1, "no_such_board_xyz"}});
        FAIL("expected an exception");
    } catch (const std::runtime_error& e) {
        CHECK(std::string(e.what()).find("no_such_board_xyz") != std::string::npos);
    }
}
