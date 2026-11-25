// obot-sim-node: simulated actuators on real CAN interfaces (normally vcan).
//
//   obot-sim-node --config robot.toml [--bus arm_a=vcan0 ...]
//
// For every bus in the configuration whose interface is not "sim" or "usbrt", opens that
// interface and answers as the actuators configured on it would: this program is the
// firmware's side of the CAN protocol. obot-rtd, motor_util and anything else built on
// motor-realtime then talk to it as to real drives, which exercises the library's CAN
// transport end to end without hardware.
//
// What a node answers (11 bit id = message << 7 | node id, CAN FD with bit rate switch):
//   1 command                      taken, no reply
//   2 command                      taken, replied to with a status (message 3)
//   3 empty frame                  a status
//   15 empty frame to the node     its description (EnumResponse)
//   15 to node 127                 every node answers with its description
// The text API (messages 4 and 5) is not simulated.
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "obot/robot/config.h"
#include "obot/robot/motor_factory.h"
#include "obot/rt/clock.h"
#include "obot/sim/devices.h"

namespace {
using namespace obot;

std::atomic<bool> g_stop{false};
void on_signal(int) {
    g_stop.store(true);
}

enum : uint32_t { kCommand = 1, kCommandStatus = 2, kStatus = 3, kEnum = 15, kBroadcast = 127 };

struct Node {
    uint32_t id;
    std::string name;
    std::shared_ptr<sim::SimMotor> motor;  // only its simulated drive is used here
};

struct Interface {
    std::string name;
    int fd = -1;
    std::vector<Node> nodes;

    void open() {
        fd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
        if (fd < 0) throw std::runtime_error("cannot open a CAN socket: " + std::string(std::strerror(errno)));
        int on = 1;
        ifreq request{};
        std::strncpy(request.ifr_name, name.c_str(), IFNAMSIZ - 1);
        sockaddr_can address{};
        address.can_family = AF_CAN;
        if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) != 0 ||
            ::ioctl(fd, SIOCGIFINDEX, &request) != 0) {
            throw std::runtime_error(name + ": " + std::strerror(errno) + " (is it a CAN FD interface that is up?)");
        }
        address.can_ifindex = request.ifr_ifindex;
        if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
            throw std::runtime_error(name + ": " + std::strerror(errno));
        }
    }

    void send(uint32_t message, uint32_t node, const void* data, uint8_t length, uint8_t padded) {
        canfd_frame frame{};
        frame.can_id = message << 7 | node;
        frame.len = padded;
        frame.flags = CANFD_BRS;
        std::memcpy(frame.data, data, length);
        (void)!::write(fd, &frame, sizeof(frame));
    }

    void send_status(Node& node) {
        MotorStatus status;
        if (node.motor->device().make_status(status)) send(kStatus, node.id, &status, 60, 64);
    }

    void send_description(const Node& node) {
        EnumResponse description{};
        description.messages_version[0] = MOTOR_MESSAGES_VERSION_MAJOR;
        description.messages_version[1] = MOTOR_MESSAGES_VERSION_MINOR;
        description.serial_number[5] = static_cast<uint8_t>(node.id);
        // These fields carry no terminator when they are full.
        std::memcpy(description.name, node.name.data(), std::min(node.name.size(), sizeof(description.name)));
        std::memcpy(description.board_name, "sim", 3);
        send(kEnum, node.id, &description, sizeof(description), 48);
    }

    // Answer everything that has arrived.
    void pump() {
        canfd_frame frame;
        while (::read(fd, &frame, sizeof(frame)) > 0) {
            const uint32_t id = frame.can_id & CAN_SFF_MASK;
            const uint32_t message = id >> 7;
            const uint32_t target = id & 0x7f;
            for (Node& node : nodes) {
                if (message == kEnum && frame.len == 0 && (target == node.id || target == kBroadcast)) {
                    send_description(node);
                }
                if (target != node.id) continue;
                if (message == kCommand || message == kCommandStatus) {
                    MotorCommand command{};
                    std::memcpy(&command, frame.data, std::min<size_t>(frame.len, sizeof(command)));
                    node.motor->device().on_command(command);
                    if (message == kCommandStatus) send_status(node);
                } else if (message == kStatus && frame.len == 0) {
                    send_status(node);
                }
            }
        }
    }
};
}  // namespace

int main(int argc, char** argv) {
    std::string config_path;
    std::vector<std::string> bus_overrides;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (a == "--bus" && i + 1 < argc) {
            bus_overrides.push_back(argv[++i]);
        } else {
            config_path.clear();
            break;
        }
    }
    if (config_path.empty()) {
        std::fprintf(stderr, "usage: obot-sim-node --config FILE [--bus NAME=IFACE ...]\n");
        return 2;
    }

    std::vector<Interface> interfaces;
    std::vector<pollfd> fds;
    try {
        robot::RobotConfig config = robot::load_config(config_path);
        robot::override_interfaces(config, false, bus_overrides);
        for (size_t b = 0; b < config.buses.size(); ++b) {
            const robot::BusConfig& bc = config.buses[b];
            if (bc.interface == "sim" || bc.interface == "usbrt") continue;
            Interface interface;
            interface.name = bc.interface;
            interface.open();
            for (const robot::JointConfig& j : config.joints) {
                if (j.bus_index != static_cast<int>(b)) continue;
                interface.nodes.push_back(Node{static_cast<uint32_t>(j.id), j.name, robot::make_sim_motor(j, config)});
            }
            std::fprintf(stderr, "obot-sim-node: %zu simulated actuators on %s\n", interface.nodes.size(),
                         bc.interface.c_str());
            fds.push_back(pollfd{interface.fd, POLLIN, 0});
            interfaces.push_back(std::move(interface));
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "obot-sim-node: %s\n", e.what());
        return 1;
    }
    if (interfaces.empty()) {
        std::fprintf(stderr, "obot-sim-node: no bus in %s uses a CAN interface\n", config_path.c_str());
        return 1;
    }

    struct sigaction sa{};
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // Physics advances with wall time in steps of at most 250 us; frames are answered as
    // soon as they arrive.
    constexpr int64_t kMaxStepNs = 250'000;
    int64_t simulated_to = rt::monotonic_ns();
    while (!g_stop.load()) {
        poll(fds.data(), fds.size(), 1);
        const int64_t now = rt::monotonic_ns();
        if (now - simulated_to > 100'000'000) simulated_to = now - 100'000'000;  // after a stall, do not replay it all
        while (simulated_to < now) {
            const int64_t step = std::min<int64_t>(kMaxStepNs, now - simulated_to);
            for (auto& interface : interfaces) {
                for (auto& node : interface.nodes) node.motor->step(static_cast<float>(step) * 1e-9f);
            }
            simulated_to += step;
        }
        for (auto& interface : interfaces) interface.pump();
    }
    return 0;
}
