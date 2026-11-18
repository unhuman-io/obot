// obot-sim-node: simulated actuators on real SocketCAN interfaces (normally vcan).
//
//   obot-sim-node --config robot.toml [--bus arm_a=vcan0 ...]
//
// For every bus in the configuration whose interface is not "sim", opens that interface
// and answers as the actuators configured on it would. obot-rtd then runs unmodified
// against it, which exercises the SocketCAN path end to end without hardware.
#include <poll.h>
#include <signal.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "obot/can/socketcan_bus.h"
#include "obot/robot/config.h"
#include "obot/robot/sim_factory.h"
#include "obot/rt/clock.h"
#include "obot/sim/devices.h"

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) {
    g_stop.store(true);
}
}  // namespace

int main(int argc, char** argv) {
    using namespace obot;
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

    std::vector<std::unique_ptr<can::SocketCanBus>> buses;
    std::vector<std::unique_ptr<sim::DeviceBus>> device_buses;
    std::vector<pollfd> fds;
    try {
        robot::RobotConfig config = robot::load_config(config_path);
        robot::override_interfaces(config, false, bus_overrides);
        for (size_t b = 0; b < config.buses.size(); ++b) {
            const robot::BusConfig& bc = config.buses[b];
            if (bc.interface == "sim") continue;
            buses.push_back(std::make_unique<can::SocketCanBus>(bc.name, bc.interface, bc.fd));
            auto device_bus = std::make_unique<sim::DeviceBus>(*buses.back());
            for (const robot::JointConfig& j : config.joints) {
                if (j.bus_index == static_cast<int>(b)) device_bus->add(robot::make_sim_device(j, config));
            }
            std::fprintf(stderr, "obot-sim-node: %zu simulated actuators on %s\n", device_bus->size(),
                         bc.interface.c_str());
            device_buses.push_back(std::move(device_bus));
            fds.push_back(pollfd{buses.back()->fd(), POLLIN, 0});
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "obot-sim-node: %s\n", e.what());
        return 1;
    }
    if (device_buses.empty()) {
        std::fprintf(stderr, "obot-sim-node: no bus in %s uses a real interface\n", config_path.c_str());
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
            for (auto& db : device_buses) db->step(static_cast<float>(step) * 1e-9f);
            simulated_to += step;
        }
        for (auto& db : device_buses) db->pump();
    }
    return 0;
}
