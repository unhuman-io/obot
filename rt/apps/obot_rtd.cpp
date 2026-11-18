// obot-rtd: the realtime daemon. The only process that talks to the actuators.
//
//   obot-rtd --config /etc/obot/robot.toml
//
// One thread runs the control cycle at the configured period under SCHED_FIFO on the
// configured CPU; the main thread only reports. Everything else on the robot talks to
// this process through the shared memory region (see obot/ipc/layout.h).
#include <poll.h>
#include <signal.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "obot/can/fake_bus.h"
#include "obot/can/socketcan_bus.h"
#include "obot/ipc/layout_json.h"
#include "obot/ipc/shm_region.h"
#include "obot/robot/config.h"
#include "obot/robot/runtime.h"
#include "obot/robot/sim_factory.h"
#include "obot/rt/clock.h"
#include "obot/rt/realtime.h"
#include "obot/sim/devices.h"

namespace {

using namespace obot;

std::atomic<bool> g_stop{false};
void on_signal(int) {
    g_stop.store(true);
}

const char* mode_name(uint32_t mode) {
    static const char* names[] = {"init", "idle", "active", "fault", "estop"};
    return mode < 5 ? names[mode] : "?";
}

struct Options {
    std::string config;
    std::string stats_json;
    double duration_s = 0;
    bool check = false;
    bool dump_layout = false;
    bool no_rt = false;
    bool sim = false;
    std::vector<std::string> bus_overrides;
    std::string shm_name;
};

void usage() {
    std::fprintf(stderr,
                 "usage: obot-rtd --config FILE [options]\n"
                 "  --sim              run every bus on built-in simulated actuators\n"
                 "  --bus NAME=IFACE   use another CAN interface for a bus (repeatable)\n"
                 "  --shm-name NAME    shared memory name, instead of the configured one\n"
                 "  --check            validate the configuration and the bus load, then exit\n"
                 "  --duration SEC     run for this long, then exit (default: until SIGINT/SIGTERM)\n"
                 "  --stats-json FILE  write loop timing statistics to FILE on exit\n"
                 "  --no-rt            do not lock memory, pin or raise priority\n"
                 "  --dump-layout      print the shared memory layout as JSON and exit\n");
}

bool parse(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const auto value = [&](std::string& out) {
            if (i + 1 >= argc) return false;
            out = argv[++i];
            return true;
        };
        std::string v;
        if (a == "--config") {
            if (!value(o.config)) return false;
        } else if (a == "--stats-json") {
            if (!value(o.stats_json)) return false;
        } else if (a == "--duration") {
            if (!value(v)) return false;
            o.duration_s = std::atof(v.c_str());
        } else if (a == "--bus") {
            if (!value(v)) return false;
            o.bus_overrides.push_back(v);
        } else if (a == "--shm-name") {
            if (!value(o.shm_name)) return false;
        } else if (a == "--sim") {
            o.sim = true;
        } else if (a == "--check") {
            o.check = true;
        } else if (a == "--dump-layout") {
            o.dump_layout = true;
        } else if (a == "--no-rt") {
            o.no_rt = true;
        } else {
            return false;
        }
    }
    return o.dump_layout || !o.config.empty();
}

// Everything the control thread touches.
struct Loop {
    robot::RobotConfig config;
    std::vector<std::unique_ptr<can::Bus>> real_buses;
    std::vector<std::unique_ptr<can::FakeBusPair>> sim_pairs;
    std::vector<std::unique_ptr<sim::DeviceBus>> sim_buses;
    std::vector<can::Bus*> buses;
    std::vector<pollfd> fds;
    std::unique_ptr<robot::Runtime> runtime;
    ipc::ShmRegion* shm = nullptr;
    rt::Histogram latency;
    rt::Histogram exec;
    std::vector<std::string> setup_problems;
    std::atomic<bool> setup_done{false};
    bool realtime = true;

    void run() {
        if (realtime) {
            setup_problems = rt::setup_thread(rt::ThreadSetup{config.rt.cpu, config.rt.priority});
        }
        setup_done.store(true);

        const int64_t period = static_cast<int64_t>(config.period_us) * 1000;
        const int64_t rx_deadline = static_cast<int64_t>(config.rt.rx_deadline_us) * 1000;
        const float dt = static_cast<float>(config.period_us) * 1e-6f;
        ipc::RobotCommand command{};
        bool have_command = false;
        ipc::RobotState state{};
        bool overrun = false;
        int shutdown_cycles = -1;
        int64_t next = rt::monotonic_ns() + period;

        for (;;) {
            const timespec wake = rt::to_timespec(next);
            while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake, nullptr) == EINTR) {
            }
            const int64_t start = rt::monotonic_ns();

            if (shutdown_cycles < 0 && g_stop.load(std::memory_order_relaxed)) {
                runtime->request_shutdown();
                shutdown_cycles = 20;  // enough cycles for every drive to receive an idle command
            }

            robot::CycleTiming timing;
            timing.now_mono_ns = start;
            timing.now_real_ns = rt::realtime_ns();
            timing.wake_latency_ns = static_cast<int32_t>(start - next);
            timing.overrun = overrun;
            if (shm->command_slot().read(command)) have_command = true;
            runtime->begin_cycle(timing, have_command ? &command : nullptr);

            // Simulated actuators answer immediately; real ones are waited for.
            for (auto& sb : sim_buses) {
                sb->pump();
                sb->step(dt);
            }
            int missing = runtime->poll_rx(rt::monotonic_ns());
            const int64_t deadline = next + rx_deadline;
            while (missing > 0 && !fds.empty()) {
                const int64_t remaining = deadline - rt::monotonic_ns();
                if (remaining <= 0) break;
                const timespec timeout = rt::to_timespec(remaining);
                const int ready = ppoll(fds.data(), fds.size(), &timeout, nullptr);
                if (ready < 0 && errno != EINTR) break;
                missing = runtime->poll_rx(rt::monotonic_ns());
            }

            const int64_t end = rt::monotonic_ns();
            runtime->end_cycle(end, state);
            shm->ring().push(state);
            latency.add(start - next);
            exec.add(end - start);

            if (shutdown_cycles > 0 && --shutdown_cycles == 0) break;

            next += period;
            const int64_t now = rt::monotonic_ns();
            overrun = now > next;
            if (overrun) {
                // Skip the periods that were missed instead of running a burst to catch up.
                next += ((now - next) / period + 1) * period;
            }
        }
    }
};

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parse(argc, argv, opt)) {
        usage();
        return 2;
    }
    if (opt.dump_layout) {
        std::fputs(ipc::layout_json().c_str(), stdout);
        return 0;
    }

    Loop loop;
    try {
        loop.config = robot::load_config(opt.config);
        robot::override_interfaces(loop.config, opt.sim, opt.bus_overrides);
        if (!opt.shm_name.empty()) loop.config.shm_name = opt.shm_name;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "obot-rtd: %s\n", e.what());
        return 1;
    }
    const robot::RobotConfig& config = loop.config;

    bool load_ok = true;
    for (const robot::BusLoad& l : robot::bus_loads(config)) {
        const bool ok = l.load <= config.max_bus_load;
        load_ok = load_ok && ok;
        std::fprintf(stderr, "obot-rtd: bus %-8s %2d frames/cycle, worst case %4.0f us of %d us = %3.0f %%%s\n",
                     l.bus.c_str(), l.frames, l.busy_s * 1e6, config.period_us, l.load * 100,
                     ok ? "" : "  <-- over max_bus_load");
    }
    if (!load_ok) {
        std::fprintf(stderr,
                     "obot-rtd: the frames for one cycle do not fit on the bus with margin. Use a longer\n"
                     "          period, move joints to another bus, or raise the data bit rate.\n");
        return 1;
    }
    if (opt.check) {
        std::fprintf(stderr, "obot-rtd: %s: ok (%zu joints, %zu buses)\n", opt.config.c_str(), config.joints.size(),
                     config.buses.size());
        return 0;
    }

    try {
        for (size_t b = 0; b < config.buses.size(); ++b) {
            const robot::BusConfig& bc = config.buses[b];
            if (bc.interface == "sim") {
                loop.sim_pairs.push_back(std::make_unique<can::FakeBusPair>(bc.name));
                auto device_bus = std::make_unique<sim::DeviceBus>(loop.sim_pairs.back()->device());
                for (const robot::JointConfig& j : config.joints) {
                    if (j.bus_index == static_cast<int>(b)) device_bus->add(robot::make_sim_device(j, config));
                }
                loop.sim_buses.push_back(std::move(device_bus));
                loop.buses.push_back(&loop.sim_pairs.back()->host());
            } else {
                loop.real_buses.push_back(std::make_unique<can::SocketCanBus>(
                    bc.name, bc.interface, bc.fd, robot::rx_ids(config, static_cast<int>(b))));
                loop.buses.push_back(loop.real_buses.back().get());
                loop.fds.push_back(pollfd{loop.real_buses.back()->fd(), POLLIN, 0});
            }
        }
        loop.runtime = std::make_unique<robot::Runtime>(config, loop.buses);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "obot-rtd: %s\n", e.what());
        return 1;
    }

    loop.realtime = !opt.no_rt;
    if (loop.realtime && config.rt.lock_memory) {
        const std::string problem = rt::lock_memory();
        if (!problem.empty()) std::fprintf(stderr, "obot-rtd: warning: %s\n", problem.c_str());
    }

    ipc::ShmRegion shm = ipc::ShmRegion::create(config.shm_name, loop.runtime->info());
    loop.shm = &shm;
    shm.publish();

    struct sigaction sa{};
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    std::thread control([&loop] { loop.run(); });
    while (!loop.setup_done.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    for (const std::string& p : loop.setup_problems) {
        std::fprintf(stderr, "obot-rtd: warning: could not %s\n", p.c_str());
    }
    std::fprintf(stderr, "obot-rtd: %s running, %d us period, %zu joints, kernel %s, shm /dev/shm/%s\n",
                 config.name.c_str(), config.period_us, config.joints.size(),
                 rt::kernel_is_preempt_rt() ? "PREEMPT_RT" : "not PREEMPT_RT", config.shm_name.c_str());

    // Reporting, from the same records every other client reads.
    const auto t0 = std::chrono::steady_clock::now();
    uint32_t last_mode = UINT32_MAX;
    uint32_t last_fault = 0;
    ipc::RobotState state{};
    while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (opt.duration_s > 0 &&
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() >= opt.duration_s) {
            g_stop.store(true);
        }
        if (!shm.ring().latest(state)) continue;
        if (state.mode != last_mode || state.fault != last_fault) {
            std::fprintf(stderr, "obot-rtd: mode %s, fault 0x%x", mode_name(state.mode), state.fault);
            for (uint32_t i = 0; i < state.num_joints; ++i) {
                if (state.joint_fault[i]) {
                    std::fprintf(stderr, ", %s fault 0x%x (drive error 0x%08x)", config.joints[i].name.c_str(),
                                 state.joint_fault[i], state.drive_error[i]);
                }
            }
            std::fprintf(stderr, "\n");
            last_mode = state.mode;
            last_fault = state.fault;
        }
    }
    control.join();
    shm.unlink();

    std::fprintf(stderr, "obot-rtd: wake latency %s\n", loop.latency.to_json().c_str());
    std::fprintf(stderr, "obot-rtd: cycle time   %s\n", loop.exec.to_json().c_str());
    if (!opt.stats_json.empty()) {
        std::ofstream f(opt.stats_json);
        f << "{\"period_us\": " << config.period_us
          << ", \"preempt_rt\": " << (rt::kernel_is_preempt_rt() ? "true" : "false")
          << ", \"overruns\": " << state.overruns << ",\n \"wake_latency\": " << loop.latency.to_json()
          << ",\n \"cycle_time\": " << loop.exec.to_json() << "}\n";
    }
    return 0;
}
