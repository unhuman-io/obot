// obot-rtd: the realtime daemon. The only process that talks to the actuators.
//
//   obot-rtd --config /etc/obot/robot.toml
//
// One thread runs the control cycle at the configured period under SCHED_FIFO on the
// configured CPU; the main thread only reports. Everything else on the robot talks to
// this process through the shared memory region (see obot/ipc/layout.h).
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

#include <motor_thread.h>

#include "obot/robot/motor_factory.h"
#include "obot/ipc/layout_json.h"
#include "obot/ipc/shm_region.h"
#include "obot/robot/config.h"
#include "obot/robot/runtime.h"
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
    bool print_can_setup = false;
    int cpu = -2;  // -2: use the configured value
    std::vector<std::string> bus_overrides;
    std::string shm_name;
};

void usage() {
    std::fprintf(stderr,
                 "usage: obot-rtd --config FILE [options]\n"
                 "  --sim              run every bus on built-in simulated actuators\n"
                 "  --bus NAME=IFACE   use another CAN interface for a bus (repeatable)\n"
                 "  --shm-name NAME    shared memory name, instead of the configured one\n"
                 "  --cpu N            pin the control thread to this CPU instead of the configured one\n"
                 "  --print-can-setup  print the 'ip link' commands that configure the CAN interfaces, and exit\n"
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
        } else if (a == "--cpu") {
            if (!value(v)) return false;
            o.cpu = std::atoi(v.c_str());
        } else if (a == "--print-can-setup") {
            o.print_can_setup = true;
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

// The realtime loop is motor-realtime's MotorThread: each period it reads every motor's
// status, calls controller_update() below, and writes the commands that leaves in the
// motor manager. In its fault tolerant mode a motor that is late or gone does not stop
// it; what the robot does about such a motor is the Runtime's business.
class RobotThread : public MotorThread {
 public:
    RobotThread(const robot::RobotConfig& config, bool realtime) : config_(config), runtime_(config) {
        set_period_ns(static_cast<uint32_t>(config.period_us) * 1000u);
        set_fault_tolerant();
        set_poll_timeout(static_cast<uint32_t>(config.rt.rx_deadline_us) * 1000u);
        // A fixed priority on a CPU of its own (isolcpus): SCHED_DEADLINE, the library's
        // default, does not take a thread that is pinned.
        set_scheduler(realtime && config.rt.priority > 0 ? FIFO : NORMAL, config.rt.priority);
        set_cpu(realtime ? config.rt.cpu : -1);
        set_lock_memory(realtime && config.rt.lock_memory);
    }

    // Opens the motors the configuration names. Throws std::runtime_error.
    void open_motors() {
        robot::OpenedMotors opened = robot::open_motors(config_);
        simulated_ = opened.simulated;
        try {
            motor_manager_.set_motors(opened.motors);  // checks the messages version
            init();
        } catch (const std::exception& e) {
            throw std::runtime_error(e.what());
        }
    }

    ipc::RobotInfo info() const { return runtime_.info(); }
    void set_shm(ipc::ShmRegion* shm) { shm_ = shm; }
    // After the drives have been sent idle commands for a few cycles following g_stop.
    bool finished() const { return finished_.load(); }
    uint32_t loop_overruns() const { return overruns(); }

    rt::Histogram latency;
    rt::Histogram exec;

 protected:
    void controller_update() override {
        if (shutdown_cycles_ < 0 && g_stop.load(std::memory_order_relaxed)) {
            runtime_.request_shutdown();
            shutdown_cycles_ = 20;  // enough cycles for every drive to receive an idle command
        }

        const auto ns = [](std::chrono::steady_clock::time_point t) {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
        };
        robot::CycleTiming timing;
        timing.now_mono_ns = ns(data_.time_start);  // steady_clock is CLOCK_MONOTONIC, as command stamps are
        timing.now_real_ns = rt::realtime_ns();
        timing.wake_latency_ns = static_cast<int32_t>(wake_latency_ns_);
        timing.overrun = overrun_;
        timing.exec_ns = last_exec_ns_;
        timing.rx_wait_ns = static_cast<int32_t>(ns(data_.read_time) - ns(data_.time_start));

        // Into a scratch copy first: a read that loses the race with a writer leaves its
        // output half written, and the previous command must survive that.
        if (shm_->command_slot().read(scratch_)) {
            command_ = scratch_;
            have_command_ = true;
        }
        const robot::MotorIo io{data_.statuses, data_.status_fresh, data_.write_failed,
                                motor_manager_.commands().data()};
        runtime_.update(timing, have_command_ ? &command_ : nullptr, io, state_);
        shm_->ring().push(state_);
        latency.add(wake_latency_ns_);

        if (shutdown_cycles_ > 0 && --shutdown_cycles_ == 0) finished_.store(true);
    }

    // After the commands have gone out.
    void post_update() override {
        const float dt = static_cast<float>(config_.period_us) * 1e-6f;
        for (sim::SimMotor* motor : simulated_) motor->step(dt);
        last_exec_ns_ = static_cast<int32_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - data_.time_start)
                .count());
        exec.add(last_exec_ns_);
    }

 private:
    robot::RobotConfig config_;
    robot::Runtime runtime_;
    std::vector<sim::SimMotor*> simulated_;
    ipc::ShmRegion* shm_ = nullptr;
    ipc::RobotCommand command_{};
    ipc::RobotCommand scratch_{};
    bool have_command_ = false;
    ipc::RobotState state_{};
    int shutdown_cycles_ = -1;
    int32_t last_exec_ns_ = 0;
    std::atomic<bool> finished_{false};
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

    robot::RobotConfig loaded;
    try {
        loaded = robot::load_config(opt.config);
        robot::override_interfaces(loaded, opt.sim, opt.bus_overrides);
        if (!opt.shm_name.empty()) loaded.shm_name = opt.shm_name;
        if (opt.cpu > -2) loaded.rt.cpu = opt.cpu;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "obot-rtd: %s\n", e.what());
        return 1;
    }
    const robot::RobotConfig& config = loaded;

    if (opt.print_can_setup) {
        // The bit rates live in the robot configuration; the system applies them with this.
        for (const robot::BusConfig& b : config.buses) {
            if (b.interface == "sim" || b.interface == "usbrt" || b.interface.rfind("vcan", 0) == 0) continue;
            std::printf("ip link set %s down\n", b.interface.c_str());
            if (b.fd) {
                std::printf("ip link set %s type can bitrate %.0f dbitrate %.0f fd on restart-ms 100\n",
                            b.interface.c_str(), b.bitrate, b.data_bitrate);
            } else {
                std::printf("ip link set %s type can bitrate %.0f fd off restart-ms 100\n", b.interface.c_str(),
                            b.bitrate);
            }
            std::printf("ip link set %s txqueuelen 64 up\n", b.interface.c_str());
        }
        return 0;
    }

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

    // On the heap: it holds the library's history of the last cycles' records.
    auto loop = std::make_unique<RobotThread>(config, !opt.no_rt);
    try {
        loop->open_motors();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "obot-rtd: %s\n", e.what());
        return 1;
    }

    ipc::ShmRegion shm = [&] {
        try {
            return ipc::ShmRegion::create(config.shm_name, loop->info());
        } catch (const std::exception& e) {
            std::fprintf(stderr, "obot-rtd: %s\n", e.what());
            std::exit(1);
        }
    }();
    loop->set_shm(&shm);
    shm.publish();

    struct sigaction sa{};
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    loop->run();  // returns once the thread has set itself up
    for (const std::string& p : loop->setup_errors()) {
        std::fprintf(stderr, "obot-rtd: warning: %s\n", p.c_str());
    }
    std::fprintf(stderr, "obot-rtd: %s running, %d us period, %zu joints, kernel %s, shm /dev/shm/%s\n",
                 config.name.c_str(), config.period_us, config.joints.size(),
                 rt::kernel_is_preempt_rt() ? "PREEMPT_RT" : "not PREEMPT_RT", config.shm_name.c_str());

    // Reporting, from the same records every other client reads.
    const auto t0 = std::chrono::steady_clock::now();
    uint32_t last_mode = UINT32_MAX;
    uint32_t last_fault = 0;
    ipc::RobotState state{};
    while (!loop->finished()) {
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
    loop->done();
    shm.unlink();

    std::fprintf(stderr, "obot-rtd: wake latency %s\n", loop->latency.to_json().c_str());
    std::fprintf(stderr, "obot-rtd: cycle time   %s\n", loop->exec.to_json().c_str());
    if (!opt.stats_json.empty()) {
        std::ofstream f(opt.stats_json);
        f << "{\"period_us\": " << config.period_us
          << ", \"preempt_rt\": " << (rt::kernel_is_preempt_rt() ? "true" : "false")
          << ", \"overruns\": " << state.overruns << ",\n \"wake_latency\": " << loop->latency.to_json()
          << ",\n \"cycle_time\": " << loop->exec.to_json() << "}\n";
    }
    return 0;
}
