// obot-rt-latency: measure how late a periodic realtime thread wakes up on this machine,
// with exactly the thread setup obot-rtd uses. Run it on the target while the rest of
// the system is under its normal (or worse) load to decide whether the kernel and the
// CPU isolation are good enough for the control period.
//
//   obot-rt-latency --cpu 5 --priority 80 --period-us 1000 --duration 60 [--json FILE]
//
// Exit status 1 if --max-us is given and the worst latency exceeded it.
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

#include "obot/rt/clock.h"
#include "obot/rt/realtime.h"

int main(int argc, char** argv) {
    using namespace obot;
    int cpu = -1;
    int priority = 80;
    int period_us = 1000;
    double duration_s = 10;
    double max_us = 0;
    std::string json;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const char* v = i + 1 < argc ? argv[i + 1] : nullptr;
        if (a == "--cpu" && v) {
            cpu = std::atoi(v), ++i;
        } else if (a == "--priority" && v) {
            priority = std::atoi(v), ++i;
        } else if (a == "--period-us" && v) {
            period_us = std::atoi(v), ++i;
        } else if (a == "--duration" && v) {
            duration_s = std::atof(v), ++i;
        } else if (a == "--max-us" && v) {
            max_us = std::atof(v), ++i;
        } else if (a == "--json" && v) {
            json = v, ++i;
        } else {
            std::fprintf(stderr,
                         "usage: obot-rt-latency [--cpu N] [--priority P] [--period-us US] [--duration SEC]\n"
                         "                       [--max-us US] [--json FILE]\n");
            return 2;
        }
    }
    if (period_us < 50) {
        std::fprintf(stderr, "obot-rt-latency: period too short\n");
        return 2;
    }

    const std::string lock_problem = rt::lock_memory();
    if (!lock_problem.empty()) std::fprintf(stderr, "warning: %s\n", lock_problem.c_str());
    for (const std::string& p : rt::setup_thread(rt::ThreadSetup{cpu, priority})) {
        std::fprintf(stderr, "warning: could not %s\n", p.c_str());
    }

    static rt::Histogram latency;
    const int64_t period = static_cast<int64_t>(period_us) * 1000;
    const int64_t end = rt::monotonic_ns() + static_cast<int64_t>(duration_s * 1e9);
    int64_t next = rt::monotonic_ns() + period;
    while (next < end) {
        const timespec wake = rt::to_timespec(next);
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake, nullptr) == EINTR) {
        }
        latency.add(rt::monotonic_ns() - next);
        next += period;
    }

    const std::string result = "{\"cpu\": " + std::to_string(cpu) + ", \"priority\": " + std::to_string(priority) +
                               ", \"period_us\": " + std::to_string(period_us) +
                               ", \"preempt_rt\": " + (rt::kernel_is_preempt_rt() ? "true" : "false") +
                               ", \"wake_latency\": " + latency.to_json() + "}";
    std::printf("%s\n", result.c_str());
    if (!json.empty()) std::ofstream(json) << result << "\n";
    if (max_us > 0 && static_cast<double>(latency.max_ns()) / 1000.0 > max_us) {
        std::fprintf(stderr, "worst latency %.1f us exceeds %.1f us\n", static_cast<double>(latency.max_ns()) / 1000.0,
                     max_us);
        return 1;
    }
    return 0;
}
