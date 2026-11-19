#include "obot/rt/realtime.h"

#include <malloc.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/utsname.h>

#include <cerrno>
#include <cstring>
#include <fstream>
#include <sstream>

namespace obot::rt {

std::string lock_memory() {
    // Keep freed memory in the process and never use mmap for allocations, so that no
    // allocation made after start-up can page fault.
    mallopt(M_TRIM_THRESHOLD, -1);
    mallopt(M_MMAP_MAX, 0);
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        return std::string("mlockall: ") + std::strerror(errno);
    }
    return "";
}

std::vector<std::string> setup_thread(const ThreadSetup& setup) {
    std::vector<std::string> problems;
    if (setup.cpu >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(static_cast<unsigned>(setup.cpu), &set);
        const int rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
        if (rc != 0) {
            problems.push_back("pin to cpu " + std::to_string(setup.cpu) + ": " + std::strerror(rc));
        }
    }
    if (setup.priority > 0) {
        sched_param param{};
        param.sched_priority = setup.priority;
        const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        if (rc != 0) {
            problems.push_back("SCHED_FIFO priority " + std::to_string(setup.priority) + ": " + std::strerror(rc));
        }
    }
    // Touch the stack this thread will use so its pages are resident.
    volatile char stack[64 * 1024];
    for (size_t i = 0; i < sizeof(stack); i += 1024) stack[i] = 0;
    return problems;
}

bool kernel_is_preempt_rt() {
    std::ifstream f("/sys/kernel/realtime");
    int value = 0;
    if (f >> value) return value == 1;
    utsname u{};
    if (uname(&u) == 0) return std::strstr(u.version, "PREEMPT_RT") != nullptr;
    return false;
}

int64_t Histogram::percentile_us(double fraction) const {
    if (count_ == 0) return 0;
    const auto target = static_cast<uint64_t>(fraction * static_cast<double>(count_));
    uint64_t seen = 0;
    for (int i = 0; i < kBuckets; ++i) {
        seen += buckets_[static_cast<size_t>(i)];
        if (seen >= target && seen > 0) return i;
    }
    return max_ns_ / 1000;
}

std::string Histogram::to_json() const {
    std::ostringstream os;
    os << "{\"count\": " << count_ << ", \"min_us\": " << static_cast<double>(min_ns()) / 1000.0
       << ", \"mean_us\": " << mean_ns() / 1000.0 << ", \"p50_us\": " << percentile_us(0.5)
       << ", \"p99_us\": " << percentile_us(0.99) << ", \"p999_us\": " << percentile_us(0.999)
       << ", \"max_us\": " << static_cast<double>(max_ns_) / 1000.0 << ", \"over_2ms\": " << overflow_ << "}";
    return os.str();
}

}  // namespace obot::rt
