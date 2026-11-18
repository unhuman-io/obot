// Realtime thread setup and timing statistics.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace obot::rt {

struct ThreadSetup {
    int cpu = -1;      // -1: leave affinity alone
    int priority = 0;  // SCHED_FIFO priority, 0: leave the policy alone
};

// Lock current and future pages and pre-fault the heap arenas. Call once, before threads start.
// Returns a description of what could not be done ("" on success).
std::string lock_memory();

// Apply affinity and scheduling policy to the calling thread and touch its stack.
// Returns one message per setting that could not be applied; the caller decides whether
// that is fatal (it is not on a development machine without privileges).
std::vector<std::string> setup_thread(const ThreadSetup& setup);

// True when the running kernel is PREEMPT_RT.
bool kernel_is_preempt_rt();

// Fixed-size histogram of durations in microseconds. add() is realtime safe.
class Histogram {
 public:
    static constexpr int kBuckets = 2000;  // 1 us wide; longer samples land in overflow
    void add(int64_t ns) {
        const int64_t us = ns < 0 ? 0 : ns / 1000;
        if (us < kBuckets) {
            ++buckets_[static_cast<size_t>(us)];
        } else {
            ++overflow_;
        }
        ++count_;
        sum_ns_ += ns;
        if (ns > max_ns_) max_ns_ = ns;
        if (count_ == 1 || ns < min_ns_) min_ns_ = ns;
    }
    uint64_t count() const { return count_; }
    int64_t min_ns() const { return count_ ? min_ns_ : 0; }
    int64_t max_ns() const { return max_ns_; }
    double mean_ns() const { return count_ ? static_cast<double>(sum_ns_) / static_cast<double>(count_) : 0; }
    uint64_t overflow() const { return overflow_; }
    // Smallest duration in microseconds that at least `fraction` of the samples do not exceed.
    int64_t percentile_us(double fraction) const;
    std::string to_json() const;

 private:
    std::array<uint64_t, kBuckets> buckets_{};
    uint64_t overflow_ = 0;
    uint64_t count_ = 0;
    int64_t sum_ns_ = 0;
    int64_t min_ns_ = 0;
    int64_t max_ns_ = 0;
};

}  // namespace obot::rt
