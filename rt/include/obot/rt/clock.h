#pragma once

#include <cstdint>
#include <ctime>

namespace obot::rt {

inline int64_t to_ns(const timespec& ts) {
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

inline timespec to_timespec(int64_t ns) {
    timespec ts;
    ts.tv_sec = static_cast<time_t>(ns / 1'000'000'000LL);
    ts.tv_nsec = static_cast<long>(ns % 1'000'000'000LL);
    return ts;
}

inline int64_t monotonic_ns() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return to_ns(ts);
}

inline int64_t realtime_ns() {
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return to_ns(ts);
}

}  // namespace obot::rt
