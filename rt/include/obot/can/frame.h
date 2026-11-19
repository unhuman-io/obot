// A CAN / CAN FD frame, independent of the transport that carries it.
#pragma once

#include <cstdint>
#include <cstring>

namespace obot::can {

inline constexpr int kMaxData = 64;

struct Frame {
    uint32_t id = 0;   // 11 bit identifier (the stack does not use extended ids)
    uint8_t len = 0;   // payload bytes, 0..64
    bool fd = false;   // CAN FD frame
    bool brs = false;  // bit rate switch (FD only)
    uint8_t data[kMaxData] = {};
    int64_t rx_time_ns = 0;  // CLOCK_MONOTONIC when the frame was read, 0 for tx frames

    void set_payload(const void* src, uint8_t n, uint8_t padded_len) {
        std::memset(data, 0, sizeof(data));
        std::memcpy(data, src, n);
        len = padded_len;
    }
};

// Smallest valid CAN FD payload length that holds n bytes (n <= 64).
constexpr uint8_t fd_padded_len(unsigned n) {
    if (n <= 8) return static_cast<uint8_t>(n);
    if (n <= 12) return 12;
    if (n <= 16) return 16;
    if (n <= 20) return 20;
    if (n <= 24) return 24;
    if (n <= 32) return 32;
    if (n <= 48) return 48;
    return 64;
}

}  // namespace obot::can
