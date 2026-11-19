// Worst-case time on the wire for CAN and CAN FD frames, used to check that everything a
// control cycle puts on a bus fits in the cycle with margin.
#pragma once

#include <cstdint>

namespace obot::can {

struct BitTiming {
    double bitrate = 1'000'000;       // nominal (arbitration) bit rate, bit/s
    double data_bitrate = 5'000'000;  // data phase bit rate for FD frames with bit rate switch
};

// Worst-case duration in seconds of one frame with an 11 bit identifier, including bit
// stuffing and the 3 bit interframe space.
constexpr double frame_time_s(unsigned payload_len, bool fd, bool brs, const BitTiming& t) {
    if (!fd) {
        // SOF + id 11 + RTR + IDE + r0 + DLC 4 + data + CRC 15 are subject to stuffing.
        const unsigned stuffable = 34 + 8 * payload_len;
        const unsigned stuff = (stuffable - 1) / 4;
        const unsigned tail = 13;  // CRC delimiter, ACK, ACK delimiter, EOF 7, IFS 3
        return (stuffable + stuff + tail) / t.bitrate;
    }
    // Arbitration phase at the nominal rate: SOF, id 11, RRS, IDE, FDF, res, BRS (+ stuffing),
    // and after the data phase: CRC delimiter, ACK, ACK delimiter, EOF 7, IFS 3.
    const unsigned arb = 17 + 4 + 13;
    // Data phase: ESI, DLC 4, data (dynamic stuffing), stuff count 4, CRC 17/21 with fixed stuff bits.
    const unsigned dyn = 5 + 8 * payload_len;
    const unsigned crc = payload_len <= 16 ? 17 : 21;
    const unsigned data = dyn + dyn / 4 + 4 + crc + crc / 4 + 1;
    const double data_rate = brs ? t.data_bitrate : t.bitrate;
    return arb / t.bitrate + data / data_rate;
}

}  // namespace obot::can
