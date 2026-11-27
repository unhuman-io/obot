#include "doctest.h"

#include "obot/can/bus_budget.h"

using namespace obot::can;

TEST_CASE("frame_time_s") {
    const BitTiming t{1'000'000, 5'000'000};
    // The textbook worst case for a classic frame with 8 data bytes is 135 bit times.
    CHECK(frame_time_s(8, false, false, t) == doctest::Approx(135e-6));
    CHECK(frame_time_s(0, false, false, t) == doctest::Approx(55e-6));
    // An FD frame with bit rate switch spends its arbitration phase at the nominal rate
    // and everything else at the data rate, so it is much shorter than without the switch.
    const double fast = frame_time_s(64, true, true, t);
    const double slow = frame_time_s(64, true, false, t);
    CHECK(fast > 34e-6);
    CHECK(fast < 200e-6);
    CHECK(slow > 4 * fast);
    // More payload never takes less time.
    CHECK(frame_time_s(48, true, true, t) < fast);
    CHECK(frame_time_s(8, true, true, t) < frame_time_s(48, true, true, t));
}
