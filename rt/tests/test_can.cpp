#include "doctest.h"

#include "obot/can/bus_budget.h"
#include "obot/can/fake_bus.h"
#include "obot/can/frame.h"

using namespace obot::can;

TEST_CASE("fd_padded_len rounds up to a valid CAN FD length") {
    CHECK(fd_padded_len(0) == 0);
    CHECK(fd_padded_len(8) == 8);
    CHECK(fd_padded_len(9) == 12);
    CHECK(fd_padded_len(44) == 48);
    CHECK(fd_padded_len(60) == 64);
    CHECK(fd_padded_len(64) == 64);
}

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

TEST_CASE("fake bus carries frames both ways in order") {
    FakeBusPair pair("test");
    Frame f;
    f.id = 0x123;
    f.len = 2;
    f.data[0] = 7;
    CHECK(pair.host().send(f));
    f.id = 0x124;
    CHECK(pair.host().send(f));

    Frame got[4];
    CHECK(pair.host().recv(got, 4) == 0);  // a node does not hear itself
    REQUIRE(pair.device().recv(got, 4) == 2);
    CHECK(got[0].id == 0x123);
    CHECK(got[1].id == 0x124);
    CHECK(got[0].data[0] == 7);

    CHECK(pair.device().send(got[0]));
    REQUIRE(pair.host().recv(got, 4) == 1);
    CHECK(pair.host().counters().tx == 2);
    CHECK(pair.host().counters().rx == 1);
}

TEST_CASE("fake bus reports a full queue and can drop frames on request") {
    FakeBusPair pair("test");
    Frame f;
    int sent = 0;
    while (pair.host().send(f)) ++sent;
    CHECK(sent == 256);
    CHECK(pair.host().counters().tx_dropped == 1);

    FakeBusPair lossy("lossy");
    lossy.set_drop_device_tx(true);
    CHECK(lossy.device().send(f));
    Frame got[1];
    CHECK(lossy.host().recv(got, 1) == 0);
}
