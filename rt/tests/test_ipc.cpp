#include "doctest.h"

#include <unistd.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "obot/ipc/channels.h"
#include "obot/ipc/shm_region.h"
#include "obot/rt/clock.h"

using namespace obot::ipc;

namespace {

// A state whose every checked field is derived from one number, so a torn copy shows.
RobotState pattern(uint64_t n) {
    RobotState s{};
    s.cycle = n;
    s.t_mono_ns = static_cast<int64_t>(n * 3);
    for (int i = 0; i < kMaxJoints; ++i) {
        s.q[i] = static_cast<float>(n % 1000);
        s.bus_voltage[i] = static_cast<float>(n % 1000);
        s.rx_miss[i] = static_cast<uint32_t>(n);
    }
    return s;
}

bool consistent(const RobotState& s) {
    const uint64_t n = s.cycle;
    if (s.t_mono_ns != static_cast<int64_t>(n * 3)) return false;
    for (int i = 0; i < kMaxJoints; ++i) {
        if (s.q[i] != static_cast<float>(n % 1000)) return false;
        if (s.bus_voltage[i] != static_cast<float>(n % 1000)) return false;
        if (s.rx_miss[i] != static_cast<uint32_t>(n)) return false;
    }
    return true;
}

std::string unique_name(const char* tag) {
    return std::string("obot_test_") + tag + "_" + std::to_string(getpid());
}

}  // namespace

TEST_CASE("state ring: read back, not-yet and overrun") {
    auto ring = std::make_unique<StateRing>();
    RobotState out{};
    CHECK_FALSE(ring->latest(out));
    CHECK(ring->read(0, out) == StateRing::Read::NotYet);

    for (uint64_t n = 0; n < 10; ++n) ring->push(pattern(n));
    CHECK(ring->head.load() == 10);
    REQUIRE(ring->read(3, out) == StateRing::Read::Ok);
    CHECK(out.cycle == 3);
    REQUIRE(ring->latest(out));
    CHECK(out.cycle == 9);
    CHECK(ring->read(10, out) == StateRing::Read::NotYet);

    // Lap the ring: old records are reported as overrun, never returned as something else.
    for (uint64_t n = 10; n < 10 + kRingCapacity; ++n) ring->push(pattern(n));
    CHECK(ring->read(3, out) == StateRing::Read::Overrun);
    REQUIRE(ring->read(10, out) == StateRing::Read::Ok);
    CHECK(out.cycle == 10);
}

TEST_CASE("state ring: a reader never sees a torn record while the writer runs") {
    auto ring = std::make_unique<StateRing>();
    std::atomic<bool> stop{false};
    std::thread writer([&] {
        uint64_t n = 0;
        while (!stop.load(std::memory_order_relaxed)) ring->push(pattern(n++));
    });

    uint64_t ok = 0, torn = 0, cursor = 0;
    RobotState s{};
    const int64_t end = obot::rt::monotonic_ns() + 300'000'000;
    while (obot::rt::monotonic_ns() < end) {
        const uint64_t head = ring->head.load(std::memory_order_acquire);
        if (cursor >= head) continue;
        switch (ring->read(cursor, s)) {
            case StateRing::Read::Ok:
                ++ok;
                if (!consistent(s) || s.cycle != cursor) ++torn;
                ++cursor;
                break;
            case StateRing::Read::Overrun:
                cursor = head > kRingCapacity / 2 ? head - kRingCapacity / 2 : 0;
                break;
            case StateRing::Read::NotYet:
                break;
        }
    }
    stop.store(true);
    writer.join();
    CHECK(ok > 1000);
    CHECK(torn == 0);
}

TEST_CASE("command slot: latest value, and never torn under a concurrent writer") {
    auto slot = std::make_unique<CommandSlot>();
    RobotCommand c{};
    CHECK_FALSE(slot->read(c));

    std::atomic<bool> stop{false};
    std::thread writer([&] {
        uint64_t n = 1;
        while (!stop.load(std::memory_order_relaxed)) {
            RobotCommand w{};
            w.seq = n;
            for (int i = 0; i < kMaxJoints; ++i) {
                w.q[i] = static_cast<float>(n % 4096);
                w.kd[i] = static_cast<float>(n % 4096);
            }
            slot->write(w);
            ++n;
        }
    });
    uint64_t ok = 0, torn = 0, last_seq = 0, backwards = 0;
    const int64_t end = obot::rt::monotonic_ns() + 300'000'000;
    while (obot::rt::monotonic_ns() < end) {
        if (!slot->read(c)) continue;
        ++ok;
        for (int i = 0; i < kMaxJoints; ++i) {
            if (c.q[i] != static_cast<float>(c.seq % 4096) || c.kd[i] != static_cast<float>(c.seq % 4096)) ++torn;
        }
        if (c.seq < last_seq) ++backwards;
        last_seq = c.seq;
    }
    stop.store(true);
    writer.join();
    CHECK(ok > 100);
    CHECK(torn == 0);
    CHECK(backwards == 0);
}

TEST_CASE("shared memory region: create, attach, command round trip") {
    const std::string name = unique_name("region");
    RobotInfo info{};
    info.num_joints = 3;
    info.period_ns = 1'000'000;

    ShmRegion daemon = ShmRegion::create(name, info);
    // Not visible to clients until published.
    CHECK_THROWS_AS(ShmRegion::attach(name), std::runtime_error);
    daemon.publish();

    ShmRegion client = ShmRegion::attach(name);
    CHECK(client.info().num_joints == 3);
    CHECK(client.header().layout_version == kLayoutVersion);
    CHECK(client.header().daemon_pid == getpid());

    daemon.ring().push(pattern(41));
    RobotState s{};
    REQUIRE(client.ring().latest(s));
    CHECK(s.cycle == 41);

    RobotCommand c{};
    c.q[1] = 0.5f;
    c.request = static_cast<uint32_t>(Request::Enable);
    client.write_command(c);
    RobotCommand got{};
    REQUIRE(daemon.command_slot().read(got));
    CHECK(got.q[1] == 0.5f);
    CHECK(got.seq == 1);
    CHECK(got.request_id == 1);
    CHECK(got.stamp_ns > 0);

    // A write without a request leaves request_id alone; one with a request advances it.
    c.request = static_cast<uint32_t>(Request::None);
    client.write_command(c);
    REQUIRE(daemon.command_slot().read(got));
    CHECK(got.seq == 2);
    CHECK(got.request_id == 1);
    CHECK(got.request == static_cast<uint32_t>(Request::Enable));  // still there to be seen
    c.request = static_cast<uint32_t>(Request::Disable);
    client.write_command(c);
    REQUIRE(daemon.command_slot().read(got));
    CHECK(got.request_id == 2);
    CHECK(got.estop_id == 0);
    c.request = static_cast<uint32_t>(Request::Estop);
    client.write_command(c);
    REQUIRE(daemon.command_slot().read(got));
    CHECK(got.estop_id == 1);
    CHECK(got.request_id == 2);

    daemon.unlink();
    CHECK_THROWS_AS(ShmRegion::attach(name), std::runtime_error);
    // The existing mapping stays usable after the name is gone.
    CHECK(client.info().num_joints == 3);
}

TEST_CASE("shared memory region: a restarted daemon gets a fresh region") {
    const std::string name = unique_name("restart");
    RobotInfo info{};
    info.num_joints = 1;
    auto first = std::make_unique<ShmRegion>(ShmRegion::create(name, info));
    first->publish();
    first->ring().push(pattern(7));
    ShmRegion old_client = ShmRegion::attach(name);

    // While the first daemon lives, a second one for the same name is refused.
    CHECK_THROWS_WITH_AS(ShmRegion::create(name, info), doctest::Contains("another obot-rtd"), std::runtime_error);
    ShmRegion still = ShmRegion::attach(name);
    CHECK(still.info().num_joints == 1);

    first.reset();  // the first daemon exits
    // Nothing but the region itself is left in /dev/shm: a leftover lock file owned by one
    // user would keep a daemon run by another user from starting.
    CHECK(::access((ShmRegion::path_for(name) + ".lock").c_str(), F_OK) != 0);
    info.num_joints = 2;
    ShmRegion second = ShmRegion::create(name, info);
    second.publish();
    ShmRegion new_client = ShmRegion::attach(name);
    CHECK(new_client.info().num_joints == 2);
    RobotState s{};
    CHECK_FALSE(new_client.ring().latest(s));
    // The old client still sees the old region, not a half-written new one.
    CHECK(old_client.info().num_joints == 1);
    second.unlink();
}
