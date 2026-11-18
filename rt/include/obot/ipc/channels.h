// Lock-free channels that live inside the shared memory region.
//
// StateRing: one writer (the realtime thread), any number of readers. The writer never
// waits. Each slot carries a sequence number: 2*i + 1 while record i is being written,
// 2*i + 2 once it is complete. A reader copies a slot and accepts the copy only if the
// sequence was 2*i + 2 before and after, so a record overwritten during the copy is
// detected rather than returned torn.
//
// CommandSlot: latest-value mailbox. Writers (not realtime) serialise among themselves
// with flock() on the shared memory file; the realtime reader takes no lock and retries a
// bounded number of times if it catches a write in progress.
#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

#include "obot/ipc/layout.h"

namespace obot::ipc {

#if defined(__clang__) || defined(__GNUC__)
#define OBOT_NO_SANITIZE_THREAD __attribute__((no_sanitize("thread")))
#else
#define OBOT_NO_SANITIZE_THREAD
#endif

// Full memory barrier between the payload copy and the sequence check (what the kernel's
// seqlock does with smp_mb / smp_rmb). ThreadSanitizer cannot model a bare fence and
// these functions are excluded from it anyway, so the fence is left out of TSan builds.
#if defined(__SANITIZE_THREAD__)
#define OBOT_TSAN 1
#elif defined(__has_feature)
#if __has_feature(thread_sanitizer)
#define OBOT_TSAN 1
#endif
#endif
#ifdef OBOT_TSAN
inline void memory_fence() {}
#else
inline void memory_fence() {
    std::atomic_thread_fence(std::memory_order_seq_cst);
}
#endif

struct StateSlot {
    std::atomic<uint64_t> seq;
    RobotState state;
};
static_assert(sizeof(StateSlot) == 8 + sizeof(RobotState));
static_assert(std::atomic<uint64_t>::is_always_lock_free);

struct StateRing {
    std::atomic<uint64_t> head;  // number of records written so far
    StateSlot slots[kRingCapacity];

    // Writer side.
    OBOT_NO_SANITIZE_THREAD void push(const RobotState& s) {
        const uint64_t i = head.load(std::memory_order_relaxed);
        StateSlot& slot = slots[i & (kRingCapacity - 1)];
        slot.seq.store(2 * i + 1, std::memory_order_relaxed);
        memory_fence();
        std::memcpy(&slot.state, &s, sizeof(RobotState));
        slot.seq.store(2 * i + 2, std::memory_order_release);
        head.store(i + 1, std::memory_order_release);
    }

    enum class Read { Ok, NotYet, Overrun };

    // Reader side: copy record i.
    OBOT_NO_SANITIZE_THREAD Read read(uint64_t i, RobotState& out) const {
        const StateSlot& slot = slots[i & (kRingCapacity - 1)];
        const uint64_t want = 2 * i + 2;
        const uint64_t before = slot.seq.load(std::memory_order_acquire);
        if (before < want) return Read::NotYet;
        if (before > want) return Read::Overrun;
        std::memcpy(&out, &slot.state, sizeof(RobotState));
        memory_fence();
        const uint64_t after = slot.seq.load(std::memory_order_acquire);
        return after == want ? Read::Ok : Read::Overrun;
    }

    // Reader side: copy the newest complete record. false if nothing has been written.
    bool latest(RobotState& out) const {
        for (int attempt = 0; attempt < 4; ++attempt) {
            const uint64_t h = head.load(std::memory_order_acquire);
            if (h == 0) return false;
            if (read(h - 1, out) == Read::Ok) return true;
        }
        return false;
    }
};

struct CommandSlot {
    std::atomic<uint64_t> seq;  // odd while a writer is in the middle of a write
    RobotCommand command;

    // Writer side. The caller holds the writer lock (see ShmRegion::write_command).
    OBOT_NO_SANITIZE_THREAD void write(const RobotCommand& c) {
        const uint64_t s = seq.load(std::memory_order_relaxed);
        seq.store(s + 1, std::memory_order_relaxed);
        memory_fence();
        std::memcpy(&command, &c, sizeof(RobotCommand));
        seq.store(s + 2, std::memory_order_release);
    }

    // Reader side. false if nothing has been written yet or a consistent copy could not be
    // taken within the retry budget (the caller keeps its previous command).
    OBOT_NO_SANITIZE_THREAD bool read(RobotCommand& out) const {
        for (int attempt = 0; attempt < 8; ++attempt) {
            const uint64_t before = seq.load(std::memory_order_acquire);
            if (before == 0) return false;
            if (before & 1) continue;
            std::memcpy(&out, &command, sizeof(RobotCommand));
            memory_fence();
            if (seq.load(std::memory_order_acquire) == before) return true;
        }
        return false;
    }
};

inline constexpr uint64_t kInfoOffset = sizeof(ShmHeader);
inline constexpr uint64_t kCommandOffset = (kInfoOffset + sizeof(RobotInfo) + 63) / 64 * 64;
inline constexpr uint64_t kRingOffset = (kCommandOffset + sizeof(CommandSlot) + 63) / 64 * 64;
inline constexpr uint64_t kTotalSize = kRingOffset + sizeof(StateRing);

}  // namespace obot::ipc
