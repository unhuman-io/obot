// The shared memory file: created by the daemon, attached by clients.
#pragma once

#include <string>

#include "obot/ipc/channels.h"
#include "obot/ipc/layout.h"

namespace obot::ipc {

// Turns the one-shot request a client put in `next.request` into the sticky request /
// request_id / estop_id fields, given the command currently in the mailbox.
void apply_request(const RobotCommand& previous, RobotCommand& next);

class ShmRegion {
 public:
    // Daemon side: create (replacing any stale file of the same name) and initialise.
    // The region becomes visible to readers when publish() is called.
    static ShmRegion create(const std::string& name, const RobotInfo& info);

    // Client side: attach to an existing region. Throws std::runtime_error if it does not
    // exist, is not published, or has a different layout.
    static ShmRegion attach(const std::string& name);

    ShmRegion(ShmRegion&& other) noexcept;
    ShmRegion& operator=(ShmRegion&& other) noexcept;
    ShmRegion(const ShmRegion&) = delete;
    ShmRegion& operator=(const ShmRegion&) = delete;
    ~ShmRegion();

    void publish();  // daemon: set the magic so that clients accept the region
    void unlink();   // daemon: remove the name (existing mappings stay valid)

    const ShmHeader& header() const { return *header_; }
    const RobotInfo& info() const { return *info_; }
    StateRing& ring() { return *ring_; }
    const StateRing& ring() const { return *ring_; }
    CommandSlot& command_slot() { return *command_; }

    // Client: write a command. Fills in seq, stamp_ns and the request bookkeeping: put a
    // Request in command.request to have it acted on once. Serialised between writers.
    void write_command(RobotCommand command);

    static std::string path_for(const std::string& name);

 private:
    ShmRegion() = default;
    void map(int fd);
    void release();

    std::string name_;
    int fd_ = -1;
    void* base_ = nullptr;
    bool owner_ = false;
    ShmHeader* header_ = nullptr;
    RobotInfo* info_ = nullptr;
    CommandSlot* command_ = nullptr;
    StateRing* ring_ = nullptr;
};

}  // namespace obot::ipc
