#include "obot/ipc/shm_region.h"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <new>
#include <stdexcept>
#include <utility>

#include "obot/rt/clock.h"

namespace obot::ipc {

namespace {
[[noreturn]] void fail(const std::string& what) {
    throw std::runtime_error(what + ": " + std::strerror(errno));
}

// Non-blocking exclusive lock that belongs to this open of the file and ends with it.
bool lock_region(int fd) {
    struct flock lock{};
    lock.l_type = F_WRLCK;
    lock.l_whence = SEEK_SET;
    lock.l_start = 0;
    lock.l_len = 1;
    return ::fcntl(fd, F_OFD_SETLK, &lock) == 0;
}
}  // namespace

std::string ShmRegion::path_for(const std::string& name) {
    return "/dev/shm/" + name;
}

void ShmRegion::map(int fd) {
    // Clients also write (commands), so the mapping is always read/write.
    void* p = ::mmap(nullptr, kTotalSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) fail("mmap " + name_);
    base_ = p;
    fd_ = fd;
    auto* bytes = static_cast<unsigned char*>(p);
    header_ = reinterpret_cast<ShmHeader*>(bytes);
    info_ = reinterpret_cast<RobotInfo*>(bytes + kInfoOffset);
    command_ = reinterpret_cast<CommandSlot*>(bytes + kCommandOffset);
    ring_ = reinterpret_cast<StateRing*>(bytes + kRingOffset);
}

ShmRegion ShmRegion::create(const std::string& name, const RobotInfo& info) {
    ShmRegion r;
    r.name_ = name;
    r.owner_ = true;
    const std::string path = path_for(name);
    // One daemon per name: a second one would replace the region the first is serving and
    // then drive the same actuators. The daemon holds a lock on its region file for as
    // long as it runs (an open-file-description lock, which the flock() that command
    // writers use does not interact with), so a region that can be locked is a dead
    // daemon's and is replaced.
    const int old = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (old >= 0) {
        if (!lock_region(old)) {
            ::close(old);
            throw std::runtime_error(path + ": another obot-rtd is already serving this name");
        }
        // A fresh inode: clients still mapped to the dead daemon's region keep seeing that
        // one instead of a half-initialised new one.
        if (::unlink(path.c_str()) != 0) {
            const int saved = errno;
            ::close(old);
            errno = saved;
            fail("replace " + path);
        }
    } else if (errno != ENOENT) {
        fail("open " + path + " (left behind by another user? remove it)");
    }
    const int fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_EXCL | O_CLOEXEC, 0660);
    if (fd < 0 || !lock_region(fd)) {
        const int saved = errno;
        if (fd >= 0) ::close(fd);
        if (old >= 0) ::close(old);
        errno = saved;
        fail("create " + path);
    }
    if (old >= 0) ::close(old);
    if (::ftruncate(fd, static_cast<off_t>(kTotalSize)) != 0) {
        ::close(fd);
        fail("size " + path);
    }
    r.map(fd);
    // ftruncate gives zero pages; zero sequence numbers mean "nothing written".
    ShmHeader h{};
    h.magic = 0;
    h.layout_version = kLayoutVersion;
    h.header_size = sizeof(ShmHeader);
    h.info_size = sizeof(RobotInfo);
    h.state_size = sizeof(RobotState);
    h.command_size = sizeof(RobotCommand);
    h.ring_capacity = kRingCapacity;
    h.ring_slot_size = sizeof(StateSlot);
    h.info_offset = kInfoOffset;
    h.command_offset = kCommandOffset;
    h.ring_offset = kRingOffset;
    h.total_size = kTotalSize;
    h.daemon_pid = ::getpid();
    h.start_time_real_ns = rt::realtime_ns();
    std::memcpy(r.header_, &h, sizeof(h));
    std::memcpy(r.info_, &info, sizeof(info));
    return r;
}

ShmRegion ShmRegion::attach(const std::string& name) {
    ShmRegion r;
    r.name_ = name;
    const std::string path = path_for(name);
    const int fd = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (fd < 0) fail("open " + path + " (is obot-rtd running?)");
    struct stat st{};
    if (::fstat(fd, &st) != 0 || static_cast<uint64_t>(st.st_size) < kTotalSize) {
        ::close(fd);
        throw std::runtime_error(path + ": wrong size, built for a different layout");
    }
    r.map(fd);
    const ShmHeader& h = *r.header_;
    if (h.magic != kMagic) throw std::runtime_error(path + ": not published yet");
    if (h.layout_version != kLayoutVersion || h.state_size != sizeof(RobotState) ||
        h.command_size != sizeof(RobotCommand) || h.info_size != sizeof(RobotInfo) ||
        h.ring_capacity != kRingCapacity || h.total_size != kTotalSize) {
        throw std::runtime_error(path + ": layout version " + std::to_string(h.layout_version) +
                                 " does not match this build (" + std::to_string(kLayoutVersion) + ")");
    }
    return r;
}

void ShmRegion::publish() {
    memory_fence();
    header_->magic = kMagic;
    memory_fence();
}

void ShmRegion::unlink() {
    if (owner_) ::unlink(path_for(name_).c_str());
}

void apply_request(const RobotCommand& previous, RobotCommand& next) {
    const auto request = static_cast<Request>(next.request);
    next.request = previous.request;
    next.request_id = previous.request_id;
    next.estop_id = previous.estop_id;
    if (request == Request::Estop) {
        next.estop_id = previous.estop_id + 1;
    } else if (request != Request::None) {
        next.request = static_cast<uint32_t>(request);
        next.request_id = previous.request_id + 1;
    }
}

void ShmRegion::write_command(RobotCommand command) {
    // flock, not a mutex in the region: a writer that dies while holding it releases it.
    while (::flock(fd_, LOCK_EX) != 0) {
        if (errno != EINTR) fail("lock " + name_);
    }
    const uint64_t slot_seq = command_->seq.load(std::memory_order_relaxed);
    if (slot_seq & 1) {
        // A previous writer died mid-write; finish its sequence so the slot is usable again.
        command_->seq.store(slot_seq + 1, std::memory_order_relaxed);
    }
    command.seq = command_->command.seq + 1;
    command.stamp_ns = rt::monotonic_ns();
    apply_request(command_->command, command);
    command_->write(command);
    ::flock(fd_, LOCK_UN);
}

void ShmRegion::release() {
    if (base_) ::munmap(base_, kTotalSize);
    if (fd_ >= 0) ::close(fd_);
    base_ = nullptr;
    fd_ = -1;
}

ShmRegion::ShmRegion(ShmRegion&& other) noexcept {
    *this = std::move(other);
}

ShmRegion& ShmRegion::operator=(ShmRegion&& other) noexcept {
    if (this != &other) {
        release();
        name_ = std::move(other.name_);
        fd_ = std::exchange(other.fd_, -1);
        base_ = std::exchange(other.base_, nullptr);
        owner_ = std::exchange(other.owner_, false);
        header_ = std::exchange(other.header_, nullptr);
        info_ = std::exchange(other.info_, nullptr);
        command_ = std::exchange(other.command_, nullptr);
        ring_ = std::exchange(other.ring_, nullptr);
    }
    return *this;
}

ShmRegion::~ShmRegion() {
    release();
}

}  // namespace obot::ipc
