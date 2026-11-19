#include "obot/can/socketcan_bus.h"

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

#include "obot/rt/clock.h"

namespace obot::can {

namespace {
[[noreturn]] void fail(int fd, const std::string& what, const std::string& interface) {
    const int saved = errno;
    if (fd >= 0) ::close(fd);
    throw std::runtime_error("can " + interface + ": " + what + ": " + std::strerror(saved));
}
}  // namespace

SocketCanBus::SocketCanBus(std::string name, const std::string& interface, bool fd_frames,
                           const std::vector<uint32_t>& rx_ids)
    : name_(std::move(name)) {
    if (interface.size() >= IFNAMSIZ) {
        throw std::runtime_error("can " + interface + ": interface name too long");
    }
    const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (fd < 0) fail(fd, "socket", interface);

    if (fd_frames) {
        const int on = 1;
        if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) != 0) {
            fail(fd, "enable CAN FD frames", interface);
        }
    }

    // Controller problems (bus off, error passive, lost arbitration) arrive as error frames.
    const can_err_mask_t err_mask = CAN_ERR_BUSOFF | CAN_ERR_CRTL | CAN_ERR_RESTARTED | CAN_ERR_BUSERROR;
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) != 0) {
        fail(fd, "set error filter", interface);
    }

    if (!rx_ids.empty()) {
        std::vector<can_filter> filters;
        filters.reserve(rx_ids.size());
        for (uint32_t id : rx_ids) {
            can_filter f{};
            f.can_id = id;
            f.can_mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
            filters.push_back(f);
        }
        if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                         static_cast<socklen_t>(filters.size() * sizeof(can_filter))) != 0) {
            fail(fd, "set receive filter", interface);
        }
    }

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd, SIOCGIFINDEX, &ifr) != 0) fail(fd, "interface not found", interface);
    const int ifindex = ifr.ifr_ifindex;

    if (fd_frames) {
        if (::ioctl(fd, SIOCGIFMTU, &ifr) != 0) fail(fd, "read mtu", interface);
        if (ifr.ifr_mtu < static_cast<int>(CANFD_MTU)) {
            ::close(fd);
            throw std::runtime_error("can " + interface + ": interface is not in CAN FD mode (mtu " +
                                     std::to_string(ifr.ifr_mtu) + "); configure it with 'fd on'");
        }
    }

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifindex;
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) fail(fd, "bind", interface);
    fd_ = fd;
}

SocketCanBus::~SocketCanBus() {
    if (fd_ >= 0) ::close(fd_);
}

bool SocketCanBus::send(const Frame& frame) {
    ssize_t written;
    if (frame.fd) {
        canfd_frame raw{};
        raw.can_id = frame.id & CAN_SFF_MASK;
        raw.len = frame.len;
        raw.flags = frame.brs ? CANFD_BRS : 0;
        std::memcpy(raw.data, frame.data, frame.len);
        written = ::write(fd_, &raw, sizeof(raw));
    } else {
        can_frame raw{};
        raw.can_id = frame.id & CAN_SFF_MASK;
        raw.len = frame.len > CAN_MAX_DLEN ? static_cast<uint8_t>(CAN_MAX_DLEN) : frame.len;
        std::memcpy(raw.data, frame.data, raw.len);
        written = ::write(fd_, &raw, sizeof(raw));
    }
    if (written < 0) {
        ++counters_.tx_dropped;
        return false;
    }
    ++counters_.tx;
    return true;
}

int SocketCanBus::recv(Frame* out, int max) {
    int n = 0;
    while (n < max) {
        canfd_frame raw;
        const ssize_t got = ::read(fd_, &raw, sizeof(raw));
        if (got < 0) {
            // EAGAIN: drained. Anything else (interface down, ...) counts as a bus error.
            if (errno != EAGAIN && errno != EWOULDBLOCK) ++counters_.rx_error;
            break;
        }
        if (got != static_cast<ssize_t>(CAN_MTU) && got != static_cast<ssize_t>(CANFD_MTU)) continue;
        if (raw.can_id & CAN_ERR_FLAG) {
            ++counters_.rx_error;
            continue;
        }
        if (raw.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)) continue;
        Frame& f = out[n];
        f.id = raw.can_id & CAN_SFF_MASK;
        f.fd = got == static_cast<ssize_t>(CANFD_MTU);
        f.brs = f.fd && (raw.flags & CANFD_BRS);
        f.len = raw.len > kMaxData ? static_cast<uint8_t>(kMaxData) : raw.len;
        std::memcpy(f.data, raw.data, f.len);
        f.rx_time_ns = rt::monotonic_ns();
        ++n;
    }
    counters_.rx += static_cast<uint64_t>(n);
    return n;
}

}  // namespace obot::can
