#include "obot/can/usbrt_bus.h"

#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <utility>

#include "obot/actuator/obot_driver.h"
#include "obot/rt/clock.h"
#include "obot/wire/obot_messages.h"

namespace obot::can {

namespace oc = actuator::obot_can;

std::vector<std::pair<std::string, std::string>> UsbRtBus::enumerate() {
    std::vector<std::pair<std::string, std::string>> found;
    const char* root = "/sys/class/usbmisc";
    DIR* dir = ::opendir(root);
    if (!dir) return found;
    while (const dirent* entry = ::readdir(dir)) {
        const std::string node = entry->d_name;
        if (node.rfind("usbrt", 0) != 0 && node.rfind("mtr", 0) != 0) continue;
        std::ifstream f(std::string(root) + "/" + node + "/device/interface");
        std::string board;
        if (std::getline(f, board) && !board.empty()) found.emplace_back(board, "/dev/" + node);
    }
    ::closedir(dir);
    return found;
}

UsbRtBus::UsbRtBus(std::string name, const std::vector<UsbRtDevice>& devices) : name_(std::move(name)) {
    const auto present = enumerate();
    std::string missing;
    for (const UsbRtDevice& d : devices) {
        std::string path;
        for (const auto& [board, node] : present) {
            if (board == d.name) path = node;
        }
        int fd = -1;
        if (!path.empty()) fd = ::open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            const std::string why = path.empty() ? "" : std::string(" (") + std::strerror(errno) + ")";
            missing += (missing.empty() ? "" : ", ") + d.name + why;
            continue;
        }
        devices_.push_back(Open{d.address, fd});
    }
    if (!missing.empty()) {
        std::string have;
        for (const auto& [board, node] : present) have += (have.empty() ? "" : ", ") + board;
        for (const Open& o : devices_) ::close(o.fd);
        throw std::runtime_error("usb " + name_ + ": cannot open " + missing +
                                 "; boards present: " + (have.empty() ? "none (is the usb_rt driver loaded?)" : have));
    }
}

UsbRtBus::UsbRtBus(std::string name, const std::vector<std::pair<uint8_t, int>>& open_fds) : name_(std::move(name)) {
    for (const auto& [address, fd] : open_fds) {
        ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK);
        devices_.push_back(Open{address, fd});
    }
}

UsbRtBus::~UsbRtBus() {
    for (const Open& o : devices_) ::close(o.fd);
}

void UsbRtBus::append_fds(std::vector<int>& out) const {
    for (const Open& o : devices_) out.push_back(o.fd);
}

bool UsbRtBus::send(const Frame& frame) {
    const uint32_t message = frame.id >> 7;
    const uint32_t address = frame.id & 0x7f;
    // A status request needs no packet: the board streams its status.
    if (message == oc::kStatus) return true;
    if (message != oc::kCmd && message != oc::kCmdStatus) return false;
    for (const Open& o : devices_) {
        if (o.address != address) continue;
        const ssize_t n = ::write(o.fd, frame.data, sizeof(MotorCommand));
        if (n != static_cast<ssize_t>(sizeof(MotorCommand))) {
            ++counters_.tx_dropped;
            return false;
        }
        ++counters_.tx;
        return true;
    }
    ++counters_.tx_dropped;
    return false;
}

int UsbRtBus::recv(Frame* out, int max) {
    int n = 0;
    for (const Open& o : devices_) {
        // Keep the newest packet. The read that finds nothing (EAGAIN) is also what
        // starts the next transfer in the usb_rt driver.
        Frame newest;
        bool got = false;
        for (;;) {
            const ssize_t len = ::read(o.fd, newest.data, kMaxData);
            if (len < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) ++counters_.rx_error;
                break;
            }
            if (len == 0) break;
            newest.len = static_cast<uint8_t>(len);
            got = true;
        }
        if (!got || n >= max) continue;
        newest.id = oc::can_id(oc::kStatus, o.address);
        newest.fd = true;
        newest.rx_time_ns = rt::monotonic_ns();
        out[n++] = newest;
    }
    counters_.rx += static_cast<uint64_t>(n);
    return n;
}

}  // namespace obot::can
