#include "obot/can/fake_bus.h"

#include <utility>

namespace obot::can {

bool FakeBusPair::Queue::push(const Frame& f) {
    if (count == kCapacity) return false;
    frames[(head + count) % kCapacity] = f;
    ++count;
    return true;
}

bool FakeBusPair::Queue::pop(Frame& f) {
    if (count == 0) return false;
    f = frames[head];
    head = (head + 1) % kCapacity;
    --count;
    return true;
}

class FakeBusEndpoint : public Bus {
 public:
    FakeBusEndpoint(std::string name, FakeBusPair::Queue& tx, FakeBusPair::Queue& rx, const bool& drop_tx)
        : name_(std::move(name)), tx_(tx), rx_(rx), drop_tx_(drop_tx) {}

    bool send(const Frame& frame) override {
        if (drop_tx_) {
            ++counters_.tx;  // the frame left this node; it was lost on the wire
            return true;
        }
        if (!tx_.push(frame)) {
            ++counters_.tx_dropped;
            return false;
        }
        ++counters_.tx;
        return true;
    }

    int recv(Frame* out, int max) override {
        int n = 0;
        while (n < max && rx_.pop(out[n])) ++n;
        counters_.rx += static_cast<uint64_t>(n);
        return n;
    }

    const std::string& name() const override { return name_; }

 private:
    std::string name_;
    FakeBusPair::Queue& tx_;
    FakeBusPair::Queue& rx_;
    const bool& drop_tx_;
};

FakeBusPair::FakeBusPair(std::string name)
    : host_(std::make_unique<FakeBusEndpoint>(name, to_device_, to_host_, drop_host_tx_)),
      device_(std::make_unique<FakeBusEndpoint>(name + ".device", to_host_, to_device_, drop_device_tx_)) {}

FakeBusPair::~FakeBusPair() = default;

Bus& FakeBusPair::host() {
    return *host_;
}
Bus& FakeBusPair::device() {
    return *device_;
}

}  // namespace obot::can
