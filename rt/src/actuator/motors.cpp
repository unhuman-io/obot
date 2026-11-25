#include "obot/actuator/motors.h"

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <motor_can.h>

namespace obot::actuator {

Motors::Motors(std::vector<std::shared_ptr<Motor>> motors) : motors_(std::move(motors)) {
    try {
        manager_.set_motors(motors_);  // checks the messages version, prepares the poll set
    } catch (const std::exception& e) {
        throw std::runtime_error(e.what());
    }
    for (auto& m : motors_) {
        if (auto* can = dynamic_cast<MotorCAN*>(m.get())) {
            // Commands go out as "command, reply with your status", and a read takes what
            // has arrived without asking for more or waiting for it.
            can->set_cmd_status_req();
            can->aread();
            can->set_timeout_ms(0);
        } else {
            m->set_nonblock();
        }
    }
    commands_.assign(motors_.size(), MotorCommand{});
    statuses_.assign(motors_.size(), MotorStatus{});
    tx_failed_.assign(motors_.size(), false);
    fresh_.assign(motors_.size(), false);
    seen_.assign(motors_.size(), false);
}

void Motors::write() {
    for (size_t i = 0; i < motors_.size(); ++i) {
        *motors_[i]->command() = commands_[i];
        bool ok = false;
        try {
            ok = motors_[i]->write() >= 0;
        } catch (const std::exception&) {  // the library reports some failures by throwing
        }
        tx_failed_[i] = !ok;
    }
}

void Motors::wait(int64_t timeout_ns) {
    if (timeout_ns <= 0) return;
    manager_.multipoll(static_cast<uint32_t>(timeout_ns));
}

void Motors::read() {
    for (size_t i = 0; i < motors_.size(); ++i) {
        fresh_[i] = false;
        try {
            motors_[i]->read();
        } catch (const std::exception&) {
            continue;
        }
        // The motor keeps its last status when nothing new arrived, and what read()
        // returns differs between transports: a status is new if it carries another time
        // from the drive, or answers another command.
        const MotorStatus& now = *motors_[i]->status();
        const MotorStatus& before = statuses_[i];
        const bool changed =
            now.mcu_timestamp != before.mcu_timestamp || now.host_timestamp_received != before.host_timestamp_received;
        if (changed || (!seen_[i] && (now.mcu_timestamp != 0 || now.host_timestamp_received != 0))) {
            statuses_[i] = now;
            fresh_[i] = true;
            seen_[i] = true;
        }
    }
}

std::shared_ptr<Motor> open_usb_motor(const std::string& name) {
    MotorManager finder;
    std::vector<std::shared_ptr<Motor>> found;
    try {
        found = finder.get_motors_by_name({name}, false);
    } catch (const std::exception& e) {
        std::string present;
        try {
            for (const auto& m : finder.get_connected_motors(false))
                present += (present.empty() ? "" : ", ") + m->name();
        } catch (const std::exception&) {
        }
        throw std::runtime_error("usb: no board named " + name +
                                 "; boards present: " + (present.empty() ? "none" : present));
    }
    // One process commands a board at a time (motor_util honours the same lock).
    if (found[0]->lock() != 0) {
        throw std::runtime_error("usb: board " + name + " is in use by another program");
    }
    return found[0];
}

std::shared_ptr<Motor> open_can_motor(const std::string& interface, int id) {
    const std::string address = interface + ":" + std::to_string(id);
    try {
        return std::make_shared<MotorCAN>(address);
    } catch (const std::exception& e) {
        throw std::runtime_error("can: node " + address + ": " + e.what());
    }
}

}  // namespace obot::actuator
