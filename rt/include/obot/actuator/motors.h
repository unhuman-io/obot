// All communication with the actuators. Every motor is a motor-realtime `obot::Motor`
// (a board on USB, a node on CAN FD, a simulated one) and every byte to or from one goes
// through that library, so a change of protocol or transport is made there and nowhere
// else. This class only arranges the library's calls into what a control cycle needs:
//
//     command(i) = ...;  write();          every motor gets its command
//     wait(timeout);                       until all have answered, or the time is up
//     read();                              fresh(i) says which status(i) are new
//
// None of these block beyond the timeout given to wait(), and none allocate.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <motor_manager.h>

namespace obot::actuator {

class Motors {
 public:
    // Takes the motors in joint order and puts each into the mode this cycle needs:
    // a command is answered with a status, and reading never waits.
    // Throws std::runtime_error (a motor speaks another messages version, ...).
    explicit Motors(std::vector<std::shared_ptr<Motor>> motors);

    size_t size() const { return motors_.size(); }
    Motor& motor(size_t i) { return *motors_[i]; }

    MotorCommand& command(size_t i) { return commands_[i]; }
    // Send every command. A motor that cannot be written to does not keep the others
    // from getting theirs; tx_failed(i) tells which.
    void write();
    bool tx_failed(size_t i) const { return tx_failed_[i]; }

    // Wait until every motor has something to read, at most timeout_ns.
    void wait(int64_t timeout_ns);

    // Take what has arrived. fresh(i) is true for a status that was not seen before.
    void read();
    bool fresh(size_t i) const { return fresh_[i]; }
    const MotorStatus& status(size_t i) const { return statuses_[i]; }

 private:
    MotorManager manager_;
    std::vector<std::shared_ptr<Motor>> motors_;
    std::vector<MotorCommand> commands_;
    std::vector<MotorStatus> statuses_;
    std::vector<bool> tx_failed_;
    std::vector<bool> fresh_;
    std::vector<bool> seen_;  // a first status has arrived
};

// A board on the usb_rt kernel driver, found by the name its firmware gives it, for this
// process alone. Throws std::runtime_error naming the boards that are connected.
std::shared_ptr<Motor> open_usb_motor(const std::string& name);

// A node on a CAN FD interface ("can0", id 3). Throws std::runtime_error if it does not answer.
std::shared_ptr<Motor> open_can_motor(const std::string& interface, int id);

}  // namespace obot::actuator
