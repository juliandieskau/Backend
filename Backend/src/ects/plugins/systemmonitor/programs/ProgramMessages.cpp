#include "ProgramMessages.hpp"

namespace ects::plugins::systemmonitor {
auto ProcessTotalMessage::to_ros(const ProcessTotalMessage &msg)
    -> ProcessTotalMessage::ros_t {
    ProcessTotalMessage::ros_t ros_msg;
    ros_msg.number_of_processes = msg._total;
    return ros_msg;
}
} // namespace ects::plugins::systemmonitor
