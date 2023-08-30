#include "CpuMessages.hpp"

namespace ects::plugins::systemmonitor {
auto CpuUsageMessage::to_ros(const CpuUsageMessage &msg) -> ros_t {
    ros_t ros_msg;
    ros_msg.total_usage = msg._total_usage;
    ros_msg.per_core_usage = msg._per_core_usage;
    ros_msg.load_averages = msg._load_averages;
    return ros_msg;
}

} // namespace ects::plugins::systemmonitor
