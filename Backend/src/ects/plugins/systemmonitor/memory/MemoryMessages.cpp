#include "MemoryMessages.hpp"

namespace ects::plugins::systemmonitor {
auto MemoryUsageMessage::to_ros(const MemoryUsageMessage &msg) -> ros_t {
    ros_t ros_msg;
    ros_msg.total = msg.total;
    ros_msg.used = msg.used;
    ros_msg.free = msg.free;
    ros_msg.shared = msg.shared;
    ros_msg.buff_cache = msg.buff_cache;
    ros_msg.available = msg.available;
    ros_msg.swap_total = msg.swap_total;
    ros_msg.swap_used = msg.swap_used;
    ros_msg.swap_free = msg.swap_free;
    return ros_msg;
}
} // namespace ects::plugins::systemmonitor
