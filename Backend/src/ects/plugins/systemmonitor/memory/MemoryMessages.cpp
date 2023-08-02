#include "MemoryMessages.hpp"


namespace ects::plugins::systemmonitor {
    auto MemoryUsageMessage::to_ros(const MemoryUsageMessage& msg) -> MemoryUsageMessage::ros_t {
        MemoryUsageMessage::ros_t ros_msg;
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

    auto MemoryUsageHistoryMessage::to_ros(const MemoryUsageHistoryMessage& msg) -> MemoryUsageHistoryMessage::ros_t {
        MemoryUsageHistoryMessage::ros_t ros_msg;
        ros_msg.aggregation = AggregationMessage::to_ros(msg.aggregation);
        for(auto& usage : msg.usage_history) {
            ros_msg.measurements.push_back(MemoryUsageMessage::to_ros(usage));
        }
        return ros_msg;
    }
} // namespace ects::plugins::systemmonitor
