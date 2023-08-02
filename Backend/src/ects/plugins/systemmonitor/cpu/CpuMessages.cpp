#include "CpuMessages.hpp"

namespace ects::plugins::systemmonitor {
auto CpuPercentageMessage::to_ros(const CpuPercentageMessage &msg) -> ros_t {
    ros_t ros_msg;
    ros_msg.usage = msg.percentage;
    return ros_msg;
}
auto CpuUsageMessage::to_ros(const CpuUsageMessage &msg) -> ros_t {
    ros_t ros_msg;
    ros_msg.total_usage = msg._total_usage;
    ros_msg.per_core_usage = msg._per_core_usage;
    ros_msg.load_averages = msg._load_averages;
    return ros_msg;
}

auto CpuUsageMessage::get_cpu_percentage() -> CpuPercentageMessage {
    return {_total_usage};
}

auto CpuUsageHistoryMessage::to_ros(const CpuUsageHistoryMessage &msg)
    -> ros_t {
    ros_t ros_msg;
    ros_msg.aggregation = AggregationMessage::to_ros(msg._aggregation);
    for (const auto &usage : msg._usage_history) {
        ros_msg.measurements.push_back(CpuUsageMessage::to_ros(usage));
    }
    return ros_msg;
}

} // namespace ects::plugins::systemmonitor
