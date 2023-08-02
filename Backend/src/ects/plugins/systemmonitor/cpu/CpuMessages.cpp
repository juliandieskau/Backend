#include "CpuMessages.hpp"

namespace ects::plugins::systemmonitor {
auto CpuPercentageMessage::to_ros(const CpuPercentageMessage &msg)
    -> CpuPercentageMessage::ros_t {
    CpuPercentageMessage::ros_t ros_msg;
    ros_msg.usage = msg.percentage;
    return ros_msg;
}
auto CpuUsageMessage::to_ros(const CpuUsageMessage &msg)
    -> CpuUsageMessage::ros_t {
    CpuUsageMessage::ros_t ros_msg;
    ros_msg.total_usage = msg._total_usage;
    ros_msg.per_core_usage = msg._per_core_usage;
    ros_msg.load_averages = msg._load_averages;
    return ros_msg;
}

auto CpuUsageMessage::get_cpu_percentage() -> CpuPercentageMessage {
    return CpuPercentageMessage(_total_usage);
}

auto CpuUsageHistoryMessage::to_ros(const CpuUsageHistoryMessage &msg) -> CpuUsageHistoryMessage::ros_t {
    CpuUsageHistoryMessage::ros_t ros_msg;
    ros_msg.aggregation = AggregationMessage::to_ros(msg._aggregation);
    auto usage_history = std::vector<CpuUsageMessage::ros_t>();
    for(auto usage : msg._usage_history){
        usage_history.push_back(CpuUsageMessage::to_ros(usage));
    }
    ros_msg.measurements = usage_history;
    return ros_msg;
}


} // namespace ects::plugins::systemmonitor
