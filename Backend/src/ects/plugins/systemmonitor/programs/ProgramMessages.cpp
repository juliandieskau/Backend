#include "ProgramMessages.hpp"

namespace ects::plugins::systemmonitor {
auto ProcessTotalMessage::to_ros(const ProcessTotalMessage &msg)
    -> ProcessTotalMessage::ros_t {
    ProcessTotalMessage::ros_t ros_msg;
    ros_msg.number_of_processes = msg._total;
    return ros_msg;
}

auto ProcessTotalHistoryMessage::to_ros(const ProcessTotalHistoryMessage &msg)
    -> ProcessTotalHistoryMessage::ros_t {
    ProcessTotalHistoryMessage::ros_t ros_msg;
    ros_msg.aggregation = AggregationMessage::to_ros(msg._aggregation);
    for (auto &usage : msg._usage_history) {
        ros_msg.measurements.push_back(ProcessTotalMessage::to_ros(usage));
    }
    return ros_msg;
}
} // namespace ects::plugins::systemmonitor
