#include "AggregationMessage.hpp"

namespace ects::plugins::systemmonitor {
    auto AggregationMessage::to_ros(const AggregationMessage& msg) -> AggregationMessage::ros_t {
        AggregationMessage::ros_t ros_msg;
        ros_msg.ectsname = msg.ects_name;
        ros_msg.type = msg.type == INTERVAL ? 0 : 1;
        ros_msg.interval = msg.interval;
        ros_msg.nreadings = msg.nreadings;
        return ros_msg;
    }
} // namespace ects::plugins::systemmonitor
