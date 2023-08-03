#include "AggregationMessage.hpp"

namespace ects::plugins::systemmonitor {
auto AggregationList::to_ros(const AggregationList &list) -> ros_t {
    ros_t r;
    for (auto aggregation : list.aggregation_list)
        r.available_aggregations.push_back(aggregation->to_ros());
    return r;
}
} // namespace ects::plugins::systemmonitor
