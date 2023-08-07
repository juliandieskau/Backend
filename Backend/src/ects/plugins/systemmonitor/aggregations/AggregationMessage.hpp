#pragma once

/* ECTS - AggregationMessage.hpp
 * Message defintion for the AggregationList service.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */
#include "Aggregation.hpp"
#include "ects/Aggregation.h"
#include "ects/AggregationList.h"
#include "ects/EmptyMessage.hpp"
#include "ects/RosInterface.hpp"
#include <string>

namespace ects::plugins::systemmonitor {

struct AggregationListService {};
using empty_aggregation_list_request =
    EmptyMessage<ects::AggregationList::Request>;
class AggregationList {
  public:
    using ros_t = ects::AggregationList::Response;
    using to_ros_t = ros_t;

    static auto to_ros(const AggregationList &) -> ros_t;
    AggregationList(std::vector<AggregationStrategy *> aggregation_list)
        : aggregation_list(std::move(aggregation_list)) {}

  private:
    std::vector<AggregationStrategy *> aggregation_list;
};

} // namespace ects::plugins::systemmonitor

namespace ects {
using namespace ects::plugins::systemmonitor;

template <> struct server_traits<AggregationListService> {
    using ros_t = ects::AggregationList;
    using request_from_ros_t = empty_aggregation_list_request;
    using response_to_ros_t = ects::plugins::systemmonitor::AggregationList;
};
} // namespace ects
