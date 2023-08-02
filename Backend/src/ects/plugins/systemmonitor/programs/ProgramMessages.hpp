#pragma once

#include "ects/ProcessTotal.h"
#include "ects/ProcessTotalHistory.h"
#include "../Usage.hpp"
#include "../aggregations/AggregationMessage.hpp"

#include "ects/MessageInterface.hpp"
#include <string>
namespace ects::plugins::systemmonitor {

class ProcessTotalMessage : public UsageData {
  public:
    using ros_t = ects::ProcessTotal;
    using from_ros_t = ros_t;
    static auto to_ros(const ProcessTotalMessage &) -> ros_t;

    ProcessTotalMessage(uint32_t total) : UsageData(), _total(total) {}

    const auto get_total() -> uint32_t { return _total; }

    auto operator+(const ProcessTotalMessage &rhs) -> ProcessTotalMessage {
        return ProcessTotalMessage(_total + rhs._total);
    }
    auto operator/(const int &rhs) -> ProcessTotalMessage {
        return ProcessTotalMessage(_total / rhs);
    }

  private:
    uint32_t _total;
};

class ProcessTotalHistoryMessage {
  public:
    using ros_t = ects::ProcessTotalHistory;
    using from_ros_t = ros_t;
    static auto to_ros(const ProcessTotalHistoryMessage &) -> ros_t;

    ProcessTotalHistoryMessage(std::vector<ProcessTotalMessage> &usage_history,
                               AggregationMessage &aggregation)
        : _usage_history(usage_history), _aggregation(aggregation) {}

  private:
    std::vector<ProcessTotalMessage> &_usage_history;
    AggregationMessage &_aggregation;
};
} // namespace ects::plugins::systemmonitor
