#pragma once

#include "ects/Aggregation.h"
#include "ects/MessageInterface.hpp"
#include <chrono>
#include <string>

namespace ects::plugins::systemmonitor {

class AggregationMessage {
  public:
    enum AggregationType { INTERVAL, READINGS };
    using ros_t = ects::Aggregation;
    using from_ros_t = ros_t;

    static auto to_ros(const AggregationMessage &) -> ros_t;
    AggregationMessage(std::string &&ects_name, AggregationType type,
                       std::chrono::seconds interval, uint16_t nreadings,
                       uint32_t keep_amount)
        : ects_name(ects_name), type(type), interval(interval),
          nreadings(nreadings), keep_amount(keep_amount) {}

    auto get_ects_name() const -> const std::string & { return ects_name; }
    auto get_type() const -> const AggregationType { return type; }
    auto get_interval() const -> const std::chrono::seconds { return interval; }
    auto get_nreadings() const -> const uint16_t { return nreadings; }
    auto get_keep_amount() const -> const uint32_t { return keep_amount; }

  private:
    const std::string ects_name;
    const AggregationType type;
    const std::chrono::seconds interval;
    const uint16_t nreadings;
    const uint32_t keep_amount;
};

} // namespace ects::plugins::systemmonitor
