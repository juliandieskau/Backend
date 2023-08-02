#pragma once
#include "../Usage.hpp"
#include "../aggregations/AggregationMessage.hpp"
#include "ects/MemoryUsage.h"
#include "ects/MemoryUsageHistory.h"

#include "ects/MessageInterface.hpp"
#include <string>

namespace ects::plugins::systemmonitor {

class MemoryUsageMessage : UsageData {
  public:
    using ros_t = ects::MemoryUsage;
    using from_ros_t = ros_t;
    static auto to_ros(const MemoryUsageMessage &) -> ros_t;

    MemoryUsageMessage(uint64_t total, uint64_t used, uint64_t free,
                       uint64_t shared, uint64_t buff_cache, uint64_t available,
                       uint64_t swap_total, uint64_t swap_used,
                       uint64_t swap_free)
        : UsageData(), _total(total), _used(used), _free(free), _shared(shared),
          _buff_cache(buff_cache), _available(available),
          _swap_total(swap_total), _swap_used(swap_used),
          _swap_free(swap_free) {}

    auto operator+(const MemoryUsageMessage &rhs) -> MemoryUsageMessage {
        return MemoryUsageMessage(
            _total + rhs._total, _used + rhs._used, _free + rhs._free,
            _shared + rhs._shared, _buff_cache + rhs._buff_cache,
            _available + rhs._available, _swap_total + rhs._swap_total,
            _swap_used + rhs._swap_used, _swap_free + rhs._swap_free);
    }
    auto operator/(const int &rhs) -> MemoryUsageMessage {
        return MemoryUsageMessage(_total / rhs, _used / rhs, _free / rhs,
                                  _shared / rhs, _buff_cache / rhs,
                                  _available / rhs, _swap_total / rhs,
                                  _swap_used / rhs, _swap_free / rhs);
    }

  private:
    uint64_t _total;
    uint64_t _used;
    uint64_t _free;
    uint64_t _shared;
    uint64_t _buff_cache;
    uint64_t _available;
    uint64_t _swap_total;
    uint64_t _swap_used;
    uint64_t _swap_free;
};

class MemoryUsageHistoryMessage {
  public:
    using ros_t = ects::MemoryUsageHistory;
    using from_ros_t = ros_t;
    static auto to_ros(const MemoryUsageHistoryMessage &) -> ros_t;

    MemoryUsageHistoryMessage(std::vector<MemoryUsageMessage> &usage_history,
                              AggregationMessage &aggregation)
        : _usage_history(usage_history), _aggregation(aggregation) {}

  private:
    std::vector<MemoryUsageMessage> &_usage_history;
    AggregationMessage &_aggregation;
};

} // namespace ects::plugins::systemmonitor
