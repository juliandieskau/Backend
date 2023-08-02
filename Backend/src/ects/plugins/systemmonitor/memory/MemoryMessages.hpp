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
    using to_ros_t = ros_t;
    using history_ros_t = ects::MemoryUsageHistory;

    static auto to_ros(const MemoryUsageMessage &) -> ros_t;

    MemoryUsageMessage(uint64_t total, uint64_t used, uint64_t free,
                       uint64_t shared, uint64_t buff_cache, uint64_t available,
                       uint64_t swap_total, uint64_t swap_used,
                       uint64_t swap_free)
        : UsageData(), total(total), used(used), free(free), shared(shared),
          buff_cache(buff_cache), available(available), swap_total(swap_total),
          swap_used(swap_used), swap_free(swap_free) {}

    auto operator+(const MemoryUsageMessage &rhs) const -> MemoryUsageMessage {
        return {total + rhs.total,
                used + rhs.used,
                free + rhs.free,
                shared + rhs.shared,
                buff_cache + rhs.buff_cache,
                available + rhs.available,
                swap_total + rhs.swap_total,
                swap_used + rhs.swap_used,
                swap_free + rhs.swap_free};
    }
    auto operator/(const int &rhs) const -> MemoryUsageMessage {
        return {total / rhs,      used / rhs,       free / rhs,
                shared / rhs,     buff_cache / rhs, available / rhs,
                swap_total / rhs, swap_used / rhs,  swap_free / rhs};
    }

  private:
    uint64_t total;
    uint64_t used;
    uint64_t free;
    uint64_t shared;
    uint64_t buff_cache;
    uint64_t available;
    uint64_t swap_total;
    uint64_t swap_used;
    uint64_t swap_free;
};

} // namespace ects::plugins::systemmonitor
