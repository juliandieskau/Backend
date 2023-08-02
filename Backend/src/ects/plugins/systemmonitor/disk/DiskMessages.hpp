#pragma once
#include "../Usage.hpp"
#include "../aggregations/AggregationMessage.hpp"
#include "ects/DiskUsage.h"
#include "ects/DiskUsageHistory.h"
#include "ects/EmptyMessage.hpp"
#include "ects/MountpointList.h"

#include "ects/MessageInterface.hpp"
#include <array>
#include <string>
#include <vector>

namespace ects::plugins::systemmonitor {

class DiskUsageMessage : UsageData {
  public:
    using ros_t = ects::DiskUsage;
    using from_ros_t = ros_t;
    static auto to_ros(const DiskUsageMessage &) -> ros_t;

    DiskUsageMessage(uint64_t total_size, uint64_t used)
        : UsageData(), _total_size(total_size), _used(used) {}

    auto operator+(const DiskUsageMessage &rhs) -> DiskUsageMessage {
        return DiskUsageMessage(_total_size + rhs._total_size,
                                _used + rhs._used);
    }
    auto operator/(const int &rhs) -> DiskUsageMessage {
        return DiskUsageMessage(_total_size / rhs, _used / rhs);
    }

  private:
    uint64_t _total_size;
    uint64_t _used;
};

class DiskUsageHistoryMessage {
  public:
    using ros_t = ects::DiskUsageHistory;
    using from_ros_t = ros_t;
    static auto to_ros(const DiskUsageHistoryMessage &) -> ros_t;

    DiskUsageHistoryMessage(std::vector<DiskUsageMessage> &usage_history,
                            AggregationMessage &aggregation)
        : _usage_history(usage_history), _aggregation(aggregation) {}

  private:
    std::vector<DiskUsageMessage> &_usage_history;
    AggregationMessage &_aggregation;
};

class Mountpoint;
using empty_mountpoint_request = EmptyMessage<ects::MountpointList::Request>;
class MountpointList {
    using ros_t = ects::MountpointList::Response;
    static auto to_ros(const MountpointList &) -> ros_t;
    MountpointList(std::vector<Mountpoint>);

  private:
    std::vector<Mountpoint> mountpoints;
};
} // namespace ects::plugins::systemmonitor
