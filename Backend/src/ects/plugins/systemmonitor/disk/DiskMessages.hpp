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
    using to_ros_t = ros_t;
    using history_ros_t = ects::DiskUsageHistory;

    static auto to_ros(const DiskUsageMessage &) -> ros_t;

    DiskUsageMessage(uint64_t total_size, uint64_t used)
        : UsageData(), _total_size(total_size), _used(used) {}

    auto operator+(const DiskUsageMessage &rhs) const -> DiskUsageMessage {
        return {_total_size + rhs._total_size, _used + rhs._used};
    }
    auto operator/(const int &rhs) const -> DiskUsageMessage {
        return {_total_size / rhs, _used / rhs};
    }

  private:
    uint64_t _total_size;
    uint64_t _used;
};

class Mountpoint;
using empty_mountpoint_request = EmptyMessage<ects::MountpointList::Request>;
class MountpointList {
  public:
    using ros_t = ects::MountpointList::Response;
    using to_ros_t = ros_t;

    static auto to_ros(const MountpointList &) -> ros_t;
    MountpointList(std::vector<Mountpoint>);

  private:
    std::vector<Mountpoint> mountpoints;
};
} // namespace ects::plugins::systemmonitor
