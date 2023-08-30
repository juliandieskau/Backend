#pragma once
/* ECTS - CpuMessages.hpp
 * Message definition for cpu usage.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */
#include "../Usage.hpp"
#include "../aggregations/AggregationMessage.hpp"
#include "ects/CpuUsage.h"
#include "ects/CpuUsageHistory.h"

#include "ects/MessageInterface.hpp"
#include "ros/ros.h"
#include <string>
#include <utility>
#include <vector>

namespace ects::plugins::systemmonitor {
class CpuUsageMessage : public UsageData {
  public:
    using ros_t = ects::CpuUsage;
    using to_ros_t = ros_t;
    using history_ros_t = ects::CpuUsageHistory;

    static auto to_ros(const CpuUsageMessage &) -> ros_t;

    CpuUsageMessage(float total, std::vector<float> per_core,
                    std::vector<float> loads)
        : UsageData(), _total_usage(total),
          _per_core_usage(std::move(per_core)),
          _load_averages(std::move(loads)) {}

    auto operator+(const CpuUsageMessage &rhs) const -> CpuUsageMessage {
        std::vector<float> added_usages;
        if (_per_core_usage.size() != rhs._per_core_usage.size()) {
            // This should never happen, if Cpi::Cpu() calls Cpu::get_usage
            ROS_ERROR("CpuUsageMessage: Tried to add two CpuUsageMessages with "
                      "different core counts");
            return *this;
        }
        added_usages.reserve(_per_core_usage.size());
        for (size_t i = 0; i < _per_core_usage.size(); i++) {
            added_usages.push_back(_per_core_usage[i] + rhs._per_core_usage[i]);
        }
        std::vector<float> added_loads;
        added_loads.reserve(_load_averages.size());
        for (size_t i = 0; i < _load_averages.size(); i++) {
            added_loads.push_back(_load_averages[i] + rhs._load_averages[i]);
        }
        return {_total_usage + rhs._total_usage, added_usages, added_loads};
    }
    auto operator/(const int &rhs) const -> CpuUsageMessage {
        std::vector<float> divided_usages;
        for (float i : _per_core_usage) {
            divided_usages.push_back(i / rhs);
        }
        std::vector<float> divided_loads;
        for (float load_average : _load_averages) {
            divided_loads.push_back(load_average / rhs);
        }

        return {_total_usage / rhs, divided_usages, divided_loads};
    }

  private:
    float _total_usage;
    std::vector<float> _per_core_usage;
    std::vector<float> _load_averages;
};

} // namespace ects::plugins::systemmonitor
