#pragma once
#include "../Usage.hpp"
#include "../aggregations/AggregationMessage.hpp"
#include "ects/CpuPercentage.h"
#include "ects/CpuUsage.h"
#include "ects/CpuUsageHistory.h"

#include "ros/ros.h"
#include "ects/MessageInterface.hpp"
#include <string>
#include <utility>
#include <vector>

namespace ects::plugins::systemmonitor {
class CpuPercentageMessage : public UsageData {
  public:
    using ros_t = ects::CpuPercentage;
    using to_ros_t = ros_t;

    static auto to_ros(const CpuPercentageMessage &) -> ros_t;
    CpuPercentageMessage(float total) : UsageData(), percentage(total) {}
    auto get_value() const -> float { return percentage; }

    auto operator+(const CpuPercentageMessage &rhs) const
        -> CpuPercentageMessage {
        return {percentage + rhs.percentage};
    }
    auto operator/(const int &rhs) const -> CpuPercentageMessage {
        return {percentage / rhs};
    }

  private:
    float percentage;
};

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

    auto get_cpu_percentage() -> CpuPercentageMessage;

    auto operator+(const CpuUsageMessage &rhs) const -> CpuUsageMessage {
        std::vector<float> added_usages;
        if(_per_core_usage.size() != rhs._per_core_usage.size()) {
            // This should never happen, if Cpi::Cpu() calls Cpu::get_usage
            ROS_ERROR("CpuUsageMessage: Tried to add two CpuUsageMessages with different core counts");
            return *this;
        }
        for (int i = 0; i < _per_core_usage.size(); i++) {
            added_usages.push_back(_per_core_usage[i] + rhs._per_core_usage[i]);
        }
        std::vector<float> added_loads;
        for (int i = 0; i < _load_averages.size(); i++) {
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
