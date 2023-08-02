#pragma once
#include "../Usage.hpp"
#include "../aggregations/AggregationMessage.hpp"
#include "ects/CpuPercentage.h"
#include "ects/CpuUsage.h"
#include "ects/CpuUsageHistory.h"

#include "ects/MessageInterface.hpp"
#include <string>
#include <array>
#include <vector>

namespace ects::plugins::systemmonitor {
class CpuPercentageMessage : public UsageData {
  public:
    using ros_t = ects::CpuPercentage;
    using from_ros_t = ros_t;
    static auto to_ros(const CpuPercentageMessage &) -> ros_t;
    CpuPercentageMessage(float total) : UsageData(), percentage(total) {}
    auto get_value() const -> float { return percentage; }

    auto operator+(const CpuPercentageMessage &rhs) -> CpuPercentageMessage {
        return CpuPercentageMessage(percentage + rhs.percentage);
    }
    auto operator/(const int &rhs) -> CpuPercentageMessage {
        return CpuPercentageMessage(percentage / rhs);
    }

  private:
    float percentage;
};

class CpuUsageMessage : public UsageData {
  public:
    using ros_t = ects::CpuPercentage;
    using from_ros_t = ros_t;
    static auto to_ros(const CpuUsageMessage &) -> ros_t;

    CpuUsageMessage(float total, std::vector<float> per_core,
                    std::array<float, 3> loads)
        : UsageData(), _total_usage(total), _per_core_usage(per_core),
          _load_averages(loads) {}

    const auto get_cpu_percentage() -> CpuPercentageMessage;

    auto operator+(const CpuUsageMessage &rhs) -> CpuUsageMessage {
        std::vector<float> added_usages;
        for (int i = 0; i < _per_core_usage.size(); i++) {
            added_usages.push_back(_per_core_usage[i] + rhs._per_core_usage[i]);
        }
        std::array<float, 3> added_loads;
        for (int i = 0; i < _load_averages.size(); i++) {
            added_loads[i] = _load_averages[i] + rhs._load_averages[i];
        }
        return CpuUsageMessage(_total_usage + rhs._total_usage, added_usages,
                               added_loads);
    }
    auto operator/(const int &rhs) -> CpuUsageMessage {
        std::vector<float> divided_usages;
        for (int i = 0; i < _per_core_usage.size(); i++) {
            divided_usages.push_back(_per_core_usage[i] / rhs);
        }
        std::array<float, 3> divided_loads;
        for (int i = 0; i < _load_averages.size(); i++) {
            divided_loads[i] = _load_averages[i] / rhs;
        }

        return CpuUsageMessage(_total_usage / rhs, divided_usages,
                               divided_loads);
    }

  private:
    float _total_usage;
    std::vector<float> _per_core_usage;
    std::array<float, 3> _load_averages;
};

class CpuUsageHistoryMessage {
  public:
    using ros_t = ects::CpuUsageHistory;
    using from_ros_t = ros_t;
    static auto to_ros(const CpuUsageHistoryMessage &) -> ros_t;

    CpuUsageHistoryMessage(std::vector<CpuUsageMessage> &usage_history,
                           AggregationMessage &aggregation)
        : _usage_history(usage_history), _aggregation(aggregation) {}

  private:
    std::vector<CpuUsageMessage> &_usage_history;
    AggregationMessage &_aggregation;
};
} // namespace ects::plugins::systemmonitor
