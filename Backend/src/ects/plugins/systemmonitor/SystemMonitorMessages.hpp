#pragma once
#include "Usage.hpp"
#include "ects/CpuPercentage.h"
#include "ects/CpuUsage.h"
#include "ects/CpuUsageHistory.h"
#include "ects/DiskUsage.h"
#include "ects/DiskUsageHistory.h"
#include "ects/MemoryUsage.h"
#include "ects/MemoryUsageHistory.h"
#include "ects/NetworkInfo.h"
#include "ects/NetworkUsage.h"
#include "ects/NetworkUsageHistory.h"
#include "ects/ProcessTotal.h"
#include "ects/ProcessTotalHistory.h"

#include "ects/MessageInterface.hpp"
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
                    std::array<float, 3> loads) : UsageData(), _total_usage(total),
                                                  _per_core_usage(per_core),
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
        return MemoryUsageMessage(_total + rhs._total, _used + rhs._used,
                                  _free + rhs._free, _shared + rhs._shared,
                                  _buff_cache + rhs._buff_cache,
                                  _available + rhs._available,
                                  _swap_total + rhs._swap_total,
                                  _swap_used + rhs._swap_used,
                                  _swap_free + rhs._swap_free);
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

class NetworkUsageMessage : UsageData {
  public:
    using ros_t = ects::DiskUsage;
    using from_ros_t = ros_t;
    static auto to_ros(const DiskUsageMessage &) -> ros_t;

    NetworkUsageMessage(uint64_t up_speed, uint64_t down_speed,
                        float wifi_signal_strength)
        : UsageData(), _up_speed(up_speed),
          _down_speed(down_speed), _wifi_signal_strength{wifi_signal_strength} {
    }
    
    auto operator+(const NetworkUsageMessage &rhs) -> NetworkUsageMessage {
        return NetworkUsageMessage(_up_speed + rhs._up_speed,
                                   _down_speed + rhs._down_speed,
                                   _wifi_signal_strength + rhs._wifi_signal_strength);
    }
    auto operator/(const int &rhs) -> NetworkUsageMessage {
        return NetworkUsageMessage(_up_speed / rhs, _down_speed / rhs,
                                   _wifi_signal_strength / rhs);
    }

  private:
    uint64_t _up_speed;
    uint64_t _down_speed;
    float _wifi_signal_strength;
};

class NetworkUsageHistoryMessage {
  public:
    using ros_t = ects::NetworkUsageHistory;
    using from_ros_t = ros_t;
    static auto to_ros(const NetworkUsageHistoryMessage &) -> ros_t;

    NetworkUsageHistoryMessage(std::vector<NetworkUsageMessage> &usage_history,
                               AggregationMessage &aggregation)
        : _usage_history(usage_history), _aggregation(aggregation) {}

  private:
    std::vector<NetworkUsageMessage> &_usage_history;
    AggregationMessage &_aggregation;
};

} // namespace ects::plugins::systemmonitor
