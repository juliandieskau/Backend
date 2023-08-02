#pragma once

#include "SystemMonitorMessages.hpp"
#include "ects/ECTS.hpp"
#include "ects/Plugin.hpp"

#include "UsageMonitor.hpp"
#include "aggregations/Aggregation.hpp"
#include "cpu/Cpu.hpp"
#include "disk/Disk.hpp"
#include "memory/Memory.hpp"
#include "network/Network.hpp"
#include "programs/Programs.hpp"

#include <memory>
#include <optional>

namespace ects::plugins::systemmonitor {
class SystemMonitor : public Plugin {
  public:
    auto init(ECTS &) -> void override;
    auto transmit_all() -> void override;
    auto transmit(const std::string &topic_name) -> void override;
    auto name() const -> const std::string override { return "systemmonitor"; }

  private:
    struct data {
        std::shared_ptr<ects::Timer> timer;
        std::vector<AggregationStrategy *> aggregations;
        UsageDataMonitor<CpuUsageMessage> cpu_monitor;
        Disk disk;
        Memory memory;
        Network network;
        Programs programs;
    };
    std::optional<data> data;
};

} // namespace ects::plugins::systemmonitor
