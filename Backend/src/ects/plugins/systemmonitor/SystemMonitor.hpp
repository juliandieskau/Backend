#pragma once

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
        Server<AggregationListService> aggregation_list_server;
        UsageDataMonitor<CpuUsageMessage> cpu_monitor;
        Server<MountpointListService> mountpoint_list_server;
        std::vector<UsageDataMonitor<DiskUsageMessage>> disk_monitors;
        UsageDataMonitor<MemoryUsageMessage> memory_monitor;
        Server<NetworkAdapterService> adapter_list_server;
        std::vector<UsageDataMonitor<NetworkUsageMessage>> network_monitors;
        std::vector<Publisher<NetworkInfoMessage>> network_info_publishers;
        UsageDataMonitor<ProcessTotalMessage> program_monitor;
        Network network;
    };
    std::optional<data> data;
    void publish_network_info();
};

} // namespace ects::plugins::systemmonitor
