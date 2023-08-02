#include "SystemMonitor.hpp"
#include "aggregations/Aggregation.hpp"
#include "ects/ECTS.hpp"
#include "nlohmann/json.hpp"
#include "ros/ros.h"

extern "C" auto create_plugin_instance() -> ects::Plugin * {
    return new ects::plugins::systemmonitor::SystemMonitor();
}
namespace ects::plugins::systemmonitor {

template <typename T, typename U>
auto make_provider() -> std::unique_ptr<UsageProvider<U>> {
    return std::make_unique<T>();
}

auto SystemMonitor::init(ECTS &ects) -> void {
    ROS_INFO("Initializing SystemMonitor");
    auto interval = ects.config().get_value_or_default<float>(
        "/systemmonitor/update_interval", 3.0);

    auto config_aggregations =
        ects.config().get_value_or_default<json::array_t>("/aggregations",
                                                          json::array());

    auto aggregations = std::vector<AggregationStrategy *>();
    for (auto &entry : config_aggregations) {
        aggregations.push_back(aggregation_from_json(entry));
    }
    ROS_INFO_STREAM("aggr " << aggregations.size());
    data = {
        ects.timer_manager().create_timer(
            interval,
            [this]() {
                ROS_INFO("SystemMonitor collecting data");
                this->data->cpu_monitor.step();
            }),
        aggregations,
        {"cpu/usage", aggregations, make_provider<Cpu, CpuUsageMessage>(),
         ects},
    };
}

auto SystemMonitor::transmit_all() -> void {}

auto SystemMonitor::transmit(const std::string &topic_name) -> void {}

} // namespace ects::plugins::systemmonitor
