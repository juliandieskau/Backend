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
template <typename T>
auto make_provider_adapter(std::function<T()> f)
    -> std::unique_ptr<UsageProvider<T>> {
    return std::make_unique<UsageProviderAdapter<T>>(f);
}

auto SystemMonitor::init(ECTS &ects) -> void {
    ROS_INFO("Initializing SystemMonitor");
    auto interval = ects.config().get_value_or_default<float>(
        SystemMonitor::update_interval_key, 3.0);

    auto config_aggregations =
        ects.config().get_value_or_default<json::array_t>(
            SystemMonitor::aggregations_key, json::array());

    auto aggregations = std::vector<std::unique_ptr<AggregationStrategy>>();
    auto aggregations_ref = std::vector<AggregationStrategy *>();
    for (auto &entry : config_aggregations) {
        aggregations.push_back(std::move(aggregation_from_json(entry)));
        aggregations_ref.push_back((--aggregations.end())->get());
    }

    // TODO: we could refresh this every once in a while, instead of checking
    //       the mountpoints only at startup
    std::vector<UsageDataMonitor<DiskUsageMessage>> disk_monitors;
    auto list = Disk::get_mountpoints();
    for (const auto &mnt : list.get_mountpoints()) {
        auto topic_name = "disk/" + mnt.get_topic_name() + "/usage";
        auto data_provider = make_provider_adapter<DiskUsageMessage>(
            [mnt] { return Disk::get_usage(mnt); });
        disk_monitors.emplace_back(topic_name, aggregations_ref,
                                   std::move(data_provider), ects);
    }

    // TODO: we could refresh this every once in a while, instead of checking
    //       the network adapters only at startup
    std::vector<UsageDataMonitor<NetworkUsageMessage>> network_monitors;
    std::vector<Publisher<NetworkInfoMessage>> network_info_publishers;
    Network net;
    for (const auto &adapter : net.get_adapters()) {
        auto topic_name = "network/" + adapter + "/";
        network_monitors.emplace_back(
            topic_name + "usage", aggregations_ref,
            make_provider_adapter<NetworkUsageMessage>(
                [this, adapter] { return data->network.get_usage(adapter); }),
            ects);
        network_info_publishers.push_back(
            ects.ros_interface().create_publisher<NetworkInfoMessage>(
                "/ects/system/" + topic_name + "info"));
    }

    data = {
        ects.timer_manager().create_timer(
            interval,
            [this]() {
                data->cpu_monitor.step();
                for (auto &disk_monitor : data->disk_monitors)
                    disk_monitor.step();
                data->memory_monitor.step();
                for (auto &network_monitor : data->network_monitors)
                    network_monitor.step();
                data->program_monitor.step();
                publish_network_info();
            }),
        std::move(aggregations),
        ects.ros_interface().create_server<AggregationListService>(
            "/ects/system/aggregations"),
        {"cpu/usage", aggregations_ref, make_provider<Cpu, CpuUsageMessage>(),
         ects},
        ects.ros_interface().create_server<MountpointListService>(
            "/ects/system/disk/mountpoints"),
        std::move(disk_monitors),
        {"mem/usage", aggregations_ref,
         make_provider<Memory, MemoryUsageMessage>(), ects},
        ects.ros_interface().create_server<NetworkAdapterService>(
            "/ects/system/network/adapters"),
        std::move(network_monitors),
        std::move(network_info_publishers),
        {"processes/total", aggregations_ref,
         make_provider<Programs, ProcessTotalMessage>(), ects},
        net,
    };

    data->aggregation_list_server.register_service(
        [aggregations_ref](empty_aggregation_list_request) {
            return aggregations_ref;
        });
    data->mountpoint_list_server.register_service(
        [](empty_mountpoint_request) { return Disk::get_mountpoints(); });
    data->adapter_list_server.register_service(
        [this](empty_adapterlist_request) {
            return data->network.get_adapters();
        });
}

auto SystemMonitor::transmit_all() -> void {
    publish_network_info();
    data->cpu_monitor.transmit_all();
    data->memory_monitor.transmit_all();
    data->program_monitor.transmit_all();
    for (auto &disk_monitor : data->disk_monitors)
        disk_monitor.transmit_all();
    for (auto &network_monitor : data->network_monitors)
        network_monitor.transmit_all();
}

auto SystemMonitor::transmit(const std::string &topic_name) -> void {
    if (!topic_name.starts_with("/ects/system/"))
        return;
    if (topic_name.starts_with("/ects/system/network/") &&
        topic_name.ends_with("/info"))
        publish_network_info();
    data->cpu_monitor.transmit(topic_name);
    data->memory_monitor.transmit(topic_name);
    data->program_monitor.transmit(topic_name);
    for (auto &disk_monitor : data->disk_monitors)
        disk_monitor.transmit(topic_name);
    for (auto &network_monitor : data->network_monitors)
        network_monitor.transmit(topic_name);
}
auto SystemMonitor::publish_network_info() -> void {
    // FIXME: adapters might change
    auto adapters = data->network.get_adapters();
    for (size_t i = 0; i < adapters.size(); ++i) {
        data->network_info_publishers[i].publish(
            data->network.get_info(adapters[i]));
    }
}

} // namespace ects::plugins::systemmonitor
