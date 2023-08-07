#include "PluginCore.hpp"

extern "C" auto create_plugin_instance() -> ects::Plugin * {
    return new ects::plugins::core::PluginCore();
}
namespace ects::plugins::core {

auto PluginCore::init(ECTS &ects) -> void {
    ROS_INFO_STREAM("Initializing PluginCore");
    data = {ects.ros_interface().create_subscriber<retransmit>(
                PluginCore::retransmit_topic),
            ects.ros_interface().create_server<EctsStatusService>(
                PluginCore::status_topic)};
    data->retransmit_subscriber.subscribe([&](retransmit r) {
        for (auto &p : ects.get_plugins()) {
            if (r.get_topic().has_value())
                p->transmit(*r.get_topic());
            else
                p->transmit_all();
        }
        if (r.get_topic().has_value())
            ROS_INFO_STREAM("retransmitted topic: " << *r.get_topic());
        else
            ROS_INFO("retransmitted all");
    });
    data->status_server.register_service(
        [&](empty_status_request) -> EctsStatus {
            std::vector<std::string> plugins;
            for (auto &p : ects.get_plugins()) {
                plugins.push_back(p->name());
            }
            auto name = ects.config().get_value<std::string>(
                PluginCore::robot_name_key);
            // TODO: determine version information
            return {plugins, name, PluginCore::backend_version};
        });
}

auto PluginCore::transmit_all() -> void {}

auto PluginCore::transmit(const std::string &topic_name) -> void {}

} // namespace ects::plugins::core
