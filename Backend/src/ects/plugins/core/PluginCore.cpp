#include "PluginCore.hpp"

extern "C" auto create_plugin_instance() -> ects::Plugin * {
    return new ects::plugins::core::PluginCore();
}
namespace ects::plugins::core {

auto PluginCore::init(ECTS &ects) -> void {
    ROS_INFO_STREAM("Initializing PluginCore");
    data = {
        ects.ros_interface().create_subscriber<retransmit>("/ects/retransmit"),
        ects.ros_interface().create_server<EctsStatusService>(
            "/ects/ects_status")};
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
            auto name =
                ects.config().get_value<std::string>("/core/robot_name");
            // TODO: determine version information
            return EctsStatus(plugins, name, "0.0.1-dev");
        });
}

auto PluginCore::transmit_all() -> void{}

auto PluginCore::transmit(const std::string &topic_name) -> void{}

} // namespace ects::plugins::core
