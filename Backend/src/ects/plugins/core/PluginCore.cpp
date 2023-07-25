#include "PluginCore.hpp"

namespace ects::plugins::core {
auto PluginCore::init(ects::ECTS *ects) -> void {
    ROS_INFO_STREAM("Initializing PluginCore");
    data = {
        ects->ros_interface().create_subscriber<retransmit>("/ects/retransmit"),
        ects->ros_interface().create_server<EctsStatusService>(
            "/ects/ects_status")};
    data->retransmit_subscriber.subscribe([](retransmit r) {
        // TODO implement
        if (r.get_topic().has_value())
            ROS_INFO_STREAM("retransmit topic: " << *r.get_topic());
        else
            ROS_INFO("retransmit all");
    });
    data->status_server.register_service(
        [](empty_status_request) -> EctsStatus {
            ROS_INFO("service called");
            // TODO implement
            return EctsStatus({"plug1", "plug2"}, "name", "ver");
        });
};

auto PluginCore::transmit_all() -> void {
    ROS_INFO_STREAM("Transmitting all");
};

auto PluginCore::transmit(std::string &topic_name) -> void {
    ROS_INFO_STREAM("Transmitting " << topic_name);
};

}; // namespace ects::plugins::core
