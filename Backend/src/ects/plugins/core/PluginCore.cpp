#include "PluginCore.hpp"

namespace ects::plugins::core {
auto PluginCore::init(ects::ECTS *ects) -> void {
    ROS_INFO_STREAM("Initializing PluginCore");
    data = {
            ects->m_timerManager->createTimer(30.0, [] { ROS_INFO("Timer callback"); }),
            ects->m_rosIf.create_subscriber<retransmit>("/ects/retransmit"),
                    ects->m_rosIf.create_server<ects_status_service>("/ects/ects_status")
                    };
    data->retransmit_subscriber.subscribe([](retransmit r) {
        //TODO implement
        if (r.get_topic().has_value())
            ROS_INFO_STREAM("retransmit topic: " << *r.get_topic());
        else
            ROS_INFO("retransmit all");
    });
    data->status_server.register_service([](ects_status_service_request) -> ects_status {
        ROS_INFO("service called");
        //TODO implement
        return ects_status({"plug1", "plug2"}, "name", "ver");
    });
};

auto PluginCore::transmit_all() -> void {
    ROS_INFO_STREAM("Transmitting all");
};

auto PluginCore::transmit(std::string &topic_name) -> void {
    ROS_INFO_STREAM("Transmitting " << topic_name);
};

}; // namespace ects::plugins::core