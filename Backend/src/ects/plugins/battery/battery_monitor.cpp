#include "battery_monitor.hpp"

namespace ects::plugins::battery {

auto battery_monitor::init(ECTS *ects) -> void {
    auto robot_battery_topic = ects->m_config->get_value<std::string>("/battery/topic");
    data = {
            std::nullopt,
            ects->m_rosIf.create_subscriber<battery_state>(robot_battery_topic),
    };
    data->battery_subscriber.subscribe([](battery_state state) {
        ROS_INFO("battery update");
    });
}

auto battery_monitor::transmit_all() -> void {

}

auto battery_monitor::transmit(std::string &topic_name) -> void {

}

}
