#include "battery_monitor.hpp"

namespace ects::plugins::battery {

auto battery_monitor::init(ECTS *) -> void {
    ROS_INFO("init battery monitor");
}

auto battery_monitor::transmit_all() -> void {

}

auto battery_monitor::transmit(std::string &topic_name) -> void {

}

}
