#include "SystemMonitor.hpp"
extern "C" auto create_plugin_instance() -> ects::Plugin * {
    return new ects::plugins::systemmonitor::SystemMonitor();
}
namespace ects::plugins::systemmonitor {

auto SystemMonitor::init(ECTS &ects) -> void {
    ROS_INFO("Initializing SystemMonitor");
    data = {};
}

auto SystemMonitor::transmit_all() -> void {}

auto SystemMonitor::transmit(const std::string &topic_name) -> void {}

} // namespace ects::plugins::systemmonitor
