#include "WaypointManager.hpp"
#include "ros/ros.h"

extern "C" auto create_plugin_instance() -> ects::Plugin * {
    return new ects::plugins::waypoints::WaypointManager();
}
namespace ects::plugins::waypoints {

void WaypointManager::init(ECTS *ects1) {
    ROS_INFO("Initializing WaypointManager");
}
void WaypointManager::transmit_all() {}
void WaypointManager::transmit(const std::string &topic_name) {}

} // namespace ects::plugins::waypoints