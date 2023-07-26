#include "WaypointManager.hpp"
#include "ros/ros.h"

namespace ects::plugins::waypoints {

void WaypointManager::init(ECTS *ects1) {
    ROS_INFO("Initializing WaypointManager");
}
void WaypointManager::transmit_all() {}
void WaypointManager::transmit(const std::string &topic_name) {}

} // namespace ects::plugins::waypoints