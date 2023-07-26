#include "WaypointListStorage.hpp"
#include "ros/ros.h"

namespace ects::plugins::waypoints {
auto WaypointListStorage::load(std::string filename) -> WaypointList {
    // TODO
    return WaypointList();
}
auto WaypointListStorage::save(std::string filename, WaypointList list)
    -> void {
    // TODO
}
auto WaypointListStorage::list_directory() -> std::vector<std::string> {
    std::vector<std::string> files;
    for (const auto &e : directory) {
        if (is_regular_file(e))
            files.push_back(e.string());
    }
    return files;
}
WaypointListStorage::WaypointListStorage(const std::string &directory)
    : directory(directory) {
    if (!is_directory(this->directory)) {
        ROS_ERROR_STREAM("the given path \"" << directory
                                             << "\n isn't a directory");
        throw std::runtime_error("waypoint list path isn't a directory");
    }
}

} // namespace ects::plugins::waypoints