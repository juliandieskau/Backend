#include "WaypointListStorage.hpp"
#include "ros/ros.h"
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace ects::plugins::waypoints {
auto WaypointListStorage::open(const std::string &filename,
                               std::ios::openmode mode) -> std::fstream {
    std::fstream f{};
    f.exceptions(std::ios_base::badbit | std::ios_base::failbit);
    f.open(directory / filename, mode);
    return f;
}

auto WaypointListStorage::load(const std::string &filename)
    -> WaypointList { // ...
    return json::parse(open(filename, std::ios_base::in));
}
auto WaypointListStorage::save(const std::string &filename, WaypointList list)
    -> void {
    json j = list;
    open(filename, std::ios_base::out | std::ios_base::trunc) << j;
}
auto WaypointListStorage::list_directory() -> std::vector<std::string> {
    std::vector<std::string> files;
    for (const auto &e : std::filesystem::directory_iterator(directory)) {
        if (is_regular_file(e))
            files.push_back(e.path().filename().string());
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