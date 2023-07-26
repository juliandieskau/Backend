#pragma once

#include "WaypointList.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace ects::plugins::waypoints {

class WaypointListStorage {
  public:
    auto load(const std::string &filename) -> WaypointList;
    auto save(const std::string &filename, WaypointList list) -> void;
    auto list_directory() -> std::vector<std::string>;
    WaypointListStorage(const std::string &directory);

  private:
    std::filesystem::path directory;
};

} // namespace ects::plugins::waypoints
