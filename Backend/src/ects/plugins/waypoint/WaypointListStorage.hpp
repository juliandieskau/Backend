#pragma once
/* ECTS - WaypointListStorage.hpp
 * This class provides methods to load and save WaypointLists from and to disk.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */

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
    auto open(const std::string &filename, std::ios::openmode mode)
        -> std::fstream;

    std::filesystem::path directory;
};

} // namespace ects::plugins::waypoints
