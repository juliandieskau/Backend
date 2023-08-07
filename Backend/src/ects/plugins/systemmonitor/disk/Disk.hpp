#pragma once
/* ECTS - Disk.hpp
 * The class Disk provides a method for returning all mountpoints,
 * as well as a method for returning the usage of a given mountpoint.
 * The usage is returned as a DiskUsageMessage, which consists of total and
 * free space.
 *
 * A Mountpoint consists of the filesystem mount-path and the topic name, in
 * which non alphanumeric characters are removed, such that this name can be
 * used in a ROS topic path, without confusion.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */
#include <utility>

#include "DiskMessages.hpp"

namespace ects::plugins::systemmonitor {

class Mountpoint {
  public:
    auto get_mountpoint() const -> std::string { return mountpoint; }
    auto get_topic_name() const -> std::string { return topic_name; }
    Mountpoint(std::string mountpoint, std::string topic_name)
        : mountpoint(std::move(mountpoint)), topic_name(std::move(topic_name)) {
    }

  private:
    std::string mountpoint;
    std::string topic_name;
};

class Disk {
  public:
    static auto get_mountpoints() -> MountpointList;
    static auto get_usage(const Mountpoint &mountpoint) -> DiskUsageMessage;
};

} // namespace ects::plugins::systemmonitor
