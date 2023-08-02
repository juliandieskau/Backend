#pragma once
#include "DiskMessages.hpp"

namespace ects::plugins::systemmonitor {

class Mountpoint {
  public:
    auto get_mountpoint() -> std::string { return mountpoint; }
    auto get_topic_name() -> std::string { return topic_name; }
    Mountpoint(std::string mountpoint, std::string topic_name)
        : mountpoint(mountpoint), topic_name(topic_name) {}

  private:
    std::string mountpoint;
    std::string topic_name;
};

class Disk {
    auto get_mountpoints() -> MountpointList;
    auto get_usage(const Mountpoint &mountpoint) -> DiskUsageMessage;
};

} // namespace ects::plugins::systemmonitor
