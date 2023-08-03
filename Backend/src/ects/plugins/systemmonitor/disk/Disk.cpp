#include "Disk.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace ects::plugins::systemmonitor {
auto Disk::get_mountpoints() -> MountpointList {
    // get all mountpoints in the system
    std::ifstream mountpoints("/proc/mounts");
    std::vector<Mountpoint> mountpoint_list;
    for (std::string line; std::getline(mountpoints, line);) {
        std::istringstream iss(line);
        std::string device, mountpoint;
        iss >> device >> mountpoint;
        // TODO: Is this a good idea?
        //  ignoring non-physical devices
        if (device.starts_with("/dev")) {
            std::string topic_name(mountpoint);
            topic_name.erase(std::remove_if(topic_name.begin(),
                                            topic_name.end(),
                                            [](auto const &c) -> bool {
                                                return !std::isalnum(c);
                                            }),
                             topic_name.end());
            mountpoint_list.emplace_back(mountpoint, topic_name);
        }
    }
    return {mountpoint_list};
}
auto Disk::get_usage(const Mountpoint &mountpoint) -> DiskUsageMessage {
    auto mountpoint_name = mountpoint.get_mountpoint();
    std::filesystem::space_info space = std::filesystem::space(mountpoint_name);
    return {space.capacity, space.free};
}

} // namespace ects::plugins::systemmonitor
