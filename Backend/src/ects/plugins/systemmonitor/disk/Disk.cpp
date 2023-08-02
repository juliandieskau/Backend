#include "Disk.hpp"
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>

namespace ects::plugins::systemmonitor {
    auto Disk::get_mountpoints() -> MountpointList {
        // get all mountpoints in the system
        std::ifstream mountpoints("/proc/mounts");
        std::vector<Mountpoint> mountpoint_list;
        for(std::string line; std::getline(mountpoints, line);) {
            std::istringstream iss(line);
            std::string device, mountpoint;
            iss >> device >> mountpoint;
            //TODO: Is this a good idea?
            // ignoring non-physical devices
            if(mountpoint.find("/dev") == 0) { 
                auto topic_name = mountpoint;
                std::replace(topic_name.begin(), topic_name.end(), '/', '_');
                mountpoint_list.push_back(Mountpoint(mountpoint, topic_name));
            }
        }
        return MountpointList(mountpoint_list);
    }
    auto Disk::get_usage(const Mountpoint &mountpoint) -> DiskUsageMessage {
        auto mountpoint_name = mountpoint.get_mountpoint();
        std::filesystem::space_info space = std::filesystem::space(mountpoint_name);
        return DiskUsageMessage(space.capacity, space.free);
    }

} // namespace ects::plugins::systemmonitor
