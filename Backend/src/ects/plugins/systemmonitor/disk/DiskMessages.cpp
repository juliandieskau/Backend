#include "DiskMessages.hpp"
#include "Disk.hpp"
namespace ects::plugins::systemmonitor {
auto DiskUsageMessage::to_ros(const DiskUsageMessage &msg) -> ros_t {
    ros_t ros_msg;
    ros_msg.size_total = msg._total_size;
    ros_msg.used = msg._used;
    return ros_msg;
}

auto MountpointList::to_ros(const MountpointList &msg) -> ros_t {
    ros_t ros_msg;
    for (auto &mountpoint : msg.mountpoints) {
        ros_msg.mountpoint.push_back(mountpoint.get_mountpoint());
        ros_msg.rosname.push_back(mountpoint.get_topic_name());
    }
    return ros_msg;
}
MountpointList::MountpointList(std::vector<Mountpoint> mountpoints)
    : mountpoints(mountpoints) {}
auto MountpointList::get_mountpoints() -> const std::vector<Mountpoint> & {
    return mountpoints;
}
} // namespace ects::plugins::systemmonitor
