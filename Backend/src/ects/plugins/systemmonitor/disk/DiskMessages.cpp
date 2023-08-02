#include "DiskMessages.hpp"
#include "Disk.hpp"
namespace ects::plugins::systemmonitor {
auto DiskUsageMessage::to_ros(const DiskUsageMessage &msg) -> ros_t {
    ros_t ros_msg;
    ros_msg.size_total = msg._total_size;
    ros_msg.used = msg._used;
    return ros_msg;
}

auto DiskUsageHistoryMessage::to_ros(const DiskUsageHistoryMessage &msg)
    -> ros_t {
    ros_t ros_msg;
    ros_msg.aggregation = AggregationMessage::to_ros(msg._aggregation);
    for (auto &usage : msg._usage_history) {
        ros_msg.measurements.push_back(DiskUsageMessage::to_ros(usage));
    }
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
} // namespace ects::plugins::systemmonitor
