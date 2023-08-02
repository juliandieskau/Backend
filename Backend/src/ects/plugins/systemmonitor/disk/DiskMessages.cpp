#include "DiskMessages.hpp"
#include "Disk.hpp"
namespace ects::plugins::systemmonitor {
auto DiskUsageMessage::to_ros(const DiskUsageMessage &msg)
    -> DiskUsageMessage::ros_t {
    DiskUsageMessage::ros_t ros_msg;
    ros_msg.size_total = msg._total_size;
    ros_msg.used = msg._used;
    return ros_msg;
}

auto DiskUsageHistoryMessage::to_ros(const DiskUsageHistoryMessage &msg)
    -> DiskUsageHistoryMessage::ros_t {
    DiskUsageHistoryMessage::ros_t ros_msg;
    ros_msg.aggregation = AggregationMessage::to_ros(msg._aggregation);
    std::vector<DiskUsageMessage::ros_t> usage_history;
    for (auto &usage : msg._usage_history) {
        usage_history.push_back(DiskUsageMessage::to_ros(usage));
    }
    ros_msg.measurements = usage_history;
    return ros_msg;
}

auto MountpointList::to_ros(const MountpointList &msg)
    -> MountpointList::ros_t {
    MountpointList::ros_t ros_msg;
    for (auto &mountpoint : msg.mountpoints) {
        ros_msg.mountpoint.push_back(mountpoint.get_mountpoint());
        ros_msg.rosname.push_back(mountpoint.get_topic_name());
    }
    return ros_msg;
}
} // namespace ects::plugins::systemmonitor
