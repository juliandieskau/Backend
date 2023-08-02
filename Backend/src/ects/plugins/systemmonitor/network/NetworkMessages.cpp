#include "NetworkMessages.hpp"

namespace ects::plugins::systemmonitor {
auto NetworkUsageMessage::to_ros(const NetworkUsageMessage &msg)
    -> NetworkUsageMessage::ros_t {
    NetworkUsageMessage::ros_t ros_msg;
    ros_msg.down_speed = msg.down_speed;
    ros_msg.up_speed = msg.up_speed;
    ros_msg.wifi_signal_strength = msg.wifi_signal_strength;
    return ros_msg;
}

auto NetworkInfoMessage::to_ros(const NetworkInfoMessage &msg)
    -> NetworkInfoMessage::ros_t {
    NetworkInfoMessage::ros_t ros_msg;
    ros_msg.interface_name = msg.interface_name;
    ros_msg.human_readable_ip_address = msg.ip_address;
    ros_msg.default_gateway = msg.default_gateway;
    ros_msg.dns_addresses = msg.dns_addresses;
    ros_msg.link_is_up = msg.link_is_up;
    ros_msg.wlan_ssid = msg.wlan_ssid;
    return ros_msg;
}

auto AdapterList::to_ros(const AdapterList &msg) -> AdapterList::ros_t {
    AdapterList::ros_t ros_msg;
    ros_msg.adapters = msg.adapter_list;
    return ros_msg;
}
} // namespace ects::plugins::systemmonitor
