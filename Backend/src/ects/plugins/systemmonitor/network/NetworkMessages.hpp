#pragma once
/* ECTS - NetworkMessages.hpp
 * Message definitions for network usage.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */
#include "../Usage.hpp"
#include "../aggregations/AggregationMessage.hpp"
#include "ects/AdapterList.h"
#include "ects/EmptyMessage.hpp"
#include "ects/NetworkInfo.h"
#include "ects/NetworkUsage.h"
#include "ects/NetworkUsageHistory.h"

#include "ects/MessageInterface.hpp"
#include <string>
#include <utility>

namespace ects::plugins::systemmonitor {
class NetworkUsageMessage : public UsageData {
  public:
    using ros_t = ects::NetworkUsage;
    using to_ros_t = ros_t;
    using history_ros_t = ects::NetworkUsageHistory;

    static auto to_ros(const NetworkUsageMessage &) -> ros_t;

    NetworkUsageMessage(uint64_t up_speed, uint64_t down_speed,
                        float wifi_signal_strength)
        : UsageData(), up_speed(up_speed), down_speed(down_speed),
          wifi_signal_strength{wifi_signal_strength} {}

    auto operator+(const NetworkUsageMessage &rhs) const
        -> NetworkUsageMessage {
        return {up_speed + rhs.up_speed, down_speed + rhs.down_speed,
                wifi_signal_strength + rhs.wifi_signal_strength};
    }
    auto operator/(const int &rhs) const -> NetworkUsageMessage {
        return {up_speed / rhs, down_speed / rhs, wifi_signal_strength / rhs};
    }

  private:
    uint64_t up_speed;
    uint64_t down_speed;
    float wifi_signal_strength;
};

class NetworkInfoMessage {
  public:
    using ros_t = ects::NetworkInfo;
    using to_ros_t = ros_t;

    static auto to_ros(const NetworkInfoMessage &) -> ros_t;

    NetworkInfoMessage(std::string interface_name, std::string ip_address,
                       std::string default_gateway, std::string dns_addresses,
                       bool link_is_up, std::string wlan_ssid)
        : interface_name(std::move(interface_name)),
          ip_address(std::move(ip_address)),
          default_gateway(std::move(default_gateway)),
          dns_addresses(std::move(dns_addresses)), link_is_up(link_is_up),
          wlan_ssid(std::move(wlan_ssid)) {}

  private:
    std::string interface_name;
    std::string ip_address;
    std::string default_gateway;
    std::string dns_addresses;
    bool link_is_up;
    std::string wlan_ssid;
};

struct NetworkAdapterService {};
using empty_adapterlist_request = EmptyMessage<ects::AdapterList::Request>;
class AdapterList {
  public:
    using ros_t = ects::AdapterList::Response;
    using to_ros_t = ros_t;

    static auto to_ros(const AdapterList &) -> ros_t;
    AdapterList(std::vector<std::string> adapter_list)
        : adapter_list(std::move(adapter_list)) {}

  private:
    std::vector<std::string> adapter_list;
};

} // namespace ects::plugins::systemmonitor

namespace ects {
using namespace ects::plugins::systemmonitor;

template <> struct server_traits<NetworkAdapterService> {
    using ros_t = ects::AdapterList;
    using request_from_ros_t = empty_adapterlist_request;
    using response_to_ros_t = ects::plugins::systemmonitor::AdapterList;
};
} // namespace ects
