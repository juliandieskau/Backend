#pragma once
#include "../Usage.hpp"
#include "../aggregations/AggregationMessage.hpp"
#include "ects/AdapterList.h"
#include "ects/EmptyMessage.hpp"
#include "ects/NetworkInfo.h"
#include "ects/NetworkUsage.h"
#include "ects/NetworkUsageHistory.h"

#include "ects/MessageInterface.hpp"
#include <string>

namespace ects::plugins::systemmonitor {
class NetworkUsageMessage : UsageData {
  public:
    using ros_t = ects::NetworkUsage;
    using from_ros_t = ros_t;
    static auto to_ros(const NetworkUsageMessage &) -> ros_t;

    NetworkUsageMessage(uint64_t up_speed, uint64_t down_speed,
                        float wifi_signal_strength)
        : UsageData(), _up_speed(up_speed),
          _down_speed(down_speed), _wifi_signal_strength{wifi_signal_strength} {
    }

    auto operator+(const NetworkUsageMessage &rhs) -> NetworkUsageMessage {
        return NetworkUsageMessage(
            _up_speed + rhs._up_speed, _down_speed + rhs._down_speed,
            _wifi_signal_strength + rhs._wifi_signal_strength);
    }
    auto operator/(const int &rhs) -> NetworkUsageMessage {
        return NetworkUsageMessage(_up_speed / rhs, _down_speed / rhs,
                                   _wifi_signal_strength / rhs);
    }

  private:
    uint64_t _up_speed;
    uint64_t _down_speed;
    float _wifi_signal_strength;
};

class NetworkUsageHistoryMessage {
  public:
    using ros_t = ects::NetworkUsageHistory;
    using from_ros_t = ros_t;
    static auto to_ros(const NetworkUsageHistoryMessage &) -> ros_t;

    NetworkUsageHistoryMessage(std::vector<NetworkUsageMessage> &usage_history,
                               AggregationMessage &aggregation)
        : _usage_history(usage_history), _aggregation(aggregation) {}

  private:
    std::vector<NetworkUsageMessage> &_usage_history;
    AggregationMessage &_aggregation;
};

class NetworkInfoMessage {
  public:
    using ros_t = ects::NetworkInfo;
    using from_ros_t = ros_t;

    static auto to_ros(const NetworkInfoMessage &) -> ros_t;

    NetworkInfoMessage(std::string interface_name, std::string ip_address,
                       std::string default_gateway, std::string dns_address,
                       bool link_is_up, std::string wlan_ssid)
        : interface_name(interface_name), ip_address(ip_address),
          default_gateway(default_gateway), dns_address(dns_address),
          link_is_up(link_is_up), wlan_ssid(wlan_ssid) {}

  private:
    std::string interface_name;
    std::string ip_address;
    std::string default_gateway;
    std::string dns_address;
    bool link_is_up;
    std::string wlan_ssid;
};

using empty_adapterlist_request = EmptyMessage<ects::AdapterList::Request>;
class AdapterList {
    public:
    using to_ros_t = ects::AdapterList::Response;
    static auto to_ros(const AdapterList &) -> to_ros_t;
    AdapterList(std::vector<std::string> adapter_list) : adapter_list(adapter_list) {}
    private:
    std::vector<std::string> adapter_list;

};

} // namespace ects::plugins::systemmonitor
