#include "Network.hpp"
#include "nlohmann/json.hpp"
#include "ros/ros.h"
#include <ifaddrs.h>

#include <array>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

static auto exec(const char *cmd) -> std::string {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

namespace ects::plugins::systemmonitor {
auto Network::get_adapters() -> std::vector<std::string> {
    std::vector<std::string> adapters;

    struct ifaddrs *ifaddr;
    if (getifaddrs(&ifaddr) == -1) {
        return {};
    }
    for (auto *ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) {
            continue;
        }
        if (ifa->ifa_addr->sa_family != AF_PACKET)
            continue;
        adapters.emplace_back(ifa->ifa_name);
    }
    freeifaddrs(ifaddr);
    return adapters;
}
auto Network::get_info(const std::string &adapter) -> NetworkInfoMessage {
    bool link_is_up = false;
    std::string ip_address;
    try {
        auto ip_result = exec(("ip -j addr show " + adapter).c_str());
        auto ip_result_json = nlohmann::json::parse(ip_result);

        link_is_up = ip_result_json[0]["operstate"] == "UP";
        ip_address = ip_result_json[0]["addr_info"][0]["local"];
    } catch (const std::exception &e) {
        ROS_ERROR_STREAM("Failed to get ip address for adapter "
                         << adapter << " : " << e.what());
    }
    if (!link_is_up) {
        return {adapter, "", "", "", false, ""};
    }
    std::string default_gateway;
    try {
        auto ip_route_result =
            exec(("ip -j route show default dev " + adapter).c_str());
        auto ip_route_result_json = nlohmann::json::parse(ip_route_result);
        default_gateway = ip_route_result_json[0]["gateway"];
    } catch (const std::exception &e) {
        ROS_ERROR_STREAM("Failed to get default gateway for adapter "
                         << adapter << " : " << e.what());
    }

    std::string dns_addresses;
    try {
        std::ifstream dns_file("/etc/resolv.conf");
        for (std::string line; std::getline(dns_file, line);) {
            if (line.starts_with("nameserver")) {
                dns_addresses += line.substr(11) + ",";
            }
        }
        dns_addresses.pop_back();
    } catch (const std::exception &e) {
        ROS_ERROR_STREAM("Failed while getting nameservers for adapter "
                         << adapter << " : " << e.what());
    }

    std::string wlan_ssid;
    if (is_ifname_wifi(adapter)) {
        try {
            wlan_ssid = exec(("iwgetid -r " + adapter).c_str());
        } catch (const std::exception &e) {
            ROS_INFO_STREAM("No wifi connection for adapter " << adapter);
        }
    }

    return {adapter,       ip_address, default_gateway,
            dns_addresses, link_is_up, wlan_ssid};
}

auto Network::is_ifname_wifi(const std::string &adapter) -> bool {
    // FIXME: There must be a better way to do this
    return adapter.starts_with("w");
}

auto Network::get_usage(const std::string &adapter) -> NetworkUsageMessage {
    int16_t signal_strength = 0;
    if (is_ifname_wifi(adapter)) {
        try {
            auto iw_result =
                exec(("iwconfig " + adapter +
                      " | sed -nr 's/^.*Signal level=([+-]?[0-9]+) dBm/\1/p'")
                         .c_str());
            signal_strength = std::stoi(iw_result);
        } catch (const std::exception &e) {
            ROS_INFO_STREAM("Failed to get signal strength for adapter "
                            << adapter << " : " << e.what());
        }
    }
    uint64_t rx_bytes = 0;
    uint64_t tx_bytes = 0;

    auto current_time = std::chrono::system_clock::now();

    try {
        auto ip_result = exec(("ip -j -s link show " + adapter).c_str());
        auto ip_result_json = nlohmann::json::parse(ip_result);
        rx_bytes = ip_result_json[0]["stats64"]["rx"]["bytes"];
        tx_bytes = ip_result_json[0]["stats64"]["tx"]["bytes"];

    } catch (const std::exception &e) {
        ROS_INFO_STREAM("Failed to get rx/tx bytes for adapter "
                        << adapter << " : " << e.what());
    }
    uint64_t upload_speed = 0;
    uint64_t download_speed = 0;

    if (last_measurements.contains(adapter)) {
        auto last_measurement = last_measurements[adapter];
        auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                             current_time - last_measurement.timestamp)
                             .count();
        if (time_diff > 0) {
            upload_speed = (tx_bytes - last_measurement.tx_bytes) / time_diff;
            download_speed = (rx_bytes - last_measurement.rx_bytes) / time_diff;
        }
    }
    last_measurements[adapter] = {current_time, rx_bytes, tx_bytes};
    // FIXME: The message spec says signal_strength is a float, but it's an
    // integer (dBm) in linux
    return {upload_speed, download_speed, (float)signal_strength};
}

} // namespace ects::plugins::systemmonitor
