#pragma once
/* ECTS - Network.hpp
 * The class Network provides a way to query all available network adapters,
 * their current usage (given an adapter) as well as information about an
 * adapter.
 *
 * NetworkInfo contains all the information about an adapter, which can not
 * be aggregated, eg. the ip address.
 *
 * NetworkUsage contains the usage of an adapter, which change over time and are
 * aggregated, such as the upload and download speed.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */
#include "NetworkMessages.hpp"
#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

namespace ects::plugins::systemmonitor {
struct NetworkStat {
    std::chrono::time_point<std::chrono::system_clock> timestamp;
    uint64_t rx_bytes;
    uint64_t tx_bytes;
};
class Network {
  public:
    auto get_adapters() -> std::vector<std::string>;
    auto get_info(const std::string &adapter) -> NetworkInfoMessage;
    auto get_usage(const std::string &adapter) -> NetworkUsageMessage;

  private:
    static auto is_ifname_wifi(const std::string &ifname) -> bool;
    std::unordered_map<std::string, NetworkStat> last_measurements;
};

} // namespace ects::plugins::systemmonitor
