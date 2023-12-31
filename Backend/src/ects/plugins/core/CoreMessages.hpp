#pragma once
/* ECTS - CoreMessages.hpp
 * Message definitions used by the core plugin.
 *
 * (c) 2023 - Peter Bohner, Julian Dieskau, Ole Hocker, Kai Erik
 * Oelbracht, Liam Derk Rembold
 */

#include "ects/EmptyMessage.hpp"
#include "ects/MessageInterface.hpp"
#include <ects/ECTSStatus.h>
#include <ects/ForceRetransmit.h>
#include <std_msgs/Empty.h>

namespace ects::plugins::core {

using status_service_t = ects::ECTSStatus;
struct EctsStatusService {};

using empty_status_request = EmptyMessage<status_service_t::Request>;

struct EctsStatus {
    using to_ros_t = status_service_t::Response;

    static auto to_ros(const EctsStatus &) -> to_ros_t;
    EctsStatus(std::vector<std::string> loaded_plugins, std::string robot_name,
               std::string version);

  private:
    std::vector<std::string> loaded_plugins;
    std::string robot_name;
    std::string version;
};

struct retransmit {
    using from_ros_t = ects::ForceRetransmit;

    static auto from_ros(const from_ros_t &) -> retransmit;
    auto get_topic() -> std::optional<std::string>;

  private:
    explicit retransmit(std::optional<std::string> topic_name);

    std::optional<std::string> topic_name;
};

} // namespace ects::plugins::core

namespace ects {
using namespace ects::plugins::core;

template <> struct server_traits<EctsStatusService> {
    using ros_t = status_service_t;
    using request_from_ros_t = empty_status_request;
    using response_to_ros_t = EctsStatus;
};
} // namespace ects
