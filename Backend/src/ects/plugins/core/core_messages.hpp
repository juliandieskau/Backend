#pragma once

#include <ects/ECTSStatus.h>
#include <ects/ForceRetransmit.h>
#include <std_msgs/Empty.h>
#include "ects/message_interface.h"
#include "ects/empty_message.h"

namespace ects::plugins::core {

using status_service_t = ects::ECTSStatus;
struct ects_status_service {
};

using empty_status_request = empty_message<status_service_t::Request>;

struct ects_status {
    using to_ros_t = status_service_t::Response;

    static to_ros_t to_ros(ects_status);
    ects_status(std::vector<std::string> loaded_plugins, std::string robot_name, std::string version);

private:
    std::vector<std::string> loaded_plugins;
    std::string robot_name;
    std::string version;
};

struct retransmit {
    using from_ros_t = ects::ForceRetransmit;

    static retransmit from_ros(from_ros_t);
    std::optional<std::string> get_topic();

private:
    explicit retransmit(std::optional<std::string> topic_name);

    std::optional<std::string> topic_name;
};

}


namespace ects {
using namespace ects::plugins::core;

template<>
struct server_traits<ects_status_service> {
    using ros_t = status_service_t;
    using request_from_ros_t = empty_status_request;
    using response_to_ros_t = ects_status;
};

}