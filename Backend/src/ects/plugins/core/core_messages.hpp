#pragma once

#include <ects/ECTSStatus.h>
#include <ects/ForceRetransmit.h>
#include <std_msgs/Empty.h>
#include "ects/message_interface.h"


using status_service_t = ects::ECTSStatus;
struct ects_status_service{};
struct ects_status_service_request {
    using from_ros_t = status_service_t::Request;
    static ects_status_service_request from_ros(from_ros_t);
};
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


struct empty_message {
    using from_ros_t = std_msgs::Empty;
    using to_ros_t = std_msgs::Empty;
};

namespace ects {

template<>
struct server_traits<ects_status_service> {
    using ros_t = status_service_t;
    using request_from_ros_t = ects_status_service_request;
    using response_to_ros_t = ects_status;
};

}