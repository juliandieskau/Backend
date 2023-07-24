#pragma once

#include "ects/ECTS.hpp"
#include "ects/Plugin.hpp"
#include "ects/Timer.hpp"
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "ects/ForceRetransmit.h"
#include "ects/ECTSStatus.h"

#include <optional>

#include "ects/ECTS.hpp"
#include "ects/ECTSStatus.h"
namespace ects::plugins::core {
class PluginCore : public Plugin {
public:
    auto init(ECTS *) -> void override;

    auto transmit_all() -> void override;

    auto transmit(std::string &topic_name) -> void override;

private:
    struct data {
        std::shared_ptr<ects::Timer> m_timer;
        subscriber<struct retransmit> retransmit_subscriber;
        //server<> m_statusServer;
    };
    std::optional<data> data;
};

struct ects_status_service{};

struct retransmit {
    using from_ros_t = ects::ForceRetransmit;
    static retransmit from_ros(from_ros_t);
    std::optional<std::string> get_topic();

private:
    retransmit(std::optional<std::string> topic_name);
    std::optional<std::string> topic_name;
};




struct empty_message {
    using from_ros_t = std_msgs::Empty;
    using to_ros_t = std_msgs::Empty;
};

} // namespace ects::plugins::core

namespace ects {

template<>
struct server_traits<plugins::core::ects_status_service> {
    using ros_t = ects::ECTSStatus;
    //using
};

}