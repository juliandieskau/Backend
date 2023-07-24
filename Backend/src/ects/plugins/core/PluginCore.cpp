#include "PluginCore.hpp"

namespace ects::plugins::core {
auto PluginCore::init(ects::ECTS *ects) -> void {
    ROS_INFO_STREAM("Initializing PluginCore");
    auto timer = ects->m_timerManager->createTimer(
            30.0, [this] { ROS_INFO("Timer callback"); });
    ROS_INFO("Timer created");
    auto s = ects->m_rosIf.create_subscriber<retransmit>("/ects/retransmit");
    ROS_INFO("subscriber created");
    ROS_INFO_STREAM("a1 " << s.m_subscriber.getTopic());
    s.subscribe([](retransmit r) {
        if (r.get_topic().has_value())
            ROS_INFO_STREAM("retransmit topic: " << *r.get_topic());
        else
            ROS_INFO("retransmit all");
    });
    ROS_INFO("subscribed");
    ROS_INFO_STREAM("af " << s.m_subscriber.getTopic());
    data = {
           timer,
           s,
    };
    ROS_INFO_STREAM("af2 " << s.m_subscriber.getTopic());
    ROS_INFO_STREAM("al " << this->data->retransmit_subscriber.m_subscriber.getTopic());
    //ros::spin();
};

auto PluginCore::transmit_all() -> void {
    ROS_INFO_STREAM("Transmitting all");
};

auto PluginCore::transmit(std::string &topic_name) -> void {
    ROS_INFO_STREAM("Transmitting " << topic_name);
};

retransmit retransmit::from_ros(retransmit::from_ros_t ros_input) {
    return {ros_input.reload_all ? std::nullopt : std::optional(ros_input.topic)};
}

std::optional<std::string> retransmit::get_topic() {
    return topic_name;
}

retransmit::retransmit(std::optional<std::string> topic_name)
        : topic_name(topic_name) {
}

}; // namespace ects::plugins::core