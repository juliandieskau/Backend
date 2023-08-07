#include "Position.hpp"

extern "C" auto create_plugin_instance() -> ects::Plugin * {
    return new ects::plugins::position::Position();
}
namespace ects::plugins::position {
auto Position::init(ECTS &ects) -> void {
    data = {
        ects.ros_interface().create_topic_forwarder<nav_msgs::Odometry>(
            ects.config().get_value_or_default<std::string>(
                Position::odometry_topic_key, default_odometry_topic),
            forward_odometry_topic),
        ects.ros_interface().create_topic_forwarder<sensor_msgs::Imu>(
            ects.config().get_value_or_default<std::string>(
                Position::imu_topic_key, default_imu_topic),
            forward_imu_topic),
        ects.ros_interface()
            .create_topic_forwarder<iosb_localization_filter::FilterState>(
                ects.config().get_value_or_default<std::string>(
                    Position::status_topic_key, default_status_topic),
                forward_status_topic),
    };
}
auto Position::transmit_all() -> void {
    data->imu_forwarder.transmit_all();
    data->odometry_forwarder.transmit_all();
    data->status_forwarder.transmit_all();
}
auto Position::transmit(const std::string &topic_name) -> void {
    data->imu_forwarder.transmit(topic_name);
    data->odometry_forwarder.transmit(topic_name);
    data->status_forwarder.transmit(topic_name);
}
} // namespace ects::plugins::position
