#include "Position.hpp"

extern "C" auto create_plugin_instance() -> ects::Plugin * {
    return new ects::plugins::position::Position();
}
namespace ects::plugins::position {
auto Position::init(ECTS &ects) -> void {
    data = {
        ects.ros_interface().create_topic_forwarder<nav_msgs::Odometry>(
            ects.config().get_value_or_default<std::string>(
                "/position/odometry_topic", default_odometry_topic),
            forward_odometry_topic),
        ects.ros_interface().create_topic_forwarder<sensor_msgs::Imu>(
            ects.config().get_value_or_default<std::string>(
                "/position/imu_topic", default_imu_topic),
            forward_imu_topic),
        ects.ros_interface().create_topic_forwarder<iosb_localization_filter::FilterState>(
            ects.config().get_value_or_default<std::string>(
                "/position/status_topic", default_status_topic),
            forward_status_topic),
    };
}
} // namespace ects::plugins::position
