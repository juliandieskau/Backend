#include "Control.hpp"

extern "C" auto create_plugin_instance() -> ects::Plugin * {
    return new ects::plugins::control::Control();
}
namespace ects::plugins::control {

auto Control::init(ECTS &ects) -> void {
    ROS_INFO("Initializing Control");
    auto robot_control_topic =
        ects.config().get_value<std::string>("/control/topic");
    data = {
        ects.ros_interface().create_subscriber<Twist>(control_topic),
        ects.ros_interface().create_publisher<Twist>(robot_control_topic),
    };
    data->control_subscriber.subscribe([this](Twist twist) {
        ROS_INFO("impulse command");
        data->control_publisher.publish(twist);
    });
}

auto Control::transmit_all() -> void {}

auto Control::transmit(const std::string &topic_name) -> void {}

} // namespace ects::plugins::control
