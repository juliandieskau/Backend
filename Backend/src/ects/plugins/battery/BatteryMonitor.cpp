#include "BatteryMonitor.hpp"

namespace ects::plugins::battery {

auto BatteryMonitor::init(ECTS *ects) -> void {
    ROS_INFO("Initializing Plugin BatteryMonitor");
    auto robot_battery_topic =
        ects->config().get_value<std::string>("/battery/topic");
    data = {
        std::nullopt,
        ects->ros_interface().create_subscriber<BatteryState>(
            robot_battery_topic),
        ects->ros_interface().create_publisher<BatteryState>(
            battery_state_topic),
        ects->ros_interface().create_publisher<ChargePercentage>(
            percentage_topic),
        ects->ros_interface().create_publisher<EstimatedRuntime>(runtime_topic),
        ects->ros_interface().create_publisher<Warning>(warning_topic),
    };
    data->battery_subscriber.subscribe([this](BatteryState state) {
        data->state = state;
        transmit_all();
    });
}

auto BatteryMonitor::transmit_all() -> void {
    if (data->state.has_value()) {
        publish_battery_state();
        publish_percentage();
        publish_runtime();
        publish_warning();
    }
}

auto BatteryMonitor::transmit(const std::string &topic_name) -> void {
    if (data->state.has_value()) {
        if (topic_name == battery_state_topic)
            publish_battery_state();
        if (topic_name == percentage_topic)
            publish_percentage();
        if (topic_name == runtime_topic)
            publish_runtime();
        if (topic_name == warning_topic)
            publish_warning();
    }
}

auto BatteryMonitor::publish_battery_state() -> void {
    data->battery_state_publisher.publish(*data->state);
}

auto BatteryMonitor::publish_percentage() -> void {
    data->percentage_publisher.publish(data->state->get_charge());
}

auto BatteryMonitor::publish_runtime() -> void {
    data->runtime_publisher.publish(data->state->get_estimated_runtime());
}

auto BatteryMonitor::publish_warning() -> void {
    data->warning_publisher.publish(data->state->is_critical());
}

} // namespace ects::plugins::battery
